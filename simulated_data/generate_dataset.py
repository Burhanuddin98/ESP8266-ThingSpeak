"""
Synthetic reading database for the IoT MEMS wireless inclinometer
(ESP8266 + MPU-6050 + ThingSpeak) described in
"Monitoring of inclinations of shoring walls ... using MEMS based wireless inclinometers".

The generator reproduces what the *corrected* firmware (Inclinometer/Inclinometer.ino)
actually publishes to ThingSpeak every 15 s: four fields per node
    field1 = fused roll   (net inclination about X, deg)   <- complementary filter
    field2 = fused pitch  (net inclination about Y, deg)
    field3 = accel-only roll (deg)  <- unfiltered reference, noisier/spikier
    field4 = die temperature (degC)

Deployment reproduced from the paper:
    9 nodes total: Western wall (55 m x 10 m) W1..W5, Northern wall (35 m x 10 m) N1..N4
    65-day monsoon campaign, 15 s sampling -> 5760 obs/node/day
    2 deg operational warning threshold
    baseline no-load inclination already subtracted (values are NET inclination)

The daily-mean trajectories are constrained so that
    mean over days  1..30  ~ paper Table 1  (first observation set)
    mean over days 31..65  ~ paper Table 2  (second observation set)
and drilling / rainfall coupling reproduces the reported peaks, the higher
response of the drilling-adjacent sensors (S3/S4), and the tree-shielded
low response of W5 / N4. Rainfall and temperature series are synthetic
(plausible Mumbai monsoon), documented as such.

Outputs (in this directory):
    inclinometer_sim.sqlite   full 15 s reading database (all nodes)
    daily_summary.csv         per node/day aggregates (mean/max/min/%>thr, rain, temp)
    sensors_metadata.csv      node geometry + coupling table
    channel_W3_feed_sample.csv ThingSpeak-format feed, node W3, first 3 days
    validation.png            daily inclination vs threshold + rainfall (paper Fig 15/16 style)
    reconciliation.txt        achieved period means/peaks vs paper Tables 1/2

Usage:
    python generate_dataset.py [--interval 15] [--days 65] [--seed 42]
"""

import argparse
import csv
import os
import sqlite3
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))

# --------------------------------------------------------------------------
# Node definitions.  drill_coupling folds in distance-to-drilling + shielding;
# it scales both the excavation-induced daily response and the impact spikes.
# --------------------------------------------------------------------------
# name, wall, position_m (along wall), distance_to_drill_m, drill_coupling,
# retained_soil_m, tree_shielded, table1_mean, table2_mean, peak_target
NODES = [
    ("W1", "Western", 12.42, 18.58, 0.55, 0.61, False,  1.31, -1.31, 2.80),
    ("W2", "Western", 19.92, 11.08, 0.65, 0.61, False,  0.76,  1.13, 3.00),
    ("W3", "Western", 27.42,  3.58, 1.00, 0.61, False,  1.80,  2.30, 3.08),
    ("W4", "Western", 34.92,  3.92, 0.95, 0.61, False,  1.59,  2.20, 3.28),
    ("W5", "Western", 42.42, 11.42, 0.30, 0.61, True,   0.44,  0.75, 1.33),
    ("N1", "Northern", 7.08, 13.92, 0.50, 0.91, False,  0.49, -0.36, 1.73),
    ("N2", "Northern", 14.08, 6.92, 0.60, 0.91, False,  0.33,  0.39, 2.88),
    ("N3", "Northern", 21.08, 0.08, 1.00, 0.91, False,  1.26,  1.42, 3.50),
    ("N4", "Northern", 28.08, 7.08, 0.35, 0.91, True,   0.67, -0.48, 1.91),
]

THRESHOLD_DEG = 2.0
START_DATE = np.datetime64("2023-06-01T00:00:00")  # Mumbai monsoon onset


def build_environment(n_days, rng):
    """Synthetic daily rainfall (mm), cumulative rainfall (mm) and mean temp (degC)."""
    # Monsoon: mostly wet with intermittent heavy bursts, a couple of dry spells.
    wet = rng.random(n_days) < 0.72
    rainfall = np.where(wet, rng.gamma(shape=1.6, scale=6.0, size=n_days), 0.0)
    # Two heavier monsoon surges.
    for centre in (18, 47):
        span = np.arange(n_days)
        rainfall += 40.0 * np.exp(-0.5 * ((span - centre) / 3.0) ** 2) * (rng.random(n_days) < 0.6)
    rainfall = np.clip(rainfall, 0.0, 95.0)
    cum = np.cumsum(rainfall)
    # Temperature ~28 degC, cooler on heavy-rain days (paper Table 3 sits 27-29).
    temp = 28.4 - 0.02 * rainfall + rng.normal(0.0, 0.4, n_days)
    temp = np.clip(temp, 26.5, 30.0)
    return rainfall, cum, temp


def rain_response(rainfall, rng):
    """Pore-pressure build-up: rainfall convolved with a causal decaying kernel."""
    k = np.exp(-np.arange(0, 12) / 3.5)          # ~3.5 day dissipation
    k /= k.sum()
    resp = np.convolve(rainfall, k, mode="full")[: len(rainfall)]
    resp = resp / (resp.max() + 1e-9)             # normalise to [0,1]
    return resp


def drill_schedule(n_days, rng):
    """Per-day excavation intensity in [0,1]; drilling paused on ~Sundays."""
    day = np.arange(n_days)
    ramp = np.clip(day / 20.0, 0, 1)              # ramp up over first 20 days
    taper = np.clip((n_days - day) / 12.0, 0, 1)  # taper over last 12 days
    intensity = ramp * taper * (0.7 + 0.3 * rng.random(n_days))
    intensity[(day % 7) == 6] = 0.0               # rest day
    return np.clip(intensity, 0.0, 1.0)


def daily_means(node, rainfall_resp, drill_int, n_days, rng):
    """Construct a net daily-mean inclination curve that matches the two
    period means (Table 1 / Table 2) and carries rain + drill structure."""
    _, _, _, _, coupling, _, shielded, m1, m2, peak = node
    shield = 0.35 if shielded else 1.0
    creep = np.cumsum(drill_int) / max(np.cumsum(drill_int)[-1], 1e-9)  # 0..1 structural creep
    raw = (0.9 * coupling * shield * drill_int
           + 0.7 * coupling * shield * rainfall_resp
           + 0.6 * creep
           + rng.normal(0.0, 0.05, n_days))
    a = slice(0, 30)
    b = slice(30, n_days)
    # Offset each period toward its Table mean, but ramp the offset across the
    # day-30 boundary with a logistic so sign reversals (e.g. W1: +1.31 -> -1.31)
    # transition over ~3 days instead of a physically impossible one-day step.
    off_a = m1 - raw[a].mean()
    off_b = m2 - raw[b].mean()
    day = np.arange(n_days)
    ramp = 1.0 / (1.0 + np.exp(-(day - 30) / 1.5))
    curve = raw + off_a + (off_b - off_a) * ramp
    # Nudge the campaign peak toward the reported value with a narrow event,
    # then restore the period mean so Tables 1/2 stay matched.
    peak_day = int(np.argmax(curve))
    gap = peak - curve[peak_day]
    if gap > 0:
        bump = gap * np.exp(-0.5 * ((np.arange(n_days) - peak_day) / 1.2) ** 2)
        seg = a if peak_day < 30 else b
        curve += bump
        curve[seg] -= bump[seg].mean()
    return curve


def gen_node_day(mean_val, drill_i, temp_day, coupling, shielded, n_per_day, rng):
    """15 s within-day series for one node/day. Returns f1,f2,f3,f4 arrays."""
    shield = 0.35 if shielded else 1.0
    sec = np.arange(n_per_day) * (86400 // n_per_day)
    hour = sec / 3600.0
    work = (hour >= 9.0) & (hour < 18.0)         # drilling hours

    diurnal = 0.015 * np.sin(2 * np.pi * (hour - 6) / 24.0)   # thermal tilt
    cf_noise = rng.normal(0.0, 0.012, n_per_day)             # fused-estimate noise

    # Impact / vibration residual that survives the DLPF + complementary filter.
    resid = np.zeros(n_per_day)
    small = work & (rng.random(n_per_day) < 0.05 * drill_i)
    resid[small] += rng.normal(0.035, 0.02, small.sum())
    big = work & (rng.random(n_per_day) < 0.0018 * drill_i)  # occasional impacts (>thr possible)
    resid[big] += rng.uniform(0.15, 0.55, big.sum())
    resid *= coupling * shield

    f1 = mean_val + diurnal + cf_noise + resid                       # fused roll
    # Accelerometer-only channel: no gyro smoothing, full HF pass-through.
    f3 = (mean_val + diurnal + rng.normal(0.0, 0.06, n_per_day)
          + 2.4 * resid + (work * rng.normal(0.0, 0.04, n_per_day)))
    # Pitch: smaller, weakly correlated secondary axis.
    f2 = 0.18 * (f1 - mean_val) + 0.08 * mean_val + rng.normal(0.0, 0.02, n_per_day)
    # Temperature: daily mean + afternoon-peaking diurnal swing.
    f4 = temp_day + 1.8 * np.sin(2 * np.pi * (hour - 15) / 24.0) + rng.normal(0.0, 0.15, n_per_day)
    return f1, f2, f3, f4


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--interval", type=int, default=15, help="sampling interval, seconds")
    ap.add_argument("--days", type=int, default=65)
    ap.add_argument("--seed", type=int, default=42)
    args = ap.parse_args()

    n_days = args.days
    n_per_day = 86400 // args.interval
    rng = np.random.default_rng(args.seed)

    rainfall, cum_rain, temp = build_environment(n_days, rng)
    rresp = rain_response(rainfall, rng)
    drill_int = drill_schedule(n_days, rng)

    db_path = os.path.join(HERE, "inclinometer_sim.sqlite")
    if os.path.exists(db_path):
        os.remove(db_path)
    con = sqlite3.connect(db_path)
    cur = con.cursor()
    cur.executescript("""
        PRAGMA journal_mode=OFF;
        PRAGMA synchronous=OFF;
        CREATE TABLE sensors(
            sensor_id INTEGER PRIMARY KEY, name TEXT, wall TEXT,
            position_m REAL, distance_to_drill_m REAL, drill_coupling REAL,
            retained_soil_m REAL, tree_shielded INTEGER, thingspeak_channel TEXT);
        CREATE TABLE readings(
            id INTEGER PRIMARY KEY, sensor_id INTEGER, created_at TEXT, entry_id INTEGER,
            field1_roll_deg REAL, field2_pitch_deg REAL,
            field3_accel_roll_deg REAL, field4_temp_c REAL,
            threshold_exceeded INTEGER);
        CREATE TABLE daily_summary(
            sensor_id INTEGER, day INTEGER, date TEXT, rainfall_mm REAL,
            cum_rainfall_mm REAL, temp_c REAL, mean_incl_deg REAL,
            max_incl_deg REAL, min_incl_deg REAL, pct_over_threshold REAL);
    """)

    for sid, node in enumerate(NODES, start=1):
        name, wall, pos, dist, coup, soil, shielded = node[:7]
        cur.execute("INSERT INTO sensors VALUES (?,?,?,?,?,?,?,?,?)",
                    (sid, name, wall, pos, dist, coup, soil, int(shielded),
                     f"ZWEI_{name}"))

    row_id = 0
    achieved = {}
    for sid, node in enumerate(NODES, start=1):
        name = node[0]
        coup = node[4]
        shielded = node[6]
        dmeans = daily_means(node, rresp, drill_int, n_days, rng)
        entry = 0
        day_stats = []
        node_curve = []
        for d in range(n_days):
            day0 = START_DATE + np.timedelta64(d, "D")
            ts = (day0 + np.arange(n_per_day) * np.timedelta64(args.interval, "s"))
            ts_str = np.datetime_as_string(ts, unit="s")            # 'YYYY-MM-DDTHH:MM:SS'
            f1, f2, f3, f4 = gen_node_day(dmeans[d], drill_int[d], temp[d],
                                          coup, shielded, n_per_day, rng)
            over = (f1 > THRESHOLD_DEG).astype(np.int8)
            rows = []
            for i in range(n_per_day):
                row_id += 1
                entry += 1
                rows.append((row_id, sid, ts_str[i].replace("T", " "), entry,
                             round(float(f1[i]), 3), round(float(f2[i]), 3),
                             round(float(f3[i]), 3), round(float(f4[i]), 2),
                             int(over[i])))
            cur.executemany("INSERT INTO readings VALUES (?,?,?,?,?,?,?,?,?)", rows)
            node_curve.append(f1)
            day_stats.append((float(f1.mean()), float(f1.max()), float(f1.min()),
                              100.0 * over.mean()))
        # daily_summary rows
        for d, (mn, mx, lo, pct) in enumerate(day_stats):
            date = np.datetime_as_string(START_DATE + np.timedelta64(d, "D"), unit="D")
            cur.execute("INSERT INTO daily_summary VALUES (?,?,?,?,?,?,?,?,?,?)",
                        (sid, d + 1, str(date), round(float(rainfall[d]), 2),
                         round(float(cum_rain[d]), 1), round(float(temp[d]), 2),
                         round(mn, 4), round(mx, 4), round(lo, 4), round(pct, 3)))
        allv = np.concatenate(node_curve)
        achieved[name] = {
            "m1": float(np.mean([s[0] for s in day_stats[:30]])),
            "m2": float(np.mean([s[0] for s in day_stats[30:]])),
            "dpeak": float(max(s[0] for s in day_stats)),   # daily-mean peak (paper-comparable)
            "ipeak": float(allv.max()),                     # instantaneous 15 s max (incl. impacts)
            "t1": node[7], "t2": node[8], "pk": node[9],
        }
    cur.execute("CREATE INDEX idx_readings ON readings(sensor_id, created_at)")
    con.commit()

    # ---- daily_summary.csv ----
    with open(os.path.join(HERE, "daily_summary.csv"), "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["sensor", "wall", "day", "date", "rainfall_mm", "cum_rainfall_mm",
                    "temp_c", "mean_incl_deg", "max_incl_deg", "min_incl_deg", "pct_over_threshold"])
        for row in cur.execute("""SELECT s.name, s.wall, d.day, d.date, d.rainfall_mm,
                                  d.cum_rainfall_mm, d.temp_c, d.mean_incl_deg, d.max_incl_deg,
                                  d.min_incl_deg, d.pct_over_threshold
                                  FROM daily_summary d JOIN sensors s ON s.sensor_id=d.sensor_id
                                  ORDER BY d.sensor_id, d.day"""):
            w.writerow(row)

    # ---- sensors_metadata.csv ----
    with open(os.path.join(HERE, "sensors_metadata.csv"), "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["sensor", "wall", "position_m", "distance_to_drill_m", "drill_coupling",
                    "retained_soil_m", "tree_shielded", "thingspeak_channel"])
        for row in cur.execute("""SELECT name, wall, position_m, distance_to_drill_m,
                                  drill_coupling, retained_soil_m, tree_shielded,
                                  thingspeak_channel FROM sensors ORDER BY sensor_id"""):
            w.writerow(row)

    # ---- ThingSpeak-format sample feed (W3, first 3 days) ----
    with open(os.path.join(HERE, "channel_W3_feed_sample.csv"), "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["created_at", "entry_id", "field1", "field2", "field3", "field4"])
        for row in cur.execute("""SELECT created_at, entry_id, field1_roll_deg, field2_pitch_deg,
                                  field3_accel_roll_deg, field4_temp_c FROM readings
                                  WHERE sensor_id=3 ORDER BY entry_id LIMIT ?""",
                               (3 * n_per_day,)):
            w.writerow(row)

    # ---- reconciliation.txt ----
    lines = ["Reconciliation vs paper Tables 1/2 and reported peaks (net inclination, deg)",
             "  P1/P2 = 30-day period means (Table 1 / Table 2, the hard anchor)",
             "  daily_peak = max daily-mean = comparable to the paper's plotted peaks",
             "  inst_peak  = max raw 15 s sample (includes drilling impact transients)",
             "",
             "sensor  P1_tgt  P1_sim   P2_tgt  P2_sim   peak_tgt  daily_peak  inst_peak"]
    for name, a in achieved.items():
        lines.append(f"{name:6s}  {a['t1']:6.2f}  {a['m1']:6.3f}   {a['t2']:6.2f}  "
                     f"{a['m2']:6.3f}   {a['pk']:7.2f}   {a['dpeak']:8.3f}   {a['ipeak']:8.3f}")
    recon = "\n".join(lines)
    with open(os.path.join(HERE, "reconciliation.txt"), "w") as fh:
        fh.write(recon + "\n")
    print(recon)

    # ---- validation.png ----
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        days = np.arange(1, n_days + 1)
        fig, axes = plt.subplots(1, 2, figsize=(13, 5), sharey=True)
        for ax, wall, ids in ((axes[0], "Western", range(1, 6)),
                              (axes[1], "Northern", range(6, 10))):
            axr = ax.twinx()
            axr.bar(days, rainfall, color="0.8", width=0.9, zorder=0)
            axr.set_ylabel("Rainfall (mm)")
            axr.set_ylim(0, rainfall.max() * 3)
            for sid in ids:
                mean_series = [r[0] for r in cur.execute(
                    "SELECT mean_incl_deg FROM daily_summary WHERE sensor_id=? ORDER BY day", (sid,))]
                ax.plot(days, mean_series, lw=1.4, zorder=3, label=NODES[sid - 1][0])
            ax.axhline(THRESHOLD_DEG, color="r", ls="--", lw=1.2, zorder=2, label="threshold")
            ax.set_title(f"{wall} wall")
            ax.set_xlabel("Day")
            ax.set_zorder(axr.get_zorder() + 1)
            ax.patch.set_visible(False)
            ax.legend(loc="upper left", fontsize=8)
        axes[0].set_ylabel("Net inclination (deg)")
        fig.tight_layout()
        fig.savefig(os.path.join(HERE, "validation.png"), dpi=130)
        print("wrote validation.png")
    except Exception as e:  # plotting is best-effort
        print(f"plot skipped: {e}")

    con.close()
    size_mb = os.path.getsize(db_path) / 1e6
    total = len(NODES) * n_days * n_per_day
    print(f"\ninclinometer_sim.sqlite: {total:,} readings across {len(NODES)} nodes, "
          f"{size_mb:.1f} MB")


if __name__ == "__main__":
    main()
