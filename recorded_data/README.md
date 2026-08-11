# Reading database — IoT MEMS wireless inclinometer

Reading database for the ESP8266 + MPU-6050 + ThingSpeak shoring-wall
inclinometer described in *"Monitoring of inclinations of shoring walls for
safety of substructure excavation work using MEMS based wireless
inclinometers."*

Covers the full 9-node / 65-day campaign at 15 s cadence, in the form the
firmware (`Inclinometer/Inclinometer.ino`) posts to ThingSpeak.

## Fields

Each node is one ESP8266 NodeMCU + MPU-6050. The firmware fuses the
accelerometer tilt and integrated gyro rate through a complementary filter and
posts four ThingSpeak fields every 15 s:

| Field  | Meaning                                   | Unit |
|--------|-------------------------------------------|------|
| field1 | fused roll  = net inclination about X     | deg  |
| field2 | fused pitch = net inclination about Y     | deg  |
| field3 | accelerometer-only roll (unfiltered ref)  | deg  |
| field4 | die temperature                           | °C   |

`field3` is the raw accelerometer estimate taken before gyro fusion, so it is
noisier and spikier than `field1`; carrying both makes the effect of the
complementary filter visible.

## Deployment

- **9 nodes.** Western wall (55 m × 10 m): W1–W5, ~7.5 m spacing. Northern wall
  (35 m × 10 m): N1–N4, ~7.0 m spacing.
- **65-day** monsoon campaign, **15 s** sampling → **5 760 readings/node/day**,
  **3 369 600 readings** total, spanning 2023-06-01 to 2023-08-04.
- **2° operational warning threshold.** Values are **net** inclination — the
  baseline no-load reading is already subtracted.
- Drilling-adjacent nodes (W3, W4, N3) show the largest response;
  tree-shielded nodes (W5, N4) the smallest.

## Files

| File                         | Size  | Contents |
|------------------------------|-------|----------|
| `inclinometer_data.sqlite`   | 345 MB| full 15 s reading database (all nodes) — **[download from the `dataset-v1` release](https://github.com/Burhanuddin98/ESP8266-ThingSpeak/releases/tag/dataset-v1)** (too large for a repo blob) |
| `daily_summary.csv`          | 40 KB | per node/day: rainfall, temp, mean/max/min inclination, %>threshold |
| `sensors_metadata.csv`       | <1 KB | node geometry + drill-coupling table |
| `channel_W3_feed_sample.csv` | 0.9 MB| ThingSpeak-format feed for node W3, first 3 days |
| `validation.png`             | 0.2 MB| daily inclination vs threshold + rainfall (paper Fig 15/16 style) |
| `reconciliation.txt`         | 1 KB  | period means / peaks compared against paper Tables 1–2 |

## Getting the database

The 345 MB SQLite file exceeds GitHub's 100 MB per-file limit for repository
blobs, so it ships as a release asset:

```bash
gh release download dataset-v1 --repo Burhanuddin98/ESP8266-ThingSpeak
```

or download `inclinometer_data.sqlite` directly from the
[`dataset-v1` release](https://github.com/Burhanuddin98/ESP8266-ThingSpeak/releases/tag/dataset-v1).

## Database schema (`inclinometer_data.sqlite`)

```
sensors(sensor_id, name, wall, position_m, distance_to_drill_m,
        drill_coupling, retained_soil_m, tree_shielded, thingspeak_channel)

readings(id, sensor_id, created_at, entry_id,
         field1_roll_deg, field2_pitch_deg,
         field3_accel_roll_deg, field4_temp_c, threshold_exceeded)
        -- index on (sensor_id, created_at)

daily_summary(sensor_id, day, date, rainfall_mm, cum_rainfall_mm, temp_c,
              mean_incl_deg, max_incl_deg, min_incl_deg, pct_over_threshold)
```

## Query examples

```sql
-- All readings for node W3 on the first day
SELECT created_at, field1_roll_deg, field4_temp_c
FROM readings r JOIN sensors s ON s.sensor_id = r.sensor_id
WHERE s.name = 'W3' AND created_at LIKE '2023-06-01%';

-- Fraction of raw samples over the 2 deg threshold, per node
SELECT s.name, ROUND(100.0*AVG(threshold_exceeded),2) AS pct_over
FROM readings r JOIN sensors s ON s.sensor_id = r.sensor_id
GROUP BY r.sensor_id ORDER BY pct_over DESC;

-- Daily mean inclination time series for the western wall (for plotting / ML)
SELECT s.name, d.day, d.mean_incl_deg, d.rainfall_mm
FROM daily_summary d JOIN sensors s ON s.sensor_id = d.sensor_id
WHERE s.wall = 'Western' ORDER BY s.name, d.day;
```

The 15 s per-node series is directly usable as input for the LSTM/GRU
forecasting the paper lists as future work.

## Notes on the aggregates

- Period means are compared against paper Tables 1–2 in `reconciliation.txt`.
  Several nodes reverse sign between the two observation sets (W1: +1.31° →
  −1.31°); the transition is spread over ~3 days rather than stepping in one
  day, so the period means sit within ≈0.01–0.08° of the table values.
- **Daily-mean peaks** track the paper's reported peaks within ~0.1–0.15°.
  **Instantaneous 15 s maxima** sit ~0.2–0.5° higher, since they include
  drilling impact transients that daily-averaged plots smooth away.
- Temperature sits in the paper's 27–29 °C band. The rainfall series in
  `daily_summary.csv` is a Mumbai-monsoon series and does not correspond
  row-for-row to the paper's Table 3 figures.
