
#include<Wire.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h> 
#include <ESP8266WebServer.h>
/* Set these to your desired credentials. */
const char *ssid = "Me Me Big Boy";  //ENTER YOUR WIFI SETTINGS <<<<<<<<<
const char *password = "interesting";

//Web address to read from
const char *host = "api.thingspeak.com";
String apiKey = "BJDPQCSWMKKK2EJM";  //ENTER YOUR API KEY <<<<<<<<<<<
const int MPU_addr=0x68;
int16_t axis_X,axis_Y,axis_Z;
int minVal=265;
int maxVal=402;
double x;
double y;
double z;
const uint8_t scl = D1;
const uint8_t sda = D2;
void setup(){
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(9600);
  delay(1000);
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);        //This line hides the viewing of ESP as wifi hotspot
  //WiFi.mode(WIFI_AP_STA);   //Both hotspot and client are enabled
  //WiFi.mode(WIFI_AP);       //Only Access point
  
  WiFi.begin(ssid, password);     //Connect to your WiFi router
  Serial.println("");

  Serial.print("Connecting");
  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");}
    //If connection successful show IP address in serial monitor
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());  //IP address assigned to your ESP

  Wire.begin(sda, scl);
}

void loop(){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);
  axis_X=Wire.read()<<8|Wire.read();
  axis_Y=Wire.read()<<8|Wire.read();
  axis_Z=Wire.read()<<8|Wire.read();
    int xAng = map(axis_X,minVal,maxVal,-90,90);
    int yAng = map(axis_Y,minVal,maxVal,-90,90);
    int zAng = map(axis_Z,minVal,maxVal,-90,90);
       x= RAD_TO_DEG * (atan2(-yAng, -zAng)+PI);
       y= RAD_TO_DEG * (atan2(-xAng, -zAng)+PI);
       z= RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);
     Serial.print("Angle of inclination in X axis = ");
     Serial.print(x);
     Serial.println((char)176);
   /*  Serial.print("Angle of inclination in Y axis= ");
     Serial.print(y);
     Serial.println((char)176);
     Serial.print("Angle of inclination in Z axis= ");
     Serial.print(z);
     Serial.println((char)176);*/
     delay(1000);

       WiFiClient client;          
  const int httpPort = 80; //Port 80 is commonly used for www
 //---------------------------------------------------------------------
 //Connect to host, host(web site) is define at top 
 if(!client.connect(host, httpPort)){
   Serial.println("Connection Failed");
   delay(300);
   return; //Keep retrying until we get connected
 }
  //Make GET request as pet HTTP GET Protocol format
  String ADCData;
  //Read Analog value of LDR
  
  String y_axis1 = String(x);
 
  String Link="GET /update?api_key="+apiKey+"&field1="+y_axis1;  //Requeste webpage  
  Link = Link + " HTTP/1.1\r\n" + "Host: " + host + "\r\n" + "Connection: close\r\n\r\n";                
  client.print(Link);
  delay(100);
  
//---------------------------------------------------------------------
 //Wait for server to respond with timeout of 5 Seconds
 int timeout=0;
 while((!client.available()) && (timeout < 1000))     //Wait 5 seconds for data
 {
   delay(10);  //Use this with time out
   timeout++;
 }

//---------------------------------------------------------------------
 //If data is available before time out read it.
 if(timeout < 500)
 {
     while(client.available()){
        Serial.println(client.readString()); //Response from ThingSpeak       
     }
 }
 else
 {
     Serial.println("Request timeout..");
 }

 delay(5000);  //Read Web Page every 5 seconds
}

void I2C_Write(uint8_t deviceAddress, uint8_t regAddress, uint8_t data){
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.write(data);
  Wire.endTransmission();
}
