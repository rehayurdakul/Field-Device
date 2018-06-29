//Purpose
//  Voltarent Field Device

//Hardware - Prepared By Reha Yurdakul(rehay@ke.ibm.com)/Peter Mbiria 
//  MFNode MCU
//  SIM900R GSM Module
//  GTOP GPS Module
//  DHT22

//Author
//  Kelvin Mwega (kelvinm@ke.ibm.com)

//Pin Assignment
//  D6: GSM-IGN
//  D0: GSM TX
//  D1: GSM RX


#include <mqtt.h>
#include <SoftwareSerial.h>
#include <ArduinoJson.h>
#include <TinyGPS.h>
#include <NewPing.h>
#include <avr/wdt.h>
#include <SPI.h>
#include <Wire.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"
#include "DHT.h"

#define DHTPIN 5 
//#define DHTTYPE DHT11   // DHT 11
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321

#define LED13     13 
#define GSM_IGN   6  
#define BUTTON_Blue 11
#define BUTTON_Black 10

DHT dht(DHTPIN, DHTTYPE);

//char server[] = "159.8.169.212";
//char clientId[34] = "d:14br69:Field:865905021075654";
//char topic[26] = "iot-2/evt/status/fmt/format_string";
//char token[13] = "ibmvehicleobdd";

char server[] = "198.100.30.116";
char clientId[20] = "";
char topic[15] = "VNT/fd/Data";

String imei = "";

int defaultLightOn = 500;
int BlackBtnState = 0;
int BlueBtnState = 0;
int flag=0;

String quality;
long anVolt, cm;
int criticalBattLevel = 3600;
int lowBattLevel = 3650;
long failures = 0;
String dateTime;

// 0X3C+SA0 - 0x3C or 0x3D
#define I2C_ADDRESS 0x3C

SSD1306AsciiWire display;

TinyGPS gps;
SoftwareSerial ss(8,0);

void setup() {
  Serial.begin(9600);
  ss.begin(9600);
  char clientid[45];
//  wdt_disable();
//  wdt_enable(WDTO_8S);
//  wdt_reset();
//  pinMode(AUX_POWER_PIN, OUTPUT);
//  auxPower(OFF);
  delay(5000);
  Wire.begin();
  display.begin(&Adafruit128x64, I2C_ADDRESS);
  display.set400kHz();
  display.setFont(font5x7);

  initHWPins();
  displayData(" ", " ", " ", "Starting..", "", "", "");
  gsm_startup();
  gsm_is_ready_MODULE(); displayData("GSM Ready!", " ", "Connecting..", "", " ", " ", " ");
  gsm_is_ready_TCP();    displayData("GSM Ready!", "", "TCP/IP ready..", "", "", " ", " ");
//  auxPower(ON);
//  DHT11.attach(5);
//  int chk = DHT11.read();
  String BatV =  "Battery     : " + String(gsmGetBattery());
  String Sig =   "GSM Signal  : " + String(gsmGetSignalQuality());
  String Temp =  "Temp        : " + String(dht.readTemperature(), DEC);
  String Hum =   "Humidity    : " + String(dht.readHumidity(), DEC);
//  auxPower(OFF);

   //clientId = "d:14br69:LevelMeter:863591029486077"
//  String clientidstr = "d:14br69:LevelMeter:" + gsmGetIMEI();
//  clientidstr.toCharArray(clientid, 45);
//  strcpy(clientId, clientid);

  delay(1000);
  
  imei = gsmGetIMEI();
  imei.toCharArray(clientId, 20);
  delay(500);

  displayData("Connected", "**Device Status**", BatV, Sig, Temp, Hum, "Sending HELLO!");
  sendHelloMsg();
  displayData("Device", "", "Ready", "", "", " ", " ");

  dht.begin();
}

void loop(){
  
  bool newdata = false;
  unsigned long start = millis();
  String gspData;

  ss.listen();
  while (millis() - start < 10000)
  {
    if (feedgps())
      newdata = true;
  }
  //Serial.println(newdata);
  //if (newdata)
  locationData(gps);

  String BatV =  "Battery     : " + String(gsmGetBattery());
  String Sig =   "GSM Signal  : " + String(gsmGetSignalQuality());
  String Temp =  "Temp        : " + String(dht.readTemperature(), DEC);
  String Hum =   "Humidity    : " + String(dht.readHumidity(), DEC);

  displayData(BatV, Sig, Temp, Hum, "---------", "~~~~~", "Sending Level");
  
  delay(100);
  
}

void displayData(String a, String b, String c, String d, String e, String f, String g) {
  wdt_reset();
  display.clear();
  //  display.set2X();
  display.println("Voltarent Eng");
  display.println(a);
  display.println(b);
  display.println(c);
  display.println(d);
  display.println(e);
  display.println(f);
  display.println(g);
}

static bool feedgps()
{
  while (ss.available())
  {
    if (gps.encode(ss.read()))
      return true;
  }
  return false;
}

void sendHelloMsg()
{
  char aBuff[18], dataToSend[70], fBuff[9], imeiBuf[20], qBuff[5], bBuff[5];
  strcpy(dataToSend, "0,");
  gsmGetIMEI().toCharArray(imeiBuf, 20);
  strcat(dataToSend, imeiBuf);
  strcat(dataToSend, ",");

  quality = gsmGetSignalQuality();
  quality.toCharArray(qBuff, 5);
  strcat(dataToSend, qBuff);
  strcat(dataToSend, ",");

  itoa(gsmGetBattery(), bBuff, 10);
  strcat(dataToSend, bBuff);
  strcat(dataToSend, ",");

  ltoa(failures, fBuff, 10);
  strcat(dataToSend, fBuff);
  
  sendMQTTMessage(clientId, server, topic, dataToSend);
}

void blinkLED(int ledID, int repeat, int wait) {
  if (repeat == 999) { 
    digitalWrite(ledID, HIGH); 
    return; 
  }
  if (repeat == 0) { 
    digitalWrite(ledID, LOW); 
    return; 
  }
  for (int i = 0; i < repeat; i++) { 
    digitalWrite(ledID, HIGH); 
    delay(wait);
    digitalWrite(ledID, LOW); 
    delay(wait);
  }
  delay(1000);
}

void initHWPins() {
  pinMode(GSM_IGN, OUTPUT);
  digitalWrite(GSM_IGN,HIGH);    
}  

// ############## GPS ###############################################################################

void locationData(TinyGPS &gps){
  wdt_reset();
  char locData[6], fBuff[9], qBuff[5], bBuff[5], imeiBuf[20];
  char strToSend[70];

  unsigned long age, date, tim, chars = 0;
  unsigned short sentences = 0, failed = 0;
  
  String locString = "";
  float flat, flon;
  unsigned long fix_age;

  gps.f_get_position(&flat, &flon, &fix_age);
  float fkmph = gps.f_speed_kmph();
  float fc = gps.f_course();
  float falt = gps.f_altitude();

  locString.concat("00,");

  if (fix_age == TinyGPS::GPS_INVALID_AGE){ //If no GPS satelite lock send **invalid**
    
    locString.concat("0");
    locString.concat(",");
    locString.concat("-1.259335");
    locString.concat(",");
    locString.concat("36.785563");
    locString.concat(",");
    locString.concat("1600");
    locString.concat(",");
    locString.concat("0");
    locString.concat(",");

    }
  else if (fix_age > 5000){ //If GPS lost satelite lock send **Stale Data**

    locString.concat("0");
    locString.concat(",");
    locString.concat("-1.259335");
    locString.concat(",");
    locString.concat("36.785563");
    locString.concat(",");
    locString.concat("1600");
    locString.concat(",");
    locString.concat("0");
    locString.concat(",");
   
  } else { //if Valid GPS data send Lat, Long and Alt

    locString.concat(String(fkmph));
    locString.concat(",");
    locString.concat(String(flat, 6));
    locString.concat(",");
    locString.concat(String(flon, 6));
    locString.concat(",");
    locString.concat(String(falt));
    locString.concat(",");
    locString.concat(String(fc));
    locString.concat(",");
    
//    dtostrf(fkmph, 5, 2, locData);
//    strcat(strToSend, locData); 
//    strcat(strToSend, ",");
//    dtostrf(fc, 5, 2, locData);
//    strcat(strToSend, locData); 
//    strcat(strToSend, ",");
//    dtostrf(date, 5, 0, locData);
//    strcat(strToSend, locData); 
//    strcat(strToSend, ",");
//    dtostrf(tim, 5, 2, locData);
//    strcat(strToSend, locData); 
//    strcat(strToSend, ",");
    gps.stats(&chars, &sentences, &failed);
    print_int(chars, 0xFFFFFFFF, 6);
    print_int(sentences, 0xFFFFFFFF, 10);
    print_int(failed, 0xFFFFFFFF, 9);    
    } 

  locString.concat(String(gsmGetBattery()));
  locString.concat(",");
  locString.concat(String(gsmGetSignalQuality()));
  locString.concat(",");
  locString.concat(String(dht.readTemperature()));
  locString.concat(",");
  locString.concat(String(dht.readHumidity()));
  locString.concat(",");

  Serial.println(locString);
  
  locString.toCharArray(strToSend, 120);
  locString = "";
  sendMQTTMessage(clientId, server, topic, strToSend);
  
  }

static void collectSensorData(){
  
}

//Function to print GPS Integer values
static void print_int(unsigned long val, unsigned long invalid, int len)
{
  char sz[32];
  if (val == invalid)
    strcpy(sz, "*******");
  else
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  //Serial.print(sz);
  //Serial.println();
  feedgps();
}

// ########################################################################################################################


void sendMQTTMessage(char* clientId, char* brokerUrl,  char* topic, char* message){  

  byte    mqttMessage[127];
  int     mqttMessageLength = 0;
  
  blinkLED(13,999,defaultLightOn);
//  mqttMessageLength=mqtt_connect_message(mqttMessage, clientId, "use-token-auth", "fieldevice");   //prepare MQTTConnect Message String & calculate length
  mqttMessageLength=mqtt_connect_message(mqttMessage, clientId, "", "");
  gsm_send_tcp_MQTT_byte(brokerUrl,mqttMessage,mqttMessageLength);                                         // Send MQTT formatted connect string to IoT Server
  // Publish to MQTT Server
  mqtt_publish_message(mqttMessage, topic, message);                                                       // prepare MQTTPublish Message Message String
  mqttMessageLength = 4 + strlen(topic) + strlen(message);                                                 // calculate the message length
  gsm_send_tcp_MQTT_byte(brokerUrl,mqttMessage, mqttMessageLength);                                        // Send MQTT formatted Message string to IoT Server
  // disconnect from MQTT Server
  mqttMessageLength = mqtt_disconnect_message(mqttMessage);                                                 // prepare MQTT disconnect Message Message String and length
  gsm_send_tcp_MQTT_byte(brokerUrl,mqttMessage, mqttMessageLength);  
  blinkLED(13,0,100);
}


///    *************************************************************************
///    *************************************************************************
///    ****             GSM/TCP FUNCTIONS   GSM/TCP FUNCTIONS               ****
///    ****             GSM/TCP FUNCTIONS   GSM/TCP FUNCTIONS               ****
///    ****             GSM/TCP FUNCTIONS   GSM/TCP FUNCTIONS               ****
///    *************************************************************************
///    *************************************************************************

boolean gsm_send_tcp_MQTT_byte( char* mqttServer, byte* message, int lenMessage) {

x:
  wdt_reset();
  if (gsm_is_ready_TCPSERVER(mqttServer, "1883") == false) goto x;
  gsm_send("AT+CIPSEND");
  delay(300);
  if ( gsm_response_check(">") == false) return false;
  for (int j = 0; j < lenMessage; j++)  Serial.write(message[j]);
  Serial.write(byte(26));
  delay(2000);
  gsm_response_check("SEND OK");     //dont use response.. because Data Transmit Check is counts!
  gsm_send("AT+CIPACK");
  delay(500);

  if ( gsm_response_check("Data Transmit Check") == false) {
    goto x;
  }
  return true;
}


boolean gsm_send_tcp( char* message) {
x:
  wdt_reset();
  if (gsm_is_ready_TCPSERVER("198.100.31.2", "19940") == false) goto x;       //VPS
  gsm_send("AT+CIPSEND");
  delay(300);
  if ( gsm_response_check(">") == false) return false;
  gsm_send(message);
  Serial.write(byte(26));
  delay(2000);
  gsm_response_check("SEND OK");     //dont use response.. because Data Transmit Check is counts!
  gsm_send("AT+CIPACK");
  delay(500);
  if ( gsm_response_check("Data Transmit Check") == false) {
    goto x;
  }
  return true;
}


boolean gsm_is_ready_TCPSERVER(char* server_ip, char* server_port) {
  int cnt = 0;
  char at_command[50];
  strcpy(at_command, "AT+CIPSTART=\"TCP\",\"" );
  strcat(at_command, server_ip);
  strcat(at_command, "\",\"");
  strcat(at_command, server_port);
  strcat(at_command, "\"");
x:
  wdt_reset();
  gsm_send("AT+CIPSTATUS");
  delay(300);
  if ( gsm_response_check("STATE: CONNECT OK") == true) return true;
  if (cnt > 7) {
    cnt = 0;
    gsm_startup();
    delay(500);
    gsm_startup();
    gsm_is_ready_MODULE();
  }
  cnt++;
  gsm_is_ready_TCP();
  gsm_send(at_command);
  delay(300);
  gsm_response_check("OK");    // not important... does not helps !!
  goto x;
}

boolean gsm_is_ready_TCP() {
x:
  wdt_reset();
  digitalWrite(LED13, LOW);
  gsm_send("AT+CIPSTATUS");
  delay(300);
  if ( gsm_response_check("STATE: IP STATUS") == true) {
    digitalWrite(LED13, HIGH);
    return true;
  }
  gsm_is_ready_GPRS();
  gsm_send("AT+CIPSHUT");
  delay(300);
  if ( gsm_response_check("SHUT OK") == false ) goto x;
  gsm_send("AT+CSTT=\"internet\"");
  delay(1000);
  if ( gsm_response_check("OK") == false ) goto x;
  gsm_send("AT+CIICR");
  delay(1000);
  if ( gsm_response_check("OK") == false ) goto x;
  gsm_send("AT+CIFSR");
  delay(300);
  if ( gsm_response_check(".") == false ) goto x;
  goto x;
}

boolean gsm_is_ready_GPRS() {
  int reps = 0;
x:
  wdt_reset();
  gsm_send("AT+CGATT=1");
  delay(300);
  gsm_response_check("OK");
  gsm_send("AT+CGATT?");
  delay(300);
  if ( gsm_response_check("+CGATT: 1") == true) {
    return true;
  }
  if (reps == 7)
  {
    gsm_is_ready_NETWORK();
    reps = 0;
    goto x;
  }
  reps++;
  goto x;
}


boolean gsm_is_ready_NETWORK() {
  int noOfTry = 0;
x:
  wdt_reset();
  noOfTry++;
  gsm_send("AT+CREG?");
  delay(1000);
  if ( gsm_response_check("+CREG: 0,1") == true) return true;
  if (noOfTry == 3) {
    noOfTry = 0;
    gsm_is_ready_MODULE();
  }
  goto x;
}

boolean gsm_is_ready_MODULE() {
  int noOfTry = 0;
  Serial.begin(19200);
x:
  wdt_reset();
  noOfTry++;
  gsm_send("AT");
  delay(500);
  if ( gsm_response_check("OK") == true) return true;
  if (noOfTry == 3) {
    noOfTry = 0;
    failures++;
    gsm_startup();
  }

  goto x;
}


boolean gsm_response_check(String expected_response) {
  wdt_reset();
  String  gprs_resp_str;

  Serial.flush();
  gprs_resp_str = gsm_read();

  if ( expected_response == "Data Transmit Check") {     //Speciall check the response !!
    int indexOfComma;
    char checkChar = '0';
    indexOfComma = gprs_resp_str.lastIndexOf(',');   // , sign before number char left for Tx.. Should be 0 !
    if ( gprs_resp_str.charAt(indexOfComma + 1) == checkChar) {
      return true;
    }
    else return false;  //still there are chars to send... NOT POSSIBLE ! reTry to SEND !
    return true;
  }

  if ( expected_response == "GET_DATE_TIME") {     //not response check.. just GET DATE/TIME
    int indexOfComma;
    char checkChar = '0';
    indexOfComma = gprs_resp_str.lastIndexOf(',');   // , sign before number char left for Tx.. Should be 0 !
    if ( gprs_resp_str.charAt(indexOfComma + 1) == checkChar) {
      return true;
    }
    else return false;  //still there are chars to send... NOT POSSIBLE ! reTry to SEND !
    return true;
  }
  if (gprs_resp_str.indexOf(expected_response) > -1) {
    return true;
  }
  else {
    return false;
  }
  return false;   //should never come here !
}

String gsm_read() {
  wdt_reset();
  String gsmString;
  while (Serial.available()) gsmString = gsmString + (char)Serial.read();
  return gsmString;
}


void gsm_send(char* command) {
  wdt_reset();
  Serial.println(command);
  delay(2500);
}

void gsm_startup() {
  wdt_reset();

  gsm_toggle();

}


void gsm_toggle() {
  wdt_reset();
  digitalWrite(GSM_IGN, HIGH);
  delay(1000);
  digitalWrite(GSM_IGN, LOW);
  delay(1500);
  digitalWrite(GSM_IGN, HIGH);
  delay(5000);
}

void gsm_shutdown() {
  wdt_reset();
  Serial.begin(19200);
  Serial.flush();
  Serial.println("AT+CPOWD=0");
  delay(2000);
  Serial.flush();
  Serial.println("AT");
  delay(500);
  Serial.end();
}

String  gsmGetIMEI() {
  wdt_reset();
  int tmp;
  String IMEI;

  gsm_send("\nAT+CGSN");  //ask  +CCGSN: "8679123519452673"
  delay(2000);
  IMEI = gsm_read();
  return IMEI.substring(tmp + 12, tmp + 12 + 15);
}


int gsmGetBattery() {
  wdt_reset();
  String voltageBattery;
  int tmp;
  gsm_send("\nAT+CBC");    // +CBC: 0,60,3790
  delay(1000);
  voltageBattery = gsm_read();
  tmp = voltageBattery.lastIndexOf(",");
  return voltageBattery.substring(tmp + 1, tmp + 5).toInt();
}


String  gsmGetSignalQuality() {
  wdt_reset();
  String signalQuality;
  int tmp, len;
  gsm_send("\nAT+CSQ");    // +CSQ: 18,0
  delay(300);
  signalQuality = gsm_read();
  tmp = signalQuality.indexOf("+CSQ:");
  len = signalQuality.length();
  return signalQuality.substring(tmp + 6, tmp + 8);
}

