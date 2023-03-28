// This file created by ryan039df on Smart Trash Bin repo

// Importing library
#include "DHT.h"
#include <SPI.h>//https://www.arduino.cc/en/reference/SPI
#include <MFRC522.h>//https://github.com/miguelbalboa/rfid
#include <Servo.h>
#include <WiFi.h>
#include <PubSubClient.h>

//____________________________________________________________________________________________________________________________________

// Initiate servo, some variables, and some rfid pins
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
#define SOUND_VELOCITY 0.034
#define SS_PIN 5
#define RST_PIN 2
#define WIFI_SSID "CALVIN-Student"
//#define WIFI_PASSWORD "cit__iee"
#define WIFI_PASSWORD "CITStudentsOnly"
//____________________________________________________________________________________________________________________________________
WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

const char* mqtt_server = "10.252.240.108";
const int ipaddress[4] = {103, 97, 67, 25};
byte nuidPICC[4] = {0, 0, 0, 0};
MFRC522::MIFARE_Key key;
MFRC522 rfid = MFRC522(SS_PIN, RST_PIN);
Servo myservo;

unsigned long previousMillis = 0;   // Stores last time temperature was published
const long interval = 3000;        // Interval at which to publish sensor readings

int pos;
const int capacityCM = 85;
float distanceCm;
int co2 = 0;
//String trash_bin = "Open    ";
//String picking_trash = "False";
int percentage_load;
float temp;
float hum;
int load;
int UScount = 0;
//____________________________________________________________________________________________________________________________________

// Initiate input digital and analog 
uint8_t led_green = 32;
uint8_t led_red1 = 33;
uint8_t led_red2 = 14;
uint8_t Buzzer = 21;
uint8_t a0 = 35; 
uint8_t trigPin = 26;
uint8_t echoPin = 25;
uint8_t servoPin = 27;
uint8_t DHTPin = 4;
DHT dht(DHTPin, DHTTYPE);
//____________________________________________________________________________________________________________________________________

// Structure
struct dht_var {
  float Temperature;
  float Humidity;
};

struct trash {
  String picking_trash;
  String trash_bin;
};

struct trash picking_trash_bin = {"False", "Open    "};
struct dht_var temp_humid = {0.0, 0.0};
//____________________________________________________________________________________________________________________________________

// Functions declaration
struct dht_var dht_check(struct dht_var A);
float US_check(void);
int MQ135_check(void);
struct trash ServoActOnSensor(float distanceCm, float capacityCM, struct trash B);
void reconnect();
void callback(char* topic, byte* message, unsigned int length);
void setup_wifi();
//____________________________________________________________________________________________________________________________________

// Setup
void setup() {
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  
  myservo.attach(servoPin);
  myservo.write(0); //Reset micro servo position  
  
  Serial.begin(115200);
  
  Serial.println(F("Initialize System"));
  SPI.begin();
  dht.begin();        
  rfid.PCD_Init();
  Serial.print(F("Reader :"));
  rfid.PCD_DumpVersionToSerial();

  // Setingg pin Mode
  pinMode(led_green, OUTPUT);
  pinMode(led_red1, OUTPUT);
  pinMode(led_red2, OUTPUT);
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  pinMode(DHTPin, INPUT);
  pinMode(a0, INPUT);   
  pinMode(Buzzer, OUTPUT);
}
//____________________________________________________________________________________________________________________________________

void loop() {
//  Serial.print("Picking Trash: ");
//  Serial.println(picking_trash);
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  temp_humid = dht_check(temp_humid);
  distanceCm = US_check();
  co2 = MQ135_check();
//  Serial.println(co2);
  percentage_load = 0;
  
  picking_trash_bin = ServoActOnSensor(distanceCm, capacityCM, picking_trash_bin);
  if (distanceCm <= capacityCM){
    percentage_load = (capacityCM - distanceCm) * 100 / capacityCM;
  }
  
  Serial.println("-------------------------------");
  Serial.print("The amount of CO2 (in PPM): ");
  Serial.println(co2);
  
  // Prints data from sensor
  Serial.print(" Humidity: ");
  Serial.print(temp_humid.Humidity);
  Serial.print("%");
  Serial.print(", ");
  Serial.print(" Temperature: ");
  Serial.print(temp_humid.Temperature);
  Serial.println(" Celcius");
  Serial.print(", ");
  Serial.println(co2);
//  Serial.print("Servo: ");
//  Serial.println(trash_bin);
  Serial.println("-------------------------------");
  
  unsigned long currentMillis = millis();
  // Every X number of seconds (interval = 10 seconds) 
  // it publishes a new MQTT message
  if (currentMillis - previousMillis >= interval) {
    // Save the last time a new reading was published
    previousMillis = currentMillis;
    // New BME280 sensor readings
    temp = temp_humid.Temperature;
    //temp = 1.8*bme.readTemperature() + 32;
    hum = temp_humid.Humidity;
    load = percentage_load;
    char tempString[8];
    dtostrf(temp, 1, 2, tempString);
    char humString[8];
    dtostrf(hum, 1, 2, humString);
    char loadString[8];
    dtostrf(load, 1, 2, loadString);
    char gasString[8];
    dtostrf(co2, 1, 2, gasString);
    client.publish("esp32/temperature", tempString);
    client.publish("esp32/humidity", humString);
    client.publish("esp32/load", loadString);
    client.publish("esp32/co2", gasString);
  }
  
  picking_trash_bin = readRFID(picking_trash_bin); 
  delay(1000);
}
//____________________________________________________________________________________________________________________________________

// Sensor check functions
struct dht_var dht_check(struct dht_var A){ // DHT Sensor
  A.Temperature = dht.readTemperature();
  A.Humidity = dht.readHumidity();
  return A;
};

float US_check(void){ // Ultra Sonic Sensor
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Reads the echoPin, returns the sound wave travel time in microseconds
  long duration;
  duration = pulseIn(echoPin, HIGH);
  distanceCm = duration * SOUND_VELOCITY/2;
  return distanceCm;
};

int MQ135_check(void){
  int sensorValue = analogRead(a0); // read the input on analog pin 15:
  return sensorValue;
}
//____________________________________________________________________________________________________________________________________

// RFID functions
struct trash readRFID(struct trash B) { /* function readRFID */
 ////Read RFID card
 for (byte i = 0; i < 6; i++) {
   key.keyByte[i] = 0xFF;
 }
 // Look for new 1 cards
 if ( ! rfid.PICC_IsNewCardPresent())
   return B;
 // Verify if the NUID has been readed
 if (  !rfid.PICC_ReadCardSerial())
   return B;
 // Store NUID into nuidPICC array
 for (byte i = 0; i < 4; i++) {
   nuidPICC[i] = rfid.uid.uidByte[i];
 }
// Serial.print(F("RFID In hex: "));
 B = printHex(rfid.uid.uidByte, rfid.uid.size, B);
 // Halt PICC
 rfid.PICC_HaltA();
 // Stop encryption on PCD
 rfid.PCD_StopCrypto1();
 return B;
}

struct trash printHex(byte *buffer, byte bufferSize, struct trash B) {
 String content= "";
 byte letter;
 for (byte i = 0; i < bufferSize; i++) {
//   Serial.print(buffer[i] < 0x10 ? " 0" : " ");
//   Serial.print(buffer[i], HEX);
//   Serial.println("");
   content.concat(String(rfid.uid.uidByte[i] < 0x10 ? " 0" : " "));
   content.concat(String(rfid.uid.uidByte[i], HEX));
   content.toUpperCase();
 }
 if (content.substring(1) == "E2 6D CE 19"){ // Make sure you change this with your own UID number
   if (B.trash_bin == "Closed  " || B.picking_trash == "True"){
     Serial.println("Authorised access");
     ledGreen();
     
     if (B.picking_trash == "False"){
       B.picking_trash = "True";
       B.trash_bin = "Open    ";
       ledRed2_off();
       servoOpen();
     }
     else if (B.picking_trash == "True"){
       B.picking_trash = "False";
     }
   }
 }
 else if (content.substring(1) =="11 A3 E6 1D"){
   Serial.println("Unauthorised access");
   ledRed1();
 }
 return B;
}

void printDec(byte *buffer, byte bufferSize) {
 for (byte i = 0; i < bufferSize; i++) {
   Serial.print(buffer[i] < 0x10 ? " 0" : " ");
   Serial.print(buffer[i], DEC);
 }
}
//____________________________________________________________________________________________________________________________________

// Simple function to move actuators
void servoClose(void){
  for (pos = 0; pos < 100; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
}

void servoOpen(void){
  for (pos = 100; pos > 0; pos -= 1) { // goes from 180x degrees to 0 degrees
   // in steps of 1 degree
   myservo.write(pos);              // tell servo to go to position in variable 'pos'
   delay(15);                       // waits 15ms for the servo to reach the position
  }
}

void buzzing(void){
  digitalWrite (Buzzer, HIGH); //turn buzzer on
  delay(100);
  digitalWrite (Buzzer, LOW);  //turn buzzer off
  delay(100);
  digitalWrite (Buzzer, HIGH); //turn buzzer on
  delay(100);
  digitalWrite (Buzzer, LOW);  //turn buzzer off
  delay(900);
}

void ledRed1(void){
  digitalWrite(led_red1, HIGH);
  delay(1500);        // ...for 2 sec
  digitalWrite(led_red1, LOW);
  delay(500);        // ...for 0.5 sec
}

void ledRed2_on(void){
  digitalWrite(led_red2, HIGH);
}

void ledRed2_off(void){
  digitalWrite(led_red2, LOW);
}

void ledGreen(void){
  digitalWrite(led_green, HIGH);
  delay(1500);        // ...for 1 sec
  digitalWrite(led_green, LOW);
  delay(500);        // ...for 0.5 sec
}
//____________________________________________________________________________________________________________________________________

// Complex function to move servo
struct trash ServoActOnSensor(float distanceCm, float capacityCM, struct trash B){
  int percentage_load;
  int loadHeightCM = distanceCm;
  
  if (B.picking_trash == "False"){
    if (distanceCm < 2000){
      percentage_load = (capacityCM - loadHeightCM) * 100 / capacityCM;
      if (percentage_load < 0){
        percentage_load = 0;
      }
      Serial.print("Load: ");
      Serial.print(percentage_load);
      Serial.println(" %");
      if (B.trash_bin == "Open    "){
        if (percentage_load > 80){
          UScount += 1;
          UScount = UScount % 5;
          if (UScount == 0){
            B.trash_bin = "Closed  ";
            servoClose();
          }
        }
      }
      else if (B.trash_bin == "Closed  "){
          buzzing();
          ledRed2_on();
      }
    }
    else if (distanceCm >= 2000){ // In case there is noise in reading
      String percentage_load = "-";
    }    
  }
  return B;
}
//____________________________________________________________________________________________________________________________________

// MQTT & Wifi Function
void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  
  if (String(topic) == "esp32/servo") {
    Serial.print("Changing output to ");
    if(messageTemp == "true"){
      if(picking_trash_bin.trash_bin == "Open    "){
        Serial.println("on");
        picking_trash_bin.trash_bin = "Closed  ";
        servoClose(); 
      }
    }
    else if(messageTemp == "false"){
      if(picking_trash_bin.trash_bin =="Closed  "){
        Serial.println("off");
        picking_trash_bin.trash_bin = "Open    ";
        ledRed2_off();
        servoOpen(); 
      }
    }
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP8266Client")) {
      Serial.println("connected");
      // Subscribe
      client.subscribe("esp32/servo");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
