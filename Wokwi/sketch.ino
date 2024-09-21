#include <ESP32Servo.h>
#include<PubSubClient.h>
#include<WiFi.h>

WiFiClient espClient;
PubSubClient mqttClient(espClient);

#define LED 5
#define LDR_LEFT_PIN 32  // Analog pin for left LDR sensor
#define LDR_RIGHT_PIN 33 // Analog pin for right LDR sensor

float controllingFactor = 0.75;    // LDR Characteristics
int thetaOffset = 30;              // Minimum angle 
const int servoPin = 26;                 // Servo Characteristics
float leftIntensity;
char L_LDR [6];
float rightIntensity;
char R_LDR [6];

Servo servo;

void setup() {
  Serial.begin(115200);
  setupWifi();
  setupMqtt();

  pinMode(LED, OUTPUT);
  pinMode(LDR_LEFT_PIN, INPUT);
  pinMode(LDR_RIGHT_PIN, INPUT);

  digitalWrite(LED, LOW);
  servo.attach(servoPin);
}

void loop() {
  if(!mqttClient.connected()){
    connectToBroke();
  }
  mqttClient.loop();
  Serial.println();
  
  int LsensorValue = analogRead(LDR_LEFT_PIN);
  int RsensorValue = analogRead(LDR_RIGHT_PIN);
  leftIntensity = float(LsensorValue - 32) / float(4063 - 32);
  rightIntensity = float(RsensorValue - 32) / float(4063 - 32);
  float maxIntensity = max(leftIntensity, rightIntensity);  
  updateSignal();
  float D;
  if (maxIntensity == leftIntensity) {
    D = 0.5;
    mqttClient.publish("Ctrl",L_LDR);
  } else {
    D = 1.5;
    mqttClient.publish("Ctrl",R_LDR);
  }

  
  int motorAngle = min((int)(thetaOffset * D + (180 - thetaOffset) * maxIntensity * controllingFactor), 180); // Calculate motor angle
  servo.write(motorAngle); // Set servo motor angle

  Serial.println(thetaOffset);
  Serial.println(controllingFactor);
  
  char buffer[7]; // Buffer to hold the converted value
  dtostrf(D, 4, 2, buffer); // Convert float to string with 5 digits and 2 decimal places
  
  mqttClient.publish("L_LDR",L_LDR);     //sending value 
  mqttClient.publish("R_LDR",R_LDR);    //sending value  
  mqttClient.publish("Message",buffer); 
  delay(1000); // Delay for stability
}

void setupWifi(){
  WiFi.begin("Wokwi-GUEST","");
  while(WiFi.status()!=WL_CONNECTED){
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("Ip address: ");
  Serial.println(WiFi.localIP()); 
}

void setupMqtt(){
  mqttClient.setServer("test.mosquitto.org",1883);
  mqttClient.setCallback(receiveCallback);
}

void connectToBroke(){
  while(!mqttClient.connected()){
    Serial.print("Attempting MQTT connection...");
    if(mqttClient.connect("ESP32-12345123451234")){
      Serial.println("connected");
      //mqttClient.subscribe("Medibox");
      mqttClient.subscribe("AngleChanger");
      mqttClient.subscribe("GAMMA_Changer"); 
    }
    else{
      Serial.print("failed");
      Serial.print(mqttClient.state());
      delay(3000);
    }
  }
}
void updateSignal(){
  String(leftIntensity,2).toCharArray(L_LDR,6);    //convert the data to string 
  String(rightIntensity,2).toCharArray(R_LDR,6);  
}
void receiveCallback(char* topic,byte* payload, unsigned int length){ 
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("]");
  Serial.println();

  char payloadCharAr[length + 1]; // Add 1 for null terminator
  for(int i = 0; i < length; i++) {
    payloadCharAr[i] = (char)payload[i];
    }
  payloadCharAr[length] = '\0'; // Null-terminate the string

  Serial.println(payloadCharAr); // Print the received payload

  float payloadValue = atof(payloadCharAr); // Convert the string to an float

  if(strcmp(topic, "AngleChanger") == 0){
      Serial.print("Angle is ");
      Serial.println(payloadValue);
      thetaOffset = payloadValue;
  }
    else{
      Serial.print("CtrlF is ");
      Serial.println(payloadValue);
      controllingFactor = payloadValue;
      Serial.println(controllingFactor);
    }  
}