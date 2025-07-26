#include <ArduinoJson.h>

// Pin definitions
const int redLED = 3;     // Danger (close animal)
const int greenLED = 4;   // Safe
const int blueLED = 5;    // Pump status indicator
const int pumpRelay = 6;  // NEW: Relay control for water pump

String inputString = "";
unsigned long pumpStartTime = 0;
const unsigned long pumpDuration = 10000; // Run pump for 10 seconds
bool pumpRunning = false;

void setup() {
  Serial.begin(9600);
  pinMode(redLED, OUTPUT);
  pinMode(greenLED, OUTPUT);
  pinMode(blueLED, OUTPUT);
  pinMode(pumpRelay, OUTPUT);  // NEW: Pump relay pin
  
  digitalWrite(redLED, LOW);
  digitalWrite(greenLED, LOW);
  digitalWrite(blueLED, LOW);
  digitalWrite(pumpRelay, LOW);  // NEW: Ensure pump is OFF initially
  
  Serial.println("Arduino ready with pump control");
}

void loop() {
  // Handle serial communication
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n') {
      handleCommand(inputString);
      inputString = "";
    }
  }
  
  // Handle pump auto-shutoff
  if (pumpRunning && (millis() - pumpStartTime >= pumpDuration)) {
    stopPump();
  }
}

void handleCommand(String jsonData) {
  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc, jsonData);
  
  if (error) {
    Serial.println("❌ JSON parse error");
    return;
  }
  
  String command = doc["command"];
  
  // Turn off all LEDs first (but keep pump state)
  digitalWrite(redLED, LOW);
  digitalWrite(greenLED, LOW);
  if (!pumpRunning) {
    digitalWrite(blueLED, LOW);
  }
  
  if (command == "RED") {
    digitalWrite(redLED, HIGH);
    Serial.println("🔴 Red LED ON - Animal Detected");
  } 
  else if (command == "GREEN") {
    digitalWrite(greenLED, HIGH);
    Serial.println("🟢 Green LED ON - Area Safe");
  } 
  else if (command == "PUMP_ON") {
    startPump();
  } 
  else {
    Serial.println("⚠️ Unknown command");
  }
}

void startPump() {
  if (!pumpRunning) {
    digitalWrite(pumpRelay, HIGH);  // Turn ON relay (pump starts)
    digitalWrite(blueLED, HIGH);    // Blue LED indicates pump is running
    pumpRunning = true;
    pumpStartTime = millis();
    Serial.println("💧 Water Pump STARTED");
  } else {
    Serial.println("💧 Pump already running");
  }
}

void stopPump() {
  digitalWrite(pumpRelay, LOW);   // Turn OFF relay (pump stops)
  digitalWrite(blueLED, LOW);     // Turn off blue LED
  pumpRunning = false;
  Serial.println("💧 Water Pump STOPPED");
}