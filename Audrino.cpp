#include <ArduinoJson.h>

// Pin definitions
const int redLED = 3;     // Danger (close animal)
const int greenLED = 4;   // Safe
const int blueLED = 5;    // Pump active

String inputString = "";

void setup() {
  Serial.begin(9600);
  pinMode(redLED, OUTPUT);
  pinMode(greenLED, OUTPUT);
  pinMode(blueLED, OUTPUT);

  digitalWrite(redLED, LOW);
  digitalWrite(greenLED, LOW);
  digitalWrite(blueLED, LOW);

  Serial.println("Arduino ready");
}

void loop() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n') {
      handleCommand(inputString);
      inputString = "";
    }
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

  // Turn off all LEDs first
  digitalWrite(redLED, LOW);
  digitalWrite(greenLED, LOW);
  digitalWrite(blueLED, LOW);

  if (command == "RED") {
    digitalWrite(redLED, HIGH);
    Serial.println("🔴 Red LED ON");
  } else if (command == "GREEN") {
    digitalWrite(greenLED, HIGH);
    Serial.println("🟢 Green LED ON");
  } else if (command == "PUMP_ON") {
    digitalWrite(blueLED, HIGH);
    Serial.println("🔵 Pump (Blue LED) ON");
  } else {
    Serial.println("⚠️ Unknown command");
  }
}
