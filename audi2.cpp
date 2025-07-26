// Arduino Code - LED Indications for Animal Detection System
#include <ArduinoJson.h>

// Pin Definitions
#define RED_LED_PIN 6      // Red LED - Animal detected
#define GREEN_LED_PIN 7    // Green LED - Safe (no detection)
#define BLUE_LED_PIN 8     // Blue LED - Pump Active

void setup() {
  Serial.begin(9600);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(BLUE_LED_PIN, OUTPUT);

  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(BLUE_LED_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, HIGH); // Default state = safe

  Serial.println("✅ Arduino Ready");
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.length() == 0) return;

    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, input);

    if (error) {
      Serial.println("❌ JSON Parse Failed");
      return;
    }

    String command = doc["command"];
    String animal = doc["animal"];
    float distance = doc["distance"];

    Serial.print("📩 Received: ");
    Serial.print(command);
    Serial.print(" | ");
    Serial.print(animal);
    Serial.print(" | ");
    Serial.print(distance);
    Serial.println("cm");

    // Default: safe mode
    digitalWrite(RED_LED_PIN, LOW);
    digitalWrite(BLUE_LED_PIN, LOW);
    digitalWrite(GREEN_LED_PIN, HIGH);

    if (command == "RED") {
      digitalWrite(GREEN_LED_PIN, LOW);
      digitalWrite(RED_LED_PIN, HIGH); // Animal detected
    }
    else if (command == "GREEN") {
      // No animal: GREEN remains HIGH
    }
    else if (command == "PUMP_ON") {
      digitalWrite(GREEN_LED_PIN, LOW);
      digitalWrite(BLUE_LED_PIN, HIGH); // Pump on
      delay(1000);                      // Blink for 1 second
      digitalWrite(BLUE_LED_PIN, LOW);
    }
  }
}
