#include <Servo.h>

Servo servo1;
Servo servo2;

#define SERVO1_PIN 9
#define SERVO2_PIN 10
#define LED_PIN 7

void setup() {
  Serial.begin(9600); // UART from Arduino Uno #1

  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);

  pinMode(LED_PIN, OUTPUT);

  // Set initial positions
  servo1.write(0);
  servo2.write(0);
  digitalWrite(LED_PIN, LOW); // LED OFF
}

void loop() {
  // Check for UART command
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');

    if (command == "OPEN_SLIDER") {
      openSlider();
    }
  }
}

void openSlider() {
  digitalWrite(LED_PIN, HIGH); // Turn on LED while operating
  servo1.write(90);
  servo2.write(90);
  delay(3000);       // Wait to simulate fill time
  servo1.write(0);
  servo2.write(0);
  delay(500);
  digitalWrite(LED_PIN, LOW); // Turn off LED after motion
}
