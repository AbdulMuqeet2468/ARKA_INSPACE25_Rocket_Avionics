#include <ESP32Servo.h>

Servo myservo;
#define SERVO_PIN 25          // attach servo to GPIO 25 (you can change this)
const int servo_on = 180;     // servo ON position
const int servo_off = 0;      // servo OFF position
int servo_status = 0; 



void setup() {
  myservo.attach(SERVO_PIN);
  myservo.write(servo_off); // start OFF
}

void loop() {
  // nothing here:

}
