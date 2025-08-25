#include <Servo.h>

const int servo_pin = 2;

Servo servo;

void setup() {
  Serial.begin(9600);
}
void loop() {

  if (Serial.available()) {
    int status = Serial.read();
      
    if (status == 1) {
      servo.attach(servo_pin);
      servo.write(180);
      delay(50);
    }
    else if (status == 0) {
      servo.attach(servo_pin);
      servo.write(0);
      delay(50);
    }
    delay(200);
    servo.detach();
  }
  
}