int pirPin = 2 ;  
int val = 0;
int buzzerPin = 3;
int pirState = LOW;

bool monitoring = false;

void setup() {
  Serial.begin(9600);
  pinMode(pirPin, INPUT);
  pinMode(buzzerPin, OUTPUT);
}

void loop() {

  if (Serial.available()) {
    int status = Serial.read();

    if (status == 1) {
      monitoring = true;
    }
    else if (status == 0) {
      monitoring = false;
    }
  }

  if (monitoring) {
    val = digitalRead(pirPin);
    if (val == HIGH) {
      digitalWrite(buzzerPin, HIGH);
      Serial.println("1");
      delay(50);
    } 
    else {
        digitalWrite(buzzerPin, LOW);
    }
    delay(200);
  }

}
