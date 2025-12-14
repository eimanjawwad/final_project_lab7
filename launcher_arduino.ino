const uint8_t inputPin  = 9;  
const uint8_t servoPin  = A1;  
const uint8_t outPin1   = 3;    
const uint8_t outPin2   = 12;  

void setup() {
  pinMode(inputPin, INPUT);
  pinMode(servoPin, OUTPUT);
  pinMode(outPin1, OUTPUT);
  pinMode(outPin2, OUTPUT);

  digitalWrite(servoPin, LOW);
  digitalWrite(outPin1, LOW);
  digitalWrite(outPin2, LOW);
}

void loop() {
  if (digitalRead(inputPin) == HIGH) {

    //turn on MOTORS ONLY at start
    digitalWrite(outPin1, HIGH);
    digitalWrite(outPin2, HIGH);

    unsigned long startTime = millis();

    while (millis() - startTime < 3000) {
      // 3sec motors only
    }

    while (millis() - startTime < 8000) {
      //then 1ms/20ms pulse (servo on CCW) + motors for 5 secs
      digitalWrite(servoPin, HIGH);
      delayMicroseconds(1000);

      digitalWrite(servoPin, LOW);
      delayMicroseconds(19000);
    }

    digitalWrite(servoPin, LOW);
    digitalWrite(outPin1, LOW);
    digitalWrite(outPin2, LOW);
  }
}