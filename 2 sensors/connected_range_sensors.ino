/*




*/


// ------ CONSTANTS
int ledPin = 17;
int ledPin2 = 30;
int inputPin = A0;
int inputPin2 = A1;
float coe = 1.344;

// time
unsigned long currentMillis;
unsigned long prevMillis1 = 0;
unsigned long prevMillis2 = 0;

// ledState
int ledState1 = LOW;
int ledState2 = LOW;
int* led1 = &ledState1;
int* led2 = &ledState2;


// Get CM
float getCM(int input) {
  int val = analogRead(input);
  float cm = val * coe;
  return cm;
}

// Get interval
int getInterval(float cm) {
  int interval;
  if (cm > 60) {
    interval = 500;
  }
  if (cm <= 60 && cm > 45) {
    interval = 350;
  }
  if (cm <= 45 && cm > 30) {
    interval = 200;
  }
  if (cm <= 30 && cm > 20) {
    interval = 50;
  }
  if (cm <= 20) {
    interval = 0;
  }
  return interval;
}

// Blink
unsigned long blinkWithMillis(int interval, int output, unsigned long prevMillis, int *ledState) {

  // Get current millis
  currentMillis = millis();

  // Blink if blink interval is reached
  if (currentMillis - prevMillis >= interval) {
    *ledState = !(*ledState);
    digitalWrite(output, *ledState);
    return currentMillis;
  }
  else return prevMillis;

}


// Setup
void setup() {
  pinMode(ledPin, OUTPUT);
  pinMode(ledPin2, OUTPUT);

  Serial.begin(9600);

}

// Loop
void loop() {

  // Left
  float cm = getCM(inputPin);
  Serial.println(cm);
  int interval1 = getInterval(cm);
  prevMillis1 = blinkWithMillis(interval1, ledPin, prevMillis1, led1);

  // Right
  float cm2 = getCM(inputPin2);
  int interval2 = getInterval(cm2);
  prevMillis2 = blinkWithMillis(interval2, ledPin2, prevMillis2, led2);

}
