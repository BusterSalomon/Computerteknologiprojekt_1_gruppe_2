
// ------ CONSTANTS
int ledPin = 17;
int inputPin = A0;
float coe = 1.344;

// time
unsigned long currentMillis;
unsigned long prevMillis = 0;

// ledState
int ledState = LOW;

// Get CM
float getCM () {
  int val = analogRead (inputPin);
  float cm = val*coe;
  return cm;
}

// Get interval
int getInterval (float cm) {
  int interval;
  if (cm > 60) {
    interval = 1000;
  }
  if (cm < 60 && cm > 45) {
    interval = 700;
  }
  if (cm < 45 && cm > 30) {
    interval = 400;
  }
  if (cm < 30 && cm > 20) {
    interval = 100;
  }
  if (cm < 20) {
    interval = 0;
  }
  return interval;
}

// Blink
void blinkWithMillis (int interval) {
  // If interval == 0, led should always be on
  if (!ledstate && interval == 0) {
    currentMillis = millis();
    // Blink if blink interval is reached
    if (currentMillis - prevMillis >= interval) {
      ledState = !ledState;
      digitalWrite (ledPin, ledState);
      prevMillis = currentMillis;  
    }  
  }
}


// Setup
void setup() {
  pinMode(ledPin, OUTPUT);
  Serial.begin(9600);
  
}

// Loop
void loop() {

  float cm = getCM ();
  int interval = getInterval (cm);
  blinkWithMillis (interval);

  
}
