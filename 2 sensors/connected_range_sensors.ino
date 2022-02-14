/*
This program takes 2 inputs from range sensors, and makes 2 led blink dependend on the reading values.
It is structured the following way:
 1. get a reading converted to cm
 2. use that reading to determine a how fast the LED should blink, ie. get a blink interval
 3. make the led's blink with their given interval

Each part is implemented with a function - description above each function. 

Date: 14/02 - 2022
By: Niels Viggo Stark Madsen, Phat Pham and Buster Salomon Rasmussen
*/


// ------ CONSTANTS
// Led pins
int ledPin = 17;
int ledPin2 = 30;

// Input pins
int inputPin = A0;
int inputPin2 = A1;

// Coeficcient for calculating from digital voltage range to CM
float coe = 1.344;

// ------ TIME
unsigned long currentMillis; // updated every loop cycle
unsigned long prevMillis1 = 0; // prevMillis for ledPin
unsigned long prevMillis2 = 0; // prevMillis for ledPin2

// ----- LED STATES
int ledState1 = LOW;
int ledState2 = LOW;
int* led1 = &ledState1; // Pointers are used to change the ledStates from the loop
int* led2 = &ledState2;

// ----- Functions
/*
getCM
Pre: Takes an analog input pin as input
Post: Returns CM corresponding to input signal
*/
float getCM(int input) {
  int val = analogRead(input);
  float cm = val * coe;
  return cm;
}

/*
getInterval
Pre: Takes a float that represents the range as input
Post: Returns an integer representing the interval an LED should blink with 
*/
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

/*
blinkWithMillis
Pre: Takes 4 arguments
 1) interval: integer - the interval the LED should blink with 
 2) output: integer - the pin the arduino should write to (send high signal)
 3) prevMillis: unsigned long - number in mill


*/
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


// ----- SETUP
void setup() {
  pinMode(ledPin, OUTPUT);
  pinMode(ledPin2, OUTPUT);

  Serial.begin(9600);

}

// ----- LOOP
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
