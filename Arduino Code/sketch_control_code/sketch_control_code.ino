#include "Adafruit_VEML7700.h"

Adafruit_VEML7700 veml = Adafruit_VEML7700();

#define OUT_PIN 3// D3 square wave output
#define POT_PIN A0// PIN_PB4
#define LUM_PIN A0// PIN_PB2
#define MODE_PIN A0 //A3 //operation mode
#define LAMP_PIN A0//A3
# define ULTRA_ECHO_PIN A0
# define ULTRA_TRIG_PIN A1

#define POT_MIN 0
#define POT_MAX 1023
#define LUM_MIN 0
#define LUM_MAX 300
#define ULTRA_THRESHOLD 20

float prev_lamp_state;

//PID control related global variables
unsigned long previousPIDMillis = 0;
const long PID_POLLING_INTERVAL = 50;
const float P_CONST = 0.5; // Increase this to reduce time to reach full brightness
const float I_CONST = 0;
const float D_CONST = 0;
float prev_error;
float total_error;


//ultrasonic related global variables
unsigned long previousUltraMillis = 0;
const long ULTRA_POLLING_INTERVAL = 500;
bool obstructedStatus = false; //true if obstruction sensed
int ultraHistory[] = {0, 0, 0};
int ultraHistPtr = 0;

//keeps track of potentiometer position in 0 to 1 range
float potentiometerStatus = 0;
unsigned long previousPotMillis = 0;
const long POT_POLLING_INTERVAL = 100;

//luminosity related vals
unsigned long previousLumMillis = 0;
const long LUM_POLLING_INTERVAL = 500;

float luminosityStatus = 0;

/*
  TODO
  - write feedback and update loop
  - write functions that interpret inputs and store to global vars
  - finish pin designations and vars

*/

//setup code, to run once:
void setup() {
  // Change pin 3 to 60 Hz PWM
  pinMode(OUT_PIN, OUTPUT);
  TCCR2B = 0b00000111; // x1024
  TCCR2A = 0b00000011; // fast pwm
  pinMode(LUM_PIN, INPUT);
  pinMode(LAMP_PIN, OUTPUT);

  pinMode(ULTRA_TRIG_PIN, OUTPUT); // Sets the trigPin as an Output
  pinMode(ULTRA_ECHO_PIN, INPUT); // Sets the echoPin as an Input


  //start lamp off
  digitalWrite(LAMP_PIN, convertToLampVal(0));
  prev_lamp_state = 0;

  total_error = 0;
  prev_error = convertLuminosityVal() - convertPotentiometerVal(); //target - current brightness
  // P_CONST;
  // I_CONST;
  // D_CONST;

  Serial.begin(9600);


  if (!veml.begin()) {
    Serial.println("Lux Sensor not found");
    while (1);
  }
  Serial.println("Lux sensor found");

  // == OPTIONAL =====
  // Can set non-default gain and integration time to
  // adjust for different lighting conditions.
  // =================
  // veml.setGain(VEML7700_GAIN_1_8);
  // veml.setIntegrationTime(VEML7700_IT_100MS);

  Serial.print(F("Gain: "));
  switch (veml.getGain()) {
    case VEML7700_GAIN_1: Serial.println("1"); break;
    case VEML7700_GAIN_2: Serial.println("2"); break;
    case VEML7700_GAIN_1_4: Serial.println("1/4"); break;
    case VEML7700_GAIN_1_8: Serial.println("1/8"); break;
  }

  Serial.print(F("Integration Time (ms): "));
  switch (veml.getIntegrationTime()) {
    case VEML7700_IT_25MS: Serial.println("25"); break;
    case VEML7700_IT_50MS: Serial.println("50"); break;
    case VEML7700_IT_100MS: Serial.println("100"); break;
    case VEML7700_IT_200MS: Serial.println("200"); break;
    case VEML7700_IT_400MS: Serial.println("400"); break;
    case VEML7700_IT_800MS: Serial.println("800"); break;
  }

  veml.setLowThreshold(10000);
  veml.setHighThreshold(20000);
  veml.interruptEnable(true);
}



// main code here, to run repeatedly:
void loop() {

  pollPotentiometer();
  pollUltrasonic();
  //  pollLuminosity();
  // determine operation mode
  //  if (getOperationMode()) {
  //    if(!obstructedStatus) {
  //if in smart mode, get target brightness and update lamp
  float PID_change = pollPID();
  float newLampSetting = constrain(prev_lamp_state + PID_change, 0.0, 1.0);
  digitalWrite(LAMP_PIN, convertToLampVal(newLampSetting));
  //      int duty_cycle = map(newLampSetting, 0.0, 1.0, 0, 255);
  Serial.print("newLampSetting: "); Serial.println(newLampSetting);
  int duty_cycle = constrain(newLampSetting * 255, 0, 255);
  Serial.print("Duty_cycle: "); Serial.println(float(duty_cycle / 255.0));
  lightPWM(duty_cycle);
  prev_lamp_state = newLampSetting;
  //    }
  //  }
  //  else {
  //    int pot_val = convertPotentiometerVal();
  //    digitalWrite(LAMP_PIN, convertToLampVal(pot_val));
  //    prev_lamp_state = pot_val;
  //  }
}

//takes val from 0 to 1 and returns the proper signal for lamp strength
float convertToLampVal(float val) {
  //TODO: convert from 0-1 to proper voltage signal to optocoupler
  return val * 5;
}

float convertLuminosityVal() { // Convert from luminosity range to 0-1
  //TODO: convert from luminosity range to 0-1
  return digitalRead(LUM_PIN);
  // return convertInputVal(digitalRead(LUM_PIN), LUM_MIN, LUM_MAX);
}



//return true for smart mode and false for user control
bool getOperationMode() {
  return (digitalRead(MODE_PIN) == 0);
}

float convertPotentiometerVal() {
  return convertInputVal(analogRead(POT_PIN), POT_MIN, POT_MAX);
}

//general function to convert a value in an input range to a target range of 0 to 1
float convertInputVal(float val, float minVal, float maxVal) {
  //converts value from mapping from min-max to mapping from targetMin to targetMax
  float targetMin = 0;
  float targetMax = 1;

  if (maxVal - minVal == 0) {
    return maxVal; //check for divide by 0
  }

  float convertedVal = (val - minVal) / (maxVal - minVal) * (targetMax - targetMin) + targetMin;

  return min(max(convertedVal, 0.0), 1.0);
}


void pollPotentiometer() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousPotMillis >= POT_POLLING_INTERVAL) {
    previousPotMillis = currentMillis;
    potentiometerStatus = convertPotentiometerVal();
  }
}

//checks if its been ULTRA_POLLING_INTERVAL (.5sec) since last poll. if so, checks ultrasonic dist and updates obstructed status
void pollUltrasonic() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousUltraMillis >= ULTRA_POLLING_INTERVAL) {
    previousUltraMillis = currentMillis;
    // Clears the trigPin
    digitalWrite(ULTRA_TRIG_PIN, LOW);
    delayMicroseconds(2);
    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(ULTRA_TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(ULTRA_TRIG_PIN, LOW);
    // Reads the echoPin, returns the sound wave travel time in microseconds
    float duration = pulseIn(ULTRA_ECHO_PIN, HIGH);
    // Calculating the distance
    int distance = duration * 0.034 / 2;

    // Serial.print("Distance: ");
    // Serial.println(distance);
    //update history array to replace oldest value using the ptr
    ultraHistory[ultraHistPtr] = distance;
    ultraHistPtr = (ultraHistPtr + 1) % 3;

    updateObstructedStatus();
    // Serial.print("obstructed stats: ");
    // Serial.println(obstructedStatus);
  }
}

//checks for a new PID poll and if so, returns the change in
float pollPID() {
  unsigned long currentMillis = millis();
  float deltaT = currentMillis - previousPIDMillis;
  if (deltaT >= PID_POLLING_INTERVAL) {
    previousPIDMillis = currentMillis;

    float targetBrightness = convertPotentiometerVal(); //0 to 1 range
    //    float curBrightness = convertLuminosityVal(); //0 to 1 range
    float curBrightness = pollLuminosity(); // 0 to 1 range
    float error = targetBrightness - curBrightness;
    Serial.print("Error: "); Serial.println(error);

    float P_change = P_CONST * error;

    float total_error = total_error + error;
    float I_change = I_CONST * total_error;

    float delta_error = error - prev_error;
    float D_change = D_CONST * delta_error;
    prev_error = error;

    float PID_change = P_change + I_change + D_change;
    return PID_change;
  }
  return 0; //if not enough time has passed for a new value, return no change in brightness
}

void resetPID() {
  unsigned long previousPIDMillis = 0;
  //TODO: set prev_error;
  //TODO: set total_error;
}


float pollLuminosity() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousLumMillis >= LUM_POLLING_INTERVAL) {
    previousLumMillis = currentMillis;

    uint16_t irq = veml.interruptStatus();
    if (irq & VEML7700_INTERRUPT_LOW) {
      Serial.println("** Low threshold");
    }
    if (irq & VEML7700_INTERRUPT_HIGH) {
      Serial.println("** High threshold");
    }

    Serial.print("raw val: ");  Serial.println(veml.readLux());
    luminosityStatus = convertInputVal(veml.readLux(), LUM_MIN, LUM_MAX);

    Serial.print("Luminosity status: "); Serial.println(luminosityStatus);
    return (luminosityStatus);
  }
  // Serial.print("raw ALS: "); Serial.println(veml.readALS());
  // Serial.print("raw white: "); Serial.println(veml.readWhite());
  // Serial.print("lux: "); Serial.println(veml.readLux());
}


//If the ultrasonic sensor has been obstructed or unobstructed for three consecutive polls, update the status accordingly
void updateObstructedStatus() {
  if (ultraHistory[0] < ULTRA_THRESHOLD & ultraHistory[0] < ULTRA_THRESHOLD & ultraHistory[2] < ULTRA_THRESHOLD) {
    obstructedStatus = true;
  }
  else {
    if (ultraHistory[0] > ULTRA_THRESHOLD & ultraHistory[0] > ULTRA_THRESHOLD & ultraHistory[2] > ULTRA_THRESHOLD) {
      if (obstructedStatus == true) {
        resetPID(); //if going back to normal operation, reset PID variables
      }
      obstructedStatus = false;
    }
  }
}

// Update the square wave PWM output based on sensors
void lightPWM(int duty_cycle) {
  analogWrite(OUT_PIN, duty_cycle);
}
