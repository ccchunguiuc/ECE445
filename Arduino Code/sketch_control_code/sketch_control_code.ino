#define POT_PIN A0// PIN_PB4
#define LUM_PIN A0// PIN_PB2
#define MODE_PIN A0 //A3 //operation mode
#define LAMP_PIN A0//A3
# define ULTRA_ECHO_PIN A0
# define ULTRA_TRIG_PIN A1

#define POT_MIN 0
#define POT_MAX 1023
#define LUM_MIN 0 //TODO update
#define LUM_MAX 1 //TODO update
#define ULTRA_THRESHOLD 20

float prev_lamp_state;

//PID control related global variables
unsigned long previousPIDMillis = 0;
const long PID_POLLING_INTERVAL = 50;
const float P_CONST = 0.5;
const float I_CONST = 0;
const float D_CONST = 0;
float prev_error;
float total_error; 


//ultrasonic related global variables
unsigned long previousUltraMillis = 0;
const long ULTRA_POLLING_INTERVAL = 500;
bool obstructedStatus = false; //true if obstruction sensed
int ultraHistory[] = {0,0,0};
int ultraHistPtr = 0;

//keeps track of potentiometer position in 0 to 1 range
float potentiometerStatus = 0; 
unsigned long previousPotMillis = 0;
const long POT_POLLING_INTERVAL = 100;

/*
TODO
- write feedback and update loop
- write functions that interpret inputs and store to global vars
- finish pin designations and vars

*/

  //setup code, to run once:
void setup() {
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
}



// main code here, to run repeatedly:
void loop() {

  pollPotentiometer();
  pollUltrasonic();

  // determine operation mode
  if (getOperationMode()) {
    if(!obstructedStatus) {
      //if in smart mode, get target brightness and update lamp
      float PID_change = pollPID();

      float newLampSetting = prev_lamp_state + PID_change;
      digitalWrite(LAMP_PIN, convertToLampVal(newLampSetting));
      prev_lamp_state = newLampSetting;
    }

  }
  else {
    int pot_val = convertPotentiometerVal();
    digitalWrite(LAMP_PIN, convertToLampVal(pot_val));
    prev_lamp_state = pot_val;
  }



}

//takes val from 0 to 1 and returns the proper signal for lamp strength
float convertToLampVal(float val){
  //TODO: convert from 0-1 to proper voltage signal to optocoupler
  return val*5;
}

float convertLuminosityVal(){
  //TODO: convert from luminosity range to 0-1
  return digitalRead(LUM_PIN);
  // return convertInputVal(digitalRead(LUM_PIN), LUM_MIN, LUM_MAX);
}



//return true for smart mode and false for user control
bool getOperationMode() {
  return (digitalRead(MODE_PIN) == 0);
}

float convertPotentiometerVal(){
  return convertInputVal(analogRead(POT_PIN), POT_MIN, POT_MAX);
}

//general function to convert a value in an input range to a target range of 0 to 1
float convertInputVal(float val, float min, float max){
  //converts value from mapping from min-max to mapping from targetMin to targetMax
  float targetMin = 0;
  float targetMax = 1;
  //TODO check for divide by 0
  return (val-min)/(max-min) * (targetMax-targetMin) + targetMin;
}


void pollPotentiometer() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousPotMillis >= POT_POLLING_INTERVAL) {
    previousPotMillis = currentMillis;
    potentiometerStatus = convertPotentiometerVal();
    Serial.println(potentiometerStatus);
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

    //UPDATE PID VARS
    
    float targetBrightness = convertPotentiometerVal(); //0 to 1 range
    float curBrightness = convertLuminosityVal(); //0 to 1 range
    float error = targetBrightness - curBrightness;

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

//If the ultrasonic sensor has been obstructed or unobstructed for three consecutive polls, update the status accordingly
void updateObstructedStatus() {  
  if (ultraHistory[0] < ULTRA_THRESHOLD & ultraHistory[0] < ULTRA_THRESHOLD & ultraHistory[2] < ULTRA_THRESHOLD) {
    obstructedStatus = true;
  }
  else {
    if (ultraHistory[0] > ULTRA_THRESHOLD & ultraHistory[0] > ULTRA_THRESHOLD & ultraHistory[2] > ULTRA_THRESHOLD) {
      if(obstructedStatus == true) {resetPID();} //if going back to normal operation, reset PID variables
      obstructedStatus = false;
    }
  }
}




