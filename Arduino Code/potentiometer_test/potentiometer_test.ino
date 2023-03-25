
# define POT_PIN A1
#define POT_MIN 0
#define POT_MAX 1023

unsigned long previousPotMillis = 0;
const long POT_POLLING_INTERVAL = 100;

float potentiometerStatus = 0;

void setup() {
  // put your setup code here, to run once:
  // pinMode(TEST_PIN, INPUT);

  Serial.begin(9600);
}


void loop() {
  // put your main code here, to run repeatedly:
  pollPotentiometer();


}

void pollPotentiometer() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousPotMillis >= POT_POLLING_INTERVAL) {
    previousPotMillis = currentMillis;
    potentiometerStatus = convertPotentiometerVal();
    Serial.println(potentiometerStatus);
  }    
}

float convertPotentiometerVal(){
  return convertInputVal(analogRead(POT_PIN), POT_MIN, POT_MAX);
}

float convertInputVal(float val, float min, float max){
  //converts value from mapping from min-max to mapping from targetMin to targetMax
  float targetMin = 0;
  float targetMax = 1;
  //TODO check for divide by 0
  return (val-min)/(max-min) * (targetMax-targetMin) + targetMin;
}

  