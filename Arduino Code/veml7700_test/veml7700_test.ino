#include "Adafruit_VEML7700.h"

Adafruit_VEML7700 veml = Adafruit_VEML7700();

#define LUM_MIN 0
#define LUM_MAX 300

unsigned long previousLumMillis = 0;
const long LUM_POLLING_INTERVAL = 500;

float luminosityStatus = 0;

void setup() {
  Serial.begin(9600); //115200
  while (!Serial) { delay(10); }
  Serial.println("Adafruit VEML7700 Test");

  if (!veml.begin()) {
    Serial.println("Sensor not found");
    while (1);
  }
  Serial.println("Sensor found");

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

void loop() {

  pollLuminosity();


}


void pollLuminosity() {
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

    // Serial.println(luminosityStatus);
  }    
  // Serial.print("raw ALS: "); Serial.println(veml.readALS());
  // Serial.print("raw white: "); Serial.println(veml.readWhite());
  // Serial.print("lux: "); Serial.println(veml.readLux());
}



float convertInputVal(float val, float min, float max){
  //converts value from mapping from min-max to mapping from targetMin to targetMax
  float targetMin = 0;
  float targetMax = 1;
  
  if (max-min == 0) {return max;} //check for divide by 0
  
  float convertedVal = (val-min)/(max-min) * (targetMax-targetMin) + targetMin;

  return min(max(convertedVal,0.0),1.0);
}
