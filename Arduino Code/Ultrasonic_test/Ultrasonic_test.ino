// # define TEST_PIN A0
# define ULTRA_ECHO_PIN A0
# define ULTRA_TRIG_PIN A1

#define ULTRA_THRESHOLD 20

unsigned long previousUltraMillis = 0;
const long ULTRA_POLLING_INTERVAL = 500;
bool obstructedStatus = false; //true if obstruction sensed
int ultraHistory[] = {0,0,0};
int ultraHistPtr = 0;

void setup() {
  // put your setup code here, to run once:
  // pinMode(TEST_PIN, INPUT);
  pinMode(ULTRA_TRIG_PIN, OUTPUT); // Sets the trigPin as an Output
  pinMode(ULTRA_ECHO_PIN, INPUT); // Sets the echoPin as an Input

  Serial.begin(9600);
}


void loop() {
  // put your main code here, to run repeatedly:

  
  pollUltrasonic();


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

    Serial.print("Distance: ");
    Serial.println(distance);
    //update history array to replace oldest value using the ptr
    ultraHistory[ultraHistPtr] = distance;
    ultraHistPtr = (ultraHistPtr + 1) % 3;

    updateObstructedStatus();
    Serial.print("obstructed stats: ");
    Serial.println(obstructedStatus);
  }
}
  

//If the ultrasonic sensor has been obstructed or unobstructed for three consecutive polls, update the status accordingly
void updateObstructedStatus() {
  if (ultraHistory[0] < ULTRA_THRESHOLD & ultraHistory[0] < ULTRA_THRESHOLD & ultraHistory[2] < ULTRA_THRESHOLD) {
    obstructedStatus = true;
  }
  else {
    if (ultraHistory[0] > ULTRA_THRESHOLD & ultraHistory[0] > ULTRA_THRESHOLD & ultraHistory[2] > ULTRA_THRESHOLD) {
      obstructedStatus = false;
    }
  }
}
