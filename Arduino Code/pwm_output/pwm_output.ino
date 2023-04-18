#define OUT_PIN 3

void lightPWM(int duty_cycle) {
  analogWrite(OUT_PIN, duty_cycle);
}

void setup() {
  // put your setup code here, to run once:
  pinMode(OUT_PIN, OUTPUT);
  // Change pin 3 to 60 Hz
  TCCR2B = 0b00000111; // x1024
  TCCR2A = 0b00000011; // fast pwm
}

void loop() {
  // put your main code here, to run repeatedly:
  int duty = random(255);
  lightPWM(duty);
  delay(500);
}
