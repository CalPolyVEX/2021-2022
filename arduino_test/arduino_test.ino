#define encoder1A 3
#define encoder1B 2
#define encoder1Out 11
#define ppr 100

int countTick = 0;
int tickA = 0;
int tickB =0;

void setup() {
  pinMode(encoder1A, INPUT);
  pinMode(encoder1B, INPUT);
  pinMode(encoder1Out, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(encoder1A), computeRotation, RISING);
  Serial.begin(9600);
}

void loop() {
  Serial.println(countTick);
  delay(500);
}

void computeRotation() {
  if (digitalRead(encoder1B)) {
    countTick += 1;
  } else {
    countTick -= 1;
  }
  analogWrite(encoder1Out, countTick);
}
