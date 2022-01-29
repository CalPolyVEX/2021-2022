#define encoder1A 3
#define encoder1B 2
#define encoder1Out 11
#define ppr 100
#define baudrate 115200

uint64_t countTick = 0;
int tickA = 0;
int tickB =0;

void setup() {
  pinMode(encoder1A, INPUT);
  pinMode(encoder1B, INPUT);
  pinMode(encoder1Out, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(encoder1A), computeRotation, RISING);
  Serial.begin(baudrate);
}

void loop() {
//  Serial.write(countTick);
  delay(500);
}

void computeRotation() {
  uint64_t val = countTick;
  if (digitalRead(encoder1B)) {
    countTick += 1;
  } else {
    countTick -= 1;
  }
  if (countTick > 281474976710655) {
    countTick -= 281474976710655; //2^48
  } else if (countTick < 0) {
    countTick += 281474976710655;
  }
  Serial.write((uint8_t *)&val, 8);
}
