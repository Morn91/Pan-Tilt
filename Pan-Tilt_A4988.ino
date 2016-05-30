#include <AccelStepper.h>

#define BATTERY 3

#define STEPX 2
#define DIRX 3
#define STEPY 4
#define DIRY 5
#define FOCUS 6
#define SHUTTER 7
#define MAX_SPEED 1536
#define LAP 15360
#define COEFFICIENT 0.9666

AccelStepper stepperX(AccelStepper::DRIVER, STEPX, DIRX);
AccelStepper stepperY(AccelStepper::DRIVER, STEPY, DIRY);

float interval = 0, time = 0, elapsed = 0;
int number = 0, n = 0;
unsigned long start = 0;
long startX = 0, startY = 0, endX = 0, endY = 0;

void setup() {
  Serial.begin(9600);
  pinMode(13, OUTPUT);
  pinMode(FOCUS, OUTPUT);
  pinMode(SHUTTER, OUTPUT);
  stepperX.setMaxSpeed(MAX_SPEED);
  stepperY.setMaxSpeed(MAX_SPEED);
}

void loop() {
  if(start)
    work();
  else
    tuning();
}

void tuning() {
  stepperX.runSpeed();
  stepperY.runSpeed();
  if(Serial.available() > 0) {
    char val = Serial.read();
    switch(val) {
    case 'r':
      stepperX.setSpeed(MAX_SPEED/2);
      break;
    case 'l':
      stepperX.setSpeed(-MAX_SPEED/2);
      break;
    case 'u':
      stepperY.setSpeed(MAX_SPEED/2);
      break;
    case 'd':
      stepperY.setSpeed(-MAX_SPEED/2);
      break;
    case 's':
      stepperX.setSpeed(0);
      stepperY.setSpeed(0);
      break;
    case 'x':
      startX = stepperX.currentPosition();
      startY = stepperY.currentPosition();
      break;
    case 'y':
      endX = stepperX.currentPosition();
      endY = stepperY.currentPosition();
      break;
    case 'a':
      stepperX.moveTo(startX);
      stepperY.moveTo(startY);
      rewind();
      break;
    case 'b':
      stepperX.moveTo(endX);
      stepperY.moveTo(endY);
      rewind();
      break;
    case '*':
      report();
      break;
    case '+':
      delay(100);
      char buffer[20];
      int i = 0;
      while(Serial.available() && i < 19)
        buffer[i++] = Serial.read();
      buffer[i++] = '\0';
      sscanf(buffer, "%d %d", &number, &i);
      interval = i / 100.0;
      startX = stepperX.currentPosition();
      startY = stepperY.currentPosition();
      float tmpTime = number * interval;
      float path = max(abs(endX - startX), abs(endY - startY));
      if(path > 0 && path / tmpTime <= MAX_SPEED) {
        time = tmpTime;
        work();
      }
      break;
    }
  }
}

void stop() {
  digitalWrite(13, LOW);
  digitalWrite(FOCUS, LOW);
  digitalWrite(SHUTTER, LOW);
  start = 0;
  time = 0;
  elapsed = 0;
  n = 0;
  while(Serial.available() > 0)
    Serial.read();
}

void work() {
  if(Serial.available() > 0) {
    char val = Serial.read();
    if(val == '-') {
      stop();
      return;
    }
    if(val == '*')
      report();
  }
  if(!start) {
    digitalWrite(13, HIGH);
    digitalWrite(FOCUS, HIGH);
    delay(100);
    start = millis();
  }
  elapsed = (millis() - start) / 1000.0;
  if(elapsed >= interval * n && n < number) {
    stepperX.moveTo(startX + (endX - startX) * n / (number - 1));
    stepperY.moveTo(startY + (endY - startY) * n / (number - 1));
    rewind();
    digitalWrite(SHUTTER, HIGH);
    delay(100);
    digitalWrite(SHUTTER, LOW);
    n++;
  }
  if(elapsed >= interval * number) {
    stop();
  }
}

void rewind() {
  float pathX = abs(stepperX.distanceToGo());
  float pathY = abs(stepperY.distanceToGo());
  float tmpTime = max(pathX, pathY) / MAX_SPEED;
  stepperX.setSpeed(pathX / tmpTime);
  stepperY.setSpeed(pathY / tmpTime);
  while(true) {
    stepperX.runSpeedToPosition();
    stepperY.runSpeedToPosition();
    if(stepperX.distanceToGo() == 0 && stepperY.distanceToGo() == 0)
      break;
  }
  stepperX.setSpeed(0);
  stepperY.setSpeed(0);
}

void report() {
  int i, voltage = 0;
  for(i = 0; i < 20; i++) {
    voltage += analogRead(BATTERY);
    delay(5);
  }
  Serial.print(number);
  Serial.print('\t');
  Serial.print(interval * 100, 0);
  Serial.print('\t');
  Serial.print(time, 0);
  Serial.print('\t');
  Serial.print(time - elapsed, 0);
  Serial.print('\t');
  Serial.print((endX - startX) * 3600.0 / LAP, 0);
  Serial.print('\t');
  Serial.print((endY - startY) * 3600.0 / LAP, 0);
  Serial.print('\t');
  Serial.println(voltage * COEFFICIENT / i, 0);
}
