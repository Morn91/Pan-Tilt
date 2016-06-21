#include <AccelStepper.h>

#define BATTERY 3

#define STEPX 2
#define DIRX 3
#define STEPY 4
#define DIRY 5
#define FOCUS 6
#define SHUTTER 7
#define LAP 15360
#define MAX_SPEED 1536
#define DELAY 100
#define VOLTAGE_K 0.9666

AccelStepper stepperX(AccelStepper::DRIVER, STEPX, DIRX);
AccelStepper stepperY(AccelStepper::DRIVER, STEPY, DIRY);

float k = 0, maxSpeed = MAX_SPEED * 1.01;
unsigned int number = 0, n = 0, holdup = 0;
unsigned long period = 0, start = 0, time = 0, elapsed = 0;
long startX = 0, startY = 0, endX = 0, endY = 0;
bool smooth = 0;

void setup() {
  Serial.begin(9600);
  pinMode(13, OUTPUT);
  pinMode(FOCUS, OUTPUT);
  pinMode(SHUTTER, OUTPUT);
  stepperX.setMaxSpeed(maxSpeed);
  stepperY.setMaxSpeed(maxSpeed);
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
      stepperX.setSpeed(maxSpeed / 2);
      break;
    case 'l':
      stepperX.setSpeed(-maxSpeed / 2);
      break;
    case 'u':
      stepperY.setSpeed(maxSpeed / 2);
      break;
    case 'd':
      stepperY.setSpeed(-maxSpeed / 2);
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
      delay(DELAY);
      int i = 0, size = 50;
      char buffer[size];
      while(Serial.available() && i < size - 1)
        buffer[i++] = Serial.read();
      buffer[i++] = '\0';
      sscanf(buffer, "%d %d %d", &number, &period, &smooth);
      startX = stepperX.currentPosition();
      startY = stepperY.currentPosition();
      long path = max(abs(endX - startX), abs(endY - startY));
      holdup = ceil(1000.0 * (smooth ? 2 : 1) * path / MAX_SPEED / (number - 1));
      if(period >= holdup + DELAY) {
        time = (unsigned long)number * period;
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
    delay(DELAY);
    start = millis();
  }
  elapsed = millis() - start;
  if(elapsed + holdup >= (unsigned long)n * period) {
    if(smooth)
      k = n < (number - 1) / 2.0 ? pow((n * 2.0 / (number - 1)), 2) / 2 : 1 - pow((n * 2.0 / (number - 1) - 2), 2) / 2;
    else
      k = n / (float)(number - 1);
    stepperX.moveTo(startX + (endX - startX) * k);
    stepperY.moveTo(startY + (endY - startY) * k);
    if(n++)
      rewind();
    digitalWrite(SHUTTER, HIGH);
    delay(DELAY);
    digitalWrite(SHUTTER, LOW);
  }
  if(n >= number || elapsed >= time)
    stop();
}

void rewind() {
  unsigned long rewindStart = millis();
  float pathX = abs(stepperX.distanceToGo());
  float pathY = abs(stepperY.distanceToGo());
  float tmpTime = max(pathX, pathY) / maxSpeed;
  stepperX.setSpeed(pathX / tmpTime);
  stepperY.setSpeed(pathY / tmpTime);
  while(stepperX.distanceToGo() != 0 || stepperY.distanceToGo() != 0) {
    stepperX.runSpeedToPosition();
    stepperY.runSpeedToPosition();
  }
  stepperX.setSpeed(0);
  stepperY.setSpeed(0);
  while(millis() < rewindStart + holdup - 1);
}

void report() {
  int i, voltage = 0;
  for(i = 0; i < 20; i++) {
    voltage += analogRead(BATTERY);
    delay(5);
  }
  Serial.print(number);
  Serial.print('\t');
  Serial.print(period);
  Serial.print('\t');
  Serial.print(smooth);
  Serial.print('\t');
  Serial.print(n);
  Serial.print('\t');
  Serial.print(time / 1000.0, 0);
  Serial.print('\t');
  Serial.print((time - elapsed) / 1000.0, 0);
  Serial.print('\t');
  Serial.print((endX - startX) * 3600.0 / LAP, 0);
  Serial.print('\t');
  Serial.print((endY - startY) * 3600.0 / LAP, 0);
  Serial.print('\t');
  Serial.println(voltage * VOLTAGE_K / i, 0);
}
