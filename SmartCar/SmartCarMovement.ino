#include <IRremote.h>
#include <Servo.h>
#include <FastLED.h>

//Motor
//A corresponds to the right set of wheels
int motor_PWMA = 5;
int motor_InA = 7;
//B corresponds to the left set of wheels
int motor_PWMB = 6;
int motor_InB = 8;
int motor_StandBy = 3;
int mSpeed = 100;
float timePassed = 0;
bool movingForward;

//Remote control
int irPin = 9;
IRrecv ir(irPin);
decode_results cmd;
String previousCmd;
String command;

//Distance Sensor
int trigPin = 13;
int echoPin = 12;
int pingTravelTime;
float pingTravelDistance;
float distanceToObstacle;
float totalDistToObstacle;
float avgDistanceToObstacle;
int measurements = 10;
bool canMoveForward = true;
float allowedDistanceToObstacle = 10;
int followDistance = 20;

//Line tracker
int rightPin = A0;
int midPin = A1;
int leftPin = A2;
int rightVal;
int midVal;
int leftVal;

//Servo
int servoPin = 10;
int servoPos = 90;
Servo servo;

//LED
#define ledPin 4
#define numLEDs 1
CRGB leds[numLEDs];
int brightness = 20;
int ledState = 0;
bool staticLED = true;
int ledColour [8][3] = {
  {0, 0, 0},
  {255, 255, 255},
  {255, 0, 0},
  {255, 255, 0},
  {0, 255, 0},
  {0, 255, 255},
  {0, 0, 255},
  {255, 0, 255}
};

//Voltage
int voltagePin = A3;

String mode;
String move1 = "Precision";
String move2 = "Mobility";
String obstAvoid = "ObstacleAvoidance";
String follow = "Follow";
String lineTracking = "LineTracking";
String areaConfinement = "AreaConfinement";
String servoMode = "Servo";
String ledMode = "LED";

void setup() {
  Serial.begin(9600);
  //Motor
  pinMode(motor_PWMA, OUTPUT);
  pinMode(motor_PWMB, OUTPUT);
  pinMode(motor_InA, OUTPUT);
  pinMode(motor_InB, OUTPUT);
  pinMode(motor_StandBy, OUTPUT);
  digitalWrite(motor_StandBy, HIGH);

  //Remote Control
  ir.enableIRIn();

  //Distance sensor
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  //Line tracker
  pinMode(rightPin, INPUT);
  pinMode(midPin, INPUT);
  pinMode(leftPin, INPUT);

  //Servo
  servo.attach(servoPin);
  int servoPos = 90;

  //LED
  FastLED.addLeds<NEOPIXEL, ledPin>(leds, numLEDs);
  FastLED.setBrightness(brightness);

  //Voltage
  pinMode(voltagePin, INPUT);

  mode = move1;
}

void loop() {
  RemoteControl();
  canMoveForward = ObstacleDetection();
  Mode();
  VoltageDetection();
  delay(500);
  ir.resume();
}

void RemoteControl() {
  while (ir.decode(&cmd) == 0) {
    command = "Stationary";
    canMoveForward = ObstacleDetection();
    Mode();
  }
  leds[0].setRGB(0, 255, 0);
  FastLED.show();
  delay(100);
  leds[0].setRGB(0, 0, 0);
  FastLED.show();
  Serial.println(cmd.value, HEX);
  if (cmd.value == 0xFF629D) {
    //Forward();
    command = "Forward";
    previousCmd = "FF629D";
  }
  else if (cmd.value == 0xFFFFFFFF && previousCmd == "FF629D") {
    //Forward();
    command = "Forward";
  }
  else if (cmd.value == 0xFFA857) {
    //Backward();
    command = "Backward";
    previousCmd = "FFA857";
  }
  else if (cmd.value == 0xFFFFFFFF && previousCmd == "FFA857") {
    //Backward();
    command = "Backward";
  }
  else if (cmd.value == 0xFF22DD) {
    //Left();
    command = "Left";
    previousCmd = "FF22DD";
  }
  else if (cmd.value == 0xFFFFFFFF && previousCmd == "FF22DD") {
    //Left();
    command = "Left";
  }
  else if (cmd.value == 0xFFC23D) {
    //Right();
    command = "Right";
    previousCmd = "FFC23D";
  }
  else if (cmd.value == 0xFFFFFFFF && previousCmd == "FFC23D") {
    //Right();
    command = "Right";
  }
  else if (cmd.value == 0xFF4AB5) {
    //Button 0
    mode = ledMode;
  }
  else if (cmd.value == 0xFF6897) {
    //Button 1
    mode = move1;
  }
  else if (cmd.value == 0xFF02FD) {
    //OK
    command = "ModeStationary";
  }
  else if (cmd.value == 0xFF9867) {
    //Button 2
    mode = move2;
  }
  else if (cmd.value == 0xFFB04F) {
    //Button 3
    mode = obstAvoid;
  }
  else if (cmd.value == 0xFF30CF) {
    //Button 4
    mode = follow;
  }
  else if (cmd.value == 0xFF18E7) {
    //Button 5
    mode = lineTracking;
  }
  else if (cmd.value == 0xFF7A85) {
    //Button 6
    mode = areaConfinement;
  }
  else if (cmd.value == 0xFF10EF) {
    //Button 7
    mode = servoMode;
  }
  else if (cmd.value == 0xFF38C7) {
    //Button 8
  }
  else if (cmd.value == 0xFF5AA5) {
    //Button 9
  }
  else if (cmd.value == 0xFF42BD) {
    //Asterisk
    command = "Blinking";
  }
  else if (cmd.value == 0xFF52AD) {
    //Hashtag
    command = "Static";
  }
  else {
    //Stationary();
    command = "Stationary";
  }
}

bool ObstacleDetection() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  pingTravelDistance = ((unsigned int)pulseIn(echoPin, HIGH) / 58.);
  //Serial.println(pingTravelDistance);
  if (pingTravelDistance <= 10) {
    return false;
  }
  else {
    return true;
  }
}

void Mode() {
  if (mode == move1) {
    PrecisionMovement();
  }
  if (mode == move2) {
    FastMovement();
  }
  if (mode == obstAvoid) {
    ObstacleAvoidance();
  }
  if (mode == follow) {
    Follow();
  }
  if (mode == lineTracking) {
    LineTracking();
  }
  if (mode == areaConfinement) {
    AreaConfinement();
  }
  if (mode == servoMode) {
    ServoControl();
  }
  if (mode == ledMode) {
    LEDControl();
  }
}
void PrecisionMovement() {
  mSpeed = 100;
  if (command == "Forward" && canMoveForward == true) {
    Forward();
  }
  if (command == "Backward") {
    Backward();
  }
  if (command == "Left") {
    Left();
  }
  if (command == "Right") {
    Right();
  }
  if (command == "Stationary" || command == "ModeStationary") {
    Stationary();
  }
}

void FastMovement() {
  mSpeed = 100;
  if (command == "Forward" && canMoveForward == true) {
    Forward();
    movingForward = true;
  }
  else if (command == "Backward") {
    Backward();
    movingForward = false;
  }
  else if (command == "Left" && movingForward == true) {
    ForwardTurnLeft();
  }
  else if (command == "Right" && movingForward == true) {
    ForwardTurnRight();
  }
  else if (command == "Left" && movingForward == false) {
    BackwardTurnLeft();
  }
  else if (command == "Right" && movingForward == false) {
    BackwardTurnRight();
  }
  else if (command == "ModeStationary") {
    Stationary();
  }
  else if (canMoveForward == false) {
    Stationary();
  }
}

void ObstacleAvoidance() {
  mSpeed = 100;
  if (command == "ModeStationary") {
    Stationary();
    mode = move1;
  }
  else if (canMoveForward) {
    Forward();
  }
  else {
    Left();
    delay(750);
    Stationary();
    delay(10);
    ObstacleDetection();
    float distOnLeft = pingTravelDistance;
    Serial.print("Dist on left: ");
    Serial.println(distOnLeft);
    Left();
    delay(1500);
    Stationary();
    delay(10);
    ObstacleDetection();
    float distOnRight = pingTravelDistance;
    Serial.print("Dist on right: ");
    Serial.println(distOnRight);
    delay(25);
    if (distOnLeft > distOnRight) {
      Left();
      delay(1500);
    }
  }
}

void Follow() {
  mSpeed = 75;
  bool movingForward;
  bool movingRight;
  bool movingLeft;
  servo.write(90);
  delay(1000);
  ObstacleDetection();
  while (pingTravelDistance > 1 && pingTravelDistance < followDistance) {
    Forward();
    movingForward = false;
    ObstacleDetection();
  }
  Stationary();
  movingForward = false;

  if (movingForward == false) {
    servo.write(45);
    delay(1000);
    ObstacleDetection();
    while (pingTravelDistance > 1 && pingTravelDistance < followDistance) {
      Right();
      movingRight = true;
      ObstacleDetection();
    }
    Stationary();
    movingRight = false;
  }

  if (movingRight == false) {
    servo.write(135);
    delay(1000);
    ObstacleDetection();
    while (pingTravelDistance > 1 && pingTravelDistance < followDistance) {
      Left();
      movingLeft = true;
      ObstacleDetection();
    }
    Stationary();
    movingLeft = false;
  }
  if (command == "ModeStationary") {
    mode = move1;
    servo.write(90);
  }
}

void ServoControl() {
  if (command == "Left") {
    if (servoPos <= 165) {
      servoPos += 5;
    }
  }
  if (command == "Right") {
    if (servoPos >= 15) {
      servoPos -= 5;
    }
  }
  if (command == "ModeStationary") {
    servoPos = 90;
  }
  servo.write(servoPos);
}

void LineTracking() {
  mSpeed = 75;
  rightVal = analogRead(rightPin);
  midVal = analogRead(midPin);
  leftVal = analogRead(leftPin);
  if (midVal > 750) {
    Forward();
  }
  else if (leftVal > 500) {
    Left();
  }
  else if (rightVal > 500) {
    Right();
  }
  else {
    Stationary();
  }
  if (midVal > 400 && leftVal > 400 && rightVal > 400) {
    //Lifted up
    Stationary();
  }
}

void AreaConfinement() {
  mSpeed = 75;

  if (command == "ModeStationary") {
    Stationary();
  }
  else if (command == "Forward") {
    rightVal = analogRead(rightPin);
    midVal = analogRead(midPin);
    leftVal = analogRead(leftPin);
    Forward();
    if (midVal > 500 || leftVal > 500 || rightVal > 500) {
      Left();
      delay(1000);
    }
  }
}

void LEDControl() {
  if (command == "Forward") {
    //Bright up
    brightness += 10;
    if (brightness > 255) {
      brightness = 255;
    }
    FastLED.setBrightness(brightness);
    leds[0].setRGB(ledColour[ledState][0], ledColour[ledState][1], ledColour[ledState][2]);
    FastLED.show();
    Serial.println(brightness);
  }
  if (command == "Backward") {
    //Bright down
    brightness -= 10;
    if (brightness < 0) {
      brightness = 0;
    }
    FastLED.setBrightness(brightness);
    leds[0].setRGB(ledColour[ledState][0], ledColour[ledState][1], ledColour[ledState][2]);
    FastLED.show();
    Serial.println(brightness);
  }
  if (command == "Left") {
    //Cycle left
    ledState --;
    if (ledState < 0) {
      ledState = 7;
    }
  }
  if (command == "Right") {
    //Cycle right
    ledState ++;
    if (ledState > 7) {
      ledState = 0;
    }
    leds[0].setRGB(ledColour[ledState][0], ledColour[ledState][1], ledColour[ledState][2]);
    FastLED.show();
  }
  if (command == "Blinking") {
    //Blinky lights
    staticLED = false;
  }
  if (command == "Static") {
    //Static lights
    staticLED = true;
  }

  if (staticLED == false) {
    leds[0].setRGB(ledColour[ledState][0], ledColour[ledState][1], ledColour[ledState][2]);
    FastLED.show();
    delay(500);
    leds[0].setRGB(0, 0, 0);
    FastLED.show();
    delay(500);
  }
  else {
    leds[0].setRGB(ledColour[ledState][0], ledColour[ledState][1], ledColour[ledState][2]);
  }
}

void Stationary() {
  digitalWrite(motor_PWMA, 0);
  digitalWrite(motor_PWMB, 0);
}
void Forward() {
  digitalWrite(motor_InA, HIGH); //Determines direction; HIGH is backwards, LOW is forwards
  analogWrite(motor_PWMA, mSpeed); //Determines speed
  digitalWrite(motor_InB, HIGH);
  analogWrite(motor_PWMB, mSpeed - 5);
}
void ForwardTurnLeft() {
  digitalWrite(motor_InA, HIGH); //Determines direction; HIGH is backwards, LOW is forwards
  analogWrite(motor_PWMA, mSpeed + (mSpeed / 3.)); //Determines speed
  digitalWrite(motor_InB, HIGH);
  analogWrite(motor_PWMB, (mSpeed - (mSpeed / 3.) / 3.));
}
void ForwardTurnRight() {
  digitalWrite(motor_InA, HIGH);
  analogWrite(motor_PWMA, (mSpeed - (mSpeed / 3.) / 3.));
  digitalWrite(motor_InB, HIGH);
  analogWrite(motor_PWMB, mSpeed + (mSpeed / 3.));
}
void BackwardTurnLeft() {
  digitalWrite(motor_InA, LOW);
  analogWrite(motor_PWMA, mSpeed + (mSpeed / 3.));
  digitalWrite(motor_InB, LOW);
  analogWrite(motor_PWMB, (mSpeed - (mSpeed / 3.) / 3.));
}
void BackwardTurnRight() {
  digitalWrite(motor_InA, LOW);
  analogWrite(motor_PWMA, (mSpeed - (mSpeed / 3.) / 3.));
  digitalWrite(motor_InB, LOW);
  analogWrite(motor_PWMB, mSpeed + (mSpeed / 3.));
}
void Backward() {
  digitalWrite(motor_InA, LOW);
  analogWrite(motor_PWMA, mSpeed);
  digitalWrite(motor_InB, LOW);
  analogWrite(motor_PWMB, mSpeed);
}
void Right() {
  digitalWrite(motor_InA, LOW);
  analogWrite(motor_PWMA, mSpeed);
  digitalWrite(motor_InB, HIGH);
  analogWrite(motor_PWMB, mSpeed);
}
void Left() {
  digitalWrite(motor_InA, HIGH);
  analogWrite(motor_PWMA, mSpeed);
  digitalWrite(motor_InB, LOW);
  analogWrite(motor_PWMB, mSpeed);
}


void VoltageDetection() {
  int voltReading = analogRead(voltagePin);
  float voltage = voltReading * 5 * ((10 + 1.5) / 1.5) / 1024.;
  voltage = voltage + (voltage * 0.08);
  Serial.print("Voltage: ");
  Serial.println(voltage);
  if (voltage > 7.8) {
    Serial.println("Battery is fully charged");
  }
  else if (voltage < 2) {
    Serial.println("Low battery");
  }
}
