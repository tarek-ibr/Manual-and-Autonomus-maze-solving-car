#include <Servo.h>
Servo myServo;

//seconds required for the car to turn
int turningSecs = 560;

//Bluetooth pins
// Start Serial1 for Bluetooth (PA9/PA10) (9 RX REGULATOR) , (10 TX)

//servo pins
const int servoPin = PA8; 

// UltraSonic sensor
const int trigPin = PA2;
const int echoPin = PB12;

// Motor PWM pins
const int leftMotorPWM = PA0;   // TIM2_CH1
const int rightMotorPWM = PA1;  // TIM2_CH2


// Motor direction pins
const int leftMotorDir1 = PB0;
const int leftMotorDir2 = PB1;
const int rightMotorDir1 = PB10;
const int rightMotorDir2 = PB11;

// Debug LEDs
const int debugLED = PC13;


//Blinkers and stop pins
const int rightBlink = PB6;
const int leftBlink = PB7;
const int stopLed = PA3;

//buzz
const int buzzPin = PB5;  //  PB5

// Command from Bluetooth
char command = 'S';

// the speed of the car
int carSpeed = 110;

void setup() {

  // Initialize motor pins
  pinMode(leftMotorPWM, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(leftMotorDir1, OUTPUT);
  pinMode(leftMotorDir2, OUTPUT);
  pinMode(rightMotorDir1, OUTPUT);
  pinMode(rightMotorDir2, OUTPUT);

  // LED pins
  // pinMode(debugLED, OUTPUT);
  

  // Start Serial1 for Bluetooth (PA9/PA10)
  Serial1.begin(9600);  // HC-05 default baud //9RX REGULATOR , 10 TX

  // Ultrasonic
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Blinkers and stop pins
  pinMode(rightBlink, OUTPUT);
  pinMode(leftBlink, OUTPUT);
  pinMode(stopLed, OUTPUT);

  //buzz
  pinMode(buzzPin, OUTPUT);  // Set PB5 as output
  digitalWrite(buzzPin, LOW);  // PB5 OFF initially

  //Servo
  myServo.attach(servoPin);
}

bool autonomousActive = false;

void loop() {
  
  if (autonomousActive) {
    AutonomousMode();
  } else {
    bluetoothMode();
  }
  delay(200);
}

void toggleLed(const int LedPin, const int duration){
  int current = 0;
  while (current<duration){
    digitalWrite(LedPin, HIGH);
    delay(100);
    digitalWrite(LedPin, LOW);
    delay(100);
    current +=200;
  }
}

// Set direction for both motors
void setMotorDirection(int leftOption, int rightOption) {

  //right motor
  if(rightOption == 0){ //stop
    digitalWrite(rightMotorDir1, 1);
    digitalWrite(rightMotorDir2, 1);
  }
  else if(rightOption == 1){ //forward
    digitalWrite(rightMotorDir1, 1);
    digitalWrite(rightMotorDir2, 0);
  }
  else if(rightOption == 2){ //backward
    digitalWrite(rightMotorDir1, 0);
    digitalWrite(rightMotorDir2, 1);
  }

  //left motor
  if(leftOption == 0){
    digitalWrite(leftMotorDir1, 1);
  digitalWrite(leftMotorDir2, 1);
  }
  else if(leftOption == 1){
    digitalWrite(leftMotorDir1, 1);
  digitalWrite(leftMotorDir2, 0);
  }
  else if(leftOption == 2){
    digitalWrite(leftMotorDir1, 0);
  digitalWrite(leftMotorDir2, 1);
  }
}

// Set speed (0–255)
void setMotorSpeed(int leftSpeed, int rightSpeed) {
  analogWrite(leftMotorPWM, leftSpeed);
  analogWrite(rightMotorPWM, rightSpeed);
}

long readUltrasonicCM() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000); // 30 ms timeout
  long distance = duration * 0.034 / 2;          // cm
  return distance;
}

void setServoAngle(int angle) {
  angle = constrain(angle, 0, 180);  // Make sure it's within bounds
  myServo.write(angle);
}

void moveBackward(){
  setMotorDirection(1, 1);
  setMotorSpeed(carSpeed, carSpeed);
  digitalWrite(rightBlink, HIGH);
  digitalWrite(leftBlink, HIGH);
  delay(turningSecs+200);
  digitalWrite(rightBlink, LOW);
  digitalWrite(leftBlink, LOW);
}

void moveForward(){
  setMotorDirection(2, 2);
  setMotorSpeed(carSpeed, carSpeed);
}

void moveRight(){
  turningSecs = -2 * carSpeed + 780;
  setMotorDirection(2, 1);
  setMotorSpeed(carSpeed, carSpeed);
  digitalWrite(rightBlink, HIGH);
  delay(turningSecs);
  digitalWrite(rightBlink, LOW);
  moveForward();
}

void moveLeft(){
  turningSecs = -2 * carSpeed + 780;
  setMotorDirection(1, 2);
  setMotorSpeed(carSpeed, carSpeed);
  digitalWrite(leftBlink, HIGH);
  delay(turningSecs);
  digitalWrite(leftBlink, LOW);
  moveForward();
}

void turn(){
  turningSecs = -2 * carSpeed + 780;
  setMotorDirection(2, 1);
  setMotorSpeed(carSpeed, carSpeed);
  digitalWrite(rightBlink, HIGH);
  digitalWrite(leftBlink, HIGH);
  digitalWrite(stopLed, HIGH);
  digitalWrite(buzzPin, HIGH);
  delay(2*turningSecs);
  digitalWrite(rightBlink, LOW);
  digitalWrite(leftBlink, LOW);
  digitalWrite(buzzPin, LOW);
  digitalWrite(stopLed, LOW);
  setMotorDirection(0, 0);
  setMotorSpeed(0, 0);

}


void bluetoothMode() {
  if (Serial1.available() > 0) {
    command = Serial1.read();
    Serial1.write(command);  // Echo back



    switch (command) {
      case 'F':  // Forward
      case 'f':  // Forward
        moveForward();
        break;
      case 'B':  // Backward
      case 'b':  // Backward
        moveBackward();
        break;
      case 'L':  // Left turn
      case 'l':  // Left turn
        moveLeft();
        break;
      case 'R':  // Right turn
      case 'r':  // Right turn
        moveRight();
        break;
      case 'S':  // Stop
      case 's':  // Stop
        setMotorDirection(0, 0);
        digitalWrite(stopLed, HIGH);
        setMotorSpeed(0, 0);
        digitalWrite(buzzPin, HIGH);
        delay(3000);
        digitalWrite(buzzPin, LOW);
        digitalWrite(stopLed, LOW);
        //delay(1000);
        autonomousActive = false;
        break;
      case 'A':
      case 'a':
        autonomousActive = true;
        break;
      case 'H':  // 
      case 'h':  // 
        digitalWrite(buzzPin, HIGH);
        delay(1000);
        digitalWrite(buzzPin, LOW);
        break;
      case 'P':  // 
      case 'p':  // 
        if(carSpeed<200)
        {
          carSpeed+=10;
          
          setMotorSpeed(carSpeed, carSpeed);
        }
        break;
      case 'M':  // 
      case 'm':  // 
        if(carSpeed>100)
        {
          carSpeed-=10;
          setMotorSpeed(carSpeed, carSpeed);
        }
        break;
      case 'T':  //
      case 't':  //
        turn();
        break;
    }
    
  }
}

void AutonomousMode(){
    carSpeed= 110;
    turningSecs = 560;
    setMotorSpeed(carSpeed, carSpeed);
  // Step 1: Face forward
  command = Serial1.read();
  Serial1.write(command);  // Echo back

  if((command == 'S') || (command == 's') || (command == 'C') || (command == 'c')){
    setMotorDirection(0, 0);
    digitalWrite(stopLed, HIGH);
    setMotorSpeed(0, 0);
    digitalWrite(buzzPin, HIGH);
    delay(3000);
    digitalWrite(buzzPin, LOW);
    digitalWrite(stopLed, LOW);
    //delay(1000);
    autonomousActive = false;
    return;
  }
  setServoAngle(90);
  uint32_t front_dist = readUltrasonicCM();
  

  if (front_dist > 40)  // No obstacle → Go forward
  {
    moveForward();
  }
  else  // Obstacle detected
  { 
    setMotorDirection(0,0);
    // Step 2: Check LEFT
    setServoAngle(180);
    delay(500);
    int left_dist = readUltrasonicCM();

    // Step 3: Check RIGHT
    setServoAngle(0);
    delay(500);
    setServoAngle(90);

    int right_dist = readUltrasonicCM();

    // Step 4: Decide where to go
    if ((left_dist >= right_dist) && left_dist > 30)
    {
      moveLeft();
    }
    else if ((right_dist > left_dist) && right_dist > 30)
    {
      moveRight();
    }
    else  // Blocked → Stop
    {
      setMotorDirection(0, 0);
      digitalWrite(stopLed, HIGH);
      setMotorSpeed(0, 0);

      digitalWrite(buzzPin, HIGH);
      delay(3000);
      digitalWrite(buzzPin, LOW);
      digitalWrite(stopLed, LOW);
      //delay(1000);
    }
  }
  
}