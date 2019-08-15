/***
 * Variable message sequence = (ForwardButton, BackwardButton, RightButton, LeftButton, ArmSelectionList, ArmMovement, HeadMovement)
 * FowardButton = Pressed(1), not pressed (0);
 * BackwardButton = Pressed(1), not pressed (0);
 * RightButton = Pressed(1), not pressed (0);
 * LeftButton = Pressed(1), not pressed (0);
 * ArmSelectionList = Left(0), Right(3), Both(2), not selected(4); 1 is not used to prevent from missending LeftButton value
 * ArmMovement = UpClick(0), DownClick(1), UpHold(2), DownHold(3), not pressed (4);
 * HeadMovement = UpClick(0), DownClick(1), UpHold(2), DownHold(3), not pressed (4);
 * 
 */

#include <SoftwareSerial.h>
#include <Servo.h>

#define motorMaxSpeed 255 // The maximum speed the motor will run and the max it can power up
#define motorMinSpeed 230 // The minimun speed the motor will run before reseting it back to max and retune it

/*** Motor ***/

/*** Encoder pins ***/
const int motorA1 = 2; // Pin for Encoder A of Motor 1
const int motorB1 = 9; // Pin for Encoder B of Motor 1
const int motorA2 = 3; // Pin for Encoder A of Motor 2
const int motorB2 = 12; //Pin for Encoder B of Motor 2

/*** Motor Shield Pins ***/
const int pwm1 = 5; // Take motorspeed1 as input
const int dir1 = 4; // Determine the direction of motor1 rotation, HIGH = clockwise
const int pwm2 = 6; // Take motorspeed2 as input
const int dir2 = 7; // Determine the direction of motor2 rotation, HIGH = clockwise

/*** Variable to store the speed of motors ***/
int motorSpeed1 = 255; // Speed of Motor1, from 0 - 255
int motorSpeed2 = 255; // Speed of Motor1, from 0 - 255

/*** Variable for adjusting the motor speed to rotate in the same speed ***/
int previousState, currentState; // Used to determine if the motor is rotating
int previousMilli, currentMilli = 0; // Serve as counting timer for rpm

const int interval = 1000; // Used to determine if 1 sec has passed
const int rmp = 150; // RPM of the motor used

const int k = 2; // Differential propotional value

int steps = 0; // Steps for motor1
int steps2 = 0; // Steps for motor2

int previousSteps = 0; // Store immediately the value of current step for one of the motor
int laststep, laststep2; // Store the last steps for each motor after calculating RPM

float rpm1, rpm2; // RPM for both motors


/*** Servo ***/

/*** Servo pins ***/
const int leftArm = 8; // Left Arm servo pin
const int rightArm = 13; // Right Arm servo pin
const int headArm = A0; // Head servo pin

/*** Servo names ***/
Servo leftServo, rightServo, headServo;

/*** Servo variable and their initial pos ***/
int leftServoPos = 0, rightServoPos = 0, headServoPos = 45;


/*** Bluetooth ***/

/*** Bluetooth delacration and pins ***/ 
SoftwareSerial Bluetooth(10, 11); // TX, RX

/*** To store values sent by bluetooth ***/
String message;


void setup() {


  pinMode (pwm1, OUTPUT);
  pinMode (dir1, OUTPUT);
  pinMode (pwm2, OUTPUT);
  pinMode (dir2, OUTPUT);
  pinMode (motorA1, INPUT_PULLUP); // Remove the need for an external resistor
  pinMode (motorB1, INPUT_PULLUP); // Remove the need for an external resistor
  pinMode (motorA2, INPUT_PULLUP); // Remove the need for an external resistor
  pinMode (motorB2, INPUT_PULLUP); // Remove the need for an external resistor

  /*** Attaching interupt to Encoder A of both motor ***/
  attachInterrupt(digitalPinToInterrupt(motorA1), encoderCount1, RISING);
  attachInterrupt(digitalPinToInterrupt(motorA2), encoderCount2, RISING);

  previousMilli = millis(); // Initialise previousMilli to a value

  /*** Attaching servos ***/
  leftServo.attach(leftArm);
  rightServo.attach(rightArm);
  headServo.attach(headArm);

  /*** Reset servos to their initial position ***/
  leftServo.write(0);
  rightServo.write(0);
  headServo.write(45);

  /*** Begin comunication ***/
  Serial.begin (9600);
  Bluetooth.begin (9600);


}

void loop() {

  /*** When there's data sent from bluetooth ***/
  while (Bluetooth.available() > 0) {
    char c; // Read the char one at a time
    c = Bluetooth.read();
    message += c; // Join all to a single string
    delay (1);
  }
  //Serial.println (message);


  /*** Forward button pressed ***/
  if (message[0] == '1') {
    forward();
    bool continueMove = true; // To stop immediately when the button is not pressed
    /*** check if this is extra ***/
    while (continueMove) // while true
    {
      if (Bluetooth.available() > 0) {
        delay(10);
        continueMove = false;
      }
      adjustment();
      forward();
    }

    Bluetooth.flush();

  }


  else if (message[1] == '1') {
    backward();
    Bluetooth.flush();
    bool continueMove = true;
    while (continueMove)
    {
      if (Bluetooth.available() > 0) {
        delay(10);
        stopMotor();
        continueMove = false;
      }
      adjustment();
      backward();
    }
  }
  else if (message[2] == '1') {
    right();
    Bluetooth.flush();
    bool continueMove = true;
    while (continueMove)
    {
      if (Bluetooth.available() > 0) {
        delay(10);
        stopMotor();
        continueMove = false;
      }
      adjustment();
      right();
    }
  }
  else if (message[3] == '1') {
    left();
    Bluetooth.flush();
    bool continueMove = true;
    while (continueMove)
    {
      if (Bluetooth.available() > 0) {
        delay(10);
        stopMotor();
        continueMove = false;
      }
      adjustment();
      left();
    }
  }

  else {
    stopMotor();
    Bluetooth.flush();

  }

  if (message[4] == '0') {
    if (message[5] == '0') {
      left_arm_up();
    }

    else if (message[5] == '2') {

      bool pressed = true;
      while (pressed) {
        if (Bluetooth.read() > 0) {
          delay(10);
          pressed = false;
        }
        left_arm_up();
        delay(10);
      }
    }
    else if (message[5] == '1') {
      left_arm_down();
    }

    else if (message[5] == '3') {

      bool pressed = true;
      while (pressed) {
        if (Bluetooth.read() > 0) {
          delay(10);
          pressed = false;
        }
        left_arm_down();
        delay(10);
      }
    }


  }


  if (message[4] == '3') {
    if (message[5] == '0') {
      right_arm_up();
    }

    else if (message[5] == '2') {

      bool pressed = true;
      while (pressed) {
        if (Bluetooth.read() > 0) {
          delay(10);
          pressed = false;
        }
        right_arm_up();
        delay(10);
      }
    }
    else if (message[5] == '1') {
      right_arm_down();
    }

    else if (message[5] == '3') {

      bool pressed = true;
      while (pressed) {
        if (Bluetooth.read() > 0) {
          delay(10);
          pressed = false;
        }
        right_arm_down();
        delay(10);
      }
    }


  }

  if (message[4] == '2') {
    if (message[5] == '0') {
      both_arm_up();
    }

    else if (message[5] == '2') {

      bool pressed = true;
      while (pressed) {
        if (Bluetooth.read() > 0) {
          delay(10);
          pressed = false;
        }
        both_arm_up();
        delay(10);
      }
    }
    else if (message[5] == '1') {
      both_arm_down();
    }

    else if (message[5] == '3') {

      bool pressed = true;
      while (pressed) {
        if (Bluetooth.read() > 0) {
          delay(10);
          pressed = false;
        }
        both_arm_down();
        delay(10);
      }
    }


  }

  if (message[6] == '0') {
    head_up();
  }

  else if (message[6] == '2') {

    bool pressed = true;
    while (pressed) {
      if (Bluetooth.read() > 0) {
        delay(10);
        pressed = false;
      }
      head_up();
      delay(10);
    }
  }
  else if (message[6] == '1') {
    head_down();
  }

  else if (message[5] == '3') {

    bool pressed = true;
    while (pressed) {
      if (Bluetooth.read() > 0) {
        delay(10);
        pressed = false;
      }
      head_down();
      delay(10);
    }
  }
  message = "";
  delay(10);
}



void forward() {
  digitalWrite(dir1, HIGH);
  analogWrite(pwm1, motorSpeed1);
  digitalWrite(dir2, LOW);
  analogWrite(pwm2, motorSpeed2);
}

void backward() {
  digitalWrite(dir1, LOW);
  analogWrite(pwm1, motorSpeed1);
  digitalWrite(dir2, HIGH);
  analogWrite(pwm2, motorSpeed2);

}

void left() {
  digitalWrite(dir1, HIGH);
  analogWrite(pwm1, motorSpeed1);
  digitalWrite(dir2, HIGH);
  analogWrite(pwm2, motorSpeed2);

}

void right() {
  digitalWrite(dir1, LOW);
  analogWrite(pwm1, motorSpeed1);
  digitalWrite(dir2, LOW);
  analogWrite(pwm2, motorSpeed2);

}

void stopMotor() {
  digitalWrite(dir1, HIGH);
  analogWrite(pwm1, 0);
  digitalWrite(dir2, HIGH);
  analogWrite(pwm2, 0);
}

/***
 * Encoder A leads Encoder B = Clockwise
 * Encoder A lags Encoder B = Anti Clockwise
 */
// counts for encoders
void encoderCount1() {

  if (digitalRead(motorA1) == 1) {

    if (digitalRead(motorB1) == 1) {
      steps++;
    }
    else {
      steps --;
    }
  }

}
void encoderCount2() {

  if (digitalRead(motorA2) == 1) {

    if (digitalRead(motorB2) == 1) {
      steps2++;
    }
    else if (digitalRead(motorB2) == 0) {
      steps2 --;
    }
  }

}


/*** Adjust the rpm of the motors ***/
void adjust() {

  int difference = abs(abs(rpm1) - abs(rpm2)); //abs is used so it doesnt matter the rotation direction
  int tune = (difference / k) * 2.5;

  /*** if motor1 turns too fast, turn the speed down ***/
  if (rpm1 > rpm2) {

    if (motorSpeed1 > motorMinSpeed) {
      motorSpeed1 = motorSpeed1 - tune;
    }
    /*** if it goes below min speed, reset it to max and retune ***/
    if (motorSpeed1 <= motorMinSpeed) {
      motorSpeed1 = motorMaxSpeed;
      motorSpeed2 = motorMaxSpeed;
    }

  }
  /*** if motor1 turns faster, turn down the speed ***/
  else if (rpm2 > rpm1) {

    if (motorSpeed2 > motorMinSpeed) {
      motorSpeed2 = motorSpeed2 - tune;
    }
    /*** if it goes below min speed, reset it to max and retune ***/
    if (motorSpeed2 <= motorMinSpeed) {
      motorSpeed2 = motorMaxSpeed;
      motorSpeed1 = motorMaxSpeed;
    }

  }
}

// Determine when to call adjust function to adjust the RPM of the motor
void adjustment() {
  currentMilli = millis(); // Take the current time
  delay(10);
  if ((currentMilli - previousMilli) > interval) { // If 1 sec has elapsed since the last time it change
    previousMilli = currentMilli; // Reset the previousmilli to the current value and get ready to adjust the speed when the next sec pass
    
    if (previousSteps != steps) { // if the motors rotate and not stationary

      rpm1 = float(abs(steps - laststep) * 60 / 150); // (current steps - lasst 1 sec steps) * 1 min / rpm of the motor
      rpm2 = float(abs(steps2 - laststep2) * 60 / 150);

//      Serial.print("Motor1 RPM: ");
//      Serial.print(rpm1);
//      Serial.print(", ");
//      Serial.print("Motor2 RPM: ");
//      Serial.print(rpm2);
//      Serial.print(", ");
//      Serial.print("Difference RPM: ");
//      Serial.print(abs(rpm1 - rpm2));
//      Serial.print(", ");
//      Serial.print("Motor1 Speed: ");
//      Serial.print(motorSpeed1);
//      Serial.print(", ");
//      Serial.print("Motor2 Speed: ");
//      Serial.println(motorSpeed2);

      adjust(); // Tune the difference

      laststep = steps; // Reset the last 1 sec value to the current 1 sec value and to be used in the next adjustment
      laststep2 = steps2;
    }
  }

  else if (previousSteps == steps) { // If the motors are stationary
    previousMilli = currentMilli;
  }

  previousSteps = steps; // Keep updated the last state whenever the function is call regardless of motors turns
}



/*** For servos ***/
void left_arm_up () {
  // Movement from 0 - 90
  if ((leftServoPos >= 0) && (leftServoPos < 90)) {
    leftServoPos ++;
    leftServo.write (leftServoPos);
    delay(1);
  }

  else if (leftServoPos >= 90) {
    Bluetooth.println("Limit Reached!   "); // When it reach the MAX Allowed
    delay(1);
  }
}

void left_arm_down () {
  // Movement from 0 - 90
  if ((leftServoPos <= 90 ) && (leftServoPos > 0)) {
    leftServoPos --;
    leftServo.write (leftServoPos);
    delay(1);
  }

  else if (leftServoPos <= 0) {
    Bluetooth.println("Limit Reached!   "); // When it reach the min Allowed
    delay(1);
  }
}

void right_arm_up () {
  // Movement from 0 - 90
  if ((rightServoPos >= 0) && (rightServoPos < 90)) {
    rightServoPos ++;
    rightServo.write (rightServoPos);
    delay(1);
  }

  else if (rightServoPos >= 90) {
    Bluetooth.println("Limit Reached!   "); // When it reach the MAX Allowed
    delay(1);
  }
}

void right_arm_down () {
  // Movement from 0 - 90
  if ((rightServoPos <= 90 ) && (rightServoPos > 0)) {
    rightServoPos --;
    //Serial.println (rightServoPos);
    rightServo.write (rightServoPos);
    delay(1);
  }

  else if (rightServoPos <= 0) {
    Bluetooth.println("Limit Reached!   "); // When it reach the MIN Allowed
    delay(1);
  }
}

void both_arm_up () {
  // Movement from 0 - 90
  if ((rightServoPos >= 0) && (rightServoPos < 90) && (leftServoPos >= 0) && (leftServoPos < 90)) {
    rightServoPos ++;
    leftServoPos ++;
    rightServo.write (rightServoPos);
    leftServo.write (leftServoPos);
    delay(1);
  }

  else if ((rightServoPos >= 90) || (leftServoPos >= 90) ) {
    Bluetooth.println("Limit Reached!   "); // When it reach the MAX Allowed
    delay(1);
  }
}

void both_arm_down () {
  // Movement from 0 - 90
  if ((rightServoPos <= 90 ) && (rightServoPos > 0) && (leftServoPos <= 90 ) && (leftServoPos > 0)) {
    rightServoPos --;
    leftServoPos --;
    rightServo.write (rightServoPos);
    leftServo.write (leftServoPos);
    delay(1);
  }

  else if ((rightServoPos <= 0) || (leftServoPos <= 0)) {
    Bluetooth.println("Limit Reached!   "); // When it reach the MIN Allowed
    delay(1);
  }
}

void head_up() {
  // Movement from 0 - 90
  if ((headServoPos >= 45) && (headServoPos < 90)) {
    headServoPos ++;
    headServo.write(headServoPos);
    delay(1);
  }

  else if (headServoPos >= 90) {
    Bluetooth.println("Limit Reached!   "); // When it reach the MAX Allowed
  }
}

void head_down() {
  // Movement from 0 - 90
  if ((headServoPos > 0) && (headServoPos <= 45)) {
    headServoPos --;
    headServo.write(headServoPos);
    delay(1);
  }

  else if (headServoPos <= 0) {
    Bluetooth.println("Limit Reached!   "); // When it reach the MIN Allowed 
  }
}
