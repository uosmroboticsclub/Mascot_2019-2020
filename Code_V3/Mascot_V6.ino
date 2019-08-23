/***
   Variable message sequence = (ForwardButton, BackwardButton, RightButton, LeftButton, ArmSelectionList, ArmMovement, HeadMovement)
   FowardButton = Pressed(1), not pressed (0);
   BackwardButton = Pressed(1), not pressed (0);
   RightButton = Pressed(1), not pressed (0);
   LeftButton = Pressed(1), not pressed (0);
   ArmSelectionList = Left(0), Right(3), Both(2), not selected(4); 1 is not used to prevent from missending LeftButton value
   ArmMovement = UpClick(0), DownClick(1), UpHold(2), DownHold(3), not pressed (4);
   HeadMovement = UpClick(0), DownClick(1), UpHold(2), DownHold(3), not pressed (4);

*/

#include <AltSoftSerial.h>
#include <ServoTimer2.h>

#define motorMaxSpeed 255 // The maximum speed the motor will run and the max it can power up
#define motorMinSpeed 230 // The minimun speed the motor will run before reseting it back to max and retune it

/*** Motor ***/

/*** Encoder pins ***/
#define motorA1  2 // Pin for Encoder A of Motor 1
#define motorB1  13 // Pin for Encoder B of Motor 1
#define motorA2  3 // Pin for Encoder A of Motor 2
#define motorB2  12 //Pin for Encoder B of Motor 2

/*** Motor Shield Pins ***/
#define pwm1  5 // Take motorspeed1 as input
#define dir1  4 // Determine the direction of motor1 rotation, HIGH = clockwise
#define pwm2  6 // Take motorspeed2 as input
#define dir2  7 // Determine the direction of motor2 rotation, HIGH = clockwise

/*** Variable to store the speed of motors ***/
int motorSpeed1 = 255; // Speed of Motor1, from 0 - 255
int motorSpeed2 = 255; // Speed of Motor1, from 0 - 255

/*** Variable for adjusting the motor speed to rotate in the same speed ***/
int previousState, currentState; // Used to determine if the motor is rotating
int previousMilli, currentMilli = 0; // Serve as counting timer for rpm

#define interval  1000 // Used to determine if 1 sec has passed
#define rmp  150 // RPM of the motor used

#define k  2 // Differential propotional value

int steps = 0; // Steps for motor1
int steps2 = 0; // Steps for motor2

int previousSteps = 0; // Store immediately the value of current step for one of the motor
int laststep, laststep2; // Store the last steps for each motor after calculating RPM

float rpm1, rpm2; // RPM for both motors


/*** Servo ***/

/*** Servo pins ***/
#define leftArm  10 // Left Arm servo pin
#define rightArm  11 // Right Arm servo pin
//#define headArm =  // Head servo pin

/*** Servo names ***/
ServoTimer2 leftServo, rightServo, headServo;

/*** Servo variable and their initial pos ***/
int leftServoPos = 1800, rightServoPos = 1250, headServoPos = 45;


/*** Bluetooth ***/

/*** Bluetooth delacration and pins ***/
AltSoftSerial Bluetooth; // TX, RX

/*** To store values sent by bluetooth ***/
String message;

int counter;

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define LOGO_HEIGHT   64
#define LOGO_WIDTH    128

int blinking = 0;
/*
  const unsigned char myBitmap [] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xff, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xff, 0xff, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xff, 0xff, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xff, 0xff, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xff, 0xff, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xff, 0xff, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xff, 0xff, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xff, 0xff, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xff, 0xff, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xff, 0xff, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xff, 0xff, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xff, 0xff, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xff, 0xff, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xff, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xff, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xff, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xff, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xff, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xff, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xff, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xff, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xff, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xff, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xff, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xff, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xff, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xff, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xff, 0xff, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xff, 0xff, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xff, 0xff, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xff, 0xff, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xff, 0xff, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xff, 0xff, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xff, 0xff, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xff, 0xff, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xff, 0xff, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xff, 0xff, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xff, 0xff, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xff, 0xff, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xff, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00

  };
*/

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
  //headServo.attach(headArm);

  /*** Reset servos to their initial position ***/
  leftServo.write(1800);
  rightServo.write(1250);
  //headServo.write(45);

  /*** Begin comunication ***/
  Serial.begin (9600);
  Bluetooth.begin (9600);

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  //  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
  //    Serial.println(F("SSD1306 allocation failed"));
  //    for (;;); // Don't proceed, loop forever
  //  }
  for (int a = 0; a < 3; a ++) {
    setDisplay(1);
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
      Serial.println(F("SSD1306 allocation failed"));
      for (;;); // Don't proceed, loop forever
    }
    display.clearDisplay();
    drawElipse(64, 32, 16, 32);

    display.display();

    setDisplay(2);
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
      Serial.println(F("SSD1306 allocation failed"));
      for (;;); // Don't proceed, loop forever
    }
    display.clearDisplay();
    drawElipse(64, 32, 16, 32);
    display.display();

    setDisplay(3);
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
      Serial.println(F("SSD1306 allocation failed"));
      for (;;); // Don't proceed, loop forever
    }
    display.clearDisplay();
    fillcircle(2);
    display.fillRect(0, 0, 128, 20, BLACK);
    display.display();
  }


  counter = 0;


}

void loop() {


  /*** When there's data sent from bluetooth ***/
  while (Bluetooth.available() > 0) {
    char c; // Read the char one at a time
    c = Bluetooth.read();
    message += c; // Join all to a single string
    delay (1);
  }
  Serial.println (message);

  if (counter == 30) {
    blinking = 1;
    counter++;
  }
  else if (counter == 32) {
    blinking = 2;
    counter++;
  }
  else if (counter == 34) {
    counter++;
  }
  else if (counter == 36) {
    blinking = 4;
    counter++;
  }

  else if (counter == 38) {
    blinking = 0;
    counter = 0;
  }
  else {
    counter++;
  }
  Serial.println(counter);
  Serial.println(blinking);


  /*** Forward button pressed ***/
  if (message[0] == '1') {
    drawElipse(64, 32, 16, 32);
    drive(HIGH, motorSpeed1, LOW, motorSpeed2);
    bool continueMove = true; // To stop immediately when the button is not pressed

    while (continueMove) // while true
    {
      if (Bluetooth.available() > 0) {
        //delay(10);
        drive(HIGH, 0, HIGH, 0);
        continueMove = false;
      }
      adjustment();
      drive(HIGH, motorSpeed1, LOW, motorSpeed2);
    }

    Bluetooth.flush();

  }


  else if (message[1] == '1') {
    int backwardSpeed1 = (motorSpeed1 - 40);
    int backwardSpeed2 = (motorSpeed2 - 40);
    
    drawElipse(64, 32, 16, 32);
    drive(LOW, backwardSpeed1, HIGH, backwardSpeed2);
    Bluetooth.flush();
    bool continueMove = true;
    while (continueMove)
    {
      if (Bluetooth.available() > 0) {
        //delay(10);
        drive(HIGH, 0, HIGH, 0);

        continueMove = false;
      }
      adjustment();
      drive(LOW, backwardSpeed1, HIGH, backwardSpeed2);
    }
  }
  else if (message[2] == '1') {
    drawElipse(96, 32, 16, 32);
    drive(LOW, motorSpeed1, LOW, motorSpeed2);
    Bluetooth.flush();
    bool continueMove = true;
    while (continueMove)
    {
      if (Bluetooth.available() > 0) {
        //delay(10);
        drive(HIGH, 0, HIGH, 0);
        continueMove = false;
      }
      adjustment();
      drive(LOW, motorSpeed1, LOW, motorSpeed2);
    }
  }
  else if (message[3] == '1') {
    drawElipse(32, 32, 16, 32);
    drive(HIGH, motorSpeed1, HIGH, motorSpeed2);
    Bluetooth.flush();
    bool continueMove = true;
    while (continueMove)
    {
      if (Bluetooth.available() > 0) {
        //delay(10);
        drive(HIGH, 0, HIGH, 0);
        continueMove = false;
      }
      adjustment();
      drive(HIGH, motorSpeed1, HIGH, motorSpeed2);
    }
  }

  else {
    drive(HIGH, 0, HIGH, 0);
    drawElipse(64, 32, 16, 32);
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
          //delay(10);
          pressed = false;
        }
        left_arm_up();
        //delay(10);
      }
    }
    else if (message[5] == '1') {
      left_arm_down();
    }

    else if (message[5] == '3') {

      bool pressed = true;
      while (pressed) {
        if (Bluetooth.read() > 0) {
          //delay(10);
          pressed = false;
        }
        left_arm_down();
        //delay(10);
      }
    }
    Bluetooth.flush();


  }


  else if (message[4] == '3') {
    if (message[5] == '0') {
      right_arm_up();
    }

    else if (message[5] == '2') {

      bool pressed = true;
      while (pressed) {
        if (Bluetooth.read() > 0) {
          //delay(10);
          pressed = false;
        }
        right_arm_up();
        //delay(10);
      }
    }
    else if (message[5] == '1') {
      right_arm_down();
    }

    else if (message[5] == '3') {

      bool pressed = true;
      while (pressed) {
        if (Bluetooth.read() > 0) {
          //delay(10);
          pressed = false;
        }
        right_arm_down();
        //delay(10);
      }
    }
    Bluetooth.flush();


  }

  else if (message[4] == '2') {
    if (message[5] == '0') {
      both_arm_up();
    }

    else if (message[5] == '2') {

      bool pressed = true;
      while (pressed) {
        if (Bluetooth.read() > 0) {
          //delay(10);
          pressed = false;
        }
        both_arm_up();
        // delay(10);
      }
    }
    else if (message[5] == '1') {
      both_arm_down();
    }

    else if (message[5] == '3') {

      bool pressed = true;
      while (pressed) {
        if (Bluetooth.read() > 0) {
          //delay(10);
          pressed = false;
        }
        both_arm_down();
        //delay(10);
      }
    }
    Bluetooth.flush();


  }

  //  if (message[6] == '0') {
  //    head_up();
  //  }
  //
  //  else if (message[6] == '2') {
  //
  //    bool pressed = true;
  //    while (pressed) {
  //      if (Bluetooth.read() > 0) {
  //        //delay(10);
  //        pressed = false;
  //      }
  //      head_up();
  //      //delay(10);
  //    }
  //  }
  //  else if (message[6] == '1') {
  //    head_down();
  //  }
  //
  //  else if (message[6] == '3') {
  //
  //    bool pressed = true;
  //    while (pressed) {
  //      if (Bluetooth.read() > 0) {
  //        //delay(10);
  //        pressed = false;
  //      }
  //      head_down();
  //      //delay(10);
  //    }
  //    Bluetooth.flush();
  //  }
  //  int lastrightServoPos;
  //  if (lastrightServoPos != rightServoPos) {
  //    rightServo.write (rightServoPos);
  //    leftServo.write (leftServoPos);
  //  }
  //  lastrightServoPos = rightServoPos;
  message = "";
  //delay(10);
}



void drive(int state1, int speed1, int state2, int speed2) {
  digitalWrite(dir1, state1);
  analogWrite(pwm1, speed1);
  digitalWrite(dir2, state2);
  analogWrite(pwm2, speed2);
}

/***
   Encoder A leads Encoder B = Clockwise
   Encoder A lags Encoder B = Anti Clockwise
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
void left_arm_down () {
  // Movement from 0 - 90
  if ((leftServoPos >= 1250) && (leftServoPos < 1800)) {
    leftServoPos += 25;
    leftServo.write (leftServoPos);
  }

  else if (leftServoPos >= 1800) {
    //Bluetooth.println("Limit Reached!   "); // When it reach the MAX Allowed
  }
}

void left_arm_up () {
  // Movement from 0 - 90
  if ((leftServoPos <= 1800 ) && (leftServoPos > 1250)) {
    leftServoPos -= 25;
    leftServo.write (leftServoPos);
  }

  else if (leftServoPos <= 1250) {
    //Bluetooth.println("Limit Reached!   "); // When it reach the min Allowed
  }
}

void right_arm_up () {
  // Movement from 0 - 90
  if ((rightServoPos >= 1250) && (rightServoPos < 1800)) {
    rightServoPos += 25;
    rightServo.write (rightServoPos);
  }

  else if (rightServoPos >= 1800) {
    //Bluetooth.println("Limit Reached!   "); // When it reach the MAX Allowed
  }
}

void right_arm_down () {
  // Movement from 0 - 90
  if ((rightServoPos <= 1800 ) && (rightServoPos > 1250)) {
    rightServoPos -= 25;
    //Serial.println (rightServoPos);
    rightServo.write (rightServoPos);
  }

  else if (rightServoPos <= 1250) {
    //Bluetooth.println("Limit Reached!   "); // When it reach the MIN Allowed
  }
}

void both_arm_up () {
  // Movement from 0 - 90
  if ((rightServoPos >= 1250) && (rightServoPos < 1800) && (leftServoPos <= 1800) && (leftServoPos > 1250)) {
    rightServoPos += 25;
    leftServoPos -= 25;
    rightServo.write (rightServoPos);
    leftServo.write (leftServoPos);
  }

  else if ((rightServoPos >= 1800) || (leftServoPos <= 1250) ) {
    //Bluetooth.println("Limit Reached!   "); // When it reach the MAX Allowed
  }
}

void both_arm_down () {
  // Movement from 0 - 90
  if ((rightServoPos <= 1800 ) && (rightServoPos > 1250) && (leftServoPos >= 1250 ) && (leftServoPos < 1800)) {
    rightServoPos -= 25;
    leftServoPos += 25;
    rightServo.write (rightServoPos);
    leftServo.write (leftServoPos);
  }

  else if ((rightServoPos <= 1250) || (leftServoPos >= 1800)) {
    //Bluetooth.println("Limit Reached!   "); // When it reach the MIN Allowed
  }
}

//void head_up() {
//  // Movement from 0 - 90
//  if ((headServoPos >= 0) && (headServoPos < 90)) {
//    headServoPos += 5;
//    headServo.write(headServoPos);
//  }
//
//  else if (headServoPos >= 90) {
//    //Bluetooth.println("Limit Reached!   "); // When it reach the MAX Allowed
//  }
//}
//
//void head_down() {
//  // Movement from 0 - 90
//  if ((headServoPos > 0) && (headServoPos <= 90)) {
//    headServoPos -= 5;
//    headServo.write(headServoPos);
//  }
//
//  else if (headServoPos <= 0) {
//    //Bluetooth.println("Limit Reached!   "); // When it reach the MIN Allowed
//  }
//}


void Ellipse (int x, int y, uint8_t width, uint8_t height, uint8_t on)
{
  long x1 = -width, y1 = 0; // II quadrant from bottom left to top right
  long e2 = height, dx = (1 + 2 * x1) * e2 * e2; // error increment
  long dy = x1 * x1, err = dx + dy; // error of 1 step

  do {
    display.drawPixel (x - x1, y + y1, on); // I Quadrant
    display.drawPixel (x + x1, y + y1, on); // II Quadrant
    display.drawPixel (x + x1, y - y1, on); // III Quadrant
    display.drawPixel (x - x1, y - y1, on); // IV Quadrant
    e2 = 2 * err;

    if (e2 >= dx) {
      x1++;
      err += dx += 2 * (long) height * height;
    } // x1 step

    if (e2 <= dy) {
      y1++;
      err += dy += 2 * (long) width * width;
    } // y1 step
  } while (x1 <= 0);

  while (y1++ < height) { // too early stop for flat ellipses with width=1
    display.drawPixel (x, y + y1, on); // -> finish tip of ellipse
    display.drawPixel (x, y - y1, on);
  }
}

void drawElipse(int x, int y, int height, int width) {
  //for (int a = 0; a < 2; a++) {
  display.clearDisplay();
  //int b = a + 1;
  int j = max(height, width);
  if (height > width) {

    for (int i = 0; i < j; i++) {
      Ellipse(x, y, i, width, WHITE);
    }
  }
  else {
    if (blinking == 0) {
      width = width;
    }
    else if (blinking == 1) {
      width = 24;
    }
    else if (blinking == 2) {
      width = 16;
    }
    else if (blinking == 3) {
      width = 8;
    }
    else if (blinking == 4) {
      width = 2;
    }
    for (int i = 0; i < j; i++) {
      Ellipse(x, y, height, i, WHITE);
    }

  }
  //  setDisplay(2);
  //  display.display();
  //}
  display.clearDisplay();
  //int b = a + 1;
  if (height > width) {

    for (int i = 0; i < j; i++) {
      Ellipse(x, y, i, width, WHITE);
    }
  }
  else {
    if (blinking == 0) {
      width = width;
    }
    else if (blinking == 1) {
      width = 24;
    }
    else if (blinking == 2) {
      width = 16;
    }
    else if (blinking == 3) {
      width = 8;
    }
    else if (blinking == 4) {
      width = 2;
    }
    for (int i = 0; i < j; i++) {
      Ellipse(x, y, height, i, WHITE);
    }

  }
  setDisplay(1);
  display.display();
  setDisplay(2);
  display.display();
}
void setDisplay(uint8_t i) {
  if (i > 7) return;                // If more than 7 out of range, do nothing

  Wire.beginTransmission(0x70);     // All data to go to address 0x70 (The Multiplexer address)
  Wire.write(1 << i);               // write value as a single bit in an 8bit byte, MSB being I2C no. 7 LSB being I2C no. 0
  Wire.endTransmission();           // Signal sending stopped
}

void fillcircle(int center) {
  display.fillCircle(display.width() / center, display.height() / 2, max(display.width(), display.height()) / 4, INVERSE);
}
