#include <ESP32Servo.h>
#include "BluetoothSerial.h"
// #include <SoftwareSerial.h>

#define MOTOR_L_1 12
#define MOTOR_L_2 14
#define MOTOR_R_1 27
#define MOTOR_R_2 26

#define arm1_servo 16
#define arm2_servo 17
#define grip_servo 5
#define center_servo 18
#define flag_servo 4


#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

Servo arm1;
Servo arm2;
Servo grip;
Servo center;
Servo flag;

float arm1_angle = 90;
float arm2_angle = 90;

bool is_grip = false;
float grip_angle = 95;
float ungrip_angle = 160;

float center_angle = 90;
float center_d = 3;

bool is_flag = false;
float flag_angle = 180;
float unflag_angle = 90;


bool isArm1In = false;
bool isArm1De = false;

bool isArm2In = false;
bool isArm2De = false;

bool isArmControl = false;

const int freq = 3000;
const int ENAChanel = 15;
const int ENBChanel = 16;
const int resolution = 8;

void setup() {
  arm1.attach(arm1_servo);
  arm2.attach(arm2_servo);
  grip.attach(grip_servo);
  center.attach(center_servo);
  flag.attach(flag_servo);


  pinMode(MOTOR_L_1, OUTPUT);
  pinMode(MOTOR_L_2, OUTPUT);
  pinMode(MOTOR_R_1, OUTPUT);
  pinMode(MOTOR_R_2, OUTPUT);

  digitalWrite(MOTOR_L_1, LOW);
  digitalWrite(MOTOR_L_2, LOW);
  digitalWrite(MOTOR_R_1, LOW);
  digitalWrite(MOTOR_R_2, LOW);

  // ledcSetup(ENAChanel, freq, resolution);
  // ledcSetup(ENBChanel, freq, resolution);
  // ledcAttachPin(MOTOR_L_2, ENAChanel);
  // ledcAttachPin(MOTOR_R_1, ENBChanel);


  Serial.begin(9600);
  SerialBT.begin("HDUNGCar");

  arm1.write(90);
  arm2.write(90);
  grip.write(ungrip_angle);
  center.write(center_angle);
}
// R-HIGH
int maxspeed = 255;
char code;
String message = "";

void loop() {

  if (SerialBT.available()) {
    message = SerialBT.readStringUntil('\n');  // read from serial or bluetooth
    Serial.println(message);
  }
}
// ###################################################### CONTROL MOTOR CAR ==============================================
void stop() {
  setmotor(0, 0);
}

void forward() {
  setmotor(maxspeed, maxspeed);
}

void forwardLeft() {
  setmotor(0, maxspeed);
}

void forwardRight() {
  setmotor(maxspeed, 0);
}

void backward() {
  setmotor(-maxspeed, -maxspeed);
}

void backwardLeft() {
  setmotor(0, -maxspeed);
}

void backwardRight() {
  setmotor(-maxspeed, 0);
}

void left() {
  setmotor(-maxspeed, maxspeed);
}
void right() {
  setmotor(maxspeed, -maxspeed);
}

void setmotor(int speedA, int speedB) {
  speedA = -speedA;
  speedB = -speedB;

  if (speedA > 0) {
    digitalWrite(MOTOR_L_1, HIGH);
    digitalWrite(MOTOR_L_2, LOW);
  } else if (speedA < 0) {
    digitalWrite(MOTOR_L_1, LOW);
    digitalWrite(MOTOR_L_2, HIGH);
  } else {
    digitalWrite(MOTOR_L_1, LOW);
    digitalWrite(MOTOR_L_2, LOW);
  }

  if (speedB > 0) {
    digitalWrite(MOTOR_R_1, HIGH);
    digitalWrite(MOTOR_R_2, LOW);
  } else if (speedB < 0) {
    digitalWrite(MOTOR_R_1, LOW);
    digitalWrite(MOTOR_R_2, HIGH);
  } else {
    digitalWrite(MOTOR_R_1, LOW);
    digitalWrite(MOTOR_R_2, LOW);
  }
}
