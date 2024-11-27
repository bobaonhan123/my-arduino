#include<math.h>
int error;
int isRunning;

const int motorA1 = 3;//bánh phải tiến
const int motorA2 = 4;
const int motorB1 = 6;//bánh trái tiến
const int motorB2 = 5;

const uint8_t SENSORS_PIN[] = { 13, 12, 11, 10, 9 };

int sensorValue = 0;
int outputValue = 0;
int IN_line, In_line_last, mode;
byte ss1, ss2, ss3, ss4, ss5, x1;
String tt;
int td3 = 120;
int td2 = 190;
int td1 = 180;
long duration;  //
int distance;   // biến khoảng cách

void setup() {
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);
  for (auto sensorPin : SENSORS_PIN) {
    pinMode(sensorPin, INPUT);
  }
  
  Serial.begin(9600);
  
}

void forward() {
  digitalWrite(motorA1, HIGH);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, HIGH);
  digitalWrite(motorB2, LOW);
}
void left() {
  digitalWrite(motorA1, HIGH);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, LOW);
}
void right() {
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, HIGH);
  digitalWrite(motorB2, LOW);
}


void loop() {
  read_sensors();
  Serial.println(error);
  if(abs(error) <=2 ) {
    forward();
  }
  else if( error<0) {
    right();
  }
  else {
    left();
  }
}

void read_sensors() {
  String sensorArray = "";
  int sumSensor = 0;
  for (auto sensorPin : SENSORS_PIN) {
    sensorArray += (char)(digitalRead(sensorPin) + 48);
    sumSensor += digitalRead(sensorPin);
  }
  Serial.println(sensorArray);

  if (sensorArray == "00001") error = 6;
  else if (sensorArray == "00011") error = 5;
  else if (sensorArray == "00111") error = 4;
  else if (sensorArray == "00010") error = 2;
  else if (sensorArray == "00110") error = 1;
  else if (sensorArray == "00100") error = 0;
  else if (sensorArray == "01110") error = 0;
  else if (sensorArray == "01100") error = 1;
  else if (sensorArray == "01000") error = -2;
  else if (sensorArray == "11100") error = -4;
  else if (sensorArray == "11000") error = -5;
  else if (sensorArray == "10000") error = -6;
  else if (sensorArray == "00000") {
    if (error > 0) error = 7;
    else error = -7;
  } else if (sensorArray == "11111" || sensorArray == "10101") {
    isRunning = 0;
  }
}