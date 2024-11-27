#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ArduinoJson.h>

const char *SSID = "309";
const char *PWD = "251914423700";

bool onoff = 0;
//26:bánh trái tiến
//27:bánh trái lùi
//12:bánh phải tiến
//13:bánh phải lùi
#define left_forward 26
#define left_backward 27
#define right_forward 12
#define right_backward 13

//Hồng ngoại trái -> phải: 4 16 17 5 18
const uint8_t SENSORS_PIN[] = { 4, 16, 17, 5, 18 };

float Kp = 31;
float Ki = 0.00005;
float Kd = 0;
int P;
float I=0;
int D;
int PIDValue = 0;
float error = 0;
int lastError = 0;
int isRunning = 1;
bool isNoLine = false;
const uint8_t initial_speed = 255;
const uint8_t maxspeeda = 255;
const uint8_t maxspeedb = 255;
const uint8_t minspeeda = 10;
const uint8_t minspeedb = 10;
int left_speed = maxspeeda;
int right_speed = maxspeeda;

const int freq = 5000;
const int ENAChanel = 5;
const int ENBChanel = 6;
const int resolution = 8;

WebServer server(80);
StaticJsonDocument<250> jsonDocument;

void handlePost() {
  if (server.hasArg("plain") == false) {
  }
  String body = server.arg("plain");
  deserializeJson(jsonDocument, body);
  Kp = jsonDocument["p"];
  Ki = jsonDocument["i"];
  Kd = jsonDocument["d"];
  isRunning=jsonDocument["isRunning"];
  server.send(200, "application/json", "{}");=
}

void stop() {
  digitalWrite(left_backward, LOW);
  digitalWrite(right_backward, LOW);
  digitalWrite(left_forward, LOW);
  digitalWrite(right_forward, LOW);
    ledcWrite(ENAChanel, 0);
  ledcWrite(ENBChanel, 0);
}

void run(int left_spd, int right_spd) {
  digitalWrite(left_forward, LOW);
  digitalWrite(right_forward, LOW);
  ledcWrite(ENAChanel, left_spd);
  ledcWrite(ENBChanel, right_spd);
  digitalWrite(left_backward, LOW);
  digitalWrite(right_backward, LOW);
}

int Constrain(int val, int mi, int ma) {
  if (val < mi) val = mi;
  if (val > ma) val = ma;
  return val;
}

void motor_control() {
  // Calculating the effective motor speed:
  int left_motor_speed = initial_speed + PIDValue;
  int right_motor_speed = initial_speed - PIDValue;
  left_motor_speed = Constrain(left_motor_speed, minspeeda, maxspeeda);
  right_motor_speed = Constrain(right_motor_speed, minspeeda, maxspeeda);
  // Serial.println("left:");
  // Serial.println(left_motor_speed);
  // Serial.println("right:");
  // Serial.println(right_motor_speed);
  run(left_motor_speed, right_motor_speed);
}


void caculate_pid() {
  P = error;
  if (abs(error) < 8)
  I = I + Ki * error;
  D = error - lastError;
  lastError=error;
  PIDValue = Kp * P + I + Kd * D;
  // Serial.println("I:");
  // Serial.println(I);
  // Serial.println(PIDValue);
}


void read_sensors() {
  String sensorArray = "";
  int sumSensor = 0;
  for (auto sensorPin : SENSORS_PIN) {
    sensorArray += (char)(digitalRead(sensorPin) + 48);
    sumSensor += digitalRead(sensorPin);
  }
  if (sumSensor > 0) {
    isNoLine = false;
  } else {
    isNoLine = true;
  }
  // Serial.println(sensorArray);

  if (sensorArray == "00001") error = 6;
  else if (sensorArray == "00011") error = 5;
  else if (sensorArray == "00111") error = 4;
  // else if (sensorArray == "00011") error = 3;
  else if (sensorArray == "00010") error = 2;
  else if (sensorArray == "00110") error = 1;
  else if (sensorArray == "00100") error = 0;
  else if (sensorArray == "01110") error = 0;
  else if (sensorArray == "01100") error = 1;
  else if (sensorArray == "01000") error = -2;
  // else if (sensorArray == "11000") error = -3;
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

void setup() {
  Serial.begin(115200);
  for (auto sensorPin : SENSORS_PIN) {
    pinMode(sensorPin, INPUT);
  }
  pinMode(left_forward, OUTPUT);
  pinMode(left_backward, OUTPUT);
  pinMode(right_forward, OUTPUT);
  pinMode(right_backward, OUTPUT);
  ledcSetup(ENAChanel, freq, resolution);
  ledcSetup(ENBChanel, freq, resolution);
  ledcAttachPin(left_forward, ENAChanel);
  ledcAttachPin(right_forward, ENBChanel);
  digitalWrite(left_backward, LOW);
  digitalWrite(right_backward, LOW);
  WiFi.begin(SSID, PWD);

  while (WiFi.status() != WL_CONNECTED) {
    Serial.println(".");
    delay(500);
  }
  Serial.print("Connected! IP Address: ");
  Serial.println(WiFi.localIP());
  server.on("/pid",HTTP_POST, handlePost);
  server.begin();
}

void loop() {
  server.handleClient();
    read_sensors();
    if (isRunning) {
      caculate_pid();
      motor_control();
    } else {
      stop();
    }
    Serial.println(Kp);
}