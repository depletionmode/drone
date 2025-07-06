#include <Modem.h>
#include <StringHelpers.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiCommands.h>
#include <WiFiFileSystem.h>
#include <WiFiS3.h>
#include <WiFiServer.h>
#include <WiFiSSLClient.h>
#include <WiFiTypes.h>
#include <WiFiUdp.h>

#include <Adafruit_BNO08x.h>
#include <sh2_err.h>
#include <sh2_hal.h>
#include <sh2_SensorValue.h>
#include <sh2_util.h>
#include <sh2.h>
#include <shtp.h>

#include "Servo.h"
#include "Arduino_LED_Matrix.h"

#include <Adafruit_BNO08x.h>
#include <EEPROM.h>
#include <AutoPID.h>

#include "secrets.h"

#define SPI_PIN_CS  10
#define SPI_PIN_INT 9
#define SPI_PIN_RESET 8
//#define SPI_PIN_MOSI   11
//#define SPI_PIN_MISO   12
//#define SPI_PIN_SCK    13

#define DEBUG
//#define DEBUG_GYRO_LOOP

#define CALLIBRATE_ON_START

#define CONTROL_PIN_KILL          A0
#define CONTROL_PIN_THROTTLE      A1
#define CONTROL_PIN_PITCH         A2
#define CONTROL_PIN_ROLL          A3
#define CONTROL_PIN_YAW           A4
#define CONTROL_PIN_CALLIBRATION  A5

#define MOTOR_PIN_FR  3
#define MOTOR_PIN_FL  2
#define MOTOR_PIN_BR  4
#define MOTOR_PIN_BL  5

#define CONTROL_VALUE_MIN 940
#define CONTROL_VALUE_MAX 1880
#define CONTROL_VALUE_NORMALIZATION_FACTOR (1000.0/CONTROL_VALUE_MIN)
#define CONTROL_UPDATE_INTERVAL 5 // every Nth loop

#define PID_UPDATE_INTERVAL 2  //ms (should be lower than the gyro report interval to avoid PID lag)
#define PID_MIN -500
#define PID_MAX 500
#define THROTTLE_MIN 1000
#define THROTTLE_MAX 2000
#define DEGREES_RANGE_MAX 3

#define GYRO_REPORT_TYPE SH2_ARVR_STABILIZED_RV
#define GYRO_REPORT_INTERVAL 5000 //us

#define IP_ADDRESS (IPAddress(192,168,1,190))

int status = WL_IDLE_STATUS;
WiFiServer server(80);

Servo br;
Servo fl;
Servo bl;
Servo fr;

bool armed;

unsigned long lastMicros = 0;
unsigned long loopCount = 0;

int yaw; // ch 4
int pitch; // ch 2
int roll;  // ch 1
int throttle; // ch 3

double yaw_error;
double pitch_error;
double roll_error;

double yaw_out;
double pitch_out;
double roll_out;

double yaw_in;
double pitch_in;
double roll_in;

double yaw_sp;
double pitch_sp;
double roll_sp;

AutoPID pid_yaw(&yaw_in, &yaw_sp, &yaw_out, PID_MIN, PID_MAX, 0, 0, 0);
AutoPID pid_pitch(&pitch_in, &pitch_sp, &pitch_out, PID_MIN, PID_MAX, 30, 0.003, 0.03);
AutoPID pid_roll(&roll_in, &roll_sp, &roll_out, PID_MIN, PID_MAX, 0, 0, 0);

struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;

Adafruit_BNO08x  bno08x(SPI_PIN_RESET);
sh2_SensorValue_t sensorValue;

ArduinoLEDMatrix matrix;

void setReports(sh2_SensorId_t reportType, long report_interval) {
  if (!bno08x.enableReport(reportType, report_interval)) {
    Serial.println("GYRO: Could not enable stabilized remote vector");
  }
}

void setup() {
  armed = false;

  pinMode(CONTROL_PIN_KILL, INPUT_PULLUP);
  pinMode(CONTROL_PIN_CALLIBRATION, INPUT_PULLUP);
  pinMode(CONTROL_PIN_THROTTLE, INPUT_PULLUP);
  pinMode(CONTROL_PIN_PITCH, INPUT_PULLUP);
  pinMode(CONTROL_PIN_ROLL, INPUT_PULLUP);
  pinMode(CONTROL_PIN_YAW, INPUT_PULLUP);  
  
  br.attach(MOTOR_PIN_BR, 1000, 2000);
  fl.attach(MOTOR_PIN_FL, 1000, 2000);
  bl.attach(MOTOR_PIN_BL, 1000, 2000);
  fr.attach(MOTOR_PIN_FR, 1000, 2000);
  br.write(0);
  fl.write(0);
  bl.write(0);
  fr.write(0);

  Serial.begin(115200);

  matrix.begin();
  matrix.clear();

  Serial.println("Starting initialization of drone...");

  pid_yaw.setTimeStep(PID_UPDATE_INTERVAL);
  pid_pitch.setTimeStep(PID_UPDATE_INTERVAL);
  pid_roll.setTimeStep(PID_UPDATE_INTERVAL);

  if (!bno08x.begin_SPI(SPI_PIN_CS, SPI_PIN_INT)) {
    Serial.println("GYRO: Failed to find BNO08x chip");
    while (1) { delay(100); }
  }
  setReports(GYRO_REPORT_TYPE, GYRO_REPORT_INTERVAL);

#ifdef CALLIBRATE_ON_START
  callibrate();
#endif

    do {
      readGyro(&ypr, false);
    } while (ypr.yaw == 0 && ypr.pitch == 0 && ypr.roll == 0);

    yaw_error = 0;
    pitch_error = 0;
    roll_error = 0;

    EEPROM.get(0, yaw_error);
    EEPROM.get(8, pitch_error);
    EEPROM.get(16, roll_error);

    Serial.println("Callibration values:");
    Serial.print(yaw_error);
    Serial.print("\t");
    Serial.print(pitch_error);
    Serial.print("\t");
    Serial.print(roll_error);
    Serial.print("\n");

    Serial.println("Initial gyro values:");
    Serial.println("Yaw\tPitch\tRoll");
    Serial.print(ypr.yaw - yaw_error);
    Serial.print("\t");
    Serial.print(ypr.pitch - pitch_error);
    Serial.print("\t");
    Serial.print(ypr.roll - roll_error);
    Serial.print("\n");

  #ifdef DEBUG
    //startWifi();
  #endif

  Serial.println("Initialization complete.");
}

void loop() {
#ifdef DEBUG
  unsigned long currentMicros = micros();
  if (currentMicros - lastMicros >= 1000000) {
    Serial.print("Loop frequency: ");
    Serial.print(loopCount);
    Serial.println(" Hz");

    loopCount = 0;
    lastMicros = currentMicros;
  }
  //handleWebServer();
#endif

#ifdef DEBUG_GYRO_LOOP
  readGyro(&ypr, true);

  yaw_in = ypr.yaw;
  pitch_in = ypr.pitch;
  roll_in = ypr.roll;

  yaw_sp = 0;
  pitch_sp = 0;
  roll_sp = 0;

  runPids();

  return;
#endif

  int kill = getValue(CONTROL_PIN_KILL, false, THROTTLE_MIN, THROTTLE_MAX);

  if (kill < 1700) {
    armed = false;

    br.write(0);
    fl.write(0);
    bl.write(0);
    fr.write(0);
  }

  if (loopCount++ % CONTROL_UPDATE_INTERVAL == 0) {
    //
    // As reading the control pins is slow, we only do it every Nth iteration
    // to avoid lagging the control loop (and impacting the PID loop).
    //

    throttle = getValue(CONTROL_PIN_THROTTLE);
    pitch = getValue(CONTROL_PIN_PITCH, true, -DEGREES_RANGE_MAX, DEGREES_RANGE_MAX);
    yaw = getValue(CONTROL_PIN_YAW, true, -DEGREES_RANGE_MAX, DEGREES_RANGE_MAX);
    roll = getValue(CONTROL_PIN_ROLL, true, -DEGREES_RANGE_MAX, DEGREES_RANGE_MAX);

#ifdef DEBUG
    Serial.print("Control values: ");
    Serial.print(kill);
    Serial.print("\t");
    Serial.print(throttle);
    Serial.print("\t");
    Serial.print(pitch);
    Serial.print("\t");
    Serial.print(yaw);
    Serial.print("\t");
    Serial.print(roll);
    Serial.print("\n");
#endif
  }

  if (!armed && throttle < 1040 && kill > 1700){
    if (getValue(CONTROL_PIN_CALLIBRATION) > 1700) {
      callibrate();
    }

    matrix.loadFrame(LEDMATRIX_EMOJI_HAPPY);

    armed = true;
  }

  if (!armed) {
    br.write(0);
    fl.write(0);
    bl.write(0);
    fr.write(0);

    matrix.clear();

    pid_yaw.stop(); pid_yaw.reset();
    pid_pitch.stop(); pid_pitch.reset();
    pid_roll.stop(); pid_roll.reset();

    return;
  }

  readGyro(&ypr, true);

  yaw_in = ypr.yaw;
  pitch_in = ypr.pitch;
  roll_in = ypr.roll;

  yaw_sp = yaw;
  pitch_sp = pitch;
  roll_sp = roll;

  runPids();

  br.writeMicroseconds(constrain(throttle - pitch_out - roll_out - yaw_out, 0, THROTTLE_MAX));
  fl.writeMicroseconds(constrain(throttle + pitch_out + roll_out - yaw_out, 0, THROTTLE_MAX));
  bl.writeMicroseconds(constrain(throttle - pitch_out + roll_out + yaw_out, 0, THROTTLE_MAX));
  fr.writeMicroseconds(constrain(throttle + pitch_out - roll_out + yaw_out, 0, THROTTLE_MAX));
}

void runPids() {
  pid_yaw.run();
  pid_pitch.run();
  pid_roll.run();

#ifdef DEBUG
  Serial.print("PID: ");
  Serial.print(yaw_out);
  Serial.print("\t");
  Serial.print(pitch_out);
  Serial.print("\t");
  Serial.print(roll_out);
  Serial.print("\n");
#endif
}

int getValue(int pin) {
  return getValue(pin, false, THROTTLE_MIN, THROTTLE_MAX);
}

int getValue(int pin, bool doMap, int min, int max) {
  int value = pulseIn(pin, HIGH, 500000); // 500ms timeout
  //Serial.println(value);
  value *= CONTROL_VALUE_NORMALIZATION_FACTOR;
  if (doMap) {
    value = map(value, CONTROL_VALUE_MIN, CONTROL_VALUE_MAX, min, max);
  } else {
    value = constrain(value, min, max);
  }

  return value;
}

void callibrate() {
  //
  // Must make sure the drone is level before running the callibration.
  //

#define CALIBRATION_SAMPLES 250

  Serial.println("Callibrating...");

  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    readGyro(&ypr, false);

    yaw_error += ypr.yaw;
    pitch_error += ypr.pitch;
    roll_error += ypr.roll;

    delay(2); // with 250 samples, this will take 500ms
  }

  yaw_error /= (double)CALIBRATION_SAMPLES;
  pitch_error /= (double)CALIBRATION_SAMPLES;
  roll_error /= (double)CALIBRATION_SAMPLES;

  EEPROM.put(0, yaw_error);
  EEPROM.put(8, pitch_error);
  EEPROM.put(16, roll_error);

  Serial.print("Calibration values: ");
  Serial.print(yaw_error);
  Serial.print("\t");
  Serial.print(pitch_error);
  Serial.print("\t");
  Serial.print(roll_error);
  Serial.print("\n");
}

void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {

    float sqr = sq(qr);
    float sqi = sq(qi);
    float sqj = sq(qj);
    float sqk = sq(qk);

    // NOTE: Orientation of the sensor on the drone defines the roll/pitch axis.
    ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    ypr->roll = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    ypr->pitch = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

    if (degrees) {
      ypr->yaw *= RAD_TO_DEG;
      ypr->pitch *= RAD_TO_DEG;
      ypr->roll *= RAD_TO_DEG;
    }
}

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void readGyro(euler_t* ypr, bool adjustForError) {
  if (bno08x.wasReset()) {
    Serial.println("GYRO: Sensor was reset");
    setReports(GYRO_REPORT_TYPE, GYRO_REPORT_INTERVAL);
  }

  if (bno08x.getSensorEvent(&sensorValue)) {
    quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, ypr, true);

    if (adjustForError) {
      ypr->yaw = ypr->yaw - yaw_error;
      ypr->pitch = ypr->pitch - pitch_error;
      ypr->roll = ypr->roll - roll_error;
    }

  #ifdef DEBUG
    Serial.print("GYRO: ");
    Serial.print(ypr->yaw);
    Serial.print("\t");
    Serial.print(ypr->pitch);
    Serial.print("\t");
    Serial.print(ypr->roll);
    Serial.print("\t");
    Serial.print(adjustForError ? "Adjusted" : "Raw");
    Serial.print("\n");
  #endif
  }
}

#ifdef DEBUG
void startWifi() {
  char ssid[] = WIFI_SSID;
  char pass[] = WIFI_PASSWORD;

  WiFi.config(IP_ADDRESS);

  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to Network named: ");
    Serial.println(WIFI_SSID);

    status = WiFi.begin(ssid, pass);

    delay(500);
  }

  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");

  server.begin();
}

void handleWebServer()
{
  WiFiClient client = server.available(); 
  if (client) {
    String request = "";

    Serial.print("WEBSERVER: ");

    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        if (c == '\n') {
            client.println("HTTP/1.1 200 OK");
            break;
        } else if (c != '\r') {
          request += c;
        }
      }
    }

    client.stop();

    Serial.println(request);

    String name;
    double kp, ki, kd;

    getPidValuesFromRequest(request, name, kp, ki, kd);

    Serial.print("WEBSERVER: ");
    Serial.print(name);
    Serial.print("\t");
    Serial.print(kp);
    Serial.print("\t");
    Serial.print(ki);
    Serial.print("\t");
    Serial.print(kd);
    Serial.print("\n");

    if (name == "yaw") {
      pid_yaw.setGains(kp, ki, kd);
    } else if (name == "pitch") {
      pid_pitch.setGains(kp, ki, kd);
    } else if (name == "roll") {
      pid_roll.setGains(kp, ki, kd);
    }

    return;
  }
}

void getPidValuesFromRequest(const String& request, String& name, double& kp, double& ki, double& kd) {
  // Copilot wrote this code. Yuch.
  kp = ki = kd = 0;
  name = "";

  // Find the first '/' and the '?' after it
  int firstSlash = request.indexOf('/');
  int question = request.indexOf('?', firstSlash);
  if (firstSlash != -1) {
    int nameEnd = (question != -1) ? question : request.indexOf(' ', firstSlash + 1);
    if (nameEnd == -1) nameEnd = request.length();
    name = request.substring(firstSlash + 1, nameEnd);
  }

  // Parse parameters after '?'
  if (question != -1) {
    int paramEnd = request.indexOf(' ', question);
    if (paramEnd == -1) paramEnd = request.length();
    String params = request.substring(question + 1, paramEnd);

    int idx = 0;
    while (idx < params.length()) {
      int eq = params.indexOf('=', idx);
      if (eq == -1) break;
      String key = params.substring(idx, eq);
      int amp = params.indexOf('&', eq + 1);
      int end = (amp == -1) ? params.length() : amp;
      String val = params.substring(eq + 1, end);
      if (key == "kp") kp = val.toDouble();
      else if (key == "ki") ki = val.toDouble();
      else if (key == "kd") kd = val.toDouble();
      idx = end + 1;
    }
  }
}
#endif