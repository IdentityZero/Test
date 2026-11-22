#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>

float GRAVITY = 9.8;
float DEGREES = 360;

float dt;
unsigned long millisOld;

float theta, phi;

float LOW_PASS_FILTER_ALPHA = 0.1;

Adafruit_BNO055 myImu = Adafruit_BNO055();

void setup() {
  Serial.begin(9600);

  myImu.begin();
  myImu.setExtCrystalUse(true);
  millisOld = millis();
}

void loop() {
  uint8_t system, gyro, accel, mg = 0;

  float psi = getHeading();
  myImu.getCalibration(&system, &gyro, &accel, &mg);

  Serial.print("System_Cal:");
  Serial.print(system);
  Serial.print(",");
  Serial.print("Gyro_Cal:");
  Serial.print(gyro);
  Serial.print(",");
  Serial.print("Accel_Cal:");
  Serial.print(accel);
  Serial.print(",");
  Serial.print("Mag_Cal:");
  Serial.print(mg);
  Serial.print(",");

  Serial.print("Theta:");
  Serial.print(theta);
  Serial.print(",");
  Serial.print("Phi:");
  Serial.print(phi);
  Serial.print(",");
  Serial.print("Psi:");
  Serial.println(psi);

  // Serial.print("Gyro_y:");
  // Serial.print(thetaG);
  // Serial.print(",");
  // Serial.print("Gyro_x:");
  // Serial.print(phiG);
  // Serial.println(",");

  // Serial.print("Phi_Measured:");s
  // Serial.print(phiM);
  // Serial.print(",");
  // Serial.print("Phi_Filtered:");
  // Serial.print(phiFnew);
  // Serial.print(",");
  // Serial.print("Theta_Measured:");
  // Serial.print(thetaM);
  // Serial.print(",");
  // Serial.print("Theta_Filtered:");
  // Serial.println(thetaFnew);
  delay(50);
}

float getHeading() {
  imu::Vector<3> acc = myImu.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> gyr = myImu.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> mag = myImu.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

  float thetaM = -atan2(acc.x() / GRAVITY, acc.z() / GRAVITY);
  thetaM = thetaM / 2 / PI * DEGREES;

  float phiM = -atan2(acc.y() / GRAVITY, acc.z() / GRAVITY);
  phiM = phiM / 2 / PI * DEGREES;

  // Gyro
  dt = (millis() - millisOld) / 1000.0;
  millisOld = millis();

  theta = (theta + gyr.y() * dt) * (1 - LOW_PASS_FILTER_ALPHA) + (thetaM * LOW_PASS_FILTER_ALPHA);
  phi = (phi - gyr.x() * dt) * (1 - LOW_PASS_FILTER_ALPHA) + (phiM * LOW_PASS_FILTER_ALPHA);

  float phiRad = phi / 360 * (2 * PI);
  float thetaRad = theta / 360 * (2 * PI);

  // Magnetometer
  float magX = mag.x() * cos(thetaRad) - mag.y() * sin(phiRad) * sin(thetaRad) + mag.z() * cos(phiRad) * sin(thetaRad);
  float magY = mag.y() * cos(phiRad) + mag.z() * sin(phiRad);
  float psi = atan2(magY, magX);

  if (psi < 0)
    psi += 2 * PI;
  if (psi > 2 * PI)
    psi -= 2 * PI;

  psi = psi * 180 / M_PI;
  psi = adjustAngle(psi);

  return psi;
}

float adjustAngle(float angle) {
  angle += 90.0;
  if (angle >= 360.0) angle -= 360.0;
  if (angle < 0) angle += 360.0;
  return angle;
}
