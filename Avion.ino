#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
Adafruit_MPU6050 mpu;


const int buzzerPin = 2;
const int servoDPin = 3;
const int servoIPin = 6;
const int servoCPin = 7;
const int servoEPin = 8;

#define PWM 5

RF24 radio(9, 10); // CE, CSN

const byte address[6] = "00001";

struct Data {
  int xr;
  int yr;
  int xl;
  int yl;
  int p;
  bool button;
};

Data data;
Servo servoD;
Servo servoI;
Servo servoC;
Servo servoE;

class PID {
  public:
    PID(float kp, float ki, float kd) {
      this->kp = kp;
      this->ki = ki;
      this->kd = kd;
      this->previousError = 0;
      this->integral = 0;
    }

    float compute(float setpoint, float measured) {
      float error = setpoint - measured;
      this->integral += error;
      float derivative = error - this->previousError;
      this->previousError = error;
      return (kp * error) + (ki * this->integral) + (kd * derivative);
    }

  private:
    float kp;
    float ki;
    float kd;
    float previousError;
    float integral;
};

//kp, ki and kd values
PID pidPitch(15, 0.1, 0.04); 
PID pidRoll(20, 0.1, 0.04);
PID pidYaw(12, 0.1, 0.04);

void setup() {
  Serial.begin(9600);
  pinMode(buzzerPin, OUTPUT);
  // Set motor control pins as outputs
  pinMode(PWM, OUTPUT);
  // Set servo pins
  servoD.attach(servoDPin);
  servoI.attach(servoIPin);
  servoC.attach(servoCPin);
  servoE.attach(servoEPin);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
  } else{
    Serial.println("MPU6050 Found!");
    // set accelerometer range to +-8G
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    // set gyro range to +- 500 deg/s
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    // set filter bandwidth to 21 Hz
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  if (radio.available()) {
    radio.read(&data, sizeof(data));
    
    int xr = data.xr;
    int yr = data.yr;
    int xl = data.xl;
    int yl = data.yl;
    int p = data.p;
    bool button = data.button;
    // Control del buzzer
    if (button) {
      tone(buzzerPin, 1000); // 1000 Hz
    } else {
      noTone(buzzerPin);
      servoC.write(0);
    }

  // Initialize motor speed
  int motorSpeed = 0;
  float motorMult = 0.0;
  int motorpSpeed = 0;
  float motorjSpeed = 0;

  motorpSpeed = map(float(p), 0, 1023, 0, 127);

  if (xl < 450) {
    motorjSpeed = map(xl, 450, 0, 100, 0);
  } else if (xl > 550) {
    motorjSpeed = map(xl, 550, 1023, 100, 200);
  } else if (450 < xl < 550){
    motorjSpeed = 100;
  }
  motorMult = motorjSpeed / 100;
  motorSpeed = motorpSpeed * motorMult;
  analogWrite(PWM, (motorSpeed));

  float pitch = g.gyro.x;
  float roll = g.gyro.y;
  float yaw = g.gyro.z;

  // Serial.print("Gyro X: ");
  // Serial.print(pitch);
  // Serial.print(" | Gyro Y: ");
  // Serial.print(roll);
  // Serial.print(" | Gyro Z: ");
  // Serial.println(yaw);

  // Joystick pitch and roll mapping
  int jpitch = map(xr, 0, 1023, 180, 0);
  int jroll = map(yr, 0, 1023, 0, 180);
  int jyaw = map(yl, 0, 1023, 180, 0);

  // Set points (0 for stability)
  float desiredPitch = 0;
  float desiredRoll = 0;
  float desiredYaw = 0;

  // PID Correction
  float correctionPitch = pidPitch.compute(desiredPitch, pitch);
  float correctionRoll = pidRoll.compute(desiredRoll, roll);
  float correctionYaw = pidYaw.compute(desiredYaw, yaw);

  // Set correction mapped to servos
  int pitchmap = constrain(map(correctionPitch, -100, 100, 0, 180), 0, 180);
  int rollmap = constrain(map(correctionRoll, -100, 100, 0, 180), 0, 180);
  int yawmap = constrain(map(correctionYaw, -100, 100, 0, 180), 0, 180);

  // Set pitch
  int rpitch = (pitchmap+jpitch)/2;

  // Set roll
  int rroll = (rollmap+jroll)/2;

  // Set yaw
  int ryaw = (yawmap+jyaw) /2;

  // Set final position  
  int servoEPosition = constrain(rpitch, 50, 150);
  int servoDPosition = constrain(map(rroll, 0, 180, 180, 0), 90, 0);
  int servoIPosition = constrain(rroll, 90, 180);
  int servoCPosition = constrain(ryaw, 40, 140);

 
  

  
  servoD.write(servoDPosition);
  servoI.write(servoIPosition);
  servoC.write(servoCPosition);
  servoE.write(servoEPosition);

  Serial.print("Servo D: ");
  Serial.print(servoDPosition);
  Serial.print(" | Servo I: ");
  Serial.print(servoIPosition);
  Serial.print(" | Servo C: ");
  Serial.print("rroll: ");
  Serial.print(rroll);
  // Serial.print(servoCPosition);
  // Serial.print(" | Motor  J Speed: ");
  // Serial.print(motorjSpeed);
  // Serial.print(" | Motor  Mult: ");
  // Serial.print(motorMult);
  // Serial.print(" | Motor  Speed: ");
  // Serial.println(motorSpeed);

  } 

}