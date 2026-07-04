#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <ESP32Servo.h>

MPU6050 mpu(0x68);
Servo myServo;

const int SERVO_PIN = 13;     
const int SERVO_CENTER = 90;  

// Complementary filter variables
float filteredAngleX = 0.0; 
unsigned long lastTime = 0;
float gyroXBias = 0.0;  // Gyroscope bias offset (calibrated at startup)

void setup() {
  Serial.begin(115200); 
  delay(1000); 

  Serial.println("--- SYSTEM STARTING ---");
  Wire.begin();        
  delay(100);

  Serial.println("Initializing MPU6050...");
  mpu.initialize();

  int retryCount = 0;
  while (!mpu.testConnection() && retryCount < 3) {
    Serial.println("MPU6050 connection failed! Retrying...");
    delay(1000);
    retryCount++;
  }

  // Gyroscope bias calibration
  // Averages 500 readings at rest to calculate the constant offset error
  // and subtract it before the complementary filter runs.
  // Keep the sensor completely still during startup.
  Serial.println("Calibrating gyroscope — keep sensor still...");
  long gx_sum = 0;
  int samples = 500;
  for (int i = 0; i < samples; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    gx_sum += gx;
    delay(2);
  }
  gyroXBias = gx_sum / (float)samples;
  Serial.print("Gyro bias calibrated: ");
  Serial.println(gyroXBias);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  
  myServo.setPeriodHertz(50); 
  myServo.attach(SERVO_PIN, 500, 2400); 
  myServo.write(SERVO_CENTER); 
  
  lastTime = millis(); 
  Serial.println("System Ready!");
}

void loop() {
  int16_t raw_ax, raw_ay, raw_az;
  int16_t raw_gx, raw_gy, raw_gz;

  mpu.getMotion6(&raw_ax, &raw_ay, &raw_az, &raw_gx, &raw_gy, &raw_gz);

  // 1. Calculate time elapsed since last loop (dt)
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;

  // 2. Convert raw Accelerometer to G's and find Accelerometer Angle
  float ay = raw_ay / 16384.0;
  float az = raw_az / 16384.0;
  float accelAngleX = atan2(ay, az) * 180.0 / M_PI;

  // 3. Convert raw Gyroscope to Degrees per Second
  // Subtract calibrated bias before converting to remove constant offset error
  float gyroRateX = (raw_gx - gyroXBias) / 131.0;

  // 4. THE COMPLEMENTARY FILTER
  // Gyroscope: fast, accurate short-term (high-pass filter)
  // Accelerometer: stable long-term reference (low-pass filter)
  // 0.98/0.02 split covers full frequency spectrum of motion
  filteredAngleX = 0.98 * (filteredAngleX + gyroRateX * dt) + 0.02 * accelAngleX;

  // 5. Turn the angle into a stabilization command
  int targetServoAngle = SERVO_CENTER - (int)filteredAngleX;
  targetServoAngle = constrain(targetServoAngle, 0, 180);

  myServo.write(targetServoAngle);

  // Serial Plotter output
  Serial.print("Accel_Angle:"); Serial.print(accelAngleX);
  Serial.print(",");
  Serial.print("Filtered_Gyro_Angle:"); Serial.println(filteredAngleX);

  delay(3); // ~100Hz update rate
}