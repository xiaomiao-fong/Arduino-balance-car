#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>


Adafruit_MPU6050 mpu;

const int L_DIR = 32;
const int L_STEP = 33;
const int R_DIR = 30;
const int R_STEP = 31;
const int stepsPerRevolution = 300;
double t_angle = 0;


char value;
void motor_move(int dir, int time_gap);

void setup()
{

  Serial.begin(115200);
  Serial3.begin(9600);
  Serial.println("BT is ready!");


  // Declare pins as Outputs
  pinMode(L_STEP, OUTPUT);
  pinMode(L_DIR, OUTPUT);
  pinMode(R_STEP, OUTPUT);
  pinMode(R_DIR, OUTPUT);

  
  if(!mpu.begin()){
    Serial.println("Failed to detect mpu6050");
    while(1){
      delay(10);
    }
  }

  
  // set accelerometer range to +-8G
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

  // set gyro range to +- 500 deg/s
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  // set filter bandwidth to 21 Hz
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(100);

  
}
void loop()
{
  // Set motor direction clockwise

 
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  

  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");
  
  if (Serial3.available()) {
    value = Serial3.read();
    Serial.print(value);
  }

  motor_move(2,500);
 
  delay(50); 
  
 }


void motor_move(int dir, int time_gap){

  //1forward 2right 3backward 4left

  if(dir == 1 || dir == 4) digitalWrite(R_DIR, HIGH);
  else digitalWrite(R_DIR, LOW);

  if(dir == 1 || dir == 2) digitalWrite(L_DIR, HIGH);
  else digitalWrite(L_DIR, LOW);

  
  for(int x = 0; x < stepsPerRevolution; x++)
  {
    digitalWrite(L_STEP, HIGH);
    digitalWrite(R_STEP, HIGH);
    delayMicroseconds(time_gap);
    digitalWrite(L_STEP, LOW);
    digitalWrite(R_STEP, LOW);
    delayMicroseconds(time_gap);
  }
  
}
