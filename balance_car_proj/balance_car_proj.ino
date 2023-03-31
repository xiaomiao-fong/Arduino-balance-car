#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>


Adafruit_MPU6050 mpu;

const int L_DIR = 32;
const int L_STEP = 33;
const int R_DIR = 30;
const int R_STEP = 31;
const int stepsPerRevolution = 1000;
int time_gap = 500;
double t_angle = 0;


char value;

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

  /*
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
*/
  
}
void loop()
{
  // Set motor direction clockwise

  /*
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  */
  if (Serial3.available()) {
    value = Serial3.read();
    Serial.print(value);
  }
    
  digitalWrite(L_DIR, HIGH);
  digitalWrite(R_DIR, HIGH);
 
  for(int x = 0; x < stepsPerRevolution; x++)
  {
    digitalWrite(L_STEP, HIGH);
    digitalWrite(R_STEP, HIGH);
    delayMicroseconds(time_gap);
    digitalWrite(L_STEP, LOW);
    digitalWrite(R_STEP, LOW);
    delayMicroseconds(time_gap);
  }
  delay(500); 
  
 }