#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define PI 3.141592//653589793238462643383279502884197169399375105820974

Adafruit_MPU6050 mpu;

const int L_DIR = 32;
const int L_STEP = 33;
const int R_DIR = 30;
const int R_STEP = 31;
const int stepsPerRevolution = 1000;
int time_gap = 500;
double t_angle = 0;

//角度參數們
float Angle, Angle_dot;   //小車最終傾斜角度、角速度
float Angle_aYZ;          //由Y軸Z軸上的加速度傳感器測得的數值，計算出傾斜角度
float Angle_gX;           //由X軸的陀螺儀傳感器測得的數值，計算出角速度

//卡爾曼參數們
float  Q_angle = 0.01;    //陀螺儀噪聲的協方差
float  Q_gyro = 0.01;     //陀螺儀漂移噪聲的協方差
float  R_angle = 0.003;   //加速度計的協方差
float  dt = 0.005;        //dt為kalman濾波器採樣時間;
char   C_0 = 1;
float  Q_bias = 0, Angle_err = 0; //Q_bias為陀螺儀漂移
float  PCt_0, PCt_1, E;
float  K_0, K_1, t_0, t_1;
float  Pdot[4] = {0, 0, 0, 0};
float  PP[2][2] = { { 1, 0 }, { 0, 1 } };

char value;
void Kalman_Filter(float, float);
void Angle_Calcu(float, float, float, float);

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
  mpu.getEvent(&a, &g, &temp);//(a=加速度、g=傾角、temp=溫度)拿溫度是要幹嘛?
  
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Angle_Calcu(a.acceleration.x,a.acceleration.x,a.acceleration.x,g.gyroscope.x);//這個g.gyroscope.x錯了不要打我 因為我真的不會
  Serial.print("Angle and Angle dot after using Kalman_Filter: ");
  Serial.print(Angle);  
  Serial.print(", ");
  Serial.print(Angle_dot);//卡爾曼濾波後的傾角及角速度
  
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
 // 卡爾曼濾波
void Kalman_Filter(float Accel, float Gyro)
{
  Angle += (Gyro - Q_bias) * dt; //先驗估計

  Pdot[0] = Q_angle - PP[0][1] -
              PP[1][0]; // Pk-先驗估計誤差協方差的微分

  Pdot[1] = - PP[1][1];
  Pdot[2] = - PP[1][1];
  Pdot[3] = Q_gyro;

  PP[0][0] += Pdot[0] * dt;   // Pk-先驗估計誤差協方差微分的積分
  PP[0][1] += Pdot[1] * dt;   // =先驗估計誤差協方差
  PP[1][0] += Pdot[2] * dt;
  PP[1][1] += Pdot[3] * dt;

  Angle_err = Accel - Angle;  //zk-先驗估計

  PCt_0 = C_0 * PP[0][0];
  PCt_1 = C_0 * PP[1][0];

  E = R_angle + C_0 * PCt_0;

  K_0 = PCt_0 / E;
  K_1 = PCt_1 / E;

  t_0 = PCt_0;
  t_1 = C_0 * PP[0][1];

  PP[0][0] -= K_0 * t_0;       //後驗估計誤差協方差
  PP[0][1] -= K_0 * t_1;
  PP[1][0] -= K_1 * t_0;
  PP[1][1] -= K_1 * t_1;

  Angle   += K_0 * Angle_err;  //後驗估計
  Q_bias  += K_1 * Angle_err;  //後驗估計
  Angle_dot = Gyro - Q_bias; //輸出值(後驗估計)的微分=角速度
}

//傾角計算（卡爾曼融合）
void Angle_Calcu(float ACCEL_YOUT_H,float ACCEL_YOUT_H,float ACCEL_ZOUT_H,float GYRO_XOUT_H)//輸入mpu6050的xyz加速度與x傾角
{
  //------根據加速度分量測得角速度--------------------------
  //不自測，加速度傳感器範圍設置  0  ±2g     16384 LSB/g
  Angle_aYZ = atan2(ACCEL_YOUT_H - 300),ACCEL_ZOUT_H - (16384 - 16450))) * 180 / PI; //去除零點偏移,計算得到角度（弧度），並把角度(弧度)轉換為度,

  //-------角速度-------------------------
  //不自測，陀螺儀測量範圍設置  0  ±250°/s  131LSB/(°/s)   0.00763358 (°/s)/LSB
  Angle_gX = (GYRO_XOUT_H - 0) * 0.00763358; //0為補償量，在靜止是測得的角速度為0LSB(老色逼)；

  //-------卡爾曼濾波融合-----------------------
  Kalman_Filter(Angle_aYZ - 0,
                Angle_gX - 0);       //卡爾曼濾波計算傾角,減去零點偏移
}
