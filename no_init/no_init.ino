#include <FlexiTimer2.h>        //定时中断
//#include <MsTimer2.h>
// #include <PID_v1.h>
#include <Wire.h>
// #include <Servo.h>
#define servo2 27
#define servo3 29
#define servo1 48
#define servo4 42

//******************PWM引脚和电机驱动引脚***************************//
const int AIN1 = 6;//A电机控制PWM波
const int AIN2 = 5;//A电机控制PWM波
const int DIN1 = 46;//D电机控制PWM波
const int DIN2 = 44;//D电机控制PWM波

const int BANGL = 18;//左边碰撞开关
const int BANGR = 3;//右边碰撞开关

const int LL = 43;//最左循迹传感器
const int LR = 41;//左循迹传感器
const int RL = 47;//右循迹传感器
const int RR = 49;//最右循迹传感器

const int FS = 35;//前面循迹传感器
const int BS = 37;//后面循迹传感器

const int MagL = 53;//左电磁铁

//******************电机启动初始值  作者：Sunny**********************//
int motorDeadZone = 0;//高频时电机启动初始值高约为130，低频时电机启动初始值低约为30,和电池电压、电机特性、PWM频率等有关，需要单独测试
double Setpointa, Inputa, Outputa;
double Setpointd, Inputd, Outputd;
//******************编码器引脚***************************//
#define ENCODER_A 2//A路电机编码器引脚AA，外部中断，中断号0
#define ENCODER_D 19//D路电机编码器引脚DA，外部中断，中断号4
#define DIRECTION_A 51//A路电机编码器引脚AB
#define DIRECTION_D 50//D路电机编码器引脚DB

//**********************全局变量***********************//
volatile long Velocity_1,Velocity_4 ;   //编码器数据
float Velocity_A, Velocity_D ;//各轮速度
float Velocity_SetA=10;
float Velocity_SetD=10 ;//各轮期望速度
int iConstrain;
int vel;
int sensorValueLL = 0;
int sensorValueLR = 0;
int sensorValueRL = 0;
int sensorValueRR = 0;
int BangLValue = 0;
int BangRValue = 0;
int sensorValueFS;
int sensorValueBS;
// PID myPIDa(&Inputa, &Outputa, &Setpointa,2,5,1,P_ON_E, DIRECT);
// PID myPIDd(&Inputd, &Outputd, &Setpointd,2,5,1,P_ON_E, DIRECT);

const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX=0, AccErrorY=0, GyroErrorX=0, GyroErrorY=0, GyroErrorZ=0;
float elapsedTime, currentTime, previousTime;
int c = 0;

float last;
//float direction_90;
//float direction_180;
//float direction_270;

void ServoControl(int servoAngle,int servoPin)
{
  double thisAngle = map(servoAngle, 0, 180, 500, 2500);//等比例角度值范围转换高电平持续时间范围
  unsigned char i = 8;//50Hz 每秒的周期次数(周期/秒) 即1S 50 个周期 每个周期20ms
  while (i--)
  {
    digitalWrite(servoPin, HIGH); 
    delayMicroseconds(thisAngle); //高电平时间
    digitalWrite(servoPin, LOW); 
    delayMicroseconds(20000 - thisAngle);//每个周期20ms减去高电平持续时间
  }
}
//单个舵机转动一个来回
void setAngel(int servoPin,int startAngle,int endAngle)
{
  if(endAngle>startAngle)
  {
       for (int i = startAngle; i <= endAngle ; i += 5)
    {
      //delay(10);
      // Serial.println(i);
      ServoControl(i,servoPin);
    }
     for (int i = endAngle; i >= startAngle ; i -= 5)
    {
      //delay(10);
      // Serial.println(i);
      ServoControl(i,servoPin);
    }
  }
  else{
     for (int i = startAngle; i >= endAngle ; i -= 5)
    {
      //delay(10);
      // Serial.println(i);
      ServoControl(i,servoPin);
    }
     for (int i = endAngle; i <= startAngle ; i += 5)
    {
      //delay(10);
      // Serial.println(i);
      ServoControl(i,servoPin);
    }
  }
}
void allServo(){
  setAngel(servo1,160,50);
  setAngel(servo2,10,120);
  setAngel(servo3,160,45);
  setAngel(servo4,10,120);
}
/**************************************************************************
函数功能：赋值给PWM寄存器 ，重载函数，适用于2或4定向轮车
入口参数：PWM
**************************************************************************/
void Set_PWM(int motorRight,int motorLeft){
  //motorLeft -->  motorb 和 motora
  //motorRight -->  motorc 和 motord

  if (motorLeft > 0)        analogWrite(AIN2, motorLeft+motorDeadZone), analogWrite(AIN1, 0); //赋值给PWM寄存器根据电机响应速度与机械误差微调,
  else if(motorLeft == 0)   analogWrite(AIN2, 0), analogWrite(AIN1, 0);   
  else if (motorLeft < 0)   analogWrite(AIN1, -motorLeft+motorDeadZone), analogWrite(AIN2, 0);//高频时电机启动初始值高约为130，低频时电机启动初始值低约为30

  if (motorRight > 0)        analogWrite(DIN1,motorRight+motorDeadZone-4),analogWrite(DIN2, 0); //赋值给PWM寄存器根据电机响应速度与机械误差微调,
  else if(motorRight == 0)   analogWrite(DIN1, 0), analogWrite(DIN2, 0);
  else if (motorRight < 0)   analogWrite(DIN2, -motorRight+motorDeadZone-4), analogWrite(DIN1, 0);//高频时电机启动初始值高约为130，低频时电机启动初始值低约为30

}

/*****函数功能：外部中断读取编码器数据，具有二倍频功能 注意外部中断是跳变沿触发********/

void READ_ENCODER_A() {
  if (digitalRead(ENCODER_A) == LOW) {     //如果是下降沿触发的中断
    if (digitalRead(DIRECTION_A) == LOW)      Velocity_1--;  //根据另外一相电平判定方向
    else      Velocity_1++;
  }
  else {     //如果是上升沿触发的中断
    if (digitalRead(DIRECTION_A) == LOW)      Velocity_1++; //根据另外一相电平判定方向
    else     Velocity_1--;
  }
}

/*****函数功能：外部中断读取编码器数据，具有二倍频功能 注意外部中断是跳变沿触发********/
void READ_ENCODER_D() {
  if (digitalRead(ENCODER_D) == LOW) { //如果是下降沿触发的中断
    if (digitalRead(DIRECTION_D) == LOW)      Velocity_4++;//根据另外一相电平判定方向
    else      Velocity_4--;
  }
  else {   //如果是上升沿触发的中断
    if (digitalRead(DIRECTION_D) == LOW)      Velocity_4--; //根据另外一相电平判定方向
    else     Velocity_4++;
  }
}
///*****函数功能：外部中断读取左边碰撞开关********/
//void READ_BANG_L() {
//  if (digitalRead(BANGL) == LOW) { //如果是下降沿触发的中断
//    while(1)
//    {
//      Set_PWM(0,0 );
//    }
//  }
//  else {   //如果是上升沿触发的中断
//    Velocity_SetA = 10;
//    Velocity_SetD = 10;
//  }
//}
///*****函数功能：外部中断读取左边碰撞开关********/
//void READ_BANG_R() {
//  if (digitalRead(BANGL) == LOW) { //如果是下降沿触发的中断
//    Set_PWM(0,0 );
//  }
//  else {   //如果是上升沿触发的中断
//    Velocity_SetA = 10;
//    Velocity_SetD = 10;
//  }
//}

/*********函数功能：10ms控制函数*******/
void control() {

  //static int print_Count;
  sei();//全局中断开启
  Velocity_A = -Velocity_1;    Velocity_1 = 0; //读取编码器数据并根据实际接线做+-调整、然后清零，这就是通过M法测速（单位时间内的脉冲数）得到速度。
  Velocity_D = -Velocity_4;    Velocity_4 = 0; //读取编码器数据并根据实际接线做+-调整、然后清零，这就是通过M法测速（单位时间内的脉冲数）得到速度。
  MPU6050();

}
/*********函数功能：输出pid调节后的pwm*******/
void PID_pwm(int vela,int veld) {
  Setpointa = vela;
  Setpointd = veld;
//  Serial.print(millis());
//  Serial.print("A:");
//  Serial.print(Velocity_A);//显示 
//  Serial.print(",");
//  Serial.print("D:");
//  Serial.print(Velocity_D);//显示 
//  Serial.print("-------------\n");
  
  Inputa = Velocity_A;
  Inputd = Velocity_D;
//   myPIDa.Compute();
//   myPIDd.Compute();
// //  if (Outputd > 80)
//    Outputd = 80;
//   if (Outputa > 70)
//    Outputa = 70;
  Set_PWM(Outputa,Outputd);
//  i++;
}

/*********函数功能：循迹传感器读数*******/
void WalkSensor() {
  sensorValueLL = digitalRead(LL);
  sensorValueLR = digitalRead(LR);
  sensorValueRL = digitalRead(RL);
  sensorValueRR = digitalRead(RR);
  vel = sensorValueLL*8+sensorValueLR*4+sensorValueRL*2+sensorValueRR*1;
//  Serial.print(sensorValueLL);
//  Serial.print(sensorValueLR);
//  Serial.print(sensorValueRL);
//  Serial.print(sensorValueRR);
//  Serial.print(vel); 
//  Serial.print("---------------\n");
}

/*********函数功能：碰撞开关读数*******/
void BangSensor() {
  BangLValue = digitalRead(BANGL);
  BangRValue = digitalRead(BANGR);
//  Serial.print(BangLValue);//显示 
//  Serial.print(BangRValue);
//  Serial.print("-------------\n");
}

void Straight_light(int i,int expect)
{   
    // Serial.println("Straight_light");
    BangSensor();
    WalkSensor();
    if(i)
    {
      switch(vel)
      {
        case 15:StraightAdjust(1,expect,0);break;//1111
        case 0:StraightAdjust(1,expect,0);break;//0000
        case 8:StraightAdjust(1,expect +3,0);
                // delay(25);
                break;
        // Set_PWM(90,130 );delay(100);Set_PWM(170,90 );delay(100);break;//1000
        case 4:StraightAdjust(1,expect,-5);
                // delay(25);
                break;//Set_PWM(100,120 );break;//0100
        case 2:StraightAdjust(1,expect,5);
                // delay(25);
                break;//Set_PWM(120,100 );break;//0010
        case 1:StraightAdjust(1,expect-3,0);
                // delay(25);
                break;
        // Set_PWM(130,90 );delay(100);Set_PWM(90,170 );delay(100);break;//0001
        case 6:StraightAdjust(1,expect,0);break;//Set_PWM(110,110);break;//0110
        case 12:StraightAdjust(1,expect+3,15);
                // delay(25);
                break;//Set_PWM(100,120 );break;//1100
        case 3:StraightAdjust(1,expect-3,15);
                // delay(25);
                break;//Set_PWM(120,100 );break;//0011

      }
    }
    else
    {
      // Straight(0,expect);//1111
      StraightAdjust(0,expect,0);//1111
    }
}
// 同时使用巡线和加速度计
void StraightAdjust(int i,int expect,int bias)
{   
  if(i)
  {
    BangSensor();
    WalkSensor();
    int diff =(int)(yaw-last-expect);
    int di=0;
    diff=diff<<2;
    if(abs(di)>20){
      if(diff>0){
        di=20;
      }else{
        di=-20;
      }
      // di=30;
    }else{
      di=diff;
    }
    // Serial.print(yaw);
    // Serial.print("/");
    // Serial.print(last+expect);
    // Serial.print("/");
    // Serial.println(diff);
    // Serial.print("bias");
    // Serial.println(bias);
    if(abs(diff)<1) //0.5
    {
      // 好像两边速度一样右边的轮子会比较快
      // Set_PWM(160,160);
      Set_PWM(190,140);
      delay(2);
      Set_PWM(120,100);
      delay(10);
    }
    else if(diff>1) //0.5
    {
      // Set_PWM(135+di+bias,135);
      // delay(2);
      // Set_PWM(105+di+bias,95);
      // delay(2);

      Set_PWM(205+di+bias,140);
      delay(2);
      Set_PWM(145+di+bias,100);
      delay(10);
    }
    else
    {
      // Set_PWM(150,135-di+bias);
      // delay(2);
      // Set_PWM(95,105-di+bias);
      // delay(2);    
      Set_PWM(165,150-di+bias);
      delay(2);
      Set_PWM(110,110-di+bias);
      delay(10);
    }
  }
  else
  {
    int diff = (int)(yaw-last-expect);
    int di=0;
    diff=diff>>1;
    if(abs(diff)>20){
      if(diff>0){
        di=20;
      }else{
        di=-20;
      }
      // di=30;
    }else{
      di=diff;
    }

    // Serial.print(yaw);
    // Serial.print("/");
    // Serial.print(last+expect);
    // Serial.print("/");
    // Serial.println(diff);
    if(abs(diff)<1)
    {
      Set_PWM(-130,-120);
    }
    else if(diff>1)
    {
      // Set_PWM(-100,-140);
    

      // Set_PWM(-135-di-bias,-135);
      // delay(2);
      // Set_PWM(-105-di-bias,-95);
      // delay(2);

      // Set_PWM(-140,-100);

      Set_PWM(-110+di-bias,-155);
      delay(2);
      // Set_PWM(-95,-105+di-bias);
      // delay(2);
    }
    else
    {
      
      // Set_PWM(-140,-100);

      // Set_PWM(-135,-135+di-bias);
      // delay(2);
      // Set_PWM(-95,-105+di-bias);
      // delay(2);
      Set_PWM(-145,-120-di-bias);
      delay(2);
      // Set_PWM(-105-di-bias,-95);
      // delay(2);

    }
  }
}
/*****函数功能：走直********/
void Straight(int i,int expect)
{   
  if(i)
  {
    BangSensor();
    WalkSensor();
    float diff = yaw-last-expect;
    // Serial.print(yaw);
    // Serial.print("/");
    // Serial.print(last+expect);
    // Serial.print("/");
    // Serial.println(diff);
    if(abs(diff)<0.5)
    {
      Set_PWM(110,110);
    }
    else if(diff>0.5)
    {
      Set_PWM(120,80);
    }
    else
    {
      Set_PWM(80,120);
    }
  }
  else
  {
    float diff = yaw-last-expect;
    // Serial.print(yaw);
    // Serial.print("/");
    // Serial.print(last+expect);
    // Serial.print("/");
    // Serial.println(diff);
    if(abs(diff)<1)
    {
      Set_PWM(-120,-120);
    }
    else if(diff>1)
    {
      Set_PWM(-100,-140);
    }
    else
    {
      Set_PWM(-140,-100);
    }
  }
}

void MPU6050()
{
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
  // Calculating Roll and Pitch from the accelerometer data
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorX; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorY; // AccErrorY ~(-1.58)
  // === Read gyroscope data === //
  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
  // Correct the outputs with the calculated error values
  //GyroX = GyroX -22.41; // GyroErrorX ~(-0.56)
  //GyroY = GyroY - 1.88; // GyroErrorY ~(2)
  //GyroZ = GyroZ + 1.05; // GyroErrorZ ~ (-0.8)
  GyroX = GyroX -GyroErrorX; // 
  GyroY = GyroY -GyroErrorY; // 
  GyroZ = GyroZ -GyroErrorZ; // 
  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  gyroAngleX = gyroAngleX + GyroX * elapsedTime; // deg/s * s = deg
  gyroAngleY = gyroAngleY + GyroY * elapsedTime;
  yaw =  yaw + GyroZ * elapsedTime;
  // Complementary filter - combine acceleromter and gyro angle values
  roll = 0.999 * gyroAngleX + 0.001 * accAngleX;
  pitch = 0.999 * gyroAngleY + 0.001 * accAngleY;
// Print the values on the serial monitor
 Serial.print(roll);
 Serial.print("/");
 Serial.print(pitch);
 Serial.print("/");
 Serial.println(yaw);
//  Serial.print(gyroAngleX);
//  Serial.print("/");
//  Serial.print(gyroAngleY);
//  Serial.print("/");
//  Serial.print(gyroAngleZ);
//  Serial.print("/");
//  Serial.print(accAngleX);
//  Serial.print("/");
//  Serial.print(accAngleY);
//  Serial.print("/");

//  Serial.print(GyroX);
//  Serial.print("/");
//  Serial.print(GyroY);
//  Serial.print("/");
//  Serial.print(GyroZ);
//  Serial.print("/");
//  Serial.println("/");
//  Serial.print(AccX);
//  Serial.print("/");
//  Serial.print(AccY);
//  Serial.print("/");
//  Serial.println(AccZ);
}


/*****函数功能：转弯********/
void turn(int i,int limit)
{
    switch(i)
    {
      case 1:
      {
        do
        {
          while(yaw>360)
          {
            yaw -=360;
          }
          while(yaw<-360)
          {
            yaw += 360;
          }
          Set_PWM(-120,110); 
          // Serial.print(yaw);
          // Serial.print("-");
          // Serial.print(limit);
          // Serial.print("=");
          // Serial.print(abs(yaw-limit));
          // Serial.print("-----------\n");
        }while(abs(yaw-limit)>1.5);
        // Set_PWM(50,-50);
        // delay(50); 
        break;
        
      }//左转
      case 2:
      {
        //      {
        do
        {
          while(yaw>360)
          {
            yaw -=360;
          }
          while(yaw<-360)
          {
            yaw += 360;
          }
          Set_PWM(110,-120); 
          // Serial.print(yaw);
          // Serial.print("-");
          // Serial.print(limit);
          // Serial.print("=");
          // Serial.print(abs(yaw-limit));
          // Serial.print("-----------\n");
        }while(abs(yaw-limit)>1.5);
        // Set_PWM(-50,50);
        // delay(50); 
        break;
      }//右转
    }
}
/*****函数功能：抓东西********/
void Catch(int expect, int back){
  turn(1,expect);
  Set_PWM(0,0);
  delay(2000);
  BangSensor();
  Set_PWM(150,100);
  delay(200);
  int i = 0;
  while(BangLValue&&BangRValue)
  {
    i++;
//    Serial.println(i);
    BangSensor();
    Straight_light(1,expect);
    // Serial.print(BangLValue);
    // Serial.print(BangRValue);
    // Serial.print("----------\n");
    if(i> 600)
    {
      break;
    }
  }
  // Serial.print("!!!!!!!!!!!!!!!!");
  Set_PWM(0,0);
//  allServo();
  BackNum(back,expect);
  Straight_light(1,expect);
  delay(400);
  turn(2,expect-90);
  Set_PWM(0,0);
  delay(800);
  Set_PWM(110,110);
  delay(1000);
//  Straight_light(1,expect);
//  delay(500);
//  Serial.println('1');
//  while(Serial.available()<0);
//  char c = Serial.read();
//  if(c=='2')
//  {
//    while(BangLValue||BangRValue)
//    {
//      Straight(1);
//    }
//    Serial.println('4');
//    while(Serial.available()<0);
//    char c = Serial.read();
//    if(c=='6')
//    {
//      while(sensorValueFS == 1&&sensorValueBS ==1)
//      {
//        Straight(0); 
//      }
//    }
//  }
//  turn(1,84);
}

/*****函数功能：数格子********/
void StraightNum(int n,int expect)
{   
  int i = 0;
  int flag =0;
  BangSensor();
  while(i<0)
  {
    BangSensor();
    if(BangLValue==0||BangRValue==0)
    {
      BackNum(1,expect);
      Catch(expect+90,2);
      return;
    }
    Straight_light(1,expect);
//    Serial.print(i);
//    Serial.print("\n");
    sensorValueFS = digitalRead(FS);
    sensorValueBS = digitalRead(BS);
//    Serial.print(sensorValueFS);
//    Serial.print(sensorValueBS);
//    Serial.print("----------\n");
    if(sensorValueFS == 1||sensorValueBS ==1)
    {
      flag =1;
    }
    else if(flag == 1)
    {
      i++;
      flag = 0;
    }
    Straight_light(1,expect);
  }
  
  i = 0;
  flag =0;
  while(1)
  {
    BangSensor();
    if(BangLValue==0||BangRValue==0)
    {
      BackNum(1,expect);
      if(i !=n-1)
      {
        i = n-2;
        flag = 1;
      }
      else{
        return;
      }
    }
    Straight_light(1,expect);
//    Serial.print(i);
//    Serial.print("\n");
    sensorValueFS = digitalRead(FS);
    sensorValueBS = digitalRead(BS);
//    Serial.print(sensorValueFS);
//    Serial.print(sensorValueBS);
//    Serial.print("----------\n");
    if(sensorValueFS == 1||sensorValueBS ==1)
    {
      flag =1;
    }
    else if(flag == 1)
    {
      i++;
      flag = 0;
      Set_PWM(0,0);
      delay(500);
      Straight_light(0,expect);
      delay(350);
      if(i == n-1)
      {
        Catch(expect+90,2);
        break;
      }
      else
      {
        Catch(expect+90,1);
        Straight_light(1,expect);
        delay(100);
      }
    }
  }
}
/*****函数功能：倒退数格子********/
void BackNum(int n,int expect)
{   
  int i = 0;
  int x = 0;
  int flag =0;
  while(i<n)
  {
    x++;
    Straight_light(0,expect);
//    Serial.print(i);
//    Serial.print("\n");
    sensorValueFS = digitalRead(FS);
    sensorValueBS = digitalRead(BS);
//    Serial.print(sensorValueFS);
//    Serial.print(sensorValueBS);
//    Serial.print("----------\n");
    if(sensorValueFS == 1||sensorValueBS ==1)
    {
      flag =1;
    }
    else if(flag == 1)
    {
      i++;
      flag = 0;
    }
    // Serial.println(x);
    if(x > 700)
    {
      break;
    }
  }
  Straight_light(0,expect);
  delay(300);
}

void calculate_IMU_error() {
  // Read accelerometer values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    // Sum all readings
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    c++;
  }
  //Divide the sum by 200 to get the error value
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  c = 0;

  // Read gyro values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();
    // Sum all readings

    GyroErrorX = GyroErrorX + (GyroX / 131.0);
    GyroErrorY = GyroErrorY + (GyroY / 131.0);
    GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
    c++;
  }
  //Divide the sum by 200 to get the error value

  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;
  // Print the error values on the Serial Monitor
  Serial.print("AccErrorX: ");
  Serial.println(AccErrorX);
  Serial.print("AccErrorY: ");
  Serial.println(AccErrorY);
  Serial.print("GyroErrorX: ");
  Serial.println(GyroErrorX);
  Serial.print("GyroErrorY: ");
  Serial.println(GyroErrorY);
  Serial.print("GyroErrorZ: ");
  Serial.println(GyroErrorZ);
}

void setup() {
  // put your setup code here, to run once:
  //int fff = 1;
  //TCCR1B =(TCCR1B & 0xF8) | fff;//调整计数器分频，频率调高至31.374KHZ
  //TCCR3B =(TCCR3B & 0xF8) | fff;//调整计数器分频，频率调高至31.374KHZ
  //TCCR4B =(TCCR4B & 0xF8) | fff;//调整计数器分频，频率调高至31.374KHZ 
  //TCCR5B =(TCCR5B & 0xF8) | fff;//调整计数器分频，频率调高至31.374KHZ
  pinMode(AIN1, OUTPUT);
  pinMode(DIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(DIN2, OUTPUT);
  pinMode(MagL, OUTPUT);
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_D, INPUT_PULLUP);
  pinMode(DIRECTION_A, INPUT_PULLUP);
  pinMode(DIRECTION_D, INPUT_PULLUP);
  pinMode(BANGL, INPUT);
  pinMode(BANGR, INPUT);
  pinMode(LL, INPUT);
  pinMode(LR, INPUT);
  pinMode(RL, INPUT);
  pinMode(RR, INPUT);
  Serial.begin(9600);  
  delay(300);   //延时等待初始化完成 
  
//  attachInterrupt(5, READ_BANG_L, CHANGE);           //开启外部中断 左边碰撞开关
//  attachInterrupt(1, READ_BANG_R, CHANGE);           //开启外部中断 右边碰撞开关

  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission
  calculate_IMU_error();
  delay(200);
  FlexiTimer2::set(15, control); //10毫秒定时中断函数
  FlexiTimer2::start ();      //中断使能
  attachInterrupt(0, READ_ENCODER_A, CHANGE);           //开启外部中断 编码器接口A
  attachInterrupt(4, READ_ENCODER_D, CHANGE);           //开启外部中断 编码器接口D
  last = yaw;

  // myPIDa.SetMode(AUTOMATIC);
  // myPIDd.SetMode(AUTOMATIC);
//  myPIDa.SetMode(MANUAL);
//  myPIDd.SetMode(MANUAL);
  pinMode(servo1, OUTPUT);
  digitalWrite(servo1, LOW);//先保证拉低
  pinMode(servo2, OUTPUT);
  digitalWrite(servo2, LOW);
   pinMode(servo3, OUTPUT);
  digitalWrite(servo3, LOW);
   pinMode(servo4, OUTPUT);
  digitalWrite(servo4, LOW); 
}
void loop() {
  BangSensor();
  WalkSensor();
  sensorValueFS = digitalRead(FS);
  sensorValueBS = digitalRead(BS);
  digitalWrite(MagL, HIGH);
  // while (1)
  // {
  //   Straight_light(1,0);
  //   /* code */
  // }

  int i=0;
  for(i=0;i<50;i++){
    Straight_light(1,0);
    i++;
  }
   turn(1,90);
   BackNum(1,90);
   turn(2,0);
   StraightNum(7,0);
   turn(2,-90);
   StraightNum(7,-90);
   turn(2,-180);
   StraightNum(7,-180);
   turn(2,-270);
   StraightNum(7,-270);
   while(1)
   {
     Straight_light(1,-270);
   }
    
//Straight_light(0,0);
// StraightNum(6,0);

//calculate_IMU_error();
//MPU6050();
//  Serial.print(millis());
//  Serial.print("A:");
//  Serial.print(Velocity_A);//显示 
//  Serial.print(",");
//  Serial.print("D:");
//  Serial.print(Velocity_D);//显示 
//  Serial.print("-------------\n");

}
