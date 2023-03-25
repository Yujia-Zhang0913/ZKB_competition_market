# 1 "e:\\code\\zkb\\113\\113.ino"
# 2 "e:\\code\\zkb\\113\\113.ino" 2
//#include <MsTimer2.h>
// #include <PID_v1.h>
# 5 "e:\\code\\zkb\\113\\113.ino" 2
// #include <Servo.h>





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





//**********************全局变量***********************//
volatile long Velocity_1,Velocity_4 ; //编码器数据
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
float direction_0;
float direction_90;


// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2016-04-18 - Eliminated a potential infinite loop
//      2013-05-08 - added seamless Fastwire support
//                 - added note about gyro calibration
//      2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility error
//      2012-06-20 - improved FIFO overflow handling and simplified read process
//      2012-06-19 - completely rearranged DMP initialization code and simplification
//      2012-06-13 - pull gyro and accel data from FIFO packet instead of reading directly
//      2012-06-09 - fix broken FIFO read sequence and change interrupt detection to RISING
//      2012-06-05 - add gravity-compensated initial reference frame acceleration output
//                 - add 3D math helper file to DMP6 example sketch
//                 - add Euler output and Yaw/Pitch/Roll output formats
//      2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
//      2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
//      2012-05-30 - basic DMP initialization working

/* ============================================

I2Cdev device library code is placed under the MIT license

Copyright (c) 2012 Jeff Rowberg



Permission is hereby granted, free of charge, to any person obtaining a copy

of this software and associated documentation files (the "Software"), to deal

in the Software without restriction, including without limitation the rights

to use, copy, modify, merge, publish, distribute, sublicense, and/or sell

copies of the Software, and to permit persons to whom the Software is

furnished to do so, subject to the following conditions:



The above copyright notice and this permission notice shall be included in

all copies or substantial portions of the Software.



THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR

IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,

FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE

AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER

LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,

OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN

THE SOFTWARE.

===============================================

*/
# 116 "e:\\code\\zkb\\113\\113.ino"
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
# 119 "e:\\code\\zkb\\113\\113.ino" 2

# 121 "e:\\code\\zkb\\113\\113.ino" 2


# 124 "e:\\code\\zkb\\113\\113.ino" 2


// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================

   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch

   depends on the MPU-6050's INT pin being connected to the Arduino's

   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is

   digital I/O pin 2.

 * ========================================================================= */
# 140 "e:\\code\\zkb\\113\\113.ino"
/* =========================================================================

   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error

   when using Serial.write(buf, len). The Teapot output uses this method.

   The solution requires a modification to the Arduino USBAPI.h file, which

   is fortunately simple, but annoying. This will be fixed in the next IDE

   release. For more info, see these links:



   http://arduino.cc/forum/index.php/topic,109987.0.html

   http://code.google.com/p/arduino/issues/detail?id=958

 * ========================================================================= */
# 153 "e:\\code\\zkb\\113\\113.ino"
// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)


// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT





// MPU control/status vars
bool dmpReady = false; // set true if DMP init was successful
uint8_t mpuIntStatus; // holds actual interrupt status byte from MPU
uint8_t devStatus; // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize; // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount; // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q; // [w, x, y, z]         quaternion container
VectorInt16 aa; // [x, y, z]            accel sensor measurements
VectorInt16 aaReal; // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector
// float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3]; // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high

uint16_t dmpcount=0;
void dmpDataReady() {
    // mpuInterrupt = true;
    // if programming failed, don't try to do anything
    // if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    // while (!mpuInterrupt && fifoCount < packetSize) {
    //     // 批量获取数据
    //     if (mpuInterrupt && fifoCount < packetSize) {
    //       // try to get out of the infinite loop 
    //       fifoCount = mpu.getFIFOCount();
    //     }  
    // }

    // reset interrupt flag and get INT_STATUS byte
    // mpuInterrupt = false;
    // mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();
    // Serial.println(dmpcount++);
    // check for overflow (this should never happen unless our code is too inefficient)
    if ( fifoCount >= 1024) {
    // if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        fifoCount = mpu.getFIFOCount();
        // Serial.println(F("FIFO overflow!"));

    } else if(fifoCount > packetSize){
    // } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
        // while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;

            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            // mpu.dmpGetGravity(&gravity, &q);
            // mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            mpu.dmpGetRoll(ypr, &q);
            ypr[0]=ypr[0]*180/
# 261 "e:\\code\\zkb\\113\\113.ino" 3
                             3.14159265358979323846 /* pi */
# 261 "e:\\code\\zkb\\113\\113.ino"
                                 ;
            // Serial.println(dmpcount++);
            // Serial.print('y');
            // Serial.print(ypr[0]);
            // Serial.print('p');
            // Serial.print(ypr[1]*180/M_PI);
            // Serial.print('r');
            // Serial.println(ypr[2]*180/M_PI);

        // mpu.resetFIFO();
    }
}



void ServoControl(int servoAngle,int servoPin)
{
  double thisAngle = map(servoAngle, 0, 180, 500, 2500);//等比例角度值范围转换高电平持续时间范围
  unsigned char i = 8;//50Hz 每秒的周期次数(周期/秒) 即1S 50 个周期 每个周期20ms
  while (i--)
  {
    digitalWrite(servoPin, 0x1);
    delayMicroseconds(thisAngle); //高电平时间
    digitalWrite(servoPin, 0x0);
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
  setAngel(42,160,50);
  setAngel(27,10,120);
  setAngel(29,160,45);
  setAngel(48,10,120);
}
/**************************************************************************

函数功能：赋值给PWM寄存器 ，重载函数，适用于2或4定向轮车

入口参数：PWM

**************************************************************************/
# 333 "e:\\code\\zkb\\113\\113.ino"
void Set_PWM(int motorRight,int motorLeft){
  //motorLeft -->  motorb 和 motora
  //motorRight -->  motorc 和 motord

  if (motorLeft > 0) analogWrite(AIN2, motorLeft+motorDeadZone), analogWrite(AIN1, 0); //赋值给PWM寄存器根据电机响应速度与机械误差微调,
  else if(motorLeft == 0) analogWrite(AIN2, 0), analogWrite(AIN1, 0);
  else if (motorLeft < 0) analogWrite(AIN1, -motorLeft+motorDeadZone), analogWrite(AIN2, 0);//高频时电机启动初始值高约为130，低频时电机启动初始值低约为30

  if (motorRight > 0) analogWrite(DIN1,motorRight+motorDeadZone-4),analogWrite(DIN2, 0); //赋值给PWM寄存器根据电机响应速度与机械误差微调,
  else if(motorRight == 0) analogWrite(DIN1, 0), analogWrite(DIN2, 0);
  else if (motorRight < 0) analogWrite(DIN2, -motorRight+motorDeadZone-4), analogWrite(DIN1, 0);//高频时电机启动初始值高约为130，低频时电机启动初始值低约为30

}

/*****函数功能：外部中断读取编码器数据，具有二倍频功能 注意外部中断是跳变沿触发********/

void READ_ENCODER_A() {
  if (digitalRead(2/*A路电机编码器引脚AA，外部中断，中断号0*/) == 0x0) { //如果是下降沿触发的中断
    if (digitalRead(51/*A路电机编码器引脚AB*/) == 0x0) Velocity_1--; //根据另外一相电平判定方向
    else Velocity_1++;
  }
  else { //如果是上升沿触发的中断
    if (digitalRead(51/*A路电机编码器引脚AB*/) == 0x0) Velocity_1++; //根据另外一相电平判定方向
    else Velocity_1--;
  }
}

/*****函数功能：外部中断读取编码器数据，具有二倍频功能 注意外部中断是跳变沿触发********/
void READ_ENCODER_D() {
  if (digitalRead(19/*D路电机编码器引脚DA，外部中断，中断号4*/) == 0x0) { //如果是下降沿触发的中断
    if (digitalRead(50/*D路电机编码器引脚DB*/) == 0x0) Velocity_4++;//根据另外一相电平判定方向
    else Velocity_4--;
  }
  else { //如果是上升沿触发的中断
    if (digitalRead(50/*D路电机编码器引脚DB*/) == 0x0) Velocity_4--; //根据另外一相电平判定方向
    else Velocity_4++;
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
  
# 399 "e:\\code\\zkb\\113\\113.ino" 3
 __asm__ __volatile__ ("sei" ::: "memory")
# 399 "e:\\code\\zkb\\113\\113.ino"
      ;//全局中断开启
  Velocity_A = -Velocity_1; Velocity_1 = 0; //读取编码器数据并根据实际接线做+-调整、然后清零，这就是通过M法测速（单位时间内的脉冲数）得到速度。
  Velocity_D = -Velocity_4; Velocity_4 = 0; //读取编码器数据并根据实际接线做+-调整、然后清零，这就是通过M法测速（单位时间内的脉冲数）得到速度。
  // MPU6050();
  dmpDataReady();
  yaw=ypr[0]-GyroErrorZ;
  Serial.print("yaw");
  Serial.println(yaw);



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
  BangLValue = !digitalRead(BANGL);
  BangRValue = !digitalRead(BANGR);
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
        case 8:StraightAdjust(1,expect +3,20);
                // delay(25);
                break;
        // Set_PWM(90,130 );delay(100);Set_PWM(170,90 );delay(100);break;//1000
        case 4:StraightAdjust(1,expect,-5);
                // delay(25);
                break;//Set_PWM(100,120 );break;//0100
        case 2:StraightAdjust(1,expect,5);
                // delay(25);
                break;//Set_PWM(120,100 );break;//0010
        case 1:StraightAdjust(1,expect-3,20);
                // delay(25);
                break;
        // Set_PWM(130,90 );delay(100);Set_PWM(90,170 );delay(100);break;//0001
        case 6:StraightAdjust(1,expect,0);break;//Set_PWM(110,110);break;//0110
        case 12:StraightAdjust(1,expect+3,20);
                // delay(25);
                break;//Set_PWM(100,120 );break;//1100
        case 3:StraightAdjust(1,expect-3,20);
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
    if(((di)>0?(di):-(di))>20){
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
    if(((diff)>0?(diff):-(diff))<1) //0.5
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

      Set_PWM(195+di+bias,140);
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
    if(((diff)>0?(diff):-(diff))>20){
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
    if(((diff)>0?(diff):-(diff))<1)
    {
      Set_PWM(-145,-130);
    }
    else if(diff>1)
    {
      // Set_PWM(-100,-140);


      // Set_PWM(-135-di-bias,-135);
      // delay(2);
      // Set_PWM(-105-di-bias,-95);
      // delay(2);

      // Set_PWM(-140,-100);

      Set_PWM(-120+di-bias,-165);
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
      Set_PWM(-155,-130-di-bias);
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
    if(((diff)>0?(diff):-(diff))<0.5)
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
    if(((diff)>0?(diff):-(diff))<1)
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
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / 3.1415926535897932384626433832795) - AccErrorX; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / 3.1415926535897932384626433832795) - AccErrorY; // AccErrorY ~(-1.58)
  // === Read gyroscope data === //
  previousTime = currentTime; // Previous time is stored before the actual time read
  currentTime = millis(); // Current time actual time read
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
  yaw = yaw + GyroZ * elapsedTime;
  // Complementary filter - combine acceleromter and gyro angle values
  roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
  pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
// Print the values on the serial monitor
 Serial.print(roll);
 Serial.print("/");
 Serial.print(pitch);
 Serial.print("/");
 Serial.println(yaw);
}


/*****函数功能：转弯********/
void turn(int i,int limit)
{
  int count_line=0;
    switch(i)
    {
      case 1:
      {
        do
        {
          WalkSensor();
          while(yaw>360)
          {
            yaw -=360;
          }
          while(yaw<-360)
          {
            yaw += 360;
          }
          Set_PWM(-140,125);
          // count_line++;
          // if(count_line>20000)
          // {
          //   break;
          // }
          // Serial.print(yaw);
          // Serial.print("-");
          // Serial.print(limit);
          // Serial.print("=");
          // Serial.print(abs(yaw-limit));
          // Serial.print("-----------\n");
        }while(((yaw-limit)>0?(yaw-limit):-(yaw-limit))>1.5);
        Set_PWM(120,-120);
        delay(100);
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
          Set_PWM(130,-135);
          // count_line++;
          // if(count_line>20000)
          // {
          //   break;
          // }
          // Serial.print(yaw);
          // Serial.print("-");
          // Serial.print(limit);
          // Serial.print("=");
          // Serial.print(abs(yaw-limit));
          // Serial.print("-----------\n");
        }while(((yaw-limit)>0?(yaw-limit):-(yaw-limit))>1.5);
        Set_PWM(-80,80);
        delay(50);
        break;
      }//右转
    }
}
/*****函数功能：抓东西********/
void Catch(int expect, int back){

// char c = '2';
    turn(1,expect);
    Set_PWM(0,0);
    delay(200);
    BangSensor();
    // Set_PWM(150,100);
    // delay(200);
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
      if(i> 6000)
      {
        break;
      }
    }
    Serial.println('7');
    Set_PWM(0,0);
    digitalWrite(52, 0x1);
    delay(7000);
    digitalWrite(52, 0x0);//先保证拉低
    // while(Serial.available()<0)
    // {
    //   i++;
    //   if(i>300000)
    //   {
    //     break;
    //   }
    // }
    BackNum(back,expect);
    Straight_light(1,expect);
    delay(400);
    turn(2,expect-90);
    Set_PWM(0,0);
    delay(800);
    // int r=0;
    // for(r=0;r<10;r++){
    // Straight_light(1,0);
    // r++;
  // }
 return;
}

/*****函数功能：数格子********/
void StraightNum(int n,int expect)
{
  int i = 0;
  int flag =0;
  BangSensor();
  while(i<1)
  {
    BangSensor();
    if(BangLValue==0||BangRValue==0)
    {
      BackNum(1,expect);
      Catch(expect+90,3);
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
      delay(450);
      // Serial.print("##############");
      // Serial.println(i);
      if(i == n-1)
      {
        Serial.print("!!!!!!!!!!!!!!!!");
        Catch(expect+90,3);
        int o=0;
        while(!(BangLValue==0&&BangRValue==0))
        {
          Straight_light(1,expect);
          o++;
          if(o>6000)
          {
            break;
          }
        }
        Set_PWM(0,0);
        direction_90=yaw;
        return;
      }
      else
      {
        Set_PWM(0,0);
        int j=0;
        Serial.println('1');
        // while(Serial.available()<=0)
        // {
        //   j++;
        // }
        // char c = Serial.read();
        char c='2';
        if(c=='2'||c=='3'||c=='4')
        {
          Catch(expect+90,1);
        }
        else if(c=='5')
        {
          ;
        }
        j =0;
        Set_PWM(170,140);
        delay(50);
        for(j=0;j<200;j++)
        {
          Serial.println(j);
          Straight_light(1,expect);
        }
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
  //  Serial.print(sensorValueFS);
  //  Serial.print(sensorValueBS);
  //  Serial.print(flag);
  //  Serial.print("----------\n");
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
    if(x > 7000)
    {
      break;
    }
  }
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
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / 3.1415926535897932384626433832795));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / 3.1415926535897932384626433832795));
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
void calculate_IMUDMP_error(){
  GyroErrorZ=0;
  for(uint8_t i=0;i<200;i++){
    dmpDataReady();
    GyroErrorZ+=ypr[0];
    delay(10);
  }
  GyroErrorZ/=200;
}
void calculate_IMUDMP_error2(){
  GyroErrorZ=0;
  for(uint8_t i=0;i<200;i++){
    // dmpDataReady();
    GyroErrorZ+=ypr[0];
    delay(10);
  }
  GyroErrorZ/=200;
}
void run_IMUDMP_error(){
  //  GyroErrorZ=0;
  for(uint8_t i=0;i<200;i++){
    dmpDataReady();
    // GyroErrorZ+=ypr[0];
    delay(2);
  }
  // GyroErrorZ/=200;

}
void line_walk(int num)
{
  BackNum(1,direction_90);
  turn(2,direction_90-90);
  StraightNum(num,direction_90-90);
}

void setup() {
      // join I2C bus (I2Cdev library doesn't do this automatically)

        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties




    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println((reinterpret_cast<const __FlashStringHelper *>(
# 1103 "e:\\code\\zkb\\113\\113.ino" 3
                  (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1103 "e:\\code\\zkb\\113\\113.ino"
                  "Initializing I2C devices..."
# 1103 "e:\\code\\zkb\\113\\113.ino" 3
                  ); &__c[0];}))
# 1103 "e:\\code\\zkb\\113\\113.ino"
                  )));
    mpu.initialize();
    // pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println((reinterpret_cast<const __FlashStringHelper *>(
# 1108 "e:\\code\\zkb\\113\\113.ino" 3
                  (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1108 "e:\\code\\zkb\\113\\113.ino"
                  "Testing device connections..."
# 1108 "e:\\code\\zkb\\113\\113.ino" 3
                  ); &__c[0];}))
# 1108 "e:\\code\\zkb\\113\\113.ino"
                  )));
    Serial.println(mpu.testConnection() ? (reinterpret_cast<const __FlashStringHelper *>(
# 1109 "e:\\code\\zkb\\113\\113.ino" 3
                                         (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1109 "e:\\code\\zkb\\113\\113.ino"
                                         "MPU6050 connection successful"
# 1109 "e:\\code\\zkb\\113\\113.ino" 3
                                         ); &__c[0];}))
# 1109 "e:\\code\\zkb\\113\\113.ino"
                                         )) : (reinterpret_cast<const __FlashStringHelper *>(
# 1109 "e:\\code\\zkb\\113\\113.ino" 3
                                                                              (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1109 "e:\\code\\zkb\\113\\113.ino"
                                                                              "MPU6050 connection failed"
# 1109 "e:\\code\\zkb\\113\\113.ino" 3
                                                                              ); &__c[0];}))
# 1109 "e:\\code\\zkb\\113\\113.ino"
                                                                              )));


    // // wait for ready
    // Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    // while (Serial.available() && Serial.read()); // empty buffer
    // while (!Serial.available());                 // wait for data
    // while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println((reinterpret_cast<const __FlashStringHelper *>(
# 1119 "e:\\code\\zkb\\113\\113.ino" 3
                  (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1119 "e:\\code\\zkb\\113\\113.ino"
                  "Initializing DMP..."
# 1119 "e:\\code\\zkb\\113\\113.ino" 3
                  ); &__c[0];}))
# 1119 "e:\\code\\zkb\\113\\113.ino"
                  )));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(0);
    mpu.setYGyroOffset(0);
    mpu.setZGyroOffset(0);
    mpu.setZAccelOffset(1688); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println((reinterpret_cast<const __FlashStringHelper *>(
# 1131 "e:\\code\\zkb\\113\\113.ino" 3
                      (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1131 "e:\\code\\zkb\\113\\113.ino"
                      "Enabling DMP..."
# 1131 "e:\\code\\zkb\\113\\113.ino" 3
                      ); &__c[0];}))
# 1131 "e:\\code\\zkb\\113\\113.ino"
                      )));
        mpu.setDMPEnabled(true);
        // mpu.setIntDMPEnabled(true);
        // mpu.setIntEnabled(true);
        // enable Arduino interrupt detection
        Serial.print((reinterpret_cast<const __FlashStringHelper *>(
# 1136 "e:\\code\\zkb\\113\\113.ino" 3
                    (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1136 "e:\\code\\zkb\\113\\113.ino"
                    "Enabling interrupt detection (Arduino external interrupt "
# 1136 "e:\\code\\zkb\\113\\113.ino" 3
                    ); &__c[0];}))
# 1136 "e:\\code\\zkb\\113\\113.ino"
                    )));

        mpuIntStatus = mpu.getIntStatus();
        Serial.println(mpuIntStatus);
        // set our DMP Ready flag so the main loop() function knows it's okay to use it

        Serial.println((reinterpret_cast<const __FlashStringHelper *>(
# 1142 "e:\\code\\zkb\\113\\113.ino" 3
                      (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1142 "e:\\code\\zkb\\113\\113.ino"
                      "DMP ready! Waiting for first interrupt..."
# 1142 "e:\\code\\zkb\\113\\113.ino" 3
                      ); &__c[0];}))
# 1142 "e:\\code\\zkb\\113\\113.ino"
                      )));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print((reinterpret_cast<const __FlashStringHelper *>(
# 1152 "e:\\code\\zkb\\113\\113.ino" 3
                    (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1152 "e:\\code\\zkb\\113\\113.ino"
                    "DMP Initialization failed (code "
# 1152 "e:\\code\\zkb\\113\\113.ino" 3
                    ); &__c[0];}))
# 1152 "e:\\code\\zkb\\113\\113.ino"
                    )));
        Serial.print(devStatus);
        Serial.println((reinterpret_cast<const __FlashStringHelper *>(
# 1154 "e:\\code\\zkb\\113\\113.ino" 3
                      (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1154 "e:\\code\\zkb\\113\\113.ino"
                      ")"
# 1154 "e:\\code\\zkb\\113\\113.ino" 3
                      ); &__c[0];}))
# 1154 "e:\\code\\zkb\\113\\113.ino"
                      )));
    }


  // put your setup code here, to run once:
  //int fff = 1;
  //TCCR1B =(TCCR1B & 0xF8) | fff;//调整计数器分频，频率调高至31.374KHZ
  //TCCR3B =(TCCR3B & 0xF8) | fff;//调整计数器分频，频率调高至31.374KHZ
  //TCCR4B =(TCCR4B & 0xF8) | fff;//调整计数器分频，频率调高至31.374KHZ 
  //TCCR5B =(TCCR5B & 0xF8) | fff;//调整计数器分频，频率调高至31.374KHZ
  pinMode(AIN1, 0x1);
  pinMode(DIN1, 0x1);
  pinMode(AIN2, 0x1);
  pinMode(DIN2, 0x1);
  pinMode(MagL, 0x1);
  pinMode(2/*A路电机编码器引脚AA，外部中断，中断号0*/, 0x2);
  pinMode(19/*D路电机编码器引脚DA，外部中断，中断号4*/, 0x2);
  pinMode(51/*A路电机编码器引脚AB*/, 0x2);
  pinMode(50/*D路电机编码器引脚DB*/, 0x2);
  pinMode(BANGL, 0x0);
  pinMode(BANGR, 0x0);
  pinMode(LL, 0x0);
  pinMode(LR, 0x0);
  pinMode(RL, 0x0);
  pinMode(RR, 0x0);
  // Serial.begin(9600);  
  delay(300); //延时等待初始化完成 

//  attachInterrupt(5, READ_BANG_L, CHANGE);           //开启外部中断 左边碰撞开关
//  attachInterrupt(1, READ_BANG_R, CHANGE);           //开启外部中断 右边碰撞开关

  // Wire.begin();                      // Initialize comunication
  // Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  // Wire.write(0x6B);                  // Talk to the register 6B
  // Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  // Wire.endTransmission(true);        //end the transmission
  // calculate_IMU_error();
  delay(200);

  calculate_IMUDMP_error();
  Serial.print("err");
  Serial.println(GyroErrorZ);

  calculate_IMUDMP_error();
  Serial.print("err");
  Serial.println(GyroErrorZ);

  run_IMUDMP_error();
  run_IMUDMP_error();
  digitalWrite(MagL, 0x1);
  calculate_IMUDMP_error();
  Serial.print("err");
  Serial.println(GyroErrorZ);

  calculate_IMUDMP_error();
  Serial.print("err");
  Serial.println(GyroErrorZ);

  run_IMUDMP_error();
  run_IMUDMP_error();
  FlexiTimer2::set(1, control); //10毫秒定时中断函数
  FlexiTimer2::start (); //中断使能
  delay(1000);
    calculate_IMUDMP_error2();
  Serial.print("err");
  Serial.println(GyroErrorZ);
    calculate_IMUDMP_error2();
  Serial.print("err");
  Serial.println(GyroErrorZ);
  Serial.print(((2 /* use pin 2 on Arduino Uno & most boards*/) == 2 ? 0 : ((2 /* use pin 2 on Arduino Uno & most boards*/) == 3 ? 1 : ((2 /* use pin 2 on Arduino Uno & most boards*/) >= 18 && (2 /* use pin 2 on Arduino Uno & most boards*/) <= 21 ? 23 - (2 /* use pin 2 on Arduino Uno & most boards*/) : -1))));
  Serial.println((reinterpret_cast<const __FlashStringHelper *>(
# 1224 "e:\\code\\zkb\\113\\113.ino" 3
                (__extension__({static const char __c[] __attribute__((__progmem__)) = (
# 1224 "e:\\code\\zkb\\113\\113.ino"
                ")..."
# 1224 "e:\\code\\zkb\\113\\113.ino" 3
                ); &__c[0];}))
# 1224 "e:\\code\\zkb\\113\\113.ino"
                )));

  // attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
  attachInterrupt(0, READ_ENCODER_A, 1); //开启外部中断 编码器接口A
  attachInterrupt(4, READ_ENCODER_D, 1); //开启外部中断 编码器接口D
  last = yaw;

  // myPIDa.SetMode(AUTOMATIC);
  // myPIDd.SetMode(AUTOMATIC);
//  myPIDa.SetMode(MANUAL);
//  myPIDd.SetMode(MANUAL);
  pinMode(52, 0x1);
  digitalWrite(52, 0x0);//先保证拉低
  pinMode(42, 0x1);
  digitalWrite(42, 0x0);//先保证拉低
  pinMode(27, 0x1);
  digitalWrite(27, 0x0);
   pinMode(29, 0x1);
  digitalWrite(29, 0x0);
   pinMode(48, 0x1);
  digitalWrite(48, 0x0);
}
void loop() {
  // int start=0;
  // char c;
  // while(1){
  //   if(Serial.available()>0)
  //   {
  //     c = Serial.read();
  //     Serial.print("ardu rec ");
  //     Serial.println(c);
  //     if(c=='0')
  //     {
  //       start=1;
  //       break;
  //     }
  //   }
  // }

// Catch(0,2);


int start=1;
  if(start)
  {
  BangSensor();
  WalkSensor();
  sensorValueFS = digitalRead(FS);
  sensorValueBS = digitalRead(BS);
  Serial.print(sensorValueFS);
  Serial.println(sensorValueBS);
  digitalWrite(MagL, 0x1);
  int i=0;
  for(i=0;i<55;i++){
    Straight_light(1,0);
    i++;
  }
   turn(1,90);
   BackNum(2,90);
   turn(2,0);
   direction_0=0;
   StraightNum(7,direction_0);

// ---------------------------------
//    BackNum(1,direction_90);
//    turn(2,direction_90-180);
//     int ai = 0;
//   int flag =0;
//   BangSensor();
//   while(i<7)
//   {
//     BangSensor();
//     if(BangLValue==0||BangRValue==0)
//     {
//       BackNum(1,direction_90-180);
//       turn(2,direction_90-270);
//       while(1)
//       {
//         Straight_light(1,direction_90-270);
//       }
//     }
//     Straight_light(1,direction_90-180);
// //    Serial.print(i);
// //    Serial.print("\n");
//     sensorValueFS = digitalRead(FS);
//     sensorValueBS = digitalRead(BS);
// //    Serial.print(sensorValueFS);
// //    Serial.print(sensorValueBS);
// //    Serial.print("----------\n");
//     if(sensorValueFS == 1||sensorValueBS ==1)
//     {
//       flag =1;
//     }
//     else if(flag == 1)
//     {
//       i++;
//       flag = 0;
//     }
//     Straight_light(1,direction_90-180);
//   }
//    Straight_light(1,direction_90-270);


//   //  -------------------------------
   int n = 0;
   while(n<3)
   {
     line_walk(7);
     n++;
   }
   Set_PWM(0,0);
   delay(20000);




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



Straight_light(0,0);
StraightNum(6,0);

calculate_IMU_error();
MPU6050();
 Serial.print(millis());
 Serial.print("A:");
 Serial.print(Velocity_A);//显示 
 Serial.print(",");
 Serial.print("D:");
 Serial.print(Velocity_D);//显示 
 Serial.print("-------------\n");

  }
}
