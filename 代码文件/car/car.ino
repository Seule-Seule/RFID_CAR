/****基于FaceID的智能小车*******/

#include<Wire.h>
#include<math.h>
#include <Servo.h>
#include <IRremote.h>

//舵机部分
Servo myservo;  //定义舵机端口
int ServoPin = 10;
int pos = 0;    // 用于存储舵机位置的变量

//电机部分
const int DIR1_RIGHT = 12;  //2路直流电机方向控制引脚
const int DIR2_RIGHT = 11;
const int DIR1_LEFT = 8;
const int DIR2_LEFT = 9;

const int PWM_LEFT = 6;  //2路电机调速引脚
const int PWM_RIGHT = 5;

int MotorAdjustmengPin = A1;
#define MOTORADJUSTMENT

//红外遥控功能
int RECV_PIN = 2;

#define IR_CODE_FORWARD 0x00FF02FD
#define IR_CODE_BACKWARDS 0x00FF9867
#define IR_CODE_TURN_LEFT 0x00FFE01F
#define IR_CODE_TURN_RIGHT 0x00FF906F
#define IR_CODE_CONTINUE -1

IRrecv irrecv(RECV_PIN);
decode_results results;

boolean isActing = false;
long timer;
const long TIME_OUT = 150;

// 魔术手功能
const int trig = 4;    // 触发信号
const int echo = 7;    // 反馈信号

//避障功能
#define S_LIMIT_1 20 //距离判断参数预设值

//摇头避障
#define S_LIMIT_3 8 //距离判断参数预设值
#define S_LIMIT_2 100
long runtime;
const long TIME_OUT_shake = 5000;
int Bit_Front, Bit_Left, Bit_Right;

//循迹功能
#define ADDR_1 0x20 //定义器件地址，地址通过硬件上A0、A1、A2位置短路选择，最大提供8个地址

//功能菜单
#define IR_CODE_Remote           0xFF30CF   //按键1-红外功能
#define IR_CODE_hand             0xFF18E7  //按键2-魔术手功能
#define IR_CODE_Obstacle         0xFF7A85 //按键3-避障功能
#define IR_CODE_Shake            0xFF10EF  //按键4-摇头避障功能
#define IR_CODE_runLineFollow    0xFF38C7  //按键5-循迹功能
#define IR_CODE_return           0xFFC23D  //按键返回-返回功能

//RFID
#define RFID  3

void setup() {
  Serial.begin(9600); //调试使用，可以去掉

  // 舵机部分
  myservo.attach(ServoPin);  // 舵机控制信号引脚

  //电机部分
  pinMode(DIR1_RIGHT, OUTPUT);
  pinMode(DIR2_RIGHT, OUTPUT);
  pinMode(DIR1_LEFT, OUTPUT);
  pinMode(DIR2_LEFT, OUTPUT);

  pinMode(PWM_LEFT, OUTPUT);
  pinMode(PWM_RIGHT, OUTPUT);

  //红外部分
  irrecv.enableIRIn(); //开启红外接收

  //魔术手
  pinMode(echo, INPUT); //触发端口设置为输出，反馈端口设置为输入
  pinMode(trig, OUTPUT);

  //循迹
  Wire.begin();        // join i2c bus (address optional for master)

  pinMode(RFID, INPUT);
}


//舵机控制函数
void steeringInit() { //舵机初始化子函数
  myservo.write(90);//舵机停在90度位置1S
  delay(1000);
  myservo.write(0);//舵机停在0度位置持续1S
  delay(1000);
  myservo.write(180);//舵机停在180度位置持续1S
  delay(1000);
  myservo.write(90);//舵机停在90度位置一直保持，用于调整设备位置
}

//电机控制子程序
void motorsWrite(int speedLeft, int speedRight) {
  float motorAdjustment = MotorAdjustment();
  if (motorAdjustment < 0) {
    speedRight *= (1 + motorAdjustment);
  }
  else {
    speedLeft *= (1 - motorAdjustment);
  }
  if (speedRight > 0) //如果PWM数值小于0，表示运行方向变化
  {
    digitalWrite(DIR1_RIGHT, 0);
    digitalWrite(DIR2_RIGHT, 1);
  }
  else
  {
    digitalWrite(DIR1_RIGHT, 1);
    digitalWrite(DIR2_RIGHT, 0);
  }
  analogWrite(PWM_RIGHT, abs(speedRight));

  if (speedLeft > 0)
  {
    digitalWrite(DIR1_LEFT, 0);
    digitalWrite(DIR2_LEFT, 1);
  }
  else
  {
    digitalWrite(DIR1_LEFT, 1);
    digitalWrite(DIR2_LEFT, 0);
  }
  analogWrite(PWM_LEFT, abs(speedLeft));
}
void stopMotor() {    //停止电机
  motorsWrite(0, 0);
}
void changeAction(float directionLeft, float directionRight) { //改变运行状态
  motorsWrite(255 * directionLeft, 255 * directionRight);
  timer = millis();
  isActing = true;
}
float  MotorAdjustment() { //轮子校正程序，当2个电机在同等条件下转速不同时需要校正，使小车可以走直线，如果没有预定义校正功能，返回0，表示没有校正
#ifdef MOTORADJUSTMENT
  float motorAdjustment = map(analogRead(MotorAdjustmengPin), 0, 1023, -30, 30) / 100.0;
  return motorAdjustment;
#else
  return 0;
#endif
}

//红外遥控功能
void processResult(unsigned long res) { //红外响应函数
  switch (res)
  {
    case IR_CODE_FORWARD:
      changeAction(0.6, 0.6); //前进
      break;
    case IR_CODE_BACKWARDS:
      changeAction(-0.6, -0.6); //后退
      break;
    case IR_CODE_TURN_LEFT:
      changeAction(-0.5, 0.5); //左转
      break;
    case IR_CODE_TURN_RIGHT:
      changeAction(0.5, -0.5); //右转
      break;
    case IR_CODE_CONTINUE:
      timer = millis(); //执行最后一次动作，然后清零时间
      break;
  }
}
void Remote() { //遥控功能子函数
  while (1) {
    if (irrecv.decode(&results))
    {
      Serial.println(results.value, HEX);
      if (results.value == IR_CODE_return) {
        return;
      }
      processResult(results.value);
      irrecv.resume(); // 接收下一个数值
    }
    //运行时间，如果没有接到其他指令，停止运行
    if (isActing && (millis() - timer >= TIME_OUT))
    {
      stopMotor();
      isActing = false;
    }
  }
}

// 魔术手功能
int ranging() { //测距
  long IntervalTime = 0; //定义一个时间变量
  digitalWrite(trig, 1);//置高电平
  delayMicroseconds(15);//延时15us
  digitalWrite(trig, 0);//设为低电平
  IntervalTime = pulseIn(echo, HIGH); //用自带的函数采样反馈的高电平的宽度，单位us
  int S = IntervalTime / 58;
  return S;
}
void hand() { //魔术手功能函数
  while (1) {
    if (irrecv.decode(&results))
    {
      Serial.println(results.value, HEX);
      if (results.value == IR_CODE_return) {
        stopMotor();
        isActing = false;
        return;
      }
      irrecv.resume(); // 接收下一个数值
    }
    myservo.write(90); //根据实际情况调整数值，使得超声波对应正前方
    int S = ranging(); //获取距离
    Serial.println(S);//通过串口输出距离数值
    int wheelSpeed;
    S = constrain(S, 0, 30);
    wheelSpeed = map(S, 3, 30, -200, 200);
    motorsWrite(wheelSpeed , wheelSpeed);
    delay(10);
  }
}

//避障功能
void motorsWritePct(int speedLeftPct, int speedRightPct) { //小车百分比输入
  int16_t speedLeft = 255 * speedLeftPct / 100.0;
  int16_t speedRight = 255 * speedRightPct / 100.0;
  motorsWrite(speedLeft, speedRight);
}
void Obstacle() {
  while (1) {
    if (irrecv.decode(&results))
    {
      Serial.println(results.value, HEX);
      if (results.value == IR_CODE_return) {
        stopMotor();
        isActing = false;
        return;
      }
      irrecv.resume(); // 接收下一个数值
    }
    myservo.write(90); //根据实际情况调整数值，使得超声波对应正前方
    int wheelSpeed, S; //定义轮速，测距距离
    S = ranging(); //获取距离
    Serial.println(S);//打印数据
    if (S <= S_LIMIT_1) {
      motorsWritePct(50, -50); //右转
    }
    else
      motorsWritePct(60, 60); //前进
    delay(400);//延时可以改变每次转弯的角度，时间越长，转弯角度越大。
  }
}

//摇头避障功能
//直行，输入0-100数字，表示百分比
void run(int pct)
{
  motorsWritePct(pct, pct);
}
//右转，并带有时间参数，调整时间可以调节转弯角度
void turnRight(int runtime)
{
  motorsWritePct(100, 0);
  delay(runtime);
}
//左转，并带有时间参数，调整时间可以调节转弯角度
void turnLeft(int runtime)
{
  motorsWritePct(0, 100);
  delay(runtime);
}
//后退，并带有时间参数，调整时间可以调节转弯角度，带转弯后退。
void back(int runtime)
{
  motorsWritePct(-80, -50);
  delay(runtime);
}
void Shake() { //摇头避障函数
  int S, SL, SR;
  int TimerFlag = 0;
  myservo.write(90);//正前方位置
  while (1)
  {
    if (irrecv.decode(&results))
    {
      Serial.println(results.value, HEX);
      if (results.value == IR_CODE_return) {
        stopMotor();
        isActing = false;
        return;
      }
      irrecv.resume(); // 接收下一个数值
    }
    if ((millis() - timer >= TIME_OUT_shake)) //定时到更新标志位
    {
      TimerFlag = 1;
    }
    myservo.write(90);//正前方位置
    delay(300);
    S =  ranging();
    Serial.println(S);//打印数据
    if (S <= S_LIMIT_3)
      back(300);
    if (S <= S_LIMIT_1 || TimerFlag == 1) //测量距离小于预设距离或者定时到时
    {
      timer = millis(); //标志位清零，并准备重新计时
      TimerFlag = 0; //清零标志位
      stopMotor();   //停止小车
      myservo.write(45); //超声波探头右转
      delay(300);        //延时稳定后测量
      int SR = ranging();//获取右边的障碍距离
      myservo.write(145); //超声波探头左转
      delay(300);      //延时
      int SL = ranging();//获取左边的障碍距离
      myservo.write(90); //探头返回中间位置
      delay(300);      //延时
      int SM  = ranging();//获取距离
      if (SL > S_LIMIT_1 && SR > S_LIMIT_1 && SM < S_LIMIT_1) //前方障碍，左右无障碍，右转
        turnRight(100);
      else if (SL <= S_LIMIT_1 && SR > S_LIMIT_1) //左障碍，右转
        turnRight(100);
      else if (SL > S_LIMIT_1 && SR <= S_LIMIT_1) //右障碍，左转
        turnLeft(100);
      else if (SL <= S_LIMIT_1 && SR <= S_LIMIT_1) //左右都有障碍，转弯后退
        back(300);
    }
    else
      run(60); //前方无障碍，前行
  }
}

//循迹程序，包含PID基础程序
void runLineFollow() {
  while (1) {
    //如果弯道的转弯半径比较小，需要降低初始速度，增加KP的数值，这样整体运行的速度会降低，但是可以准确过弯道。反之调大初始速度可以增加整体运行速度，适合大弯道。
    //不同大小和速度的车这些参数都需要重新调整。车的转弯半径和弯道的半径都是最终影响运行的重要因素。
    int robotSpeed = 45; //初始速度
    int KP = 30;    //比例系数
    int KI = 0;     //积分系数 这里不使用
    int KD = 11;     //微分系数
    static int error, last_error; //定义本次误差、上次误差
    error = sensor(); //读取误差值
    // Serial.println(error);
    if (irrecv.decode(&results))
    {
      Serial.println(results.value, HEX);
      if (results.value == IR_CODE_return) {
        stopMotor();
        isActing = false;
        return;
      }
      irrecv.resume(); // 接收下一个数值
    }
    if (error < 100) { //如果误差值小于限定数值，进行PID调节
      error = constrain(error, -100, 100); //限制误差最小和最大数值


      int vel = (error * KP) / 10 + (error - last_error) * KD; //简化PD公式

      last_error = error; //保存误差

      int motor_left = constrain((robotSpeed - vel), -100, 100); //把（误差增量+初始值）赋值给左右轮，这里用百分比，最大100%
      int motor_right = constrain((robotSpeed + vel), -100, 100);
      Serial.print(motor_left);
      Serial.print(",");
      Serial.println(motor_right);
      motorsWritePct(motor_left, motor_right); //把对应的数值变换成PWM输出
      delay(1); //循环采样时间
    }
    else
    {
      stopMotor();//如果超出范围，停止小车
    }
  }
}
int sensor() { //传感器采样函数
  int ERROR_PCT = 5; //定义误差最小单位，这里用百分比表示
  //int error;//定义误差
  int num;  //定义临时变量
  num = 0x0F & pcf8574Read(ADDR_1);
  //Serial.println(num,BIN);
  switch (num) //判断这个字节数据并作出不同反应
  {
    case 0x01:
      return (-ERROR_PCT << 2); //偏离中间位置越大，得到的误差值越大
    case 0x03:
      return (-ERROR_PCT << 1);
    case 0x02:
      return (-ERROR_PCT);
    case 0x06:
      return 0;           //中间位置，表示没有误差
    case 0x04:
      return (ERROR_PCT);
    case 0x0C:
      return (ERROR_PCT << 1);
    case 0x08:
      return (ERROR_PCT << 2);
    case 0x0F:      //传感器组完全超出黑线，或者全部在黑线上，误差为100%
    case 0x00:
      return 100;
  }
}
void pcf8574Write( char _Addr, char _data) { //写入IO口数据
  Wire.beginTransmission(_Addr);
  Wire.write(_data);
  Wire.endTransmission();
}
unsigned char pcf8574Read(char _Addr) { //读取IO口数据
  unsigned char c;
  pcf8574Write(_Addr, 0xff);
  Wire.requestFrom(_Addr, 1);
  while (Wire.available())
  {
    c = Wire.read();
  }
  Wire.endTransmission();
  return c;
}


void loop() {
  steeringInit();  //舵机初始化
   while (1){
    Serial.println(digitalRead(RFID));
    if (digitalRead(RFID)){
      Serial.println("RFID HIGH");
    if ((irrecv.decode(&results)))
    {
      Serial.println(results.value, HEX);
      switch (results.value)
      {
        case IR_CODE_Remote:
          Remote();        //红外功能
          break;
        case IR_CODE_hand:
          hand();          //魔术手功能
          break;
        case IR_CODE_Obstacle:
          Obstacle();      //避障功能
          break;
        case IR_CODE_Shake:
          Shake();         //摇头避障
          break;
        case IR_CODE_runLineFollow:
          runLineFollow();         //循迹功能
          break;
        case IR_CODE_CONTINUE:
          timer = millis(); //执行最后一次动作，然后清零时间
          break;
      }
      irrecv.resume(); // 接收下一个数值
    }
    }
    //运行时间，如果没有接到其他指令，停止运行
    if (isActing && (millis() - timer >= TIME_OUT))
    {
      stopMotor();
      isActing = false;
    }
  }
}
