#include <Servo.h>
#define INTERACTIONS 50
/***********************/
#include <SPI.h>
#include <Mirf.h>
#include <nRF24L01.h>
#include <MirfHardwareSpiDriver.h>
/***********************/
/*================================================================================*/
enum direction {
  front,
  back,
  left,
  right,
  stop
};
/*================================================================================*/
class car {
    int pin_a;
    int pin_b;
    int pwm_a;
    int pwm_b;
  public:
    void set_pins(int a, int b, int pa, int pb);
    void move(direction dir, int pwm);
    void moveR(direction dir, int pwm);
    void moveL(direction dir, int pwm);
};

void car::set_pins(int a, int b, int pa, int pb) {
  pin_a = a;
  pin_b = b;
  pwm_a = pa;
  pwm_b = pb;
  pinMode(pin_a, OUTPUT);
  pinMode(pin_b, OUTPUT);
}
/*================================================================================*/
void car::move(direction dir, int pwm = 0) {
  if (dir == front) {
    digitalWrite(pin_a, 0);
    digitalWrite(pin_b, 1);
    analogWrite(pwm_a, pwm);
    analogWrite(pwm_b, 255 - pwm);
  }
  else if (dir == back) {
    digitalWrite(pin_a, 1);
    digitalWrite(pin_b, 0);
    analogWrite(pwm_a, 255 - pwm);
    analogWrite(pwm_b, pwm);
  }
  else if (dir == right) {
    digitalWrite(pin_a, 1);
    digitalWrite(pin_b, 1);
    analogWrite(pwm_a, 255 - pwm);
    analogWrite(pwm_b, 255 - pwm);
  }
  else if (dir == left) {
    digitalWrite(pin_a, 0);
    digitalWrite(pin_b, 0);
    analogWrite(pwm_b, pwm);
    analogWrite(pwm_a, pwm);
  }
  else if (dir == stop) {
    digitalWrite(pin_a, 0);
    digitalWrite(pin_b, 0);
    analogWrite(pwm_b, 0);
    analogWrite(pwm_a, 0);
  }
}
void car::moveR(direction dir, int pwm)
{
    if (dir == front) {
    digitalWrite(pin_a, 0);
    analogWrite(pwm_a, pwm);
  }
  if (dir == back){
    digitalWrite(pin_a, 1);
    analogWrite(pwm_a, 255 - pwm);
  }
}
void car::moveL(direction dir, int pwm)
{
  if (dir == front) {
    digitalWrite(pin_b, 1);
    analogWrite(pwm_b, 255 - pwm);
  }
  if (dir == back) {
    digitalWrite(pin_b, 0);
    analogWrite(pwm_b, pwm);
  }
}
/*================================================================================*/
/******************************************************************************/
class pid
{
  public:
    float kp;
    float ki;
    float kd;
    float error;
    float setpoint;
    float input;
    float output;
    float Iterm;
    float outMin;
    float outMax;

    void set_limits(float a, float b);
    float compute(float in);
};
/******************************************************************************/

/******************************************************************************/
void pid::set_limits(float a, float b)
{
  if (a < b)
  {
    outMin = a;
    outMax = b;
  }
  else
  {
    outMin = b;
    outMax = a;
  }
}
/******************************************************************************/

/******************************************************************************/
float pid::compute(float in)
{
  input = in;
  error = setpoint - input;
  Iterm += (ki * error);
  if (Iterm > outMax) Iterm = outMax;
  else if (Iterm < outMin) Iterm = outMin;
  output = kp * error + Iterm;
  if (output > outMax) output = outMax;
  else if (output < outMin) output = outMin;
  return output;
}
/******************************************************************************/
Servo servo_x, servo_y;
int servo_n = 0;
car carro;
int val_pwm = 100;
pid pid_rodas;

void setup() {
  Mirf.spi = &MirfHardwareSpi;
  Mirf.init();
  Mirf.setRADDR((byte *)"carro");
  Mirf.payload = sizeof(unsigned long);
  Mirf.channel = 10;
  Mirf.config();
  
  Serial.begin(9600);
  
  carro.set_pins(2, 3, 5, 6);
  servo_x.attach(10);
  servo_y.attach(11);
  servo_x.write(90);
  servo_y.write(110);
  //Controlzador PID
  pid_rodas.kp = 1;
  pid_rodas.ki = 0.3;
  pid_rodas.set_limits(-255.0, 255.0);
  pid_rodas.setpoint = 90.0;          //Setpoint  a posiço inicial do eixo x.
}

byte _data=0;
bool obj_detected = false;
float out=0;
long time;
byte data[4];

void loop() {
  
  if(Mirf.dataReady()){  
    Mirf.getData(data);
  if(data[2]==0 && data[3]==0)
  {
    carro.moveR(back,data[0]);
    carro.moveL(back,data[1]);
  }
  if(data[2]==1 && data[3]==0)
  {
    carro.moveR(front,data[0]);
    carro.moveL(back,data[1]);
  }
  if(data[2]==0 && data[3]==1)
  {
    carro.moveR(back,data[0]);
    carro.moveL(front,data[1]);
  }
  if(data[2]==1 && data[3]==1)
  {
    carro.moveR(front,data[0]);
    carro.moveL(front,data[1]);
  }
  }
  
  if (Serial.available()) {
    _data = Serial.read();

    if (_data == 200)servo_n = 1;                     //Seleçao para servox
    else if (_data == 201)servo_n = 2;              //Seleçao para servoy
    else if (_data <= 180)                                  //Alinha a camera
    {
      if (servo_n == 1)
        servo_x.write(_data);
      else if (servo_n == 2)
        servo_y.write(140 - _data);
    }
    
    if(!obj_detected)//Pega uma amostra de tempo
    {
      time = millis();
      obj_detected = true;
    }
  }
  
  if (time+1000 >= millis())//Se passar um segundo, alinhar carro
  {
    out = pid_rodas.compute(servo_x.read());
    if(out<0)
  {
    carro.move(left,int(abs(out)));
  }
  else
  {
    carro.move(right,int(out));
  }
  }
  delay(10);

}
