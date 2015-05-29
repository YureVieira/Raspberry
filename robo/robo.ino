
#include <Servo.h>
#define PIN_DEBUG A3
ISR(TIM2_OVER_vect){
}
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
  void move(direction dir, int pwmR ,int pwmL);
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
void car::move(direction dir, int pwmR = 0,int pwmL=0) {
  if (dir == right) {
    digitalWrite(pin_a, 0);
    digitalWrite(pin_b, 1);
    analogWrite(pwm_a, pwmR);
    analogWrite(pwm_b, 255- pwmL);
  }
  else if (dir == left) {
    digitalWrite(pin_a, 1);
    digitalWrite(pin_b, 0);
    analogWrite(pwm_a, 255 - pwmR);
    analogWrite(pwm_b, pwmL);
  }
  else if (dir == front) {
    digitalWrite(pin_a, 1);
    digitalWrite(pin_b, 1);
    analogWrite(pwm_a, 255 - pwmR);
    analogWrite(pwm_b, 255 - pwmL);
  }
  else if (dir == back) {
    digitalWrite(pin_a, 0);
    digitalWrite(pin_b, 0);
    analogWrite(pwm_b, pwmR);
    analogWrite(pwm_a, pwmL);
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
int fnc = 0;
car carro;
int val_pwm = 100;
pid pid_rodas;

void setup() {
  Serial.begin(9600);
  pinMode(PIN_DEBUG,INPUT_PULLUP);
  carro.set_pins(2, 3, 5, 6);
  servo_x.attach(10);
  servo_y.attach(11);
  servo_x.write(90);
  servo_y.write(110);
  //Controlzador PID
  //  pid_rodas.kp = 1;
  //  pid_rodas.ki = 0.3;
  //  pid_rodas.set_limits(-255.0, 255.0);
  //  pid_rodas.setpoint = 90.0;          //Setpoint  a posiço inicial do eixo x.
}

byte _data=0;
byte target_dist=0;
boolean target=false;

int pwm_debug;
void loop() {

  if (Serial.available()) {
    _data = Serial.read();

    if (_data == 200)fnc = 1;                     //Seleçao para servox
    if (_data == 201)fnc = 2;              //Seleçao para servoy
    if (_data == 202)fnc = 3;
    if (_data <= 180)                                  //Alinha a camera
    {
      if (fnc == 1)
        servo_x.write(_data);
      if (fnc == 2)
        servo_y.write(140 - _data);
      if(fnc == 3){
        target_dist = _data;
        target = true;
      }
    }
  }

  if (digitalRead(PIN_DEBUG)==0)
  {
    if(target)
    if(target_dist == 1)
    {
      carro.move(front,255,255);//Aproximar
    }
    if(target_dist == 2)
  {
    carro.move(back,255,255);//Afastar
  }
  }
  delay(10);
}



