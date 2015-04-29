#include <Servo.h>
#define INTERACTIONS 50
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

byte data=0, count=0;
bool obj_detected = false;
float out=0;

void loop() {
  if (Serial.available()) {
    data = Serial.read();

    if (data == 200)servo_n = 1;                     //Seleçao para servox
    else if (data == 201)servo_n = 2;              //Seleçao para servoy
    else if (data == 202) //Sinaliza que nao ha objetos viziveis
    {
      obj_detected = false;
      count = 0;
    }
    else if (data == 203)obj_detected = true;  //Sinaliza que um objeto foi detectado
    else if (data <= 180)                                  //Alinha a camera
    {
      if (servo_n == 1)
        servo_x.write(data);
      else if (servo_n == 2)
        servo_y.write(140 - data);
    }
    
  count++;
  count = min(INTERACTIONS,count);
  }
  
  if (count >= INTERACTIONS && obj_detected)//Alinhar carro
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
