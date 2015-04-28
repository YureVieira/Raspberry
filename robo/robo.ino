#include <Servo.h> 
/*================================================================================*/
enum direction{
  front,
  back,
  left,
  right,
  stop
};
/*================================================================================*/
class car{
  int pin_a;
  int pin_b;
  int pwm_a;
  int pwm_b;
public:
  void set_pins(int a,int b,int pa,int pb);
  void move(direction dir,int pwm);
};

void car::set_pins(int a,int b,int pa,int pb){
  pin_a=a;
  pin_b=b;
  pwm_a=pa;
  pwm_b=pb;
  pinMode(pin_a,OUTPUT);
  pinMode(pin_b,OUTPUT);
}
/*================================================================================*/
void car::move(direction dir,int pwm=0){
  if(dir == front){
    digitalWrite(pin_a,0);
    digitalWrite(pin_b,1);
    analogWrite(pwm_a,pwm);
    analogWrite(pwm_b,255-pwm); 
  }
  else if(dir == back){
    digitalWrite(pin_a,1);
    digitalWrite(pin_b,0);
    analogWrite(pwm_a,255-pwm);
    analogWrite(pwm_b,pwm); 
  }
  else if(dir == right){
    digitalWrite(pin_a,1);
    digitalWrite(pin_b,1);
    analogWrite(pwm_a,255-pwm);
    analogWrite(pwm_b,255-pwm); 
  }
  else if(dir == left){
    digitalWrite(pin_a,0);
    digitalWrite(pin_b,0);
    analogWrite(pwm_b,pwm);
    analogWrite(pwm_a,pwm); 
  }
  else if(dir == stop){
    digitalWrite(pin_a,0);
    digitalWrite(pin_b,0);
    analogWrite(pwm_b,0);
    analogWrite(pwm_a,0); 
  }
}
/*================================================================================*/
Servo servo_x,servo_y;
int servo_n=0;
car carro;
int val_pwm=100;

void setup(){
  Serial.begin(9600);
  carro.set_pins(2,3,5,6);
  servo_x.attach(10); 
  servo_y.attach(11); 
  servo_x.write(90);
  servo_y.write(110);
}
byte data;
void loop(){
  if(Serial.available()){
    data = Serial.read();
    
    if(data == 200)servo_n = 1;        //Seleçao para servox
    else if(data == 201)servo_n = 2;  //Seleçao para servoy    
    else if(data <=180)
    {
      if(servo_n == 1)
        servo_x.write(data);
      else if(servo_n == 2)
        servo_y.write(140-data);
    }
  }
  delay(10);
}

