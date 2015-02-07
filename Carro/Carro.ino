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
  if(dir == left){
    digitalWrite(pin_a,0);
    digitalWrite(pin_b,1);
    analogWrite(pwm_a,pwm);
    analogWrite(pwm_b,255-pwm); 
  }
  else if(dir == right){
    digitalWrite(pin_a,1);
    digitalWrite(pin_b,0);
    analogWrite(pwm_a,255-pwm);
    analogWrite(pwm_b,pwm); 
  }
  else if(dir == front){
    digitalWrite(pin_a,1);
    digitalWrite(pin_b,1);
    analogWrite(pwm_a,255-pwm);
    analogWrite(pwm_b,255-pwm); 
  }
  else if(dir == back){
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
car carro;
int val_pwm=100;
void setup(){
  Serial.begin(9600);
  carro.set_pins(2,3,10,11);
  pinMode(13,OUTPUT);
}
void loop(){
  if(Serial.available())
  {
    byte data = Serial.read(); 
    val_pwm =map(data,0,255,-255,255) ;
  }
  //digitalWrite(13,1);
  if(val_pwm>0){
    digitalWrite(13,1);
    carro.move(right,val_pwm);
  }
  else {
    digitalWrite(13,0);
    carro.move(left,abs(val_pwm));
  }
//  delay(1000);
//  digitalWrite(13,0);
//  carro.move(stop);
//  delay(5000);
}
/*================================================================================*/




