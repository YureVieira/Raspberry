#include <Servo.h> 

Servo servo_x,servo_y;
int servo_n=0;

void setup(){
  Serial.begin(9600);
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

