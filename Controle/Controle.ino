#include <SPI.h>
#include <Mirf.h>
#include <nRF24L01.h>
#include <MirfHardwareSpiDriver.h>

void setup(){
  Serial.begin(9600);
  Mirf.spi = &MirfHardwareSpi;
  Mirf.init();
  Mirf.setRADDR((byte *)"contr");
  Mirf.setTADDR((byte *)"carro");
  Mirf.payload = sizeof(unsigned long);
  Mirf.channel = 10;
  Mirf.config();
  byte init[] = {0,0,0,0};
  Mirf.send((byte *)&init);
  while(Mirf.isSending());
}
void loop(){
  byte analog[4];
  int a0 = analogRead(A0);
  int a1 = analogRead(A1);
  a0 = map(a0, 0, 1023, -255, 255);
  a1 = map(a1, 0, 1023, -255, 255);
  /********************************/  
  if(a0>0){
    analog[0]=byte(a0);
    analog[2]=1;
  }
  else
  {
    analog[0]=byte(abs(a0));
    analog[2]=0;
  }
  /********************************/
  if(a1>0){
    analog[1]=byte(a1);
    analog[3]=1;
  }
  else
  {
    analog[1]=byte(abs(a1));
    analog[3]=0;
  }
  /********************************/
  Mirf.send((byte *)&analog);
  while(Mirf.isSending());
  delay(10);
}
