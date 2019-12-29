#include <VirtualWire.h>
struct Data {
  int x;
  int y;
  int val;
  };
  struct Data *DataIn;
void setup()
{
    pinMode(2,OUTPUT);
    vw_set_ptt_inverted(true); // Required for DR3100
    vw_set_rx_pin(12);
    vw_setup(2000);  // Bits per sec
    Serial.begin(9600);
    vw_rx_start();       // Start the receiver PLL running
}

int temp=1000;
void loop()
{
    uint8_t buf[VW_MAX_MESSAGE_LEN];
    uint8_t buflen = VW_MAX_MESSAGE_LEN;

    if (vw_get_message(buf, &buflen)) // Non-blocking
    {
      DataIn=(struct Data*)buf;
      
      int xx=(DataIn->x);
      if(xx<520 &&xx>490)
         xx=512;
       int t= constrain(map(xx,0,1024,-255,255),-255,+255);
       temp+=t/32;
       temp=constrain(temp,1000,1500);
      int pushbutton=(DataIn->val);
      if(pushbutton==0)
        digitalWrite(2,HIGH);
      else digitalWrite(2,LOW);
      Serial.println(pushbutton);
               
     }
 }
