#include <VirtualWire.h>
int xaxis=A0;
int yaxis=A1;
const int but_Pin=2;
struct{
  int x;
  int y;
  int val;
  }Data;
void setup()
{
  pinMode(xaxis,INPUT);
  pinMode(yaxis,INPUT);
  pinMode(but_Pin,INPUT_PULLUP);
  vw_set_tx_pin(12);
  vw_setup(2000);// speed of data transfer Kbps
  
}
void loop()
{
  Data.x=analogRead(xaxis);
  Data.y=analogRead(yaxis);
  Data.val=digitalRead(but_Pin);
  vw_send((uint8_t *)&Data, sizeof(Data));
  vw_wait_tx(); // Wait until the whole message is gone
  }
