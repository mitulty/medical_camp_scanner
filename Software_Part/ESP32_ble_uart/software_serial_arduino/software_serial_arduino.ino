#include <SoftwareSerial.h>

SoftwareSerial SoftSerial(2, 3);//(Rx,Tx)
String dt;
String uart_data;
void setup()
{
  Serial.begin(115200);
  SoftSerial.begin(115200);
}

void loop()
{
            if(SoftSerial.available())
            {
              dt=SoftSerial.readString();
              Serial.print("Data Received:");
              Serial.println(dt);
            }
           else if(Serial.available())
           {
              uart_data=Serial.readString();
              SoftSerial.print(uart_data);  
           }
}
