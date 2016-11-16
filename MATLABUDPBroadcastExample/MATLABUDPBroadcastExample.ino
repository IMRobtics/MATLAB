// MATLABUDPBroadcastExample.ino
// Copyright 2016 The MathWorks, Inc.

#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

int ledPin = 5;                                    // Connect an LED->330Ohm to pin 5 

int packetSize;
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xAD, 0xDE};
unsigned int broadcastPort = 31416;      
char packetBuffer[UDP_TX_PACKET_MAX_SIZE];  
char  ArduinoAckMessage[] = "Arduino:ACK\r";       // "\r" is the carriage return, i.e., char(13)
char MATLABBlinkItCommand[] = "MATLAB:BlinkIt\r";  
EthernetUDP Udp;
IPAddress broadcastAddress;

void setup()
{
  Serial.begin(9600);
  if (Ethernet.begin(mac)==0){
    Serial.println("Failed to configure Ethernet. Halt.");
    delay(1000);                                  // Allow some time for message to be printed
    abort();
  }
  Serial.print("Arduino's IP Address is: ");
  Serial.println(Ethernet.localIP());
  Udp.begin(broadcastPort);  
  pinMode(ledPin, OUTPUT);      
  digitalWrite(ledPin, LOW);    
}

void loop()
{
  // if there's data available, read a packet
  packetSize = Udp.parsePacket();
  if (packetSize) {
    Serial.print("Message: ");
    Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
    packetBuffer[packetSize] = '\0';
    Serial.println(packetBuffer);
    if (strcmp(packetBuffer, MATLABBlinkItCommand) == 0)
    {
      broadcastAddress = Udp.remoteIP();
      broadcastAddress[3] = 255;
      Udp.beginPacket(broadcastAddress, broadcastPort);
      Udp.write(ArduinoAckMessage);
      Udp.endPacket();
      digitalWrite(ledPin, HIGH);                   // sets the LED on
      delay(1000);                                  // waits for a second
      digitalWrite(ledPin, LOW);                    // sets the LED off
    }
  }
  delay(10);  
}
