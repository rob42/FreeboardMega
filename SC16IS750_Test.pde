/*Basic Sketch to Setup the SC16IS750 and
  test transmitting and revceiving characters
  
  Originally taken from http://inmcm-hdl.googlecode.com/files/SC16IS750_Test.zip
  
UART Setup
-9600bps baudrate
-8 data bits
-1 stop bit
-No Parity
-No RTS,CTS
-Enable FIFO w/o extra features

Connections:
  Arduino <---> SC16IS750 Breakout
  Pin#/Name <---> Pin Name  
  3V  <---> VIN                    
  GND <---> GND 
  GND <---> SDA-VSS
  GND <---> I2C-SPI
  2/NSS_TX  <--->  RX
  3/NSS_RX  <--->  TX
  10/SS  <---> A0-CS
  11/MOSI  <---> A1-SI
  12/MISO  <---> NC-S0
  13/SCK  <--->  SCL-SCK

Some bits and pieces of code borrowed from WiFly library: Copyright (c) 2010 SparkFun Electronics
*/
#include <SPI.h>
#include <NewSoftSerial.h>  //Need NSS, Available Here: http://arduiniana.org/libraries/newsoftserial/

#define RXPIN 3
#define TXPIN 2
NewSoftSerial mySerial(RXPIN, TXPIN);

const int chipSelectPin = 10;  //Also known as Slave Select (SS)
const byte Write = B00000000;
const byte Read =  B10000000;

//Register Mapping
const byte XHR = 0x00;
const byte FCR = 0x02;
const byte LCR = 0x03;
const byte MCR = 0x04;
const byte LSR = 0x05;
const byte MSR = 0x06;
const byte TXLVL = 0x8;
const byte RXLVL = 0x9;
const byte DLL = 0x00;
const byte DLH = 0x01;

byte work = 0;
char testStrng[] = "Goodbye World!";
int i;

void setup() {
  //Start Serial Units on Arduino
  Serial.begin(115200);
  mySerial.begin(9600);
  
  // start the SPI library:
  SPI.begin();
  
  //Data is transmitted and received MSB first
  SPI.setBitOrder(MSBFIRST);  
  
  //SPI interface will run at 1MHz if 8MHz chip or 2Mhz if 16Mhz
  SPI.setClockDivider(SPI_CLOCK_DIV8); 
  
  //Data is clocked on the rising edge and clock is low when inactive
  SPI.setDataMode(SPI_MODE0);
  
  // initalize chip select pin:
  pinMode(chipSelectPin, OUTPUT);  
  //Clear Chip Select
  digitalWrite(chipSelectPin,HIGH);
  
  
  delay(30);
  Serial.println("Setuping Up SC16IS750:");
  Serial.println("9600bps, 8 Data bits, 1 Stop bit, No Parity");
  
  //Setup SPI/UART Function
  configUARTregs();
  
  delay(2000);
  Serial.println("Reading Back Config Registers");
  Serial.println("Expected Value in ()");  
  Serial.print("FIFO Control Register (C1): ");
  Serial.println(readRegister(FCR),HEX); 
  Serial.print("Line Control Register(3): ");
  Serial.println(readRegister(LCR),HEX); 
  Serial.print("Modem Control Register(0): ");
  Serial.println(readRegister(MCR),HEX);
  delay(2000);
 }

void loop(){
  //First Test Will Use NewSoftSerial to write characters to the RX pin
  // of the SC16IS750. Then it will read the number of characters in the 
  // RX FIFO. Finally it will read out those characters via SPI
  Serial.println("Testing Setup");
  Serial.println("Transmitting \"Hello, World?\" (13 chars) to RX of UART");
  mySerial.print("Hello, world?");
  delay(100);
  Serial.print("RX FIFO Level: ");
  work = readRegister(RXLVL);
  Serial.println(work,DEC);
  Serial.println("Reading back contents of UART FIFO");
  
  while (0 < work) {
    Serial.print(readRegister(XHR));
    work--;
  } 
  delay(3000);

  //Second Test sends characters to the SC16IS750 TX FIFO via SPI and then recieves them
  // via the NewSoftSerial UART.
  Serial.println();
  Serial.println("Writing \"Goodbye World!\" to UART TX");
  Serial.println("and printing received result");
  for (i = 0; i < sizeof(testStrng) - 1; i++){
    writeRegister(XHR,testStrng[i]);
  }
  while (mySerial.available()) {
    Serial.print((char)mySerial.read());
  }      
  Serial.println();
  Serial.println("Test Complete!");
  while(1){
  }
   
   
}


//Config the SC16IS740 UART  
void configUARTregs() {
    
  //Line Control Register: Enable Writing DLH & DLL
  //& set no Parity, 1 stop bit, and 8 bit word length
  writeRegister(LCR,B10000011);
  
  //Division registers DLL & DLH
  // Write '96' to get 9600 baud rate
  //Assumes you have the version with the ~14MHz crystal
  // (16x9600 = 153600 = 14.7456Mhz/96)
  writeRegister(DLL,96);
  writeRegister(DLH,00); 
  
  //Line Control Register: Disnable Writing DLH & DLL
  //Same setup 
  writeRegister(LCR,B00000011);
  
  //Modem Control Register
  //Normal Operating Mode
   writeRegister(MCR,B00000000);
 
  //FIFO Control Register: Enable the FIFO and no other features
  writeRegister(FCR,B00000111);  
}   
  
  
  
  
  
  
void writeRegister(byte thisRegister, byte thisValue) {

  // SC16IS740 expects a R/W  bit first, followed by the 4 bit
  // register address of the byte.
  // So shift the bits left by three bits:
  thisRegister = thisRegister << 3;
  // now combine the register address and the command into one byte:
  byte dataToSend = thisRegister | Write;

  // take the chip select low to select the device:
  digitalWrite(chipSelectPin, LOW);

  SPI.transfer(dataToSend); //Send register location
  SPI.transfer(thisValue);  //Send value to record into register

  // take the chip select high to de-select:
  digitalWrite(chipSelectPin, HIGH);

}

byte readRegister(byte thisRegister) {
  byte readback;
  
  // SC16IS740 expects a R/W  bit first, followed by the 4 bit
  // register address of the byte.
  // So shift the bits left by three bits:
  thisRegister = thisRegister << 3;
  // now combine the register address and the command into one byte:
  byte dataToSend = thisRegister | Read;

  // take the chip select low to select the device:
  digitalWrite(chipSelectPin, LOW);
  
  SPI.transfer(dataToSend); //Send register location
  readback = SPI.transfer(0);  //Get Value from register
  
  digitalWrite(chipSelectPin, HIGH);
  return(readback);
} 
  
  
  
  
  
  
  
  

    
  
  
