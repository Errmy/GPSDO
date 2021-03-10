#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <NeoSWSerial.h>
#include <TinyGPS++.h>

#define GPS_RX 3
#define GPS_TX 4
#define GPS_BAUD 9600

#define LockDetection A0 // PLL lock detection
#define Vosc A1          // Oscillator Voltage

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
 
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// GPS Software Serial connection
TinyGPSPlus gps;
NeoSWSerial gpsSerial(GPS_RX, GPS_TX);

uint8_t numofsatellites = 0;
byte gps_set_sucess = 0 ;
void setup() {
  pinMode(LockDetection, INPUT_PULLUP);
  //Serial.begin(9600);
  
  // UBX code for 100kHz TP5
  uint8_t ublox_init[] = {

  0xB5, 0x62, // header
  0x06, 0x31, // time pulse get/set
  0x20,  // lenght 32
  0x00, // tpIdx time pulse selection = 0 = timepulse, 1 = timepulse2  (U1 Char)
  0x00,  // reserved0 U1
  0x01, 0x00, // reserved1 U2
  0x00, 0x32, // antCableDelay ns I2
  0x00, 0x00, // rf group delay I2
  0x00, 0x01, 0x00, 0x00, // freqPeriod U4
  0x00, 0xA0, 0x86, 0x01, // freqPeriodLoc U4
  0x00, 0x00, 0x00, 0x00, // pulselenRatio U4
  0x80, 0x00, 0x00, 0x00, // pulselenRatio lock U4
  0x80, 0x00, 0x00, 0x00, // userConfigDelay ns I4
  0x00, 0xEF, 0x00, 0x00, // flags - page 135 u-blox 7 Receiver Description Including Protocol Specification V14.pdf X4
  0x00, 0xA1, 0xBA
};



  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
   // Serial.println(F("SSD1306 allocation failed"));
   // for(;;);
  }
  delay(1000);
  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  // Display static text
  display.println("GPSDO by DM4RK v1");
  //char buf[20];
  //sprintf(buf, "Satellites: %d.", numofsatellites);
  //display.println(buf);
  display.println("booting...");
  
  delay(500);
  display.print("initializing GPS...");
  
  gpsSerial.begin(GPS_BAUD);
  // actual u-blox 6m programing 
//  while(!gps_set_sucess)
//  {
    sendUBX(ublox_init, sizeof(ublox_init)/sizeof(uint8_t));
//    gps_set_sucess=getUBX_ACK(ublox_init);
//  }
//  gps_set_sucess=0;
  /*
   for(int i = 0; i < sizeof(ublox_init); i++) {                       
    gpsSerial.write( pgm_read_byte(ublox_init+i) );
    delay(5); // simulating a 38400baud pace (or less), otherwise commands are not accepted by the device.
  }
// ends here  */
  display.println("done");
  display.display(); 
}

void loop() {
  char buf[20];
  float Voscvalue = 0.0F;
  while (gpsSerial.available() > 0)
  {
    gps.encode(gpsSerial.read());
  }
  if (gps.satellites.isUpdated())
    {
      numofsatellites = gps.satellites.value();
      display.fillRect(0,8,128,32,BLACK);
      display.setCursor(0,8);
      sprintf(buf, "Satellites: %d.", numofsatellites);
      display.println(buf);
      gps.location.isValid()?sprintf(buf, "GPS locked."):sprintf(buf, "GPS not locked!");
      display.println(buf);
    }
    display.fillRect(0,24,128,48,BLACK);
    display.setCursor(0,24);
    digitalRead(LockDetection)?sprintf(buf, "PLL locked."):sprintf(buf, "PLL unlocked.");
    display.println(buf);
    Voscvalue =  (float)analogRead(Vosc) * (5.0 / 1023.0);
    sprintf(buf, "Vosc: %d.%02dV", (int)Voscvalue, (int)(Voscvalue*100)%100);
    //dtostrf(Vosc, 2, 2, &buf[strlen(buf)]);
    display.println(buf);
    display.display();

}
void sendUBX(uint8_t *MSG, uint8_t len) {
  for(int i=0; i<len; i++) {
    gpsSerial.write(MSG[i]);
    //Serial.print(MSG[i], HEX);
  }
  gpsSerial.println();
}
/*
// Calculate expected UBX ACK packet and parse UBX response from GPS
boolean getUBX_ACK(uint8_t *MSG) {
  uint8_t b;
  uint8_t ackByteID = 0;
  uint8_t ackPacket[10];
  unsigned long startTime = millis();
  display.print(" * Reading ACK response: ");
 
  // Construct the expected ACK packet    
  ackPacket[0] = 0xB5;  // header
  ackPacket[1] = 0x62;  // header
  ackPacket[2] = 0x05;  // class
  ackPacket[3] = 0x01;  // id
  ackPacket[4] = 0x02;  // length
  ackPacket[5] = 0x00;
  ackPacket[6] = MSG[2];  // ACK class
  ackPacket[7] = MSG[3];  // ACK id
  ackPacket[8] = 0;   // CK_A
  ackPacket[9] = 0;   // CK_B
 
  // Calculate the checksums
  for (uint8_t i=2; i<8; i++) {
    ackPacket[8] = ackPacket[8] + ackPacket[i];
    ackPacket[9] = ackPacket[9] + ackPacket[8];
  }
 
  while (1) {
 
    // Test for success
    if (ackByteID > 9) {
      // All packets in order!
      display.println(" (SUCCESS!)");
      return true;
    }
 
    // Timeout if no valid response in 3 seconds
    if (millis() - startTime > 3000) { 
      display.println(" (FAILED!)");
      return false;
    }
 
    // Make sure data is available to read
    if (gpsSerial.available()) {
      b = gpsSerial.read();
 
      // Check that bytes arrive in sequence as per expected ACK packet
      if (b == ackPacket[ackByteID]) { 
        ackByteID++;
        gpsSerial.print(b, HEX);
      } 
      else {
        ackByteID = 0;  // Reset and look again, invalid order
      }
 
    }
  }
}
*/
