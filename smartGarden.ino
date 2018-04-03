// IMPORTANT: ELEGOO_TFTLCD LIBRARY MUST BE SPECIFICALLY
// CONFIGURED FOR EITHER THE TFT SHIELD OR THE BREAKOUT BOARD.
// SEE RELEVANT COMMENTS IN Elegoo_TFTLCD.h FOR SETUP.
//Technical support:goodtft@163.com

#include <Servo.h>
#include <BME280.h>
 
Servo myServo; // create a servo object

BME280 bme;                   // Default : forced mode, standby time = 1000 ms
                              // Oversampling = pressure ×1, temperature ×1, humidity ×1, filter off,
bool metric = true;
float temp(NAN), hum(NAN), pres(NAN);
   uint8_t pressureUnit(1);   // unit: B000 = Pa, B001 = hPa, B010 = Hg, B011 = atm, B100 = bar, B101 = torr, B110 = N/m^2, B111 = psi

/* ==== Prototypes ==== */
/* === Print a message to stream with the temp, humidity and pressure. === */
void printBME280Data(Stream * client);
/* === Print a message to stream with the altitude, and dew point. === */
void printBME280CalculatedData(Stream* client);
/* ==== END Prototypes ==== */
 


#include <Elegoo_GFX.h>    // Core graphics library
#include <Elegoo_TFTLCD.h> // Hardware-specific library
#include <TouchScreen.h>

#if defined(__SAM3X8E__)
    #undef __FlashStringHelper::F(string_literal)
    #define F(string_literal) string_literal
#endif
// The control pins for the LCD can be assigned to any digital or
// analog pins...but we'll use the analog pins as this allows us to
// double up the pins with the touch screen (see the TFT paint example).
#define LCD_CS A3 // Chip Select goes to Analog 3
#define LCD_CD A2 // Command/Data goes to Analog 2
#define LCD_WR A1 // LCD Write goes to Analog 1
#define LCD_RD A0 // LCD Read goes to Analog 0

#define LCD_RESET A4 // Can alternately just connect to Arduino's reset pin

// When using the BREAKOUT BOARD only, use these 8 data lines to the LCD:
// For the Arduino Uno, Duemilanove, Diecimila, etc.:
//   D0 connects to digital pin 8  (Notice these are
//   D1 connects to digital pin 9   NOT in order!)
//   D2 connects to digital pin 2
//   D3 connects to digital pin 3
//   D4 connects to digital pin 4
//   D5 connects to digital pin 5
//   D6 connects to digital pin 6
//   D7 connects to digital pin 7
// For the Arduino Mega, use digital pins 22 through 29
// (on the 2-row header at the end of the board).

// Assign human-readable names to some common 16-bit color values:
#define  BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

#define YP A3  // must be an analog pin, use "An" notation!
#define XM A2  // must be an analog pin, use "An" notation!
#define YM 9   // can be a digital pin
#define XP 8   // can be a digital pin
/*
#define TS_MINX 50
#define TS_MAXX 920

#define TS_MINY 100
#define TS_MAXY 940
*/
//Touch For New ILI9341 TP
#define TS_MINX 120
#define TS_MAXX 900

#define TS_MINY 70
#define TS_MAXY 920

// For better pressure precision, we need to know the resistance
// between X+ and X- Use any multimeter to read it
// For the one we're using, its 300 ohms across the X plate
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);


#define BOXSIZE 40
#define PENRADIUS 3
int oldcolor, currentcolor;
Elegoo_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);
// If using the shield, all control and data lines are fixed, and
// a simpler declaration can optionally be used:
// Elegoo_TFTLCD tft;

void setup(void) {
  
  Serial.begin(9600);
  Serial.println(F("TFT LCD test"));


  

#ifdef USE_Elegoo_SHIELD_PINOUT
  Serial.println(F("Using Elegoo 2.4\" TFT Arduino Shield Pinout"));
#else
  Serial.println(F("Using Elegoo 2.4\" TFT Breakout Board Pinout"));
#endif

  Serial.print("TFT size is "); Serial.print(tft.width()); Serial.print("x"); Serial.println(tft.height());

 // tft.reset();

   uint16_t identifier = tft.readID();
   if(identifier == 0x9325) {
    Serial.println(F("Found ILI9325 LCD driver"));
  } else if(identifier == 0x9328) {
    Serial.println(F("Found ILI9328 LCD driver"));
  } else if(identifier == 0x4535) {
    Serial.println(F("Found LGDP4535 LCD driver"));
  }else if(identifier == 0x7575) {
    Serial.println(F("Found HX8347G LCD driver"));
  } else if(identifier == 0x9341) {
    Serial.println(F("Found ILI9341 LCD driver"));
  } else if(identifier == 0x8357) {
    Serial.println(F("Found HX8357D LCD driver"));
  } else if(identifier==0x0101)
  {     
      identifier=0x9341;
       Serial.println(F("Found 0x9341 LCD driver"));
  }
  else if(identifier==0x1111)
  {     
      identifier=0x9328;
       Serial.println(F("Found 0x9328 LCD driver"));
  }
  else {
    Serial.print(F("Unknown LCD driver chip: "));
    Serial.println(identifier, HEX);
    Serial.println(F("If using the Elegoo 2.8\" TFT Arduino shield, the line:"));
    Serial.println(F("  #define USE_Elegoo_SHIELD_PINOUT"));
    Serial.println(F("should appear in the library header (Elegoo_TFT.h)."));
    Serial.println(F("If using the breakout board, it should NOT be #defined!"));
    Serial.println(F("Also if using the breakout, double-check that all wiring"));
    Serial.println(F("matches the tutorial."));
    identifier=0x9328;
  
  }
  tft.begin(identifier);
 tft.setRotation(2);

  tft.fillScreen(BLACK);

  tft.fillRect(0, 0, BOXSIZE, BOXSIZE, RED);
  tft.fillRect(BOXSIZE, 0, BOXSIZE, BOXSIZE, BLUE);
  tft.fillRect(BOXSIZE*2, 0, BOXSIZE, BOXSIZE, GREEN);
  tft.fillRect(BOXSIZE*3, 0, BOXSIZE, BOXSIZE, CYAN);
  tft.fillRect(BOXSIZE*4, 0, BOXSIZE, BOXSIZE, YELLOW);
  tft.fillRect(BOXSIZE*5, 0, BOXSIZE, BOXSIZE, MAGENTA);
  // tft.fillRect(BOXSIZE*6, 0, BOXSIZE, BOXSIZE, WHITE);
 
  tft.drawRect(0, 0, BOXSIZE, BOXSIZE, WHITE);
  currentcolor = RED;

 while(!bme.begin()){
    Serial.println("Could not find BME280 sensor!");
    
    delay(1000);
  }


  
   pinMode(13, OUTPUT);

   pinMode(11, OUTPUT);
   pinMode(12,OUTPUT);
   pinMode(13,OUTPUT);
   pinMode(10,OUTPUT);
   //myServo.attach(10); // attaches the servo on pin 9 to the servo object

}

#define MINPRESSURE 300
#define MAXPRESSURE 1000
bool led1=0,led2=0,led3=0,motor=0;
int loopCnt=0;
void loop(void) {
   printBME280Data(&Serial);
   printBME280CalculatedData(&Serial);

digitalWrite(10,LOW);
 /* digitalWrite(11, LOW);
 if(led1==1){
  digitalWrite(12, LOW);
  led1=0;}
  else{digitalWrite(13, LOW);
  led1=1;
  }
  delay(1000);
   digitalWrite(3,LOW);
    */
 // digitalWrite(9,HIGH);
  // tft.fillScreen(BLACK);
//myServo.write(1023);
//   digitalWrite(13, HIGH);
   TSPoint p = ts.getPoint();
  // digitalWrite(13, LOW);
   pinMode(XM, OUTPUT);
   pinMode(YP, OUTPUT);
     
   tft.setRotation(2);
  unsigned long start = micros();

  tft.setCursor(0, 0);

  if(loopCnt>100){
         tft.fillRect(0, BOXSIZE, tft.width(), tft.height()-BOXSIZE, BLACK);
         tft.setRotation(1);
        tft.setTextColor(RED); tft.setTextSize(3);
  tft.println();
  tft.print(temp);
  tft.println(" C");
  tft.print(hum);
  tft.println(" %");
  tft.print(pres);
  tft.println(" hPa");
  tft.println();
  tft.println();
  loopCnt=0;
  }
  loopCnt++;
 // delay(1000);


 tft.setRotation(2);
    if (p.z > MINPRESSURE && p.z < MAXPRESSURE) {
   
    
    if (p.y < (TS_MINY-5)) {
      Serial.println("erase");
      // press the bottom of the screen to erase 
      tft.fillRect(0, BOXSIZE, tft.width(), tft.height()-BOXSIZE, BLACK);
    }
    // scale from 0->1023 to tft.width
    p.x = map(p.x, TS_MINX, TS_MAXX, tft.width(), 0);
    //p.x = tft.width()-map(p.x, TS_MINX, TS_MAXX, tft.width(), 0);
    p.y = (tft.height()-map(p.y, TS_MINY, TS_MAXY, tft.height(), 0));
     //p.y = map(p.y, TS_MINY, TS_MAXY, tft.height(), 0);
   
    if (p.y < BOXSIZE) {
       oldcolor = currentcolor;

       if (p.x < BOXSIZE) { 
    
 if(led3==1){
  digitalWrite(11, LOW);
  led3=0;}
  else{digitalWrite(11, HIGH);
  led3=1;
  }
  delay(500);

  
   tft.setRotation(2);
         currentcolor = RED; 
         tft.drawRect(0, 0, BOXSIZE, BOXSIZE, WHITE);
       } else if (p.x < BOXSIZE*2) {
        
 if(led2==1){
  digitalWrite(12, LOW);
  led2=0;}
  else{digitalWrite(12, HIGH);
  led2=1;
  }
  delay(500);
  Serial.println(p.z);
  Serial.println(led2);
   tft.setRotation(2);
         currentcolor = BLUE;
         tft.drawRect(BOXSIZE, 0, BOXSIZE, BOXSIZE, WHITE);
       } else if (p.x < BOXSIZE*3) {
         currentcolor = GREEN;
         tft.drawRect(BOXSIZE*2, 0, BOXSIZE, BOXSIZE, WHITE);
    if(led1==1){
  digitalWrite(13, LOW);
  led1=0;}
  else{digitalWrite(13, HIGH);
  led1=1;
  }
  delay(500);
   Serial.println(p.z);
  Serial.println(led1);
       } else if (p.x < BOXSIZE*4) {
        //myServo.write(1023);
         if(motor==1){
  digitalWrite(10, LOW);
  motor=0;}
  else{digitalWrite(10, HIGH);
  motor=1;
  }
  delay(2000);
         currentcolor = CYAN;
         tft.drawRect(BOXSIZE*3, 0, BOXSIZE, BOXSIZE, WHITE);
       } else if (p.x < BOXSIZE*5) {
         currentcolor = YELLOW;
         tft.drawRect(BOXSIZE*4, 0, BOXSIZE, BOXSIZE, WHITE);
       } else if (p.x < BOXSIZE*6) {
         currentcolor = MAGENTA;
         tft.drawRect(BOXSIZE*5, 0, BOXSIZE, BOXSIZE, WHITE);
       }

       if (oldcolor != currentcolor) {
          if (oldcolor == RED) tft.fillRect(0, 0, BOXSIZE, BOXSIZE, RED);
          if (oldcolor == BLUE) tft.fillRect(BOXSIZE, 0, BOXSIZE, BOXSIZE, BLUE);
          if (oldcolor == GREEN) tft.fillRect(BOXSIZE*2, 0, BOXSIZE, BOXSIZE, GREEN);
          if (oldcolor == CYAN) tft.fillRect(BOXSIZE*3, 0, BOXSIZE, BOXSIZE, CYAN);
          if (oldcolor == YELLOW) tft.fillRect(BOXSIZE*4, 0, BOXSIZE, BOXSIZE, YELLOW);
          if (oldcolor == MAGENTA) tft.fillRect(BOXSIZE*5, 0, BOXSIZE, BOXSIZE, MAGENTA);
       }
    }
    if (((p.y-PENRADIUS) > BOXSIZE) && ((p.y+PENRADIUS) < tft.height())) {
      tft.fillCircle(p.x, p.y, PENRADIUS, currentcolor);
    }
  }


  

}
void printBME280Data(Stream* client){
    bme.ReadData(pres, temp, hum, metric, pressureUnit);                // Parameters: (float& pressure, float& temp, float& humidity, bool hPa = true, bool celsius = false)
  /* Alternatives to ReadData():
    float ReadTemperature(bool celsius = false);
    float ReadPressure(uint8_t unit = 0);
    float ReadHumidity();

    Keep in mind the temperature is used for humidity and
    pressure calculations. So it is more effcient to read
    temperature, humidity and pressure all together.
   */
  client->print("Temp: ");
  client->print(temp);
  client->print("°"+ String(metric ? 'C' :'F'));
  client->print("\t\tHumidity: ");
  client->print(hum);
  client->print("% RH");
  client->print("\t\tPressure: ");
  client->print(pres);
  client->print(" hPa");

  
}
void printBME280CalculatedData(Stream* client){
  float altitude = bme.CalculateAltitude(metric);
  float dewPoint = bme.CalculateDewPoint(metric);
  client->print("\t\tAltitude: ");
  client->print(altitude);
  client->print((metric ? "m" : "ft"));
  client->print("\t\tDew point: ");
  client->print(dewPoint);
  client->println("°"+ String(metric ? 'C' :'F'));

}
