/*  Arduino display of Altitude, density Altitude, Press, Temp, and Kollsman setting
 *  from Bosch Sensor BMP280 using ILI9341 touchscreen shield for Arduino Uno.
 *  SPI via hardware ICSP with Pin #19 for the sensor CSB.
 *  All remaining TFT LCD shield pins used by the 2.8" ILI9341 touch screen.
 *  Range of Ground Level Pressure(QNH) 27.50 to 31.50 inches of Hg.
 *  Any other input of inHg will result in "invalid" message with wait for
 *  correct input.
 *  To use, enter the most recent QNH in inches of Hg obtained from a nearby
 *  airport METAR.  QNH is a sea level pressure equivalent, but not exactly
 *  the same as Sea Level Pressure (SLP), as SLP is adjusted for a 12-hour 
 *  average temperature, rather than for current conditions.
 *  
 *  Made publicly available under the terms of the Creative Commons License.
 *  
 *  Copyright January 20, 2020.  Ken Burrell.
 */
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

// **** IF USING THE LCD BREAKOUT BOARD, COMMENT OUT THIS NEXT LINE. ****
// **** IF USING THE LCD SHIELD, LEAVE THE LINE ENABLED:             ****
#define USE_ADAFRUIT_SHIELD_PINOUT 1


#include <Adafruit_GFX.h>
#include <MCUFRIEND_kbv.h>
MCUFRIEND_kbv tft;
#include <TouchScreen.h>
#define MINPRESSURE 200
#define MAXPRESSURE 1000

// ALL Touch panel wiring is DIFFERENT
// copy-paste results from TouchScreen_Calibr_native.ino
const int XP = 8, XM = A2, YP = A3, YM = 9; //ID=0x9341
const int TS_LEFT = 927, TS_RT = 122, TS_TOP = 100, TS_BOT = 905;

TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);

Adafruit_GFX_Button enter_btn;

#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

#define BMP_CS 19
//initialize hardware SPI
Adafruit_BMP280 bmp(BMP_CS);

// Assign human-readable names to some common 16-bit color values:
#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

int pixel_x, pixel_y, pixel_z;     //Touch_getXY() updates global vars
bool Touch_getXY(void)
{
    TSPoint p = ts.getPoint();
    pinMode(YP, OUTPUT);      //restore shared pins
    pinMode(XM, OUTPUT);
    digitalWrite(YP, HIGH);   //because TFT control pins
    digitalWrite(XM, HIGH);
    bool pressed = (p.z > MINPRESSURE && p.z < MAXPRESSURE);
    if (pressed) {
        pixel_x = map(p.x, TS_LEFT, TS_RT, 0, tft.width());
        pixel_y = map(p.y, TS_TOP, TS_BOT, 0, tft.height());
        pixel_z = p.z;
    }
    return pressed;
}

float B = 29.92 ;
int delta = 25;
int clockCenterX=120;
int clockCenterY=110+delta;
/*
Uncomment Serial lines for debugging
*/

void setup() {

//  Serial.begin(9600);
  
  tft.reset();

#ifdef USE_ADAFRUIT_SHIELD_PINOUT
//  Serial.println(F("Using Adafruit 2.8\" TFT Arduino Shield Pinout"));
#else
//  Serial.println(F("Using Adafruit 2.8\" TFT Breakout Board Pinout"));
#endif
//  Serial.print("TFT size is "); // Serial.print(tft.width()); // Serial.print("x"); // Serial.println(tft.height());

  uint16_t identifier = tft.readID();
  
  if(identifier == 0x9341) {
//    Serial.println(F("Found ILI9341 LCD driver"));
    } else {
//    Serial.print(F("Unknown LCD driver chip: "));
//    Serial.println(identifier, HEX);
//    Serial.println(F("If using the Adafruit 2.8\" TFT Arduino shield, the line:"));
//    Serial.println(F("  #define USE_ADAFRUIT_SHIELD_PINOUT"));
//    Serial.println(F("should appear in the library header (Adafruit_TFT.h)."));
//    Serial.println(F("If using the breakout board, it should NOT be #defined!"));
//    Serial.println(F("Also if using the breakout, double-check that all wiring"));
//    Serial.println(F("matches the tutorial."));
    return;
    }
     
    tft.begin(identifier);
  
    tft.fillScreen(BLACK);
    // set orientation to portrait (0 or 2) or landscape (1 or 3)
    tft.setRotation(0);

    if (!bmp.begin()) {
//      Serial.println("BMP init failed!");
      while (1);
    } else {
//      Serial.println("BMP init success!");
    /* Default settings from datasheet. */
      bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                      Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                      Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                      Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                      Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
     }
  }

void drawDisplay(float B)
{
    tft.fillScreen(BLACK);
// Set up fixed text
    tft.setTextColor(GREEN,BLACK);  tft.setTextSize(2);
    tft.setCursor(13,12);
    tft.println("Practice Altimeter");
    tft.setTextColor(GREEN,BLACK);  tft.setTextSize(1);  
    tft.setCursor(0, 280);
    tft.println(" Feet");
    tft.setCursor(0, 290);
    tft.println(" dAlt");
    tft.setCursor(0, 300);
    tft.println(" mBar");
    tft.setCursor(0, 310);
    tft.println(" degC");
  
// Draw Clockface 
  // first clear clock
  tft.fillCircle(clockCenterX, clockCenterY, 100, BLACK);
  
  for (int i=0; i<2; i++)
  {
    tft.drawCircle(clockCenterX, clockCenterY, 100-i, GREEN);
  }
  for (int i=0; i<3; i++)
  {
    tft.drawCircle(clockCenterX, clockCenterY, i, RED);
  }
  
  // Draw a small mark for 100 feet
  drawMarks();

  // Label each mark
  drawNumbs();
  
  // insert Kollsman knob
  tft.fillCircle(190, 225+delta, 35, BLUE);
  tft.fillCircle(190, 225+delta, 30, BLACK);
  tft.setTextColor(MAGENTA); tft.setTextSize(2);
  tft.setCursor(162, 220+delta);
  tft.println(B,2);
  tft.setTextColor(GREEN); tft.setTextSize(1);
  tft.setCursor(95, 220+delta);
  tft.println("x100 Feet");
}

void drawMarks()
{
  float x1, y1, x2, y2, h, phi;

  tft.setTextColor(GREEN); tft.setTextSize(1);
   
  for (int i=0; i<10; i++)
  {
    h   = i*36.0;
    phi = radians(h);

    x1= 99.0*sin(phi);
    y1=-99.0*cos(phi);
    x2= 89.0*sin(phi);
    y2=-89.0*cos(phi);
  
    tft.drawLine(x1+clockCenterX, y1+clockCenterY, x2+clockCenterX, y2+clockCenterY, GREEN);
   
  }
    for (int i=0; i<50; i++)
  {
    h   = i*7.20;
    phi = radians(h);

    x1= 99.0*sin(phi);
    y1=-99.0*cos(phi);
    x2= 94.0*sin(phi);
    y2=-94.0*cos(phi);
  
    tft.drawLine(x1+clockCenterX, y1+clockCenterY, x2+clockCenterX, y2+clockCenterY, GREEN);
   
  }
}

void drawNumbs()
{
  float x3, y3, h, phi;
  int j,k;
    
  tft.setTextColor(GREEN); tft.setTextSize(1);
  
  for (int i=0; i<10; i++)
  {
    h=i*36.0;
    phi = radians(h);
    
    j = 85.0*sin(phi)+clockCenterX;
    k =-85.0*cos(phi)+clockCenterY;
    
    tft.setCursor(j,k );
    tft.println(i);
  }
}

void draw10K(float elev)
{
  float x1, y1, x2, y2, h, phi;
  float x3, y3, x4, y4, x5, y5, chi, omg;

  h = elev * 0.0036;

  phi = radians(h);
  chi = radians(h-6.0);
  omg = radians(h+6.0);

  x1= 94.0*sin(phi)+clockCenterX;
  y1=-94.0*cos(phi)+clockCenterY;
  x2=  5.0*sin(phi)+clockCenterX;
  y2= -5.0*cos(phi)+clockCenterY;
  
  x3= 96.0*sin(chi)+clockCenterX;
  y3=-96.0*cos(chi)+clockCenterY;
  x4= 96.0*sin(omg)+clockCenterX;
  y4=-96.0*cos(omg)+clockCenterY;
  x5= 84.0*sin(phi)+clockCenterX;
  y5=-84.0*cos(phi)+clockCenterY;
   
  
  tft.drawLine(x1, y1, x2, y2, WHITE);
  tft.fillTriangle(x3, y3, x5, y5, x4, y4, WHITE);
}

void draw100(float elev)
{
  float x1, y1, x2, y2, x3, y3, x4, y4, h, phi, chi, omg;

  h = ((int) elev % 1000 ) * 0.36;

  phi = radians(h);
  chi = radians(h+8.0);
  omg = radians(h-8.0);
  
  x1= 85.0*sin(phi);
  y1=-85.0*cos(phi);
  x2= 10.0*sin(phi);
  y2=-10.0*cos(phi);
  x3= 35.0*sin(chi);
  y3=-35.0*cos(chi);
  x4= 35.0*sin(omg);
  y4=-35.0*cos(omg);
  
  tft.drawLine(x1+clockCenterX, y1+clockCenterY, x3+clockCenterX, y3+clockCenterY, RED);
  tft.drawLine(x3+clockCenterX, y3+clockCenterY, x2+clockCenterX, y2+clockCenterY, RED);
  tft.drawLine(x2+clockCenterX, y2+clockCenterY, x4+clockCenterX, y4+clockCenterY, RED);
  tft.drawLine(x4+clockCenterX, y4+clockCenterY, x1+clockCenterX, y1+clockCenterY, RED);

}

void draw1K(float elev)
{
  float x1, y1, x2, y2, x3, y3, x4, y4, h, phi, chi, omg;

  h = elev * 0.036;

  phi = radians(h);
  chi = radians(h+12.0);
  omg = radians(h-12.0);
  
  x1=  70.0*sin(phi);
  y1= -70.0*cos(phi);
  x2=  10.0*sin(phi);
  y2= -10.0*cos(phi);
  x3=  25.0*sin(chi);
  y3= -25.0*cos(chi);
  x4=  25.0*sin(omg);
  y4= -25.0*cos(omg);
  
  tft.drawLine(x1+clockCenterX, y1+clockCenterY, x3+clockCenterX, y3+clockCenterY, RED);
  tft.drawLine(x3+clockCenterX, y3+clockCenterY, x2+clockCenterX, y2+clockCenterY, RED);
  tft.drawLine(x2+clockCenterX, y2+clockCenterY, x4+clockCenterX, y4+clockCenterY, RED);
  tft.drawLine(x4+clockCenterX, y4+clockCenterY, x1+clockCenterX, y1+clockCenterY, RED);

}

#define BUTTON_X 40
#define BUTTON_Y 100
#define BUTTON_W 60
#define BUTTON_H 30
#define BUTTON_SPACING_X 20
#define BUTTON_SPACING_Y 20
#define BUTTON_TEXTSIZE 2
#define TEXT_X 10
#define TEXT_Y 10
#define TEXT_W 220
#define TEXT_H 50
#define TEXT_TSIZE 3
#define TEXT_TCOLOR MAGENTA
// the data (in Hg) stored in the textfield
#define TEXT_LEN 5
char textfield[TEXT_LEN+3] = "";
uint8_t textfield_i=0;

//Variables
Adafruit_GFX_Button buttons[12];
/* create 15 buttons, in classic candybar phone style */
char buttonlabels[12][5] = { "1", "2", "3", "4", "5", "6", "7", "8", "9", "0",".", "Clr" };
uint16_t buttoncolors[12] = {BLUE, BLUE, BLUE, 
                             BLUE, BLUE, BLUE, 
                             BLUE, BLUE, BLUE, 
                             BLUE, BLUE, RED};

float H0 = 0.0, OldB;

void loop(void) 
{
     float QNH = 33.8639 * B ;
     float T = bmp.readTemperature();
     float P = 0.01* bmp.readPressure();
     float H = round(3.28*bmp.readAltitude(QNH)) ;  
     float PA = H + 27.0 * (1013.00 - QNH);
     float DA = round((1.2376 * PA) + ( 118.8 * T ) - 1782.0) ;
     
     if ( B != OldB ) {
        drawDisplay(B);
     } else if ( H != H0 ) {
     // first clear hands of clock and re-draw clock numbers, then hands
        tft.fillCircle(clockCenterX, clockCenterY, 85, BLACK);
        for (int i=0; i<3; i++)
        {
           tft.drawCircle(clockCenterX, clockCenterY, i, RED);
        }
        drawNumbs();
     }
     // draw hands
     draw10K(H);
     draw1K(H);
     draw100(H);

     // draw text
     tft.setTextColor(YELLOW,BLACK); tft.setTextSize(1);
     tft.setCursor(40, 280);
     tft.println(H, 0);
     tft.setCursor(40, 290);
     tft.println(DA, 0);
     tft.setCursor(40, 300);
     tft.println(P, 2);
     tft.setCursor(40, 310);
     tft.println(T, 2);
     
     H0 = H;
     OldB = B;
     bool down = Touch_getXY();
     bool box = ( (pixel_x >= 160 && pixel_x <= 220) &&
                (  pixel_y >= 220 && pixel_y <= 280) );
     if ( down  && box ) 
     {
         tft.fillScreen(BLACK);
      // create buttons
         for (uint8_t row=0; row<4; row++) {
          for (uint8_t col=0; col<3; col++) {
           buttons[col + row*3].initButton(&tft, BUTTON_X+col*(BUTTON_W+BUTTON_SPACING_X), 
             BUTTON_Y+row*(BUTTON_H+BUTTON_SPACING_Y),    // x, y, w, h, outline, fill, text
             BUTTON_W, BUTTON_H, WHITE, buttoncolors[col+row*3], WHITE,
             buttonlabels[col + row*3], BUTTON_TEXTSIZE); 
             buttons[col + row*3].drawButton();
           }
         }
      // create 'text field'
         tft.drawRect(TEXT_X, TEXT_Y, TEXT_W, TEXT_H, WHITE);
      // create ENTER button
         enter_btn.initButton(&tft,120, 300, 100, 40, WHITE, CYAN, BLACK, "ENTER", 2);
         enter_btn.drawButton(false);
         do {
          TSPoint p = ts.getPoint();
          pinMode(YP, OUTPUT);      //restore shared pins
          pinMode(XM, OUTPUT);
          digitalWrite(YP, HIGH);   //because TFT control pins
          digitalWrite(XM, HIGH);
          if (p.z > MINPRESSURE && p.z < MAXPRESSURE) {
             p.x = map(p.x, TS_LEFT, TS_RT, 0, tft.width());
             p.y = map(p.y, TS_TOP, TS_BOT, 0, tft.height());
            }
            // go thru all the buttons, checking if they were pressed
            for (uint8_t b=0; b<12; b++) {
              if (buttons[b].contains(p.x, p.y)) {
                buttons[b].press(true);  // tell the button it is pressed
              } else {
                buttons[b].press(false);  // tell the button it is NOT pressed
              }
            }
         // now we can ask the buttons if their state has changed
            for (uint8_t b=0; b<12; b++) {
             if (buttons[b].justReleased()) {
               buttons[b].drawButton();  // draw normal
             }
    
             if (buttons[b].justPressed()) {
               buttons[b].drawButton(true);  // draw invert!
        
          //   if a numberpad button, append the relevant # to the textfield
               if (b < 11) {
                 if (textfield_i < TEXT_LEN) {
                   textfield[textfield_i] = buttonlabels[b][0];
                   textfield_i++;
                   textfield[textfield_i] = 0; // zero terminate
                 }
               }

        //     clr button! delete char
               if (b == 11) {
                 textfield[textfield_i] = 0;
                 if (textfield > 0) {
                   textfield_i--;
                   textfield[textfield_i] = ' ';
                 }
               }

        //     update the current text field
               tft.setCursor(TEXT_X + 2, TEXT_Y+10);
               tft.setTextColor(TEXT_TCOLOR, WHITE);
               tft.setTextSize(TEXT_TSIZE);
               tft.print(textfield);        
               
               delay(100); // UI debouncing
             }
            }
      //    check if enter key pressed, set B, and exit.
            enter_btn.press(down && enter_btn.contains(p.x, p.y));
            if ( enter_btn.justReleased() ) enter_btn.drawButton();
            if ( enter_btn.justPressed() ) {
              enter_btn.drawButton(true);
              B = atof ( textfield );
              if ( B >= 27.50 && B <= 31.50 )  break;
   // Since 2016 aviation altimeters restrict range from 27.50 to 31.50 in Hg           
              textfield[0] = 'I';
              textfield[1] = 'n';
              textfield[2] = 'v';
              textfield[3] = 'a';
              textfield[4] = 'l';
              textfield[5] = 'i';
              textfield[6] = 'd';
              textfield_i = 7;
              textfield[textfield_i] = 0; // zero terminate
              tft.setCursor(TEXT_X + 2, TEXT_Y+10);
              tft.setTextColor(TEXT_TCOLOR, WHITE);
              tft.setTextSize(TEXT_TSIZE);
              tft.print(textfield);
              delay( 1000 );
              for (uint8_t b=0; b<7; b++) {
                   textfield[b] = ' ';
              }
              textfield_i = 0;
              tft.fillRect(TEXT_X, TEXT_Y, TEXT_W, TEXT_H, BLACK);
              tft.drawRect(TEXT_X, TEXT_Y, TEXT_W, TEXT_H, WHITE);
              enter_btn.drawButton(false);
            }
          } while ( 1 );
     }
}
