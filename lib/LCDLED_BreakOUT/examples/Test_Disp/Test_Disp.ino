/*********************

Example code for the Adafruit RGB Character LCD Shield and Library

This code displays text on the shield, and also reads the buttons on the keypad.
When a button is pressed, the backlight changes color.

**********************/

// include the library code:
#include <Wire.h>
#include <LCDLED_BreakOUT.h>
#include <utility/Adafruit_MCP23017.h>


// The shield uses the I2C SCL and SDA pins. On classic Arduinos
// this is Analog 4 and 5 so you can't use those for analogRead() anymore
// However, you can connect other I2C sensors to the I2C bus and share
// the I2C bus.
LCDLED_BreakOUT lcd = LCDLED_BreakOUT();

// These #defines make it easy to set the backlight color
#define BACKLIGHT 0x1
int secWAIT =90;
int secBLINK = 60;
int secFULL = 10;
long time;

void setup() {
  // Debugging output
  Serial.begin(9600);
  // set up the LCD's number of columns and rows: 
  lcd.begin(20, 4);

  // Print a message to the LCD. We track how long it takes since
  // this library has been optimized a bit and we're proud of it :)
  time = millis();
  lcd.print("Hello, world!");
  time = millis() - time;
  Serial.print("Took "); Serial.print(time); Serial.println(" ms");
  lcd.setBacklight(BACKLIGHT);
  lcd.pinLEDs(5, LOW);
  lcd.pinLEDs(4, LOW);
  lcd.pinLEDs(3, LOW);
  lcd.pinLEDs(2, LOW);
  lcd.pinLEDs(1, LOW);
  lcd.pinLEDs(0, LOW);
  time = millis();
}

uint8_t i=0;
void loop() {
  // set the cursor to column 0, line 1
  // (note: line 1 is the second row, since counting begins with 0):
//  if (time ==0) time = millis();
  uint8_t buttons = lcd.readButtons();
  
  if (buttons) {
//    Serial.println(buttons);
//    lcd.clear();
    lcd.setCursor(0,2);
    if (buttons & BUTTON_P2) {
      lcd.print("STOP ");
      lcd.pinLEDs(4, HIGH);
      lcd.pinLEDs(5, LOW);
      lcd.pinLEDs(2, HIGH);
      lcd.pinLEDs(3, HIGH);
      delay(250);
      lcd.pinLEDs(2, LOW);
      lcd.pinLEDs(3, LOW);
    }
    if (buttons & BUTTON_P1) {
      lcd.print("EXTRA ");
      lcd.pinLEDs(5, HIGH);
      lcd.pinLEDs(4, LOW);
      lcd.pinLEDs(2, HIGH);
      lcd.pinLEDs(3, HIGH);
      delay(125);
      lcd.pinLEDs(2, LOW);
      lcd.pinLEDs(3, LOW);
    }
  }
// print the number of seconds since reset:
  if ((millis() - time) % 250 ==0) {   
    lcd.setCursor(0, 1);   
    int secStart =((millis() - time)/1000);
    lcd.print(secStart);
    
    lcd.setCursor(5, 1);
    int rest =secWAIT - secStart;
    lcd.print(rest);
    lcd.print(" ");

//    Serial.print(time);
//    Serial.print("\t");
//    Serial.print(millis());
//    Serial.print("\t");
//    Serial.print(secStart);
//    Serial.print("\t");
//    Serial.println(rest);
  
    if (rest ==0 ) {
      lcd.pinLEDs(0, LOW);
      lcd.pinLEDs(1, LOW);        
      lcd.setCursor(0, 1);   
      lcd.print("               ");
      time = millis();
    }
    else if (rest <= secBLINK) {
      if (rest <= secFULL) {
        lcd.pinLEDs(0, HIGH);
        lcd.pinLEDs(1, HIGH);
      }
      else if (rest % 2 ==0) {
        lcd.pinLEDs(1, HIGH);
        lcd.pinLEDs(0, LOW);
      }
      else if (rest % 2 !=0) {
        lcd.pinLEDs(1, LOW);
        lcd.pinLEDs(0, HIGH);
      }        
    }   
  }
}
