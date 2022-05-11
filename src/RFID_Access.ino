/* DESCRIPTION
  ====================
  started on 01JAN2017 - uploaded on 06.01.2017 by Dieter
  Code for machine and cleaner control over RFID
  reading IDENT from xBee, retrait sending ...POR until time responds
  switch on claener by current control and separate cleaner on

  Commands to Raspi --->
  xBeeName - from xBee (=Ident) [max 4 caracter including numbers] {xBeeName + nn}
  'POR'    - machine power on reset (Ident;por)

  'Ident;on'   - machine reporting ON-Status
  'Ident;off'  - machine reporting OFF-Status
  'card;nn...' - uid_2 from reader
  If current detection then:
  'cd_'    - current detected
  'nc_'    - no current

  Commands from Raspi
  'time'   - format time33.33.3333 33:33:33
  'onp'    - Machine permanent ON
  'ontxx'  - Machine xxx minutes ON
  'off'    - Machine OFF
  'noreg'  - RFID-Chip not registed
  'onDust' - Dust Collector on  (New)
  'ofDust' - Dust Collector off (New)

  'setce'  - [15 min] set time before ClosE machine
  'setcn'  - [6 sec] set time for longer CleaN on
  'setcl'  - [0] set Current Level for switching on and off
  'setrt'  - [1] set RepeaT messages
  'dison'  - display on for 60 sec
  'r3t...' - display text in row 3 "r3tabcde12345", max 20
  'r4t...' - display text in row 4 "r4tabcde12345", max 20

  last change: 26.04.2022 by Michael Muehl
  changed: new commands for dust collector, loged in | out, machine on | off,
           combine all commands and check messages, if kown and number is ok.
*/
#define Version "9.7.2" // (Test = 9.7.x ==> 9.7.3)
#define xBeeName "MA"   // Name and number for xBee
#define checkFA      2  // event check for every (1 second / FActor)

#include <Arduino.h>
#include <TaskScheduler.h>
#include <Wire.h>
#include <LCDLED_BreakOUT.h>
#include <utility/Adafruit_MCP23017.h>
#include <SPI.h>
#include <MFRC522.h>

// PIN Assignments
// RFID Control -------
#define RST_PIN      4  // RFID Reset
#define SS_PIN      10  // RFID Select

// Machine Control (ext)
#define currMotor   A0  // Motor current (Machine)
#define OUT_Machine A2  // OUT Machine on / off  (Machine)
#define OUT_Dust    A3  // OUT Dust on / off  (Dust Collector)

#define xBuError     8  // xBee and Bus error (13)

// I2C IOPort definition
byte I2CFound = 0;
byte I2CTransmissionResult = 0;
#define I2CPort   0x20  // I2C Adress MCP23017

// Pin Assignments Display (I2C LCD Port A/LED +Button Port B)
// Switched to LOW
#define FlashLED_A   0  // Flash LEDs oben
#define FlashLED_B   1  // Flash LEDs unten
#define buzzerPin    2  // Buzzer Pin
#define BUT_P1_LED   3  // not used
// Switched High - Low - High - Low
#define StopLEDrt    4  // StopLEDrt (LED + Stop-Taster)
#define StopLEDgn    5  // StopLEDgn (LED - Stop-Taster)
// switch to HIGH Value (def .h)
// BUTTON_P1         6  // not used
// BUTTON_P2         7  // StopSwitch
// BACKLIGHT for LCD-Display
#define BACKLIGHToff 0x0
#define BACKLIGHTon  0x1

// DEFINES
#define porTime         5 // [  5] wait seconds for sending Ident + POR
#define disLightOn     30 // [.30] display light on for seconds
#define CLOSE2END      60 // [ 60] MINUTES until activation is off
#define CLEANON         6 // [  6] TASK_SECOND dust collector on after current off
#define repMES          1 // [  1] repeat commands
#define periRead      100 // [100] read 100ms analog input for 50Hz (current)
#define currHyst       10 // [ 10] hystereses for current detection
#define currMean        3 // [  3] current average over ...

// CREATE OBJECTS
Scheduler runner;
LCDLED_BreakOUT lcd = LCDLED_BreakOUT();
MFRC522 mfrc522(SS_PIN, RST_PIN);  // Create MFRC522 instance

// Callback methods prototypes
void checkXbee();        // Task connect to xBee Server
void BlinkCallback();    // Task to let LED blink - added by D. Haude 08.03.2017
void UnLoCallback();     // Task to Unlock machine
void repeatMES();        // Task to repeat messages

void BuzzerOn();         // added by DieterH on 22.10.2017
void FlashCallback();    // Task to let LED blink - added by D. Haude 08.03.2017
void DispOFF();          // Task to switch display off after time

void Current();          // current measurement and detection

// Functions define for C++
void OnTimed(long);
void flash_led(int);

// TASKS
Task tM(TASK_SECOND / 2, TASK_FOREVER, &checkXbee);	    // 500ms main task
Task tR(TASK_SECOND / 2, 0, &repeatMES);                // 500ms * repMES repeat messages
Task tU(TASK_SECOND / checkFA, TASK_FOREVER, &UnLoCallback);  // 1000ms / checkFA ctor
Task tB(TASK_SECOND * 5, TASK_FOREVER, &BlinkCallback); // 5000ms added M. Muehl

Task tBU(TASK_SECOND / 10, 6, &BuzzerOn);               // 100ms 6x =600ms added by DieterH on 22.10.2017
Task tBD(1, TASK_ONCE, &FlashCallback);                 // Flash Delay
Task tDF(1, TASK_ONCE, &DispOFF);                       // display off

// --- Current measurement --
Task tCU(TASK_SECOND / 2, TASK_FOREVER, &Current);      // current measure

// VARIABLES
unsigned long val;
unsigned int timer = 0;
bool onTime = false;
int minutes = 0;
bool toggle = false;
unsigned long code;
byte atqa[2];
byte atqaLen = sizeof(atqa);
byte intervalRFID = 0;      // 0 = off; from 1 sec to 6 sec after Displayoff
// Cleaner Control
bool displayIsON = false;   // if display is switched on = true
bool isCleaner  = false;    // is cleaner under control (installed)
byte steps4push = 0;        // steps for push button action
unsigned int pushCount = 0; // counter how long push button in action

// Variables can be set externaly: ---
// --- on timed, time before new activation
unsigned int CLOSE = CLOSE2END; // RAM cell for before activation is off
bool firstCLOSE = false;        // only display message once
// --- for cleaning
unsigned int CLEAN = CLEANON; // RAM cell for Dust vaccu cleaner on after no current
unsigned int CURLEV = 0;      // RAM cell for before activation is off

// current measurement (cleaning on):
unsigned int currentVal =0;   // mean value
unsigned int currentMax =0;   // read max value
int currNR  = 0;              // number off machine with current detection
byte stepsCM = 0;             // steps for current measurement
byte countCM = 0;             // counter for current measurement

// Serial with xBee
String inStr = "";      // a string to hold incoming data
String IDENT = "";      // Machine identifier for remote access control
String SFMes = "";      // String send for repeatMES
byte co_ok = 0;        // send +++ control AT sequenz
byte getTime = porTime;

// ======>  SET UP AREA <=====
void setup()
{
  //init Serial port
  Serial.begin(57600);  // Serial
  inStr.reserve(40);    // reserve for instr serial input
  IDENT.reserve(5);     // reserve for IDENT serial output

  // initialize:
  Wire.begin();         // I2C

  SPI.begin();             // SPI

  mfrc522.PCD_Init();      // Init MFRC522
  mfrc522.PCD_SetAntennaGain(mfrc522.RxGain_avg);
//  mfrc522.PCD_SetAntennaGain(mfrc522.RxGain_max);

  // IO MODES
  pinMode(xBuError, OUTPUT);
  pinMode(OUT_Machine, OUTPUT);
  pinMode(OUT_Dust, OUTPUT);

  // Set default values
  digitalWrite(xBuError, HIGH); // turn the LED ON (init start)
  digitalWrite(OUT_Machine, LOW);
  digitalWrite(OUT_Dust, LOW);

    runner.init();
    runner.addTask(tM);
    runner.addTask(tB);
    runner.addTask(tR);
    runner.addTask(tU);
    runner.addTask(tBU);
    runner.addTask(tBD);
    runner.addTask(tDF);

    // Current --------
    runner.addTask(tCU);

  // Check if I2C _ Ports are avilable
  Wire.beginTransmission(I2CPort);
  I2CTransmissionResult = Wire.endTransmission();
  if (I2CTransmissionResult == 0)
  {
    I2CFound++;
  }
  // I2C Bus mit slave vorhanden
  if (I2CFound == 1)
  {
    lcd.begin(20, 4);    // initialize the LCD
    lcd.clear();
    lcd.pinLEDs(buzzerPin, LOW);
    lcd.pinLEDs(BUT_P1_LED, LOW);
    but_led(1);
    flash_led(1);
    dispRFID();
    tM.enable();         // xBee check
    tCU.enable();        // Current
    Serial.print("+++"); //Starting the request of IDENT
  }
  else 
  {
    tB.enable();  // enable Task Error blinking
    tB.setInterval(TASK_SECOND);
  }
}
// Setup End -----------------------------------

// TASK (Functions) ----------------------------
void checkXbee()
{
  if (IDENT.startsWith("MA") && co_ok == 2)
  {
    ++co_ok;
    tB.setCallback(retryPOR);
    tB.enable();
    digitalWrite(xBuError, LOW); // turn the LED off (Programm start)
  }
}

void retryPOR() {
  tDF.restartDelayed(TASK_SECOND * disLightOn); // restart display light
  if (getTime < porTime * 5) {
    Serial.println(String(IDENT) + ";POR;V" + String(Version));
    ++getTime;
    tB.setInterval(TASK_SECOND * getTime);
    lcd.setCursor(0, 0); lcd.print(String(IDENT) + " ");
    lcd.setCursor(16, 1); lcd.print((getTime - porTime) * porTime);
  }
  else if (getTime == 255) {
    tR.setIterations(repMES);
    tM.setCallback(checkRFID);
    tM.enable();
    tB.disable();
    displayON();
  }
}

void checkRFID()
{   // 500ms Tick
  if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial())
  {
    code = 0;
    firstCLOSE = false;
    for (byte i = 0; i < mfrc522.uid.size; i++) {
      code = ((code + mfrc522.uid.uidByte[i]) * 10);
    }
    if (!digitalRead(OUT_Machine))
    { // Check if machine is switched on
      flash_led(4);
      tBD.setCallback(&FlashCallback);
      tBD.restartDelayed(100);
      tDF.restartDelayed(TASK_SECOND * disLightOn);
    }
    Serial.println("card;" + String(code));
    // Display changes
    lcd.setCursor(5, 0); lcd.print("               ");
    lcd.setCursor(0, 0); lcd.print("Card# "); lcd.print(code);
    displayON();
  }

  if (displayIsON && steps4push > 0)
  {
    tB.disable();   //  time == 0 and timed or Button
    digitalWrite(OUT_Dust, LOW);
    pushCount = 0;
    steps4push = 0;
    flash_led(1);
  }
}

void UnLoCallback() {   // 500ms Tick
  uint8_t buttons = lcd.readButtons();
  if (timer > 0)
  {
    if (timer / 120 < CLOSE)
    { // Close to end time reached
      toggle = !toggle;
      if (toggle)
      { // toggle GREEN Button LED
        but_led(1);
        flash_led(1);
      }
      else
      {
        but_led(3);
        flash_led(4);
      }
      if (!firstCLOSE)
      {
        lcd.setCursor(0, 0);
        lcd.print("Place Tag @ Reader");
        lcd.setCursor(0, 1);
        lcd.print("to extend Time      ");
        tB.disable();
        tM.enable();
        firstCLOSE = true;
      }
    }
    timer -= 1;
    minutes = timer / 120;
    if (timer % 120 == 0)
    {
      char tbs[8];
      sprintf(tbs, "% 4d", minutes);
      lcd.setCursor(16, 3); lcd.print(tbs);
    }
  }
  if (timer == 0 && onTime && stepsCM >3) timer = 1;
  if (((timer == 0 && onTime) || buttons & BUTTON_P2) && stepsCM <=3)
  {   //  time == 0 and timed or Button
      onTime = false;
      shutdown();
  }
}

// Task repeatMES: ------------------------
void repeatMES() {
  // --repeat messages from machines
  Serial.println(String(SFMes));
}

void BlinkCallback() {
  // --Blink if BUS Error
  digitalWrite(xBuError, !digitalRead(xBuError));
}

void FlashCallback() {
  flash_led(1);
}

void DispOFF() {
  displayIsON = false;
  lcd.setBacklight(BACKLIGHToff);
  lcd.clear();
  but_led(1);
  flash_led(1);
}

// Current measure -----
void Current() {   // 500ms Tick
  // detect current for switching
  currentMax = getCurrMax();
  // steps ------------------
  switch (stepsCM) {
    case 0:   // set level values to min
      CURLEV = currentVal + currHyst;
      if (digitalRead(OUT_Machine)) {
        stepsCM = 1;
        countCM = 0;
      }
      break;
    case 1:   // current > level
      if (currentVal > CURLEV) {
        SFMes = "CD"+ String(currNR);
        Serial.println(SFMes);
        tR.restart();
        digitalWrite(OUT_Dust, HIGH);
        stepsCM = 2;
      }
      break;
    case 2:   // generate level for detecting on
      if (currentVal > CURLEV && countCM < currMean + 1) {
        ++countCM;
      } else if (countCM > currMean)  {
        CURLEV = currentVal / 3 * 2;  // 2 / 3 measured current = level
        stepsCM = 3;
        countCM = 0;
      }
      break;
    case 3:   // current > level
      if (currentVal > CURLEV) {
        SFMes = "CD"+ String(currNR);
        Serial.println(SFMes);
        tR.restart();
        digitalWrite(OUT_Dust, HIGH);
        stepsCM = 4;
      }
      break;
    case 4:   // wait for level less then level 3 times
      if (currentVal < CURLEV && countCM <= CLEAN * 2) {
        ++countCM;
        if (countCM >= CLEAN * 2) {
          stepsCM = 5;
          countCM = 0;
          break;
        }
      } else if (currentVal > CURLEV && countCM < CLEAN * 2) {
        countCM =0;
      }
      break;
    case 5:   // switch off clean after x sec later
      SFMes = "NC"+ String(currNR);
      Serial.println(SFMes);
      digitalWrite(OUT_Dust, LOW);
      stepsCM = 3;
      break;
  }
  currentVal = (currentVal + currentMax) / 2;
}
// END OF TASKS ---------------------------------

// FUNCTIONS ------------------------------------
int getNum(String strNum) // Check if realy numbers
{
  strNum.trim();
  for (byte i = 0; i < strNum.length(); i++)
  {
    if (!isDigit(strNum[i])) 
    {
      Serial.println(String(IDENT) + ";Num?;" + strNum);
      lcd.setCursor(19,0);
      lcd.print("?");
      return 0;
    }
  }
  return strNum.toInt();
}

void noreg() {
  digitalWrite(OUT_Machine, LOW);
  digitalWrite(OUT_Dust, LOW);
  lcd.setCursor(0, 2); lcd.print("Tag not registered  ");
  lcd.setCursor(0, 3); lcd.print("===> No access! <===");
  tM.enable();
  BadSound();
  but_led(1);
  flash_led(1);
  tDF.restartDelayed(TASK_SECOND * disLightOn);
}

void OnTimed(long min) {   // Turn on machine for nnn minutes
  onTime = true;
  timer = timer + min * 120;
  Serial.println(String(IDENT) + ";ont");
  char tbs[8];
  sprintf(tbs, "% 4d", timer / 120);
  lcd.setCursor(0, 3); lcd.print("Time left (min):"); lcd.print(tbs);
  granted();
}

void OnPerm(void)  {    // Turn on machine permanently (VIP-Users only)
  onTime = false;
  Serial.println(String(IDENT) + ";onp");
  lcd.setCursor(0, 3); lcd.print("Press Stop to EXIT");
  granted();
}

// Tag registered
void granted()
{
  tM.disable();
  tDF.disable();
  tU.enable();
  but_led(3);
  flash_led(1);
  GoodSound();
  digitalWrite(OUT_Machine, HIGH);
  lcd.setCursor(0, 2); lcd.print("Access granted      ");
  tR.disable();
}

// Switch off machine and stop
void shutdown(void)
{
  tU.disable();
  timer = 0;
  but_led(2);
  digitalWrite(OUT_Machine, LOW);
  digitalWrite(OUT_Dust, LOW);
  Serial.println(String(IDENT) + ";off");
  tDF.restartDelayed(TASK_SECOND * disLightOn);
  BadSound();
  flash_led(1);
  tM.enable();  // added by DieterH on 18.10.2017
  stepsCM = 0;
  // Display change
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("System shut down at");
}

void but_led(int var)
{
  switch (var)
  {
    case 1:   // LEDs off
      lcd.pinLEDs(StopLEDrt, HIGH);
      lcd.pinLEDs(StopLEDgn, HIGH);
      break;
    case 2:   // RED LED on
      lcd.pinLEDs(StopLEDrt, LOW);
      lcd.pinLEDs(StopLEDgn, HIGH);
      break;
    case 3:   // GREEN LED on
      lcd.pinLEDs(StopLEDrt, HIGH);
      lcd.pinLEDs(StopLEDgn, LOW);
      break;
  }
}

void flash_led(int var)
{
  switch (var)
  {
    case 1:   // LEDs off
      lcd.pinLEDs(FlashLED_A, LOW);
      lcd.pinLEDs(FlashLED_B, LOW);
      break;
    case 2:
      lcd.pinLEDs(FlashLED_A, HIGH);
      lcd.pinLEDs(FlashLED_B, LOW);
      break;
    case 3:
      lcd.pinLEDs(FlashLED_A, LOW);
      lcd.pinLEDs(FlashLED_B, HIGH);
      break;
    case 4:
      lcd.pinLEDs(FlashLED_A, HIGH);
      lcd.pinLEDs(FlashLED_B, HIGH);
      break;
  }
}

void BuzzerOff()
{
  lcd.pinLEDs(buzzerPin, LOW);
  tBU.setCallback(&BuzzerOn);
}

void BuzzerOn()
{
  lcd.pinLEDs(buzzerPin, HIGH);
  tBU.setCallback(&BuzzerOff);
}

void BadSound(void)
{   // added by DieterH on 22.10.2017
  tBU.setInterval(100);
  tBU.setIterations(6); // I think it must be Beeps * 2?
  tBU.setCallback(&BuzzerOn);
  tBU.enable();
}

void GoodSound(void)
{
  lcd.pinLEDs(buzzerPin, HIGH);
  tBD.setCallback(&BuzzerOff);  // changed by DieterH on 18.10.2017
  tBD.restartDelayed(200);      // changed by DieterH on 18.10.2017
}

//  RFID ------------------------------
void dispRFID(void)
{
  lcd.print("Sys  V" + String(Version).substring(0,3) + " starts at:");
  lcd.setCursor(0, 1); lcd.print("Wait Sync xBee:");
}

void displayON()
{
  displayIsON = true;
  lcd.setBacklight(BACKLIGHTon);
  tB.disable();
  tM.enable();
  intervalRFID = 0;
}

/*Function: Sample for 100ms and get the maximum value from the SIG pin*/
int getCurrMax()
{
  int curMax = 0;
  int curValue;   //value read from the sensor
  uint32_t start_time = millis();
  while((millis()-start_time) < periRead)
  {
    curValue = analogRead(currMotor);
    if (curValue > curMax)
    {
      curMax = curValue; //record the maximum sensor value
    }
  }
  return curMax;
}
// End Funktions --------------------------------

// Funktions Serial Input (Event) ---------------
void evalSerialData()
{
  inStr.toUpperCase();
  if (inStr.startsWith("OK"))
  {
    if (co_ok == 0)
    {
      Serial.println("ATNI");
      ++co_ok;
    }
    else
    {
      ++co_ok;
    }
  }
  else if (co_ok ==1  && inStr.length() == 4)
  {
    if (inStr.startsWith(xBeeName))
    {
      IDENT = inStr;
      currNR = inStr.substring(2).toInt();
    }
    else
    {
      lcd.setCursor(0, 0); lcd.print(inStr);
    }
    Serial.println("ATCN");
    ++co_ok;
  }
  else if (inStr.startsWith("TIME") && stepsCM <=3)
  {
    inStr.concat("                   ");     // add blanks to string
    lcd.setCursor(0, 1); lcd.print(inStr.substring(4,24));
    tB.setInterval(TASK_SECOND / 2);
    getTime = 255;
  }
  else if (inStr.startsWith("NOREG") && inStr.length() ==5)
  {
    noreg();  // changed by D. Haude on 18.10.2017
  }
  else if (inStr.startsWith("ONT") && inStr.length() >= 4 && inStr.length() < 7) 
  {
    val = getNum(inStr.substring(3));
    if (val > CLOSE2END) OnTimed(val);
  }
  else if (inStr.startsWith("ONP") && inStr.length() ==3)
  {
    OnPerm();
  }
  else if (inStr.startsWith("OFF") && inStr.length() ==3)
  {
    shutdown(); // Turn OFF Machine, only in case of emergency!
  }
  else if (inStr.startsWith("ONDUST") && inStr.length() ==6)
  {
    digitalWrite(OUT_Dust, HIGH);  
  }
  else if (inStr.startsWith("OFDUST") && inStr.length() ==6)
  {
    digitalWrite(OUT_Dust, LOW);  
  }
  else if (inStr.startsWith("SETCE") && inStr.length() >= 5 && inStr.length() < 8)
  { // set time before ClosE machine
    val = getNum(inStr.substring(5));
    if (val >= CLOSE2END) CLOSE = val;
  }
  else if (inStr.startsWith("SETCN") && inStr.length() >= 5 && inStr.length() < 8)
  { // set time for longer CleaN on
    val = getNum(inStr.substring(5));
    if (val >= CLEANON) CLEAN = val;
  }
  else if (inStr.startsWith("SETRT") && inStr.length() >= 5 && inStr.length() < 7)
  { // set repeat messages
    val = getNum(inStr.substring(5));
    if (val >= repMES) tR.setIterations(val);
  }
  else if (inStr.startsWith("SETCL") && inStr.length() >= 5 && inStr.length() < 8)
  { // set Current Level for switching on and off
    val = getNum(inStr.substring(5));
    if (val > 0) CURLEV = val;
  }
  else if (inStr.startsWith("DISON") && !digitalRead(OUT_Machine))
  { // Switch display on for disLightOn secs
    displayON();
    tDF.restartDelayed(TASK_SECOND * disLightOn);
  }
  else if (inStr.substring(0, 3) == "R3T" && inStr.length() >3)
  {  // print to LCD row 3
    inStr.concat("                    ");     // add blanks to string
    lcd.setCursor(0,2);
    lcd.print(inStr.substring(3,23)); // cut string lenght to 20 char
  }
  else if (inStr.substring(0, 3) == "R4T" && inStr.length() >3)
  {  // print to LCD row 4
    inStr.concat("                    ");     // add blanks to string
    lcd.setCursor(0,3);
    lcd.print(inStr.substring(3,23));   // cut string lenght to 20 char  changed by MM 10.01.2018
  }
  else
  {
    Serial.println(String(IDENT) + ";?;" + inStr);
    inStr.concat("                    ");    // add blanks to string
    lcd.setCursor(0,2);
    lcd.print("?:" + inStr.substring(0,18)); // cut string lenght to 20 char
  }
  inStr = "";
}

/* SerialEvent occurs whenever a new data comes in the
  hardware serial RX.  This routine is run between each
  time loop() runs, so using delay inside loop can delay
  response.  Multiple bytes of data may be available.
*/
void serialEvent()
 {
  char inChar = (char)Serial.read();
  if (inChar == '\x0d')
  {
    evalSerialData();
    inStr = "";
  }
  else if (inChar != '\x0a')
  {
    inStr += inChar;
  }
}
// End Funktions Serial Input -------------------

// PROGRAM LOOP AREA ----------------------------
void loop() {
  runner.execute();
}
