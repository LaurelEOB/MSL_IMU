#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_GPS.h>


#define inputCLK 14    // CLK pin on rotary encoder
#define inputDT 15   // DT pin on rotary encoder
#define inputBtn 16   //  SW pin on rotary encoder (pushbutton)
#define inputFix 6   //  Pin for GPS fix

#define mySerial Serial1    // Set serial port
Adafruit_GPS GPS(&mySerial);    //  Make GPS object and connect it to the port

Adafruit_BNO055 bnoSensor = Adafruit_BNO055(55);    //  Make BNO sensor object

LiquidCrystal_I2C lcd(0x27,20,4);   //  Set LCD screen

const int chipSelect = BUILTIN_SDCARD;    //  For SD card "pin"


int unixTime = 0;   // Placeholder for Posix time
int waitPeriod = 0;
int firstTwoSkip = 0;   // Used to skip the first two bad readings during a scan
File sensorUnixFile;    // Sets the file for SD card
uint32_t timer = millis();    // For timer
imu::Vector<3> euler;   // For euler measurements 


// Cheat for testing
boolean cheat = true;   // Set to false for normal, true for debugging

enum DeviceState {calibrationState, scanningState, menuState};
DeviceState deviceStates = menuState;


String pageOptions[4] = {"(1) Start Scan", "(2) Calibrate", "(3) something"};

// Defining variables for rotary encoder and button
int RotateCounter = 0; // counts the rotation clicks
bool rotated = true; // info of the rotation
bool ButtonPressed = false; // info of the button

// Statuses
int CurrentCLK;
int previousCLK;
int currentDT;
int previousDT;

// Timers
float timerOne;
float timerTwo;




void setup(){
  Serial.begin(115200);   //  Starts serial 
  pinMode(inputBtn, INPUT);   //  Sets button at input
  pinMode(inputFix, INPUT);   // Sets the fix pin as input

  //  Initialize the LCD
  lcd.init(); 
  lcd.backlight();
  
  startingText(0, 0,"Setup starting..");
  
  //  Set up GPS
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);    //  Only gets RMC NEMA sentances
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);    //  Updates at 1Hz

  //  Set up BNO055
  if(!bnoSensor.begin()){   //  If BNO can start, it does. If not...
    startingText(0, 0, "IMU not detected");  //  IMU is BNO055, check wiring or I2C ADDR 
    startingText(0, 1, "Fix and restart");
    while(1);
  }
  bnoSensor.setExtCrystalUse(true);   //  Uses external crystal for accuracy 

  // Set up SD card 
  while (!SD.begin(chipSelect)){   // Sets up SD card if it can, else...
    startingText(0, 0, "Insert SD card");
    startingText(0, 1, "Fix and restart");
    while(1);
  }
  if (SD.exists("testData.csv")) { // Delete old data files to start fresh
    SD.remove("testData.csv");
  }  
  
  if (cheat == false){   //  Has cheat to skip calibration
    getCalStatus(true);   //  Have to calibrate before use
  }
  
  startingText(0,0,"Finding fix...");
    
  while ((checkFix() == false) && (cheat == false)) {
    startingText(0, 1, checkFix());
    startingText(3, 1, digitalRead(inputFix));
  }


  // Store states
  previousCLK = digitalRead(inputCLK);
  previousDT = digitalRead(inputDT);
    
  attachInterrupt(digitalPinToInterrupt(inputCLK), checkRotate, CHANGE);
  attachInterrupt(digitalPinToInterrupt(inputBtn), checkButtonPressed, FALLING); // either falling or rising but never "change".

  timerOne = millis();    // Starts the first timer  

}


void loop(){

  switch (deviceStates){
    case menuState:
      printMenu();
      buttonChecker();
      break;
    case scanningState:
      scan();
      buttonChecker();
      break;
    case calibrationState:
      getCalStatus(true);
      deviceStates = menuState;
      rotated = true;
      break;
  }
}

boolean checkFix (){
  int blinks = 0;
  for (int i = 0; i < 10; i++){
    if (digitalRead(inputFix) == HIGH){
      blinks++;
    }
    delay(300);
  }
  if (blinks < 2){
    return true;
  } else {
    return false;
  }
}


void startingText (int x, int y, String string){    // Used to print to both the lcd and Serial 
   lcd.setCursor(x,y);
   lcd.print(string);
   Serial.println(string);
}



void menuText(int x, int y, String text){   // Used for the menu
  lcd.setCursor(x,y);
  lcd.print(text);
}



void printMenu(){   // Prints out the correct page depending on rotation
  if (rotated == true){   // If the rotary encoder has been moved...
    lcd.clear();
    menuText(0, 0, ">");
    
    switch (RotateCounter){   // Decides which page to set the LCD to
      case 0:
        menuText (1, 0, pageOptions[0]);
        menuText (1, 1, pageOptions[1]);
        break;
      case 1:
        menuText (1, 0, pageOptions[1]);
        menuText (1, 1, pageOptions[2]);
        break;
      case 2:
        menuText (1, 0, pageOptions[2]); 
        menuText (1, 1, pageOptions[0]);        
       break;
    }
    rotated = false;
  }
}



void scan(){
  char c = GPS.read();

  if (GPS.newNMEAreceived()){    //  if a sentence is received, parse it...
  if (!GPS.parse(GPS.lastNMEA()))   //  Sets the newNMEAreceived() flag to false
     return;  // Wait for another sentance
  }
  unixTime = unixConvert(GPS.year, GPS.month, GPS.day, GPS.hour, GPS.minute, GPS.seconds); // Get Unix time
  euler = bnoSensor.getVector(Adafruit_BNO055::VECTOR_EULER);

  // Timer that runs about every second
  if ((millis() - timer > 1000) && (waitPeriod == 3)){
    timer = millis(); //  reset the timer

    if (firstTwoSkip < 3)
      firstTwoSkip++;
    else
      timerLog(unixTime, GPS.year, GPS.month, GPS.day, GPS.hour, GPS.minute, GPS.seconds);
      
  } else if (waitPeriod < 3)
    waitPeriod++;
    
}



int unixConvert (int year, int month, int day, int hour, int minute, int second){
  int total = 1609459200;   // Starts at Jan 1, 2021

  for (int i = 2021; i < year; i++){   // Adds the seconds of the past full years
    if ((i % 4) == 0){
      total += (366 * 86400);
    } else {
      total += (365 * 86400);
    }
  }

  for (int i = 1; i < month; i++){   // Adds the seconds of the past full months in the current year
    switch(i){
      case 1 :
      case 3 :
      case 5 :
      case 7 :
      case 8 :
      case 10 :
      case 12 :
        total += (31 * 86400);
        break;
      case 4 :
      case 6 :
      case 9 :
      case 11 :
        total += (30 * 86400);
        break;
      case 2 :
        total += (28 * 86400);
        break;
    }  
   }
 
  total += (day*86400)-86400;   // Adds the days in the current month
  total += (hour*3600);   // Adds the hours in the current day
  total += (minute*60);   // Adds the minutes in the current hour
  total += second;    // Adds the seconds in the current minute

  // Adds a day's worth of seconds if it is a leap year and it is past the leap day
  if ((year % 4 == 0)&&(month >= 3)){
    total += 86400;
  }
  
  return total;   // Returns the current time in a Unix timestamp
}



void timerLog (int unix, int GPSyear, int GPSmonth, int GPSday, int GPShour, int GPSmin, int GPSsec){
   char namefile[15] = "testData.csv";
   sensorUnixFile = SD.open(namefile, FILE_WRITE);
   sensorUnixFile.print(unix);
   sensorUnixFile.print(", ");
   sensorUnixFile.print(euler.x());
   sensorUnixFile.print(", ");
   sensorUnixFile.print(euler.y());
   sensorUnixFile.print(", ");
   sensorUnixFile.print(euler.z());
   sensorUnixFile.print(", ");
   sensorUnixFile.println(getCalStatus(false));
   sensorUnixFile.close();
   
   if (GPS.fix)
     startingText(0, 0, "Fix: True        ");
   else 
     startingText(0, 0, "Fix: False       ");
   lcd.setCursor(0,1);
   lcd.print("Time:");
   lcd.setCursor(6,1);
   lcd.print(unixTime);
    
   Serial.print ("Time: ");
   Serial.println(unixTime);

   Serial.print ("Year: ");
   Serial.print (GPSyear);
   Serial.print ("Month: ");
   Serial.print (GPSmonth);
   Serial.print ("Day: ");
   Serial.print (GPSday);
   Serial.print ("Hour: ");
  Serial.print (GPShour);
   Serial.print ("Minute: ");
   Serial.print (GPSmin);
   Serial.print ("Second: ");
   Serial.println (GPSsec);
   Serial.println(" ");
}



String getCalStatus(boolean calibrateSetting){
  uint8_t System, gyro, accel, mago;
  System = gyro = accel = mago = 0;
  bnoSensor.getCalibration(&System, &gyro, &accel, &mago);    // Gets the states of calibration (3 = full, 0 = none)

   
  
  if (calibrateSetting == true){    // Lets you calibrate the device, used in start and option in menu
    lcd.clear();
    startingText(0, 0, "Calibrate (333)");
    
    while (!((System == 3) && (mago == 3) && (gyro == 3))){   // Waits untill the button is pressed or until fully calibrated
      delay(800); 
      bnoSensor.getCalibration(&System, &gyro, &accel, &mago);
      startingText(0, 1, "S:");
      startingText(3, 1, System);
      startingText(6, 1, "M:");
      startingText(9, 1, mago);
      startingText(12, 1, "G:");
      startingText(15, 1, gyro);
      buttonChecker();
    }
    
    lcd.clear();
  }

  // Printout for debugging
  Serial.print("Cal:  ");   
  Serial.print("Sys:");
  Serial.print(System, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.println(mago, DEC);
  if (!System){
    Serial.print("! ");
  }
  
  String calPrintOut = "Cal: ";   // Returns the calibration status to log in the csv file
  calPrintOut += System;
  calPrintOut += gyro;
  calPrintOut += accel;
  calPrintOut += mago;
  return calPrintOut;
}



void checkButtonPressed(){    // The timer is called a "software debounce"
  timerTwo = millis();
  if(timerTwo - timerOne > 500){    // Checks the encoder button twice to make sure it was fully pushed    
    ButtonPressed = true;    
  }
  timerOne = millis();    // This resets the timer
}



void checkRotate(){
  CurrentCLK = digitalRead(inputCLK);   // Reads the current state of the CLK pin

  if (CurrentCLK != previousCLK  && CurrentCLK == 1){   // If last and current state of CLK are different, then a pulse occurred  
    
    if (digitalRead(inputDT) != CurrentCLK){    // If the DT state is different than the CLK state then it is rotating CCW       
      RotateCounter++; 
      if(RotateCounter > 2)   // Menu loops
        RotateCounter = 0;
    } else {    // Anything else means it is rotating CW        
      RotateCounter--;
      if(RotateCounter < 0)   // Menu loops
        RotateCounter = 2;
    }   
    
  }   
  
  previousCLK = CurrentCLK;   // Current state is now the next previous state
  rotated = true;
}



void buttonChecker(){ 
  if(ButtonPressed == true){   // If the button is pressed
    switch(deviceStates){
      case menuState:   // If it is in the menu state, button changes states
        switch(RotateCounter){
          case 0:
            deviceStates = scanningState; 
            lcd.clear();    
            break;
          case 1:
            deviceStates = calibrationState;
            break;
        }
        break;
      case scanningState:   // If it is in the scannning state, button stops the scan
        deviceStates = menuState;
        rotated = true;
        firstTwoSkip = 0;
        break;
    }
  }  
  ButtonPressed = false; // Resets this variable
}
