#include<LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>
#include <AltSoftSerial.h>
#include <TinyGPS++.h>
#include <math.h>
#include <Wire.h>
#include <SD.h>
#include <SPI.h>

#define HELP_BUTTON 7
#define ABORT_BUTTON 5
#define BUZZER 6

#define RED_LED 31
#define GREEN_LED 32
#define BLUE_LED 33

#define X A1
#define Y A2
#define Z A3

#define TX 11
#define RX 10

#define SENSITIVITY 100

#define GPS_DATA_FILENAME "gps-data.txt"
#define CONTACTS_FILENAME "contacts.txt"

SoftwareSerial sim800(TX, RX);
AltSoftSerial neogps;
TinyGPSPlus gps;
LiquidCrystal_I2C lcd(0x27, 16, 2);

boolean askHelp = false;
boolean flag = false;
boolean isAlerted = false;

String EMERGENCY_CONTACT = "+639XXXXXXXXX";
String MDRRMO = "+639XXXXXXXXX";

String URL = "https://animated-gumdrop-c7294b.netlify.app/?location=";

long latitude = 0, longitude = 0, speed = 0;

int xaxis = 0, yaxis = 0, zaxis = 0;
int magnitude = 0;

long gpsMillis = 0L;

void setup() {
  // put your setup code here, to run once:
  // initialize pins to be use
  pinMode(HELP_BUTTON, INPUT_PULLUP);
  pinMode(ABORT_BUTTON, INPUT_PULLUP);
  pinMode(BUZZER, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  
  Serial.begin(9600);
  sim800.begin(9600);
  neogps.begin(9600);

  lcd.init();
  lcd.backlight();
  lcd.blink();
  lcd.setCursor(0,0);
  lcd.print("INITIALIZING");
  lcd.setCursor(0,1);
  lcd.print("GSM.........    ");

  Serial.println("Initializing...");

  sim800.println("AT"); //Once the handshake test is successful, it will back to OK
  updateSerial();
  sim800.println("AT+CSQ"); //Signal quality test, value range is 0-31 , 31 is the best
  updateSerial();
  sim800.println("AT+CCID"); //Read SIM information to confirm whether the SIM is plugged
  updateSerial();
  sim800.println("AT+CREG?"); //Check whether it has registered in the network
  updateSerial();

  // AT command to set SIM900 to SMS mode
  sim800.print("AT+CMGF=1\r"); 
  updateSerial();
  // Set module to send SMS data to serial out upon receipt 
  sim800.print("AT+CNMI=2,2,0,0,0\r");
  updateSerial();

  lcd.setCursor(12,1);
  lcd.print("done");
  delay(1000);
  lcd.setCursor(0,1);
  lcd.print("GPS.........    ");
  delay(1000);
  initGps();
  lcd.setCursor(12,1);
  lcd.print("done");
  delay(1000);
  lcd.setCursor(0,1);
  lcd.print("SD..........    ");
  delay(1000);
  initSd();
  lcd.setCursor(12,1);
  lcd.print("done");
  delay(1000);
  initContact();
}

void loop() {
  // put your main code here, to run repeatedly:
  getLocation();

  long previousMillis = 0L;
  long helpMillis = 0L;

  if (!askHelp) {
    if (millis() - previousMillis > 1000) {
      lightItUp(LOW, HIGH, LOW);
      lcd.noBlink();
      lcd.setCursor(0,0);
      lcd.print("  SYSTEM READY  ");
      lcd.setCursor(0,1);
      lcd.print("----------------");
      Serial.println("System Ready.");
      //lightItUp(LOW, HIGH, LOW);
      previousMillis = millis();
    }
  }

  if (isPressed(HELP_BUTTON)) {
    help();
  }

  if(isCollisionDetected()) {
    askHelp = true;
    Serial.println("Collision Detected!");
    lcd.clear();
    lcd.setCursor(0,0); //col=0 row=0
    lcd.print(" CRASH DETECTED ");
    lcd.setCursor(0,1); //col=0 row=1
    lcd.print("MAGNITUDE:  " + String(magnitude));
  }

  while(askHelp) {

    lcd.setCursor(0,0); //col=0 row=0
    lcd.print("ASKING FOR HELP!");

    if (isPressed(ABORT_BUTTON))
      abort();

    long currentMillis = millis();
    if (currentMillis - previousMillis > 1000) {
      lightItUp(digitalRead(RED_LED) == HIGH ? LOW : HIGH, LOW, LOW);
      previousMillis = currentMillis;
    }
  
    if (currentMillis - helpMillis > 300000) {
      isAlerted = false;
      helpMillis = currentMillis;
    }

    if (millis() - helpMillis > 5000) {
      digitalWrite(BUZZER, LOW);
    }

    if (!isAlerted) {
      digitalWrite(BUZZER, HIGH);
      askForHelp();
      isAlerted = true;
    }

    if (millis() - gpsMillis > 5000 ) {
      Serial.println("Display GPS data in LCD I2C");
      lcd.setCursor(0, 0);
      lcd.print("Lat: " + String(latitude, 6));
      lcd.setCursor(0, 1);
      lcd.print("Lon: " + String(longitude, 6));
      delay(1000);
      lcd.setCursor(0, 0);
      lcd.print("Speed: " + String(latitude));
      lcd.setCursor(0, 1);
      lcd.print("----------------");
      delay(1000);
      gpsMillis = millis();
    }

    checkForIncomingMessages();
  }

  updateSerial();
}

boolean isPressed(uint8_t pin) {
  return digitalRead(pin) == LOW;
}

void help() {
  askHelp = true;
  Serial.println("Help button is triggered!");
}

void abort() {
  askHelp = false;
  Serial.println("Abort button is triggered!");
  lcd.setCursor(0,0); //col=0 row=0
  lcd.print("ABORT HELP ASKED");
  delay(3000);
}

void askForHelp() {
  makeCall(EMERGENCY_CONTACT);
  //String message = "Need Help. Here's my location (" + String(latitude) + ", " + String(longitude) + ")";     // non URL message
  String message = "Need Help. Locate me on map here: " + URL + String(latitude) + ", " + String(longitude);    
  sendSms(EMERGENCY_CONTACT, message);
  sendSms(MDRRMO, message);
}

// function to simply LED light management
void lightItUp(int red, int green, int blue) {
  digitalWrite(RED_LED, red);
  digitalWrite(GREEN_LED, green);
  digitalWrite(BLUE_LED, blue);
}

void sendSms(String number, String message) {
  lightItUp(LOW, LOW, HIGH);
  lcd.setCursor(0, 0);
  lcd.print("SENDING LOCATION");
  lcd.setCursor(0, 1);
  lcd.print("----------------");
  Serial.println("Sending SMS...");
  sim800.print("AT+CMGF=1\r");
  updateSerial();
  delay(100);
  sim800.print("AT+CMGS=\""+ number + "\"\r");
  updateSerial();
  delay(500);
  sim800.print(message);
  delay(500);
  sim800.print((char)26);
  updateSerial();
  delay(500);
  sim800.println();
  Serial.println("Text Sent.");
  delay(500);
  lightItUp(LOW, LOW, LOW);
}

void makeCall(String number) {
  lightItUp(LOW, LOW, HIGH);
  lcd.setCursor(0, 0);
  lcd.print("CALLING FOR HELP");
  lcd.setCursor(0, 1);
  lcd.print("----------------");
  Serial.println("calling....");
  sim800.println("ATD"+ number + ";");
  updateSerial();
  delay(20000); //20 sec delay
  sim800.println("ATH");
  updateSerial();
  delay(1000); //1 sec delay
  lightItUp(LOW, LOW, LOW);
}

void updateSerial() {
  delay(500);
  while (Serial.available()) 
  {
    sim800.write(Serial.read());
  }
  while(sim800.available()) 
  {
    Serial.write(sim800.read());
  }
}

// function to detect received messages
void checkForIncomingMessages() {
  if (sim800.available() > 0) 
  {
    decodeMessage(sim800.readString());
  }
}

void initSd() {
  Serial.print("Initializing SD card...");

  if (!SD.begin()) {
    Serial.println("failed!");
    while (1);
  }

  Serial.println("success.");
}

File createOrOpenFile(String filename) {
  if (!SD.exists(filename)) {
    File gpsFile = SD.open(filename, FILE_WRITE);
    gpsFile.close();
  }
}

// function to obtain sender number from the message and check for "GET LOCATION"
// that correspond to asking user (device owner) current location
void decodeMessage(String text) {
  String sender = text.substring(text.indexOf("+63")).substring(0,13);
  if (sender == EMERGENCY_CONTACT || sender == MDRRMO) {
    if (text.indexOf("GET LOCATION") >= 0) {
      sendSms(sender, "Need Help. Here's my location (" + String(latitude) + ", " + String(longitude) + ")");
    }
  }
}

int vibration = 2;
int devibrate = 75;

boolean isCollisionDetected() {
  
  int oldx = xaxis;
  int oldy = yaxis;
  int oldz = zaxis;

  xaxis = analogRead(X);
  yaxis = analogRead(Y);
  zaxis = analogRead(Z);
  vibration--;
  if(vibration < 0) vibration = 0;
  
  if(vibration > 0) return false;
  int deltx = xaxis - oldx;
  int delty = yaxis - oldy;
  int deltz = zaxis - oldz;
  
  magnitude = sqrt(sq(deltx) + sq(delty) + sq(deltz));
  Serial.println(magnitude);
  boolean result = false;

  if (magnitude >= SENSITIVITY) {
    result = true;
    vibration = devibrate;
  } else {
    result = false;
    magnitude=0;
  }
  return result;
}

void initGps() {
  long gpsMillis = 0L;
  while (neogps.available() > 0) {
    if (millis() - gpsMillis > 5000) {
      if (!gps.encode(neogps.read())) {
        Serial.println("No GPS data is available");
      }
      gpsMillis = millis();
    }
  }
}

void getLocation() {
  long gpsMillis = 0L;
  while (neogps.available() > 0) {
    if (millis() - gpsMillis > 5000) {
      if (gps.encode(neogps.read())) {
      
        if (gps.location.isUpdated()) {
          latitude = gps.location.lat();
          longitude = gps.location.lng();
          speed = gps.speed.kmph();

          String data = "Location: ";
          data += String(latitude, 6);
          data += ",";
          data += String(longitude, 6);
          data += " Speed: ";
          data += speed;

          data += " Date/Time: ";
          data += gps.date.month() + "/";
          data += gps.date.day() + "/";
          data += gps.date.year() + " ";
          data += gps.time.hour() + ":";
          data += gps.time.minute() + ":";
          data += gps.time.second();

          logGpsData(data, GPS_DATA_FILENAME);
        }
      }
      gpsMillis = millis();
    }
  }
}

void logGpsData(String data, String filename) {
  File gpsFile = createOrOpenFile(filename);
  Serial.println("Logging location data to gps_data.txt...");
  gpsFile.println(data);
  gpsFile.close();
  Serial.println("Done.");
}

void initContact() {
  File file;
  if (!SD.exists(CONTACTS_FILENAME)) {
    file = SD.open(CONTACTS_FILENAME, FILE_WRITE);
    file.print("EMERGENCY_CONTACT="+EMERGENCY_CONTACT+"\nMDRRMO=" + MDRRMO);
  }

  //File file = createOrOpenFile(CONTACTS_FILENAME);
  file = SD.open(CONTACTS_FILENAME, FILE_WRITE);
  if (file) {
    Serial.print("Reading to test.txt...");
    
    while (file.available()) {
      Serial.write(file.read());
    }
    
    file.close();
    Serial.println("done.");
   
    // close the file:
    file.close();
  } else {
    Serial.println("error opening test.txt");
  }
  
}