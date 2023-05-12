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

#define GPS_DATA_DIR "gps-data"

SoftwareSerial sim800(TX, RX);
AltSoftSerial neogps;
TinyGPSPlus gps;
LiquidCrystal_I2C lcd(0x27, 16, 2);

boolean askHelp = false;
boolean isAlerted = false;

String EMERGENCY_CONTACT = "+639XXXXXXXXX"; // change this to reflect the actual Emergency Number
String MDRRMO = "+639XXXXXXXXX"; // change this to reflect the actual MDRRMO
String THIRD_NUMBER = "+639XXXXXXXXX"; // change this to reflect the actual MDRRMO

String URL = "https://vehikulo.netlify.app/?location=";

String currentDate = "";

File gpsFile;

double latitude = 0.0, longitude = 0.0, speed = 0.0;

int xaxis = 0, yaxis = 0, zaxis = 0;
int magnitude = 0;
const int THRESHOLD = 25;
const int DELAY_MS = 100;

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

  if(isAccidentDetected()) {
    askHelp = true;
    Serial.println("Accident detected.");
    lcd.clear();
    lcd.setCursor(0,0); //col=0 row=0
    lcd.print(" CRASH DETECTED ");
    lcd.setCursor(0,1); //col=0 row=1
    //lcd.print("MAGNITUDE:  " + String(magnitude));
    lcd.print("----------------");
  }

  if (askHelp) {
    long abortMillis = 10000L;
    while (abortMillis >= 0) {
      lcd.setCursor(0,0);
      lcd.print("ASKING HELP IN..");
      lcd.setCursor(0,1);
      lcd.print("..." + String((abortMillis)/1000)  + "           ");
      if (isPressed(ABORT_BUTTON))
        abort();

      abortMillis -= 1000;
      delay(1000);
    }
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

    checkForIncomingMessages();
  }

  if (millis() - gpsMillis > 5000 ) {
    Serial.println("Display GPS data in LCD I2C");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Lat: " + String(latitude, 6));
    lcd.setCursor(0, 1);
    lcd.print("Lon: " + String(longitude, 6));
    delay(1000);
    lcd.setCursor(0, 0);
    lcd.print("Speed: " + String(speed));
    lcd.setCursor(0, 1);
    lcd.print("----------------");
    delay(1000);
    gpsMillis = millis();
  }

  checkForIncomingMessages();
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
  //String message = "Need Help. Here's my location (" + String(latitude, 6) + "," + String(longitude, 6) + ")";     // non URL message
  String message = "Need Help. Locate me on map here: " + URL + String(latitude, 6) + "," + String(longitude, 6);    
  sendSms(EMERGENCY_CONTACT, message);
  sendSms(MDRRMO, message);
  sendSms(THIRD_NUMBER, message);
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
  sim800.print(message);
  sim800.print((char)26);
  updateSerial();
  delay(5000);
  Serial.println("Text Sent.");
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
  
  String filename = String(GPS_DATA_DIR) + "/" + currentDate + ".txt";

  while(!SD.begin()) {
    Serial.print("Initializing SD card...");
    delay(1000);
    Serial.println("failed!");
  }
  
  Serial.print("Initializing SD card...");
  delay(1000);
  Serial.println("success.");

  Serial.print("Creating gps-data directory...");
  if (!SD.exists(GPS_DATA_DIR))
    SD.mkdir(GPS_DATA_DIR);
  delay(1000);
  Serial.println("success.");

  Serial.print("Creating/Opening " + filename + "...");
  gpsFile = createOrOpenFile(filename);
  delay(1000);
  Serial.println("success.");
}

File createOrOpenFile(String filename) {
  return SD.open(filename, FILE_WRITE); 
}

// function to obtain sender number from the message and check for "GET LOCATION"
// that correspond to asking user (device owner) current location
void decodeMessage(String text) {
  String sender = text.substring(text.indexOf("+63")).substring(0,13);
  if (sender == EMERGENCY_CONTACT || sender == MDRRMO) {
    if (text.indexOf("GET LOCATION") >= 0) {
      sendSms(sender, "Need Help. Locate me on map here: " + URL + String(latitude, 6) + "," + String(longitude, 6));
    }
  }
}

boolean isAccidentDetected() {

  xaxis = analogRead(X);
  yaxis = analogRead(Y);
  zaxis = analogRead(Z);

  // Get acceleration value uisng 3 accelerometer value
  int accel_value = sqrt(sq(xaxis)+sq(yaxis)+sq(zaxis));

  // Convert the acceleration sensor value to mph
  float mph_value = (accel_value * 0.0049 * 2.23694);
  
  // Calculate the deceleration in mph/s
  float deceleration = mph_value / (float(DELAY_MS) / float(1000));

  // Check if the deceleration exceeds the threshold
  return deceleration >= THRESHOLD;
}

void initGps() {
  while(latitude == 0) {
    while(neogps.available()) {
      if (gps.encode(neogps.read())) {
        latitude = gps.location.lat();
        longitude = gps.location.lng();

        currentDate += String(gps.date.month());
        currentDate += String(gps.date.day());
        currentDate += String(gps.date.year());
      }
    }
  }
}

void getLocation() {
  while (neogps.available() > 0) {
    if (gps.encode(neogps.read())) {
      getInfo();
    }
  }
}

void getInfo() {
  if (gps.location.isValid()) {

    // skip latitude and longitude values if match from previous values
    if (gps.location.lat() == latitude && gps.location.lng() == longitude) 
      return;

    latitude = gps.location.lat();
    longitude = gps.location.lng();
    speed = gps.speed.kmph();

    String locationData = "Location: ";
    locationData += String(latitude, 6);
    locationData += ",";
    locationData += String(longitude, 6);
    locationData += " Speed: ";
    locationData += speed;

    String dateData = " Date/Time: ";
    if (gps.date.isValid()) {
      dateData += gps.date.month();
      dateData += "/";
      dateData += gps.date.day();
      dateData += "/";
      dateData += gps.date.year();
    }

    String timeData = " ";
    if (gps.time.isValid()) {
      timeData += gps.time.hour();
      timeData += ":";
      timeData += gps.time.minute();
      timeData += ":";
      timeData += gps.time.second();
    }

    String data = locationData + dateData + timeData;
    Serial.println(data);

    logGpsData(data);
    delay(1000);

  } else {
    Serial.println("No GPS location data.");
  }
}

void logGpsData(String data) {
  if (gpsFile) {
    Serial.println("Logging location data...");
    gpsFile.println(data);
    gpsFile.close();
    Serial.println("Done.");
  } else {
    Serial.println("Error logging location data...");
    initSd();
  }
}