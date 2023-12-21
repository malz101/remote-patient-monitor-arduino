#include <Wire.h>
#include <LiquidCrystal.h>
#include <SoftwareSerial.h>
#include "MAX30105.h"
#include "heartRate.h"

/*NEED FOR SERVER COMMUNICATION*/
//#define HOST_IP "192.168.100.69"    //IP address of HOST server
//#define HOST_PORT "3000"     //Port for HOST server
//#define patientID "0001"
#define espRX A2
#define espTX A3
#define espTimeout_ms 3000
uint32_t tsLastReport_esp = 0;

const char string_0[] PROGMEM = "AT+CIPSTART=\"TCP\",\""; // "String 0" etc are strings to store - change to suit.
const char string_1[] PROGMEM = "192.168.100.69";
const char string_2[] PROGMEM = "3000";
const char string_3[] PROGMEM = "0001";
const char string_4[] PROGMEM = "POST /Insert HTTP/1.1\r\nHost: ";
const char string_5[] PROGMEM = "\r\nContent-Type: application/x-www-form-urlencoded\r\nContent-Length: ";
const char string_6[] PROGMEM = "\r\n\r\n";
const char string_7[] PROGMEM = "AT+CIPSEND=";
const char string_8[] PROGMEM = "AT+CIPCLOSE\r\n";
const char string_9[] PROGMEM = "Initializing...";
const char string_10[] PROGMEM = "id=";
const char string_11[] PROGMEM = "&hr=";
const char string_12[] PROGMEM = "&temp=";
const char string_13[] PROGMEM = "&pos=";  
const char string_14[] PROGMEM = "&alert=";
const char string_15[] PROGMEM = "&accl=";
const char string_16[] PROGMEM = "HELP!";
const char string_17[] PROGMEM = "FELL!";                     
const char string_18[] PROGMEM = "bpm";

const char *const string_table[] PROGMEM = {string_0, string_1, string_2, string_3, string_4, string_5, string_6, string_7, string_8, string_9, string_10, string_11, string_12,string_13, string_14, string_15, string_16, string_17, string_18};

char buffer[75];

SoftwareSerial esp(espRX, espTX);

String params = "";
String paramsOld = "";
String espCommandString = "";


/*CREATE LCD OBJECT*/
//sets how often new value should be output to LCD
#define REPORTING_PERIOD_MS 1000
uint32_t tsLastReport = 0;//last time information was printed to LCD
//const int rs = 12, en = 11, d4 = 7, d5 = 6, d6 = 5, d7 = 4;
LiquidCrystal lcd (12, 11, 7, 6, 5, 4);
int printCount = 0; //number of times one set of values are printed to LCD


/*CREATE PULSE MONITOR OBJECT*/
MAX30105 particleSensor;

const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred

float beatsPerMinute;
int beatAvg;//heart rate of patient as integer
String heart_rate="";//heart rate of patient as String
String prev = "";


/*VARIABLES FOR TEMPERATURE SENSOR*/
const byte tempPin = A0;
float lm_temp;//temperature of patient as float
float print_temp;
float vout;
//String temperature="";//temperature of patient as String

/*VARIABLES FOR MPU-6050*/
const int MPU_addr = 0x68;  // I2C address of the MPU-6050
const int num_bytes = 14;

//Variables for Gyroscope
int gyX, gyY, gyZ;
long gyX_cal, gyY_cal, gyZ_cal;
boolean set_gyro_angles;

//Variables for Accelerometer
long acX, acY, acZ, acc_total_vector;
float angle_roll_acc, angle_pitch_acc;

float angle_pitch, angle_roll;
float angle_pitch_output, angle_roll_output;
//String  patient_accl="";

// Setup timers and temperature variables
long loop_timer;

/*VARIABLES FOR PATIENT POSITION*/
String pos;

/*VARIABLES FOR PATIENT FALL*/
float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
boolean fall = false; //stores if a fall has occurred
boolean trigger1 = false; //stores if first trigger (lower threshold) has occurred
boolean trigger2 = false; //stores if second trigger (upper threshold) has occurred
boolean trigger3 = false; //stores if third trigger (orientation change) has occurred

byte trigger1count = 0; //stores the counts past since trigger 1 was set true
byte trigger2count = 0; //stores the counts past since trigger 2 was set true
byte trigger3count = 0; //stores the counts past since trigger 3 was set true
int angleChange = 0;
const byte fallBut = 3;
int AM=0;


/*VARIABLES FOR ALERT*/
const int buzz_freq = 700;//stores buzzer frequency
const byte buzzPin = 9;
const byte butPin = 2;
//const byte led = 10;
boolean alertTrig = false;
String bpmAlert = "";
String tempAlert = "";
String alertString = "";


void setup() {
  //Serial.begin(115200);
  pinMode(butPin, INPUT);
  pinMode(buzzPin, OUTPUT);

  //response.reserve(400);
  espInit();
  initInterrupts();
  initLCD();

  Wire.begin();
  initPox();

  setupMPU();
  calibrateMPU();

  //loop_timer = micros();
}

void loop() {
  updateLCD();
  getPulseRate();
  getTemperature();
  getMpuData();
  correctMpuData();
  processPositionData();
  getSleepPosition();
  processAcclData();
  checkConditions();
  getParams();
  espUpdate();

  //Serial.print("Time: ");
  //Serial.println(micros()- loop_timer);
  //Reset the loop timer
  //loop_timer = micros();
}


void initInterrupts() {
  pinMode(butPin, INPUT);
  pinMode(fallBut, INPUT);
  attachInterrupt(digitalPinToInterrupt(butPin), alert, RISING);
  attachInterrupt(digitalPinToInterrupt(fallBut), alertResponse, RISING);
}


void initLCD() {
  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.print(strcpy_P(buffer, (char *)pgm_read_word(&(string_table[9]))));//Initializing
}

void initPox() {
  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    while (1);
  }

  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED
}


void updateLCD() {

  if (millis() - tsLastReport > REPORTING_PERIOD_MS) {

    lcd.clear();

    lcd.setCursor(0, 0);
    lcd.print("H: ");

    lcd.setCursor(3, 0);

//    if (printCount == 7) {
//      printCount = 0;
//      prev = heart_rate;
//      lcd.print(prev + strcpy_P(buffer, (char *)pgm_read_word(&(string_table[18])))+ bpmAlert);
//    }
//    else {
      lcd.print(heart_rate + strcpy_P(buffer, (char *)pgm_read_word(&(string_table[18]))) + bpmAlert);//"bpm"
//    }


    lcd.setCursor(0, 1);
    lcd.print("T: ");
    lcd.setCursor(3, 1);
    // print the number of seconds since reset:
    lcd.print(String(lm_temp) + tempAlert);
    print_temp=lm_temp;

    lcd.setCursor(10, 0);
    lcd.print("P: " + pos);

    lcd.setCursor(10, 1);
    lcd.print(alertString);
    //lcd.setCursor(5,1);
    //lcd.print(String(patient_accl)+" "+alertString);

    printCount++;

    //stores time of last lcd update
    tsLastReport = millis();
  }//IF Millis

}//END updateLCD


void getPulseRate() {
  long irValue = particleSensor.getIR();

  if (checkForBeat(irValue) == true) {
    //We sensed a beat!
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20) {
      rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
      rateSpot %= RATE_SIZE; //Wrap variable

      //Take average of readings
      beatAvg = 0;
      //heart_rate="";
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      beatAvg /= (RATE_SIZE);
      heart_rate=String(beatAvg);
      //Serial.print("Heart Rate is: "+heart_rate+"\tBPM avg is: ");
      //Serial.println(beatAvg);
    }
  }


  if (irValue < 50000) { //No finger Detected
    beatAvg = 0;
    heart_rate="";
  }

}

void getTemperature() {
  
  lm_temp = 0.0;
  //lm_temp = (5 * vout * 100) / 1023;
  for (byte x = 0 ; x < 3 ; x++){
    vout = analogRead(tempPin);
    lm_temp += (5 * vout * 100) / 1023;
  }
  
  lm_temp/=3;
  
  //Serial.print("LM TEMP is: ");
  //Serial.println(lm_temp);
  //temperature = String(lm_temp);
}


void alert() {
  alertString = strcpy_P(buffer, (char *)pgm_read_word(&(string_table[16])));//"HELP!";
  //sounds buzzer if alert is triggered by patientm
  analogWrite(buzzPin, buzz_freq);
}

void alertResponse() {
  alertString = "";
  tempAlert = "";
  bpmAlert = "";
  //disables buzzer
  analogWrite(buzzPin, 0);
}


void setupMPU() {

  //Activate the MPU-6050

  //Start communicating with the MPU-6050
  Wire.beginTransmission(MPU_addr);
  //Send the requested starting register
  Wire.write(0x6B);
  //Set the requested starting register
  Wire.write(0x00);
  //End the transmission
  Wire.endTransmission();

  //Configure the accelerometer (+/-8g)

  //Start communicating with the MPU-6050
  Wire.beginTransmission(MPU_addr);
  //Send the requested starting register
  Wire.write(0x1C);
  //Set the requested starting register
  Wire.write(0x10);
  //End the transmission
  Wire.endTransmission();

  //Configure the gyro (500dps full scale)

  //Start communicating with the MPU-6050
  Wire.beginTransmission(MPU_addr);
  //Send the requested starting register
  Wire.write(0x1B);
  //Set the requested starting register
  Wire.write(0x08);
  //End the transmission
  Wire.endTransmission();

}


void getMpuData() {
  int mpu_temp;
  //Read the raw gyro and accelerometer data

  //Start communicating with the MPU-6050
  Wire.beginTransmission(MPU_addr);
  //Send the requested starting register
  Wire.write(0x3B);
  //End the transmission
  Wire.endTransmission();
  //Request 14 bytes from the MPU-6050
  Wire.requestFrom(MPU_addr, num_bytes);
  //Wait until all the bytes are received
  while (Wire.available() < num_bytes);

  //Following statements left shift 8 bits, then bitwise OR.
  //Turns two 8-bit values into one 16-bit value
  acX = Wire.read() << 8 | Wire.read();
  acY = Wire.read() << 8 | Wire.read();
  acZ = Wire.read() << 8 | Wire.read();
  mpu_temp = Wire.read() << 8 | Wire.read();
  gyX = Wire.read() << 8 | Wire.read();
  gyY = Wire.read() << 8 | Wire.read();
  gyZ = Wire.read() << 8 | Wire.read();
}


void calibrateMPU() {

  //Read the raw acc and gyro data from the MPU-6050 1000 times
  for (int cal_int = 0; cal_int < 1000 ; cal_int ++) {
    getMpuData();
    //Add the gyro x offset to the gyro_x_cal variable
    gyX_cal += gyX;
    //Add the gyro y offset to the gyro_y_cal variable
    gyY_cal += gyY;
    //Add the gyro z offset to the gyro_z_cal variable
    gyZ_cal += gyZ;

    //Delay 5us to have 50Hz for-loop
    delay(5);
  }

  // Divide all results by 1000 to get average offset
  gyX_cal /= 1000;
  gyY_cal /= 1000;
  gyZ_cal /= 1000;
  // Init Timer
  //loop_timer = micros();
}


void getSleepPosition() {

  if (acZ > 3700) {
    pos = "FU";
  }
  else if (acZ < -3700) {
    pos = "FD";
  }
  else if (angle_pitch_output < -25 and angle_pitch_output  > -120) {
    pos = "UpR";
  }
  else if (angle_roll_output > 40 and angle_roll_output < 180) {
    pos = "L";
  }
  else if (angle_roll_output < -30 and angle_roll_output > -110 ) {
    pos = "R";
  }
//      Serial.print("Pitch: ");
//      Serial.print(angle_pitch_output);
//      Serial.print("\tRoll: ");
//      Serial.print(angle_roll_output);
//      Serial.print("AcZ: ");
//      Serial.println(acZ);
}//End getPatient


void correctMpuData() {
  //Subtract the offset values from the raw gyro values
  gyX -= gyX_cal;
  gyY -= gyY_cal;
  gyZ -= gyZ_cal;
}//correctMpuData


void processPositionData() {

  //50Hz = 1/loop_time
  //Gyro angle calculations . Note .0002993564 = 1 / (50Hz x 65.5)
  //Calculate the traveled pitch angle and add this to the angle_pitch variable
  angle_pitch += gyX * 0.0002993564;


  //Calculate the traveled roll angle and add this to the angle_roll variable
  angle_roll += gyY * 0.0002993564;


  //0.000005225 = 0.0002993564 * (PI / 180degr) The Arduino sin function is in radians
  //If the IMU has yawed transfer the roll angle to the pitch angle
  angle_pitch += angle_roll * sin(gyZ * 0.000005225);
  //If the IMU has yawed transfer the pitch angle to the roll angle
  angle_roll -= angle_pitch * sin(gyZ * 0.000005225);


  //Accelerometer angle calculations

  //Calculate the total accelerometer vector
  acc_total_vector = sqrt((acX * acX) + (acY * acY) + (acZ * acZ));

  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  //Calculate the pitch angle
  angle_pitch_acc = asin((float)acY / acc_total_vector) * 57.296;
  //Calculate the roll angle
  angle_roll_acc = asin((float)acX / acc_total_vector) * -57.296;

  //Accelerometer calibration value for pitch
  angle_pitch_acc -= -3.30;//-1.45;
  //Accelerometer calibration value for roll
  angle_roll_acc -= -4.40;//-6.35;


  if (set_gyro_angles) {

    //If the IMU has been running
    //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;

    //Correct the drift of the gyro roll angle with the accelerometer roll angle
    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;
  }
  else {
    //IMU has just started
    //Set the gyro pitch angle equal to the accelerometer pitch angle
    angle_pitch = angle_pitch_acc;
    //Set the gyro roll angle equal to the accelerometer roll angle
    angle_roll = angle_roll_acc;
    //Set the IMU started flag
    set_gyro_angles = true;
  }

  //To dampen the pitch and roll angles a complementary filter is used
  //Take 90% of the output pitch value and add 10% of the raw pitch value
  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;

  //Take 90% of the output roll value and add 10% of the raw roll value
  angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;

}//processPositionData


void processAcclData() {

  ax = acX / 16384.00;
  ay = acY / 16384.00;
  az = acZ / 16384.00;

  //270, 351, 136 for gyroscope
  gx = gyX / 131.07;
  gy = gyY / 131.07;
  gz = gyZ / 131.07;

  // calculating Amplitute vactor for 3 axis
  float Raw_AM = pow(ax * ax + ay * ay + az * az, 0.5);
  AM = Raw_AM * 10;  // as values are within 0 to 1, I multiplied
  //patient_accl = String(AM);

  //  Serial.print("AM: ");
  //  Serial.println(AM);

  if (trigger3 == true) {
    trigger3count++;
    //       Serial.print("Trig 3 Angle Change: ");
    //       Serial.println(angleChange);
    if (trigger3count >= 5) {
      angleChange = pow(pow(gx, 2) + pow(gy, 2) + pow(gz, 2), 0.5);
      //delay(10);

      if ((angleChange >= 0) && (angleChange <= 10)) { //if orientation changes remains between 0-10 degrees
        fall = true;
        trigger3 = false;
        trigger3count = 0;
      }
      else { //user regained normal orientation
        trigger3 = false;
        trigger3count = 0;
      }
    }
  }


  if (fall == true) { //in event of a fall detection
    fall = false;
  }

  if (trigger2count >= 6) { //allow 0.5s for orientation change
    trigger2 = false;
    trigger2count = 0;
  }


  if (trigger1count >= 6) { //allow 0.5s for AM to break upper threshold
    trigger1 = false;
    trigger1count = 0;
  }

  if (trigger2 == true) {
    trigger2count++;

    //angleChange=acos(((double)x*(double)bx+(double)y*(double)by+(double)z*(double)bz)/(double)AM/(double)BM);
    angleChange = pow(pow(gx, 2) + pow(gy, 2) + pow(gz, 2), 0.5);

    //         Serial.print("Trig 2 Angle Change: ");
    //     Serial.println(angleChange);
    if (angleChange >= 20 && angleChange <= 400) { //if orientation changes by between 80-100 degrees
      trigger3 = true;
      trigger2 = false;
      trigger2count = 0;
      alertString = strcpy_P(buffer, (char *)pgm_read_word(&(string_table[17])));//"FELL!";
    }
  }

  if (trigger1 == true) {
    trigger1count++;

    if (AM >= 8) { //if AM breaks upper threshold (3g)
      trigger2 = true;
      trigger1 = false;
      trigger1count = 0;
    }
  }

  if (AM <= 2 && trigger2 == false) { //if AM breaks lower threshold (0.4g)
    trigger1 = true;
  }

}


void checkConditions() {
  if (alertString == strcpy_P(buffer, (char *)pgm_read_word(&(string_table[17])))) {//"Fell!"
    analogWrite(buzzPin, buzz_freq);
  }

  if (print_temp < 20 || print_temp > 40) {
    tempAlert = "!!";
    analogWrite(buzzPin, buzz_freq);
  }

  if (beatAvg > 160) {
    bpmAlert = "!!";
    analogWrite(buzzPin, buzz_freq);
  }

}


/*FUNCTIONS FOR COMMUNICATION*
  WEB WITH SERVER
 *****************************/

void espSend(String command) {
  //String response = "";
  char responseChar = 0;
  //response="";
  esp.print(command); // send the read character to the esp

  //unsigned long time = millis();

  //while ((millis() - time) < 300) {
    //while (esp.available() > 0) {
      //responseChar = (esp.read());
      //response += responseChar;
    //}
  //}
  //Serial.print("New Response is: ");
  //Serial.println(response);
}

void startServerCommunication() {

  espCommandString = strcpy_P(buffer, (char *)pgm_read_word(&(string_table[0])));//"AT+CIPSTART=\"TCP\",\""
  espCommandString += strcpy_P(buffer, (char *)pgm_read_word(&(string_table[1])));
  espCommandString += "\",";
  espCommandString += strcpy_P(buffer, (char *)pgm_read_word(&(string_table[2])));
  espCommandString += ",1";
  espCommandString += "\r\n";
  //Serial.print(espCommandString);
  espSend(espCommandString);    //starts the connection to the server
  //delay(10);
  espCommandString = "";
}

void espUpdate() {
  String post;
  if (millis() - tsLastReport_esp > espTimeout_ms) {
      //response  = "";
    //if(!paramsOld.equals(params)){//only updates if one of the parameters has changed
      //paramsOld = params;
      startServerCommunication();//starts communication to server
  
      post = strcpy_P(buffer, (char *)pgm_read_word(&(string_table[4])));"POST /Insert HTTP/1.1\r\nHost: ";
      post += strcpy_P(buffer, (char *)pgm_read_word(&(string_table[1])));"192.168.100.69";
      post += strcpy_P(buffer, (char *)pgm_read_word(&(string_table[5])));"\r\nContent-Type: application/x-www-form-urlencoded\r\nContent-Length: ";
      post += params.length();
      post += strcpy_P(buffer, (char *)pgm_read_word(&(string_table[6])));;"\r\n\r\n";
      post += params;
      post += strcpy_P(buffer, (char *)pgm_read_word(&(string_table[6])));;"\r\n\r\n";
  
      //espCommandString ="";
      espCommandString = strcpy_P(buffer, (char *)pgm_read_word(&(string_table[7])));//"AT+CIPSEND=";
      espCommandString += post.length();
      espCommandString += "\r\n";
  
      espSend(espCommandString);//sends post length
      //delay(10);
      espCommandString = "";
  
      espSend(post);//sends POST request with the parameters
      //delay(10);
      //parseResponse();//extracts data from server response
  
      espSend(strcpy_P(buffer, (char *)pgm_read_word(&(string_table[8]))));//"AT+CIPCLOSE\r\n");   //closes server connection
      //delay(10);
    //}//End of (!paramsOld.equals(params)
    
    //stores time of last esp update
    tsLastReport_esp = millis();
  }//End of (millis() - tsLastReport_esp > espTimeout_ms)
}

void espInit() {
  esp.begin(115200);
  //Serial.println("Initialized...");

  while (esp.available()) {
    esp.read();
    delay(50);
  }

  //esp.print("AT+CWJAP?\r\n");

  delay(1000);  //gives ESP some time to get IP
}

void getParams() {

  params = strcpy_P(buffer, (char *)pgm_read_word(&(string_table[10])));//"id=";
  params += strcpy_P(buffer, (char *)pgm_read_word(&(string_table[3])));//patientID;
  //params += "0001";
  params += strcpy_P(buffer, (char *)pgm_read_word(&(string_table[11])));//"&hr=";
  params += heart_rate;

  params += strcpy_P(buffer, (char *)pgm_read_word(&(string_table[12])));//"&temp=";
  params += String(lm_temp);


  params += strcpy_P(buffer, (char *)pgm_read_word(&(string_table[13])));//"&pos=";
  params += pos;

  params += strcpy_P(buffer, (char *)pgm_read_word(&(string_table[14])));//"&alert=";

  params += alertString;
  params += strcpy_P(buffer, (char *)pgm_read_word(&(string_table[15])));//"&accl=";
  params += String(AM);
}
