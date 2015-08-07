#include <Wire.h>
#include <Adafruit_INA219.h>
#include <Adafruit_BMP085.h>

#define ADDRESS 0x76 //0x76

Adafruit_INA219 ina219_1(0x40); //none
Adafruit_INA219 ina219_2(0x44); //A1
Adafruit_BMP085 bmp;

char buff[50];
int fileNumber;

int statLED = 13;
boolean statusled = HIGH;
int resetOpenLog =2;

unsigned long delta_t, timer;

byte _buff[6];

uint32_t D1 = 0;
uint32_t D2 = 0;
int64_t dT = 0;
int32_t TEMP = 0;
int64_t OFF = 0; 
int64_t SENS = 0; 
int32_t P = 0;
uint16_t C[7];

float Temperature;
float Pressure;

void setup(void) {
  
  // Disable internal pullups, 10Kohms are on the breakout
  PORTC |= (1 << 4);
  PORTC |= (1 << 5);

  Wire.begin();
  Serial.begin(9600); //9600 changed 'cos of timing?
  delay(100);
  initial(ADDRESS);
  
  Serial.begin(9600);
  delay(500);
  Serial1.begin(9600);
  delay(500);
  Serial2.begin(9600);
  delay(500);
  set_SDcard();
  delay(500);
  ina219_1.begin();
  delay(20);
  ina219_2.begin();
  delay(200);
  initial(ADDRESS);
  Serial.println("GO!");
}

void loop(void){
  delta_t = millis() - timer; // calculate time through loop i.e. acq. rate
  Serial3.print(delta_t);
  Serial3.print(",");
  statusled = !statusled;
  getcurrent();
  BMP085_routine();
  MS5611_routine();
  digitalWrite(13, statusled);

}

void set_SDcard() {
  pinMode(statLED, OUTPUT);
  pinMode(resetOpenLog, OUTPUT);
  randomSeed(analogRead(0));
  delay(500);
  Serial3.begin(9600);
  delay(500);
  //Reset OpenLog
  digitalWrite(resetOpenLog, LOW);
  delay(100);
  digitalWrite(resetOpenLog, HIGH);

  //Wait for OpenLog to respond with '<' to indicate it is alive and recording to a file
  while(1) {
    if(Serial3.available())
      if(Serial3.read() == '<') break;
  }
  //Send three control z to enter OpenLog command mode
  //This is how Arduino v0022 used to do it. Doesn't work with v1.0
  //Serial.print(byte(26));
  //Serial.print(byte(26));
  //Serial.print(byte(26));
  
  //Works with Arduino v1.0
  Serial3.write(26);
  Serial3.write(26);
  Serial3.write(26);
  
  //Wait for OpenLog to respond with '>' to indicate we are in command mode
  while(1) {
    if(Serial3.available())
      if(Serial3.read() == '>') break;
  }
  
  fileNumber = random(0,999);
  
  //Send new (random from 0 to 999) file name
  //Old way
  sprintf(buff, "new LAICA%02d.txt\r", fileNumber);
  Serial3.print(buff); //\r in string + regular print works with older v2.5 Openlogs

  //New way
//  sprintf(buff, "new rock%03d.txt", fileNumber);
//  Serial2.println(buff); //regular println works with v2.51 and above
  
  //Wait for OpenLog to return to waiting for a command
  while(1) {
    if(Serial3.available())
      if(Serial3.read() == '>') break;
  }
  
  sprintf(buff, "append LAICA%02d.txt\r", fileNumber);
  Serial3.print(buff);
  
  //Wait for OpenLog to indicate file is open and ready for writing
  while(1) {
    if(Serial3.available())
      if(Serial3.read() == '<') break;
  }
}


void getcurrent(void) 
{
  float shuntvoltage1 = 0;
  float busvoltage1 = 0;
  float current_mA1 = 0;
  float loadvoltage1 = 0;
  float shuntvoltage2 = 0;
  float busvoltage2 = 0;
  float current_mA2 = 0;
  float loadvoltage2 = 0;

  shuntvoltage1 = ina219_1.getShuntVoltage_mV();
  busvoltage1 = ina219_1.getBusVoltage_V();
  current_mA1 = ina219_1.getCurrent_mA();
  loadvoltage1 = busvoltage1 + (shuntvoltage1 / 1000);
  
  //Serial.print("1 - Bus Voltage:   "); Serial.print(busvoltage1); Serial.println(" V");
  //Serial.print("1 - Shunt Voltage: "); Serial.print(shuntvoltage1); Serial.println(" mV");
  //Serial.print("1 - Load Voltage:  "); Serial.print(loadvoltage1); Serial.println(" V");
  //Serial.print("1 - Current:       "); Serial.print(current_mA1); Serial.println(" mA");
  

  shuntvoltage2 = ina219_2.getShuntVoltage_mV();
  busvoltage2 = ina219_2.getBusVoltage_V();
  current_mA2 = ina219_2.getCurrent_mA();
  loadvoltage2 = busvoltage2 + (shuntvoltage2 / 1000);
  
  //Serial.print("2 - Bus Voltage:   "); Serial.print(busvoltage2); Serial.println(" V");
  //Serial.print("2 - Shunt Voltage: "); Serial.print(shuntvoltage2); Serial.println(" mV");
  //Serial.print("2 - Load Voltage:  "); Serial.print(loadvoltage2); Serial.println(" V");
  //Serial.print("2 - Current:       "); Serial.print(current_mA2); Serial.println(" mA");

  Serial3.print(busvoltage1);
  Serial3.print(",");
  Serial3.print(shuntvoltage1);
  Serial3.print(",");
  Serial3.print(loadvoltage1);
  Serial3.print(",");
  Serial3.print(current_mA1);
  Serial3.print(",");
  Serial3.print(busvoltage2);
  Serial3.print(",");
  Serial3.print(shuntvoltage2);
  Serial3.print(",");
  Serial3.print(loadvoltage2);
  Serial3.print(",");
  Serial3.print(current_mA2);
  Serial3.print(",");
}

void BMP085_routine(){
//    Serial.print("Temperature = ");
//    Serial.print(bmp.readTemperature());
//    Serial.println(" *C");
    
    Serial3.print(bmp.readPressure());
    Serial3.print(",");
    
    // Calculate altitude assuming 'standard' barometric
    // pressure of 1013.25 millibar = 101325 Pascal
    Serial3.print(bmp.readAltitude());
    Serial3.print(",");

  // you can get a more precise measurement of altitude
  // if you know the current sea level pressure which will
  // vary with weather and such. If it is 1015 millibars
  // that is equal to 101500 Pascals.
//    Serial2.print(bmp.readAltitude(101500));
//    Serial2.print(",");
}

void MS5611_routine(){
  D1 = getVal(ADDRESS, 0x46); // Pressure raw
  D2 = getVal(ADDRESS, 0x56);// Temperature raw

  dT   = D2 - ((uint32_t)C[5] << 8);
  OFF  = ((int64_t)C[2] << 16) + ((dT * C[4]) >> 7);
  SENS = ((int32_t)C[1] << 15) + ((dT * C[3]) >> 8);

  TEMP = (int64_t)dT * (int64_t)C[6] / 8388608 + 2000;

  Temperature = (float)TEMP / 100; 
  
  P  = ((int64_t)D1 * SENS / 2097152 - OFF) / 32768;

  Pressure = (float)P / 100;
  
  Serial3.print(Temperature);
  Serial3.print(",");
  Serial3.print(Pressure);
  Serial3.print(",");
  Serial2.println(Pressure);
  
//  Serial.print(" RAW Temp D2=  ");
  Serial3.print(D2);
  Serial3.print(",");
//  Serial.print(" RAW Pressure D1=  ");
  Serial3.println(D1);

//  Serial.print(" dT=  ");
//  Serial.println(dT); can't print int64_t size values
//  Serial.println();
//  Serial.print(" C1 = ");
//  Serial.println(C[1]);
//  Serial.print(" C2 = ");
//  Serial.println(C[2]); 
//  Serial.print(" C3 = ");
//  Serial.println(C[3]); 
//  Serial.print(" C4 = ");
//  Serial.println(C[4]); 
//  Serial.print(" C5 = ");
//  Serial.println(C[5]); 
//  Serial.print(" C6 = ");
//  Serial.println(C[6]); 
//  Serial.print(" C7 = ");
//  Serial.println(C[7]);
//  Serial.println();

  delay(1000);
}
    
long getVal(int address, byte code){
  unsigned long ret = 0;
  Wire.beginTransmission(address);
  Wire.write(code);
  Wire.endTransmission();
  delay(10);
  // start read sequence
  Wire.beginTransmission(address);
  Wire.write((byte) 0x00);
  Wire.endTransmission();
  Wire.beginTransmission(address);
  Wire.requestFrom(address, (int)3);
  if (Wire.available() >= 3)
  {
    ret = Wire.read() * (unsigned long)65536 + Wire.read() * (unsigned long)256 + Wire.read();
  }
  else {
    ret = -1;
  }
  Wire.endTransmission();
  return ret;
}

void initial(uint8_t address){

  Wire.beginTransmission(address);
  Wire.write(0x1E); // reset
  Wire.endTransmission();
  delay(10);


  for (int i=0; i<6  ; i++) {

    Wire.beginTransmission(address);
    Wire.write(0xA2 + (i * 2));
    Wire.endTransmission();

    Wire.beginTransmission(address);
    Wire.requestFrom(address, (uint8_t) 6);
    delay(1);
    if(Wire.available())
    {
       C[i+1] = Wire.read() << 8 | Wire.read();
    }
    else {
      Serial.println("Error reading PROM 1"); // error reading the PROM or communicating with the device
    }
    Serial.println(C[i+1]);
  }
  Serial.println();
}
    


