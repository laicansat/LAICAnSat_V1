#include <Wire.h>
#include <OneWire.h>
#include <Adafruit_INA219.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include <GPS_UBLOX.h> // UBLOX GPS Library
#include <SHT1x.h>
#include "TSL2561.h"


#define ADDRESS 0x76 //0x76

#define dataPin  14
#define clockPin 15
SHT1x sht1x(dataPin, clockPin);

Adafruit_INA219 ina219_1(0x40); //none
Adafruit_INA219 ina219_2(0x44); //A1
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);

// The address will be different depending on whether you let
// the ADDR pin float (addr 0x39), or tie it to ground or vcc. In those cases
// use TSL2561_ADDR_LOW (0x29) or TSL2561_ADDR_HIGH (0x49) respectively
TSL2561 tsl(TSL2561_ADDR_FLOAT); 


OneWire  ds(12);  // on pin 19 (a 4.7K resistor is necessary)


char buff[50];
int fileNumber;

int statLED = 13;
int contador_de_leitura = 0;
//int contador_de_leitura2 = 0;
//int contador_de_leitura3 = 0;
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
  Serial.begin(57600);
  delay(1000);
  Serial1.begin(9600);
  delay(1000);
  GPS.Init();   // GPS Initialization
  delay(1000);
  Serial2.begin(9600);
  delay(1000);
  set_SDcard();
  delay(500);
  //ina219_1.begin();
  delay(200);
  ina219_2.begin();
  delay(200);
  initial(ADDRESS);
  delay(200);
  bmp.begin();
  delay(200);
  
  // You can change the gain on the fly, to adapt to brighter/dimmer light situations
  //tsl.setGain(TSL2561_GAIN_0X);         // set no gain (for bright situtations)
  tsl.setGain(TSL2561_GAIN_16X);      // set 16x gain (for dim situations)
  
  // Changing the integration time gives you a longer time over which to sense light
  // longer timelines are slower, but are good in very low light situtations!
  tsl.setTiming(TSL2561_INTEGRATIONTIME_13MS);  // shortest integration time (bright light)
  //tsl.setTiming(TSL2561_INTEGRATIONTIME_101MS);  // medium integration time (medium light)
  //tsl.setTiming(TSL2561_INTEGRATIONTIME_402MS);  // longest integration time (dim light)
  
}

void loop()
{
  GPS.Read();
  if (GPS.NewData){
    Serial3.print(GPS.Time);
    Serial3.print(",");
    Serial3.print((int)GPS.Fix);
    Serial3.print(",");
    Serial3.print(GPS.Lattitude);
    Serial3.print(",");
    Serial3.print(GPS.Longitude);
    Serial3.print(",");
    Serial3.print(GPS.Altitude/1000.0);
    Serial3.print(",");
    Serial3.print(GPS.Ground_Speed/100.0);
    Serial3.print(",");
    Serial3.print(GPS.Ground_Course/100000.0);
    Serial3.print(",");
//    Serial2.print(GPS.Time);
//    Serial2.print(",");
//    Serial2.print((int)GPS.Fix);
//    Serial2.print(",");
//    Serial2.print(GPS.Lattitude);
//    Serial2.print(",");
//    Serial2.print(GPS.Longitude);
//    Serial2.print(",");
//    Serial2.print(GPS.Altitude/1000.0);
//    Serial2.print(",");
//    Serial2.print(GPS.Ground_Speed/100.0);j
  if (contador_de_leitura == 20){// 80 = intervalo de 20 segundos // retorna 
    getcurrent(); // retorna 8 campos
    BMP085_routine();
    delay(200);
    MS5611_routine();
    delay(200);
    DS18x20_routine();
    delay(200);
    TSL2561_routine();
    delay(200);
    SHT1x_routine();
    contador_de_leitura = 0;
  }else{
     for (int i=0; i < 22; i++){ //escreve o que esta abaixo 23 vezes
      Serial3.print(0);
      Serial3.print(",");
    }
    Serial3.println(0);
  }
//    for (int i=0; i < 22; i++){ //escreve o que esta abaixo 23 vezes
//      Serial3.print(0);
//      Serial3.print(",");
//    }
//    Serial3.println(0);
  
//  if (contador_de_leitura2 == 40){// 40 = intervalo de 10 segundos
//    BMP085_routine(); // retorna 3 campos
//    MS5611_routine(); // retorna 4 campos
//    DS18x20_routine(); // retorna 2 campos
//    contador_de_leitura2 = 0;
//  }else{
//    if (contador_de_leitura == 0 && contador_de_leitura2 != 0){
//      for (int i=0; i < 14; i++){ //escreve o que esta abaixo 15 vezes
//        Serial3.print(0);
//        Serial3.print(",");
//        }
//    }else{
//    }
//  }
//  
//  if (contador_de_leitura3 == 120){
//    TSL2561_routine(); // retorna 4 campos
//    SHT1x_routine(); // retorna 2 campos
//    contador_de_leitura3 = 0;
//  }else{
//    if (contador_de_leitura == 0 && contador_de_leitura2 == 0 && contador_de_leitura3 != 0){
//      for (int i=0; i < 5; i++){ //escreve o que esta abaixo 6 vezes
//        Serial3.print(0);
//        Serial3.print(",");
//        }
//    }else{
//    }
//  }
  
  
//    char c = Serial2.read();
//    if(c == 97){ // a ascii table
//      servoleft.write(180);
//      Serial3.print(",");
//      Serial3.print("180");
//      Serial2.print(",");
//      Serial2.print("180");
//      servoright.write(90);
//      Serial3.print(",");
//      Serial3.println("90");
//      Serial2.print(",");
//      Serial2.println("90");
//      servobuzzer.write(0);
//    }else{
//      if(c == 100){ // d ascii table
//        servoleft.write(90);
//        Serial3.print(",");
//        Serial3.print("90");
//        Serial2.print(",");
//        Serial2.print("90");
//        servoright.write(0);
//        Serial3.print(",");
//        Serial3.println("0");
//        Serial2.print(",");
//        Serial2.println("0");
//        servobuzzer.write(0);
//      }else{
//        if(c == 115){ // s ascii table
//          servoleft.write(90);
//          Serial3.print(",");
//          Serial3.print("90");
//          Serial2.print(",");
//          Serial2.print("90");
//          servoright.write(90);
//          Serial3.print(",");
//          Serial3.println("90");
//          Serial2.print(",");
//          Serial2.println("90");
//          servobuzzer.write(0);
//        }else{
//          if(c == 101){
//            servobuzzer.write(180);
//            Serial3.println();
//            Serial2.println();
//          }else{
//            Serial3.println();
//            Serial2.println();
//          }
//        }
//      }
//    }
    GPS.NewData = 0; // We have read the data
    contador_de_leitura = contador_de_leitura++;
    }
  delay(10);
}


void set_SDcard(){
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



void getcurrent(void) {
  float shuntvoltage1 = 0;
  float busvoltage1 = 0;
  float current_mA1 = 0;
  float loadvoltage1 = 0;
  float shuntvoltage2 = 0;
  float busvoltage2 = 0;
  float current_mA2 = 0;
  float loadvoltage2 = 0;

//  shuntvoltage1 = ina219_1.getShuntVoltage_mV();
//  busvoltage1 = ina219_1.getBusVoltage_V();
//  current_mA1 = ina219_1.getCurrent_mA();
//  loadvoltage1 = busvoltage1 + (shuntvoltage1 / 1000);
  
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
  
    sensors_event_t event;
    bmp.getEvent(&event);
  
    if (event.pressure){
    /* Display atmospheric pressue in hPa */
    Serial3.print(event.pressure);
    Serial3.print(",");

    /* First we get the current temperature from the BMP085 */
    float temperature;
    bmp.getTemperature(&temperature);
    Serial3.print(temperature);
    Serial3.print(",");

    /* Then convert the atmospheric pressure, SLP and temp to altitude    */
    /* Update this next line with the current SLP for better results      */
    float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
    Serial3.print(bmp.pressureToAltitude(seaLevelPressure, event.pressure, temperature));
    Serial3.print(","); 
  }
  else{
  }
  
//    Serial3.print(bmp.readTemperature());
//    Serial3.print(",");
//    
//    Serial3.print(bmp.readPressure());
//    Serial3.print(",");
//    
//    // Calculate altitude assuming 'standard' barometric
//    // pressure of 1013.25 millibar = 101325 Pascal
//    Serial3.print(bmp.readAltitude());
//    Serial3.print(",");

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
  
//  Serial2.print("Tint");
//  Serial2.print(Temperature);
//  Serial2.print("P");
//  Serial2.print(Pressure);
//  Serial2.print("!");
  
//  Serial.print(" RAW Temp D2=  ");
  Serial3.print(D2);
  Serial3.print(",");
//  Serial.print(" RAW Pressure D1=  ");
  Serial3.print(D1);
  Serial3.print(",");

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
    
void DS18x20_routine(){
  byte i;
  byte present = 0;
  byte type_s = 0;
  byte data[12];
  byte addr[8];
  float celsius, fahrenheit;
  
  if (!ds.search(addr)) {
    ds.reset_search();
    delay(50);
    return;
  }
  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);        // start conversion, with parasite power on at the end
  
  delay(750);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.
  
  present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE);         // Read Scratchpad

  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
  }

  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }
  celsius = (float)raw / 16.0;
  Serial3.print(celsius);
  Serial3.print(",");
}

void SHT1x_routine(){
  float temp_c;
  float humidity;

  // Read values from the sensor
  temp_c = sht1x.readTemperatureC();
  delay(1000);
  humidity = sht1x.readHumidity();
  delay(1000);

  // Print the values to the serial port
  Serial3.print(temp_c, DEC);
  Serial3.print(",");
  Serial3.print(humidity);
  Serial3.println();
}

void TSL2561_routine(){

  // More advanced data read example. Read 32 bits with top 16 bits IR, bottom 16 bits full spectrum
  // That way you can do whatever math and comparisons you want!
  uint32_t lum = tsl.getFullLuminosity();
  uint16_t ir, full;
  ir = lum >> 16;
  full = lum & 0xFFFF;
  Serial3.print(ir);
  Serial3.print(",");
  Serial3.print(full);   
  Serial3.print(",");
  Serial3.print(full - ir);   
  Serial3.print(",");
  Serial3.print(tsl.calculateLux(full, ir));
  Serial3.print(",");
}
