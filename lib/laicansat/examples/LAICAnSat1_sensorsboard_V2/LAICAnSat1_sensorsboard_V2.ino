
//TODOS OS SENSORES DA IMU FUNCIONANDO
//TERMOMETRO, BAROMETRO, ACELEROMETRO, GIROSCOPIO
//vou poder usar mais de uma instancia da mesma função?
#include <SPI.h>
#include <SD.h>
#include <laicansat.h>

const int led = 13; //MUDAR PARA 30
double accelerations[3] = {0,0,0};
double angularSpeeds[3] = {0,0,0};
double temperatureSHT15 = 0.0;
double gpsData[7] = {0,0,0,0,0,0,0};

void setup()
{
  Serial.begin(38400);
 
  laicansat.gps->beginGPS();
  laicansat.bar->begin(BMP180_BAROMODE);
  laicansat.thermo->begin(BMP180_THERMOMODE);
  laicansat.accel->begin();  
  laicansat.gyro->begin(); 
  //laicansat.hygro->begin(); 
  //laicansat.thermo->begin(SHT15_THERMOMODE);
  pinMode(led, OUTPUT);
  delay(100);
}

void loop()
{
   
  laicansat.gps->getData(gpsData);
  double temperature1 = laicansat.thermo->getTemperature();
  double pressure = laicansat.bar->getPressure();
  //double humidity = laicansat.hygro->getHumidity();
  //double temperature2 = laicansat.thermo->getTemperature();
  laicansat.accel->getAcceleration(accelerations);
  laicansat.gyro->getAngularSpeed(angularSpeeds);
  

  
  //Serial.print("Humidity: ");
  //Serial.println(humidity);
  Serial.print("Temperature BMP180: ");
  Serial.println(temperature1);
  //Serial.print("Temperature STH15: ");
  //Serial.println(temperature2);
  Serial.print("Pressure: ");
  Serial.println(pressure);
  Serial.print("Accelerations: ");
  Serial.print(accelerations[0]);
  Serial.print(", ");
  Serial.print(accelerations[1]);
  Serial.print(", ");
  Serial.println(accelerations[2]);
  Serial.print("Angular speeds: ");
  Serial.print(angularSpeeds[0]);
  Serial.print(", ");
  Serial.print(angularSpeeds[1]);
  Serial.print(", ");
  Serial.println(angularSpeeds[2]);
  digitalWrite(led, HIGH);
  delay(1000);
  digitalWrite(led, LOW);
  delay(1000);
}
