// V1 en monothread, on rajoutera les semaphores, threads, mutex... avec la librairie arduino_free_RTOS en V2

//################################## Libraries ##################################

// P,T,z,H
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BME280.h>

// Accel-gyro
//#include <SparkFunLSM6DS3.h>
//#include <Wire.h>
//#include <math.h>
//#include <MsTimer2.h>

//T
#include <DS18B20.h>

// GPS
#include <SoftwareSerial.h>
#include <TinyGPS.h>



// SD https://www.arduino.cc/en/Tutorial/LibraryExamples/Datalogger
//#include <SD.h>

// TX
//#include <LoRa.h>



// ################################## Ordonnanceur ##################################
#define   NUMTACHE      3 //capteur P,T,z,H / capteur accel,gyro / GPS / écriture carte SD / TX / RX en continu donc pas une tâche / Commande chaufferette / 
#define   DUREEREP_MS   6000
#define   DEBUG         1 //variable d'affiche des données sur le moniteur série, actif à 1 inactif à 0
unsigned char Tache_on[NUMTACHE], Tache_off[NUMTACHE];
unsigned long Tache_start[NUMTACHE], i;
unsigned long Master_time = 0, Tache_duree[NUMTACHE];
unsigned char Tache_Active = 0;


// ################################## P,T,z,H ##################################
#define SEALEVELPRESSURE_HPA (1013.25)
#define RHO (1,225)
#define G (9.81)

Adafruit_BME280 bme;
float pTzH_values[4];
unsigned long delayTime;
void get_pTzH();

// ################################## T ################################## https://github.com/matmunk/DS18B20
DS18B20 ds(2); 

// ################################## Accel-gyro ###############################
//double[] accelGyroValues;
//Accélérations linéaires selon les 3 axes
//float accX;
//float accY;
//float accZ;

//Vitesses angulaires selon les 3 axes
//float gyro_angle_rateX;
//float gyro_angle_rateY;
//float gyro_angle_rateZ;

//Positions angulaires selon 2 axes données par l'accéléromètre
//float accAngleX;
//float accAngleY;
//float accAngleZ;

//Positions angulaires selon 3 axes données par le gyroscope
//float gyroAngleX;
//float gyroAngleY;


//Angles premettant la détermination de l'orientation du capteur
//float roll; //angle selon l'axe x
//float pitch; //angle selon l'axe y
//float yaw; //angle selon l'axe z

//Conversion radians degrées
//float rad_deg = 57.2957;

//valeur alpha utilisée pour le filtre complémantaire
//float alphaX;
//float alphaY;

//float Time = 1000;

//float elapsedTime = Time/1000;

//Create a instance of class LSM6DS3
//LSM6DS3 myIMU(I2C_MODE, 0x6A);    //I2C device address 0x6A


//################################## GPS ##################################

 
long lat,lon; // create variable for latitude and longitude object
 
SoftwareSerial gpsSerial(3, 4); // create gps sensor connection
TinyGPS gps; // create gps object

// ################################## SD ##################################
//const int chipSelect = 10;

// ################################## Lora ##################################
//const int csPin = 7;          // LoRa radio chip select
//const int resetPin = 6;       // LoRa radio reset
//const int irqPin = 1;         // change for your board; must be a hardware interrupt pin

//byte msgCount = 0;            // count of outgoing messages
//int interval = 2000;          // interval between sends
//long lastSendTime = 0;        // time of last packet send

void setup()
{

  Serial.begin(9600);
  while (!Serial);   // time to get serial running

  // ################################## Ordonnanceur ##################################
  // Init écheances des taches (en ms)
  Tache_duree[0] = 1000; // capteur P,T,z,H
  Tache_duree[1] = 1000; // capteur accel, gyro
  Tache_duree[2] = 1000; // capteur GPS
  Tache_duree[3] = 1000; // commande chaufferette
  Tache_duree[4] = 1000; // écriture carte SD
  Tache_duree[5] = 1000; // TX

  // Init temps de début des tâches (en ms)
  Tache_start[0] = 1000;
  Tache_start[1] = 2000;
  Tache_start[2] = 3000;
  Tache_start[3] = 4000;
  Tache_start[4] = 5000;
  Tache_start[5] = 6000;

  // ################################## P,T,z,H ##################################

  Serial.println("BME280 test");

  unsigned status;

  // si defauts pour le setting
  status = bme.begin(0x76);  /* the function bme.begin takes in parameter the adress of the sensor*/
  /* if at the beginnign the adress in unknown, run the code "code_to_found_adress"*/

  if (!status) 
  {
    Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
    Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(), 16);
    Serial.print("ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("ID of 0x60 represents a BME 280.\n");
    Serial.print("ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  }

  Serial.println("-- Default Test --"); /* once the status is ok*/
  delayTime = 1000;

  Serial.println();
  //################################## GPS ##################################
  Serial.begin(9600); // connect serial
  gpsSerial.begin(9600); // connect gps sensor

  // ################################## Accel-gyro ###############################

  // ################################## SD ##################################
 
  //################################## Lora //##################################
}

void loop()
{
  // Lecture de l'ordonnanceur
  Master_time = millis();

  // Init de l'ordonnanceur avec la période de répétition
  Master_time = (Master_time % DUREEREP_MS);

  // Génération des conditions de début des taches
  for (i = 0; i < NUMTACHE; i++)
  {
    Tache_on[i] = (Master_time > Tache_start[i]);
  }

  // Génération des conditions d'arrêts
  for (i = 0; i < NUMTACHE; i++)
  {
    Tache_off[i] = Tache_on[i] & (Master_time < (Tache_start[i] + Tache_duree[i]));
  }

  // Exécution des tâches

  // Tâche 1 : P,T,z,H
  if (Tache_off[0])
  {Serial.println("a");
    get_pTzH(); /*fonction defined after*/
    Tache_Active = 1;
    Serial.println("b");
  }

  // Tâche 2 : Température
   if (Tache_off[1])
  {

   if(DEBUG)
   {
        Serial.print("*******");
        Serial.print("T_value");
        Serial.print("*******");
   }
  Serial.print(ds.getTempC());
  }
  // Tâche 3 : GPS
  if (Tache_off[2])
  {
    get_GPS();
  }

  // Tâche 4 : Gyro

  // Tâche 5 : Lora

  // Tâche 6 : SD

// Affichage de la tache active
  Serial.print(Master_time / 1000);
  Serial.print(":");
  Serial.println(Master_time % 1000);
}

// ################################## P,T,z,H ##################################
void get_pTzH(){
        
    pTzH_values[0] = bme.readPressure() / 100.0F;
    pTzH_values[1] = bme.readTemperature() ;
    pTzH_values[2] = (SEALEVELPRESSURE_HPA - pTzH_values[0])/(RHO*G) ;
    pTzH_values[3] = bme.readHumidity();
        
    //TODO delete after test
    /*
    /*temperature in Celsius degree*/
    if(DEBUG)
    {
      Serial.print("***********");
      Serial.print("pTzH_values");
      Serial.print("***********");
      Serial.printf("\n" );
      Serial.print("Temperature = ");
      Serial.print(pTzH_values[0]);
      Serial.println(" *C");

      /*temperature Kelvin*/
      Serial.print("Temperature = ");
      Serial.print(pTzH_values[1]+273,15);
      Serial.println(" K");

      /*pressure in hPa*/
      Serial.print("Pressure = ");
      Serial.print(pTzH_values[2] / 100.0F);
      Serial.println(" hPa");

      /*humidity in pourcentage*/
      Serial.print("Humidity = ");
      Serial.print(pTzH_values[3]);
      Serial.println(" %");

      Serial.println("\n");
    }
    
}

//################################## Accel-gyro ##################################

// Mettre ici la fonction accel

//################################## GPS ##################################
void get_GPS()
{
if(gpsSerial.available()){ // check for gps data
   if(gps.encode(gpsSerial.read())) // encode gps data
   { 
    gps.get_position(&lat,&lon); // get latitude and longitude
    // display position
    if(DEBUG)
    {
      Serial.print("Position: ");
      Serial.print("lat: ");Serial.print(lat);Serial.print(" ");// print latitude
      Serial.print("lon: ");Serial.println(lon); // print longitude
    }
   }
  }
   
 } 
 void clearBufferArray()                     // function to clear buffer array
 {
   for (int i=0; i<count;i++)
   {
       buffer[i]=0;
   }                      // clear all index of array with command NULL
 }
//################################## SD ##################################
// Mettre ici le code du capteur de carte sd 
// Lora
