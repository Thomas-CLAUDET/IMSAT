// V1 en monothread, on rajoutera les semaphores, threads, mutex... avec la librairie arduino_free_RTOS en V2

//################################## Libraries ##################################

// P,T,z,H
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BME280.h>

// Accel-gyro
#include <SparkFunLSM6DS3.h>
#include <Wire.h>
#include <math.h>
#include <MsTimer2.h>

// GPS
#include <SoftwareSerial.h>

// SD https://www.arduino.cc/en/Tutorial/LibraryExamples/Datalogger
#include <SD.h>

// TX
#include <LoRa.h>


// ################################## Ordonnanceur ##################################
#define   NumTache      6 //capteur P,T,z,H / capteur accel,gyro / GPS / écriture carte SD / TX / RX en continu donc pas une tâche / Commande chaufferette / 
#define   DureeRep_ms   6000
unsigned char Tache_on[NumTache],Tache_off[NumTache];
unsigned long Tache_start[NumTache],i;
unsigned long Master_time=0, Tache_duree[NumTache];
unsigned char Tache_Active=0; 

// ################################## P,T,z,H ##################################
#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; 
double[] pTzHvalues;
unsigned long delayTime;
// ################################## Accel-gyro ###############################
double[] accelGyroValues;
//Accélérations linéaires selon les 3 axes 
float accX;     
float accY;
float accZ; 

//Vitesses angulaires selon les 3 axes 
float gyro_angle_rateX;   
float gyro_angle_rateY;
float gyro_angle_rateZ;

//Positions angulaires selon 2 axes données par l'accéléromètre
float accAngleX;    
float accAngleY;
float accAngleZ;

//Positions angulaires selon 3 axes données par le gyroscope
float gyroAngleX;    
float gyroAngleY;


//Angles premettant la détermination de l'orientation du capteur
float roll; //angle selon l'axe x      
float pitch; //angle selon l'axe y
float yaw; //angle selon l'axe z

//Conversion radians degrées
float rad_deg = 57.2957;

//valeur alpha utilisée pour le filtre complémantaire
float alphaX;
float alphaY;

float Time = 1000;

float elapsedTime = Time/1000;

//Create a instance of class LSM6DS3
LSM6DS3 myIMU(I2C_MODE, 0x6A);    //I2C device address 0x6A


//################################## GPS ##################################
SoftwareSerial SoftSerial(2, 3);
unsigned char buffer[64];                   // buffer array for data receive over serial port
int count=0;  
double[] GPSValues;

// ################################## SD ##################################
const int chipSelect = 10;

// ################################## Lora ##################################
const int csPin = 7;          // LoRa radio chip select
const int resetPin = 6;       // LoRa radio reset
const int irqPin = 1;         // change for your board; must be a hardware interrupt pin

byte msgCount = 0;            // count of outgoing messages
int interval = 2000;          // interval between sends
long lastSendTime = 0;        // time of last packet send

void setup()
{

    Serial.begin(9600);
    while(!Serial);    // time to get serial running

    // ################################## Ordonnanceur ##################################
    // Init écheances des taches (en ms)  
    Tache_duree[0]=1000; // capteur P,T,z,H
    Tache_duree[1]=1000; // capteur accel, gyro
    Tache_duree[2]=1000; // capteur GPS
    Tache_duree[3]=1000; // commande chaufferette
    Tache_duree[4]=1000; // écriture carte SD
    Tache_duree[5]=1000; // TX

    // Init temps de début des tâches (en ms) 
    Tache_start[0]=1000;
    Tache_start[1]=2000;
    Tache_start[2]=3000;
    Tache_start[3]=4000;
    Tache_start[4]=5000;
    Tache_start[5]=6000;

    // ################################## P,T,z,H ##################################

    Serial.println(F("BME280 test"));

    unsigned status;
    
    // si defauts pour le setting 
    status = bme.begin(0x76);  /* the function bme.begin takes in parameter the adress of the sensor*/
    /* if at the beginnign the adress in unknown, run the code "code_to_found_adress"*/

    if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
        Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
        Serial.print("ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        Serial.print("ID of 0x56-0x58 represents a BMP 280,\n");
        Serial.print("ID of 0x60 represents a BME 280.\n");
        Serial.print("ID of 0x61 represents a BME 680.\n");
        while (1) delay(10);
    }
    
    Serial.println("-- Default Test --"); /* once the status is ok*/
    delayTime = 1000;

    Serial.println();

    // ################################## Accel-gyro ###############################
    //Call .begin() to configure the IMUs
    if (myIMU.begin() != 0) {
        Serial.println("Device error");
    } else {
        Serial.println("Device OK!");
    }

    //Récupération des accélérations linéaires 
    accX = myIMU.readFloatAccelX();           
    accY = myIMU.readFloatAccelY(); 
    accZ = myIMU.readFloatAccelZ();

    //Calcul des positions angulaires en degrés avec les données de l'accéléromètre
    accAngleX = atan2(accY , accZ) * rad_deg;
    accAngleY = atan2((- accX), sqrt(pow(accY, 2)) + pow(accZ, 2)) * rad_deg;

    gyroAngleX = accAngleX;
    gyroAngleY = accAngleY;

    //################################## GPS ##################################
    SoftSerial.begin(9600);                 // the SoftSerial baud rate

    // ################################## SD ##################################
    Serial.print("Initializing SD card...");

    if (!SD.begin(chipSelect)) {

    Serial.println("initialization failed. Things to check:");

    Serial.println("1. is a card inserted?");

    Serial.println("2. is your wiring correct?");

    Serial.println("3. did you change the chipSelect pin to match your shield or module?");

    Serial.println("Note: press reset or reopen this serial monitor after fixing your issue!");

    while (true);

  }

    Serial.println("SD initialization done.");

    ////################################## Lora //##################################
    Serial.println("LoRa Duplex - Set spreading factor");

    // override the default CS, reset, and IRQ pins (optional)
    LoRa.setPins(csPin, resetPin, irqPin); // set CS, reset, IRQ pin

    if (!LoRa.begin(868E6)) {             // initialize ratio at 868 MHz
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                       // if failed, do nothing
    }

    Lora.setCodingRate4(6)          // à faire varier entre 5 et 8. Il doit être a priori le même en récepteur et émetteur, à vérifier également par les tests.
    Serial.println("LoRa init succeeded.");
}



void loop()
{
  // Lecture de l'ordonnanceur 
  Master_time=millis(); 

  // Init de l'ordonnanceur avec la période de répétition 
  Master_time=(Master_time % DureeRep_ms);
  
  // Génération des conditions de début des taches  
  for (i=0;i<NumTache;i++)
  {
    Tache_on[i]=(Master_time>Tache_start[i]);
  }

  // Génération des conditions d'arrêts   
  for (i=0;i<NumTache;i++)
  {
    Tache_off[i]=Tache_on[i] & (Master_time<(Tache_start[i]+Tache_duree[i]));
  }

  // Exécution des tâches 

  // Tâche 1 : P,T,z,H
  if(Tache_off[0])
  {
    pTzHvalues = pTzH(); /*fonction defined after*/
    Tache_Active=1;
  }

  // Tâche 2 : accel-gyro
  else if(Tache_off[1])
  {
    accelGyroValues = AcccelGyro();
    Tache_Active=2;
  }

  // Tâche 3 : GPS
  else if(Tache_off[3])
  {
    GPSValues = GPS();
    Tache_Active=3;
  }

  // Tâche 4 : SD
  else if(Tache_off[2])
  {
    message = writeIntoSD(pTzHvalues, accelGyroValues, GPSValues);
    Tache_Active=3;
  }

  // Tâche 5 : LoraTX
  else if(Tache_off[4])
  {
    LoraTX(message);
    Tache_Active=4;
  }

  // Tâche 5 : Chaufferette
  else if(Tache_off[5])
  {
    heater(pTzHvalues[1]);
    Tache_Active=5;
  }

  // Autres: Désactivation des sorties 
  else 
  {
    Tache_Active=0;
  }
   
  // Affichage de la tache active 
  Serial.print(Master_time/1000);
  Serial.print(":");
  Serial.println(Master_time%1000);
}

// ################################## P,T,z,H ##################################

double[] pTzH() {
    /*temperature in Celsius degree*/
    Serial.print("Temperature = ");
    Serial.print(bme.readTemperature());
    Serial.println(" *C");

    /*temperature Kelvin*/
    Serial.print("Temperature = ");
    Serial.print(bme.readTemperature()+273,15);
    Serial.println(" K");

    /*pressure in hPa*/
    Serial.print("Pressure = ");
    Serial.print(bme.readPressure() / 100.0F);
    Serial.println(" hPa");

    /*humidity in pourcentage*/
    Serial.print("Humidity = ");
    Serial.print(bme.readHumidity());
    Serial.println(" %");

    Serial.println();

    //mettre tout dans une liste et return
}

//################################## Accel-gyro ##################################

double[] AcccelGyro() { // debut de la fonction d'interruption Timer2

    //Récupération des accélérations linéaires 
    accX = myIMU.readFloatAccelX();           
    accY = myIMU.readFloatAccelY(); 
    accZ = myIMU.readFloatAccelZ();

    gyro_angle_rateX = myIMU.readFloatGyroX() - 0.33;    
    gyro_angle_rateY = myIMU.readFloatGyroY() + 0.8; 
    gyro_angle_rateZ = myIMU.readFloatGyroZ() + 0.4;
    
    //Calcul des positions angulaires en degrés avec les données de l'accéléromètre
    accAngleX = atan2(accY , accZ) * rad_deg;
    accAngleY = atan2((- accX), sqrt(pow(accY, 2)) + pow(accZ, 2)) * rad_deg;

    //Calcul des positions angulaires en degrés avec les données du gyroscope   
    
    // Intégration de la vitesse angulaire pour récupérer les angles
    gyroAngleX = gyroAngleX + gyro_angle_rateX * elapsedTime; // deg/s * s = deg
    gyroAngleY = gyroAngleY + gyro_angle_rateY * elapsedTime;
    yaw =  yaw + gyro_angle_rateZ * elapsedTime;

    //Tests
    if (abs(accX) > 1){            //une forte accélération latérale fausse les mesures d'angles, elles ne sont donc plus fiables
      alphaX = 1; 
    }
    else if (abs(accY) > 1){
      alphaY = 1;
    }
    else if (abs(gyroAngleX) > 120){         //en cas de divergence trop forte des valeurs d'angles calculée à l'aide des données du gyro,
      alphaX = 0;                                                    //on ne les prend pas en compte et on "réinitialise" le gyro
      gyroAngleX = accAngleX;
    }
    else if (abs(gyroAngleY) > 120){
      alphaY = 0;                                                
      gyroAngleY = accAngleY;
    }
    else{
      alphaX = 0.25;
      aplhaY = 0.25;
    }
    
    // Flitre complémentaire combinant les valeurs de l'accéléromètre et du gyroscope
    roll = alphaX * gyroAngleX + (1-alphaX) * accAngleX;
    pitch = alphaY * gyroAngleY + (1-alphaY) * accAngleY;

    /*
    //Affichage des accélérations linéaires
    Serial.print("\nAccelerometer:\n");
    Serial.print(" X1 = ");
    Serial.println(accX, 4);
    Serial.print(" Y1 = ");
    Serial.println(accY, 4);
    Serial.print(" Z1 = ");
    Serial.println(accZ, 4);
    
    //Affichage des vitesses angulaires
    Serial.print("\nGyroscope:\n");
    Serial.print(" X1 = ");
    Serial.println(gyro_angle_rateX, 4);
    Serial.print(" Y1 = ");
    Serial.println(gyro_angle_rateY, 4);
    Serial.print(" Z1 = ");
    Serial.println(gyro_angle_rateZ, 4);
    */
    
    //Affichage des positions angulaires selon l'accéléromètre et le gyroscope
    Serial.print("\nAngles:\n");
    
    Serial.print("accAngleX = ");
    Serial.println(accAngleX, 4);
    Serial.print("gyroAngleX = ");
    Serial.println(gyroAngleX, 4);
    Serial.print("accAngleY = ");
    Serial.println(accAngleY, 4);
    Serial.print("gyroAngleY = ");
    Serial.println(gyroAngleY, 4);


    
    Serial.print("Roll = ");
    Serial.println(roll, 4);
    Serial.print("Pitch = ");
    Serial.println(pitch, 4);
    Serial.print("Yaw = ");
    Serial.println(yaw, 4);

    //tout mettre dans une liste et la return
}

//################################## GPS ##################################
double GPS(){
    if (SoftSerial.available())                     // if date is coming from software serial port ==> data is coming from SoftSerial shield
    {
        while(SoftSerial.available())               // reading data into char array
        {
            buffer[count++]=SoftSerial.read();      // writing data into array
            if(count == 64)break;
        }
        Serial.write(buffer,count);                 // if no data transmission ends, write buffer to hardware serial port
        clearBufferArray();                         // call clearBufferArray function to clear the stored data from the array
        count = 0;                                  // set counter of while loop to zero 
    }
    if (Serial.available())                 // if data is available on hardware serial port ==> data is coming from PC or notebook
    SoftSerial.write(Serial.read());        // write it to the SoftSerial shield

    //parser, mettre dans une liste et return
}

void clearBufferArray()                     // GPS function to clear buffer array
{
    for (int i=0; i<count;i++)
    {
        buffer[i]=NULL;
    }                      // clear all index of array with command NULL
}

//################################## SD ##################################
void writeIntoSD(pTzHvalues, accelGyroValues, GPSValues){
    // make a string for assembling the data to log:

    String dataString = "P : " + pTzHvalues[0] + "," + " T : " + pTzHvalues[1] + "," + " z : " + pTzHvalues[2] + "," + " H : " + pTzHvalues[3] + ","
    + " AccX : " + accelGyroValues[0] + "," + " AccY : " + accelGyroValues[1] + "," + " GyroX : " + accelGyroValues[2] + "," + " GyroY : " + accelGyroValues[3]
    + "," + " GyroZ : " + accelGyroValues[4] + "," + " AngleX : " + accelGyroValues[5] + "," + " AngleY : " + accelGyroValues[6] + ","  + " AngleZ : " 
    + accelGyroValues[7] + ","  + " Longitude : " + GPSValues[0] + "," + " Latitude : " + GPSValues[1] + "," + " Altitude : " + GPSValues[2];


    // open the file. note that only one file can be open at a time,

    // so you have to close this one before opening another.

    File dataFile = SD.open("datalog.txt", FILE_WRITE);

    // if the file is available, write to it:

    if (dataFile) {

    dataFile.println(dataString);

    dataFile.close();

    }

    // if the file isn't open, pop up an error:

    else {

    Serial.println("error opening datalog.txt");

    }
}

// Lora
void LoraTX(message) {
    LoRa.beginPacket();                   // start packet
    LoRa.print(message);                 // add payload
    LoRa.endPacket();                     // finish packet and send it 
    Serial.println("Sent");
}

// Chaufferette
void heater(T) {
    if (T<-15) {
     // activer la chaufferette : à coder avec un interrupteur 
    }
    else {
        // désactiver la chaufferette;
    }
}
