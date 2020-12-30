/*
  LoRa Duplex communication with Spreading Factor
  Sends a message every half second, and polls continually
  for new incoming messages. Sets the LoRa radio's spreading factor.
  Spreading factor affects how far apart the radio's transmissions
  are, across the available bandwidth. Radios with different spreading
  factors will not receive each other's transmissions. This is one way you
  can filter out radios you want to ignore, without making an addressing scheme.
  Spreading factor affects reliability of transmission at high rates, however,
  so avoid a hugh spreading factor when you're sending continually.
  See the Semtech datasheet, http://www.semtech.com/images/datasheet/sx1276.pdf
  for more on Spreading Factor.
  created 28 April 2017
  by Tom Igoe
*/
#include <SPI.h>              // include libraries
#include <LoRa.h>

const int csPin = 7;          // LoRa radio chip select
const int resetPin = 6;       // LoRa radio reset
const int irqPin = 1;         // change for your board; must be a hardware interrupt pin

byte msgCount = 0;            // count of outgoing messages
int interval = 2000;          // interval between sends
long lastSendTime = 0;        // time of last packet send

void setup() {
  Serial.begin(9600);                   // initialize serial
  while (!Serial);

  Serial.println("LoRa Duplex - Set spreading factor");

  // override the default CS, reset, and IRQ pins (optional)
  LoRa.setPins(csPin, resetPin, irqPin); // set CS, reset, IRQ pin

  if (!LoRa.begin(868E6)) {             // initialize ratio at 915 MHz
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                       // if failed, do nothing
  }
  
 // Paramètre à faire varier en re-téléversant le programme à chaque fois, ou une boucle for mais on a moins de temps d'analyser les résultats, à voir...
  
  LoRa.setSpreadingFactor(8);          //Entre 6 et 12, 7 par défaut, il doit être a priori le même en récepteur et émetteur, à vérifier également par les tests.
  Serial.println("LoRa init succeeded.");
}

void loop() {
  // parse for a packet, and call onReceive with the result:
  onReceive(LoRa.parsePacket());
}

void onReceive(int packetSize) {
  if (packetSize == 0) return;          // if there's no packet, return

  // read packet header bytes:
  String incoming = "";

  while (LoRa.available()) {
    incoming += (char)LoRa.read();
  }

  Serial.println("Message: " + incoming);
  Serial.println("RSSI: " + String(LoRa.packetRssi()));
  Serial.println("Snr: " + String(LoRa.packetSnr()));
  Serial.println();
}
