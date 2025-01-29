#include <Arduino.h>   // needed for PlatformIO
#include <Mesh.h>
#include <SPIFFS.h>
#include <Adafruit_BME280.h>

#define RADIOLIB_STATIC_ONLY 1
#include <RadioLib.h>
#include <helpers/RadioLibWrappers.h>
#include <helpers/ArduinoHelpers.h>
#include <helpers/StaticPoolPacketManager.h>
#include <helpers/SimpleMeshTables.h>

/* ------------------------------ Config -------------------------------- */

#ifndef LORA_FREQ
  #define LORA_FREQ   915.0
#endif
#ifndef LORA_BW
  #define LORA_BW     125
#endif
#ifndef LORA_SF
  #define LORA_SF     9
#endif
#ifndef LORA_CR
  #define LORA_CR      5
#endif

#ifdef HELTEC_LORA_V3
  #include <helpers/HeltecV3Board.h>
  static HeltecV3Board board;
#elif defined(ARDUINO_XIAO_ESP32C3)
  #include <helpers/XiaoC3Board.h>
  #include <helpers/CustomSX1262Wrapper.h>
  #include <helpers/CustomSX1268Wrapper.h>
  static XiaoC3Board board;
#elif defined(SEEED_XIAO_S3)
  #include <helpers/ESP32Board.h>
  #include <helpers/CustomSX1262Wrapper.h>
  static ESP32Board board;
#else
  #error "need to provide a 'board' object"
#endif

/* ------------------------------ Code -------------------------------- */

Adafruit_BME280 bme;

class MyMesh : public mesh::Mesh {

public:
  MyMesh(mesh::Radio& radio, mesh::MillisecondClock& ms, mesh::RNG& rng, mesh::RTCClock& rtc, mesh::MeshTables& tables)
     : mesh::Mesh(radio, ms, rng, rtc, *new StaticPoolPacketManager(16), tables)
  {
  }
};

SPIClass spi;
StdRNG fast_rng;
SimpleMeshTables tables;
SX1262 radio = new Module(P_LORA_NSS, P_LORA_DIO_1, P_LORA_RESET, P_LORA_BUSY, spi);
MyMesh the_mesh(*new RadioLibWrapper(radio, board), *new ArduinoMillis(), fast_rng, *new VolatileRTCClock(), tables);

unsigned long nextAnnounce;

void halt() {
  while (1) ;
}

void setup() {
  Serial.begin(115200);

  board.begin();
#if defined(P_LORA_SCLK)
  spi.begin(P_LORA_SCLK, P_LORA_MISO, P_LORA_MOSI);
  int status = radio.begin(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR, RADIOLIB_SX126X_SYNC_WORD_PRIVATE, 22, 8);
#else
  int status = radio.begin(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR, RADIOLIB_SX126X_SYNC_WORD_PRIVATE, 22, 8);
#endif
  if (status != RADIOLIB_ERR_NONE) {
    Serial.print("ERROR: radio init failed: ");
    Serial.println(status);
    halt();
  }
  fast_rng.begin(radio.random(0x7FFFFFFF));
  the_mesh.begin();

  RadioNoiseListener true_rng(radio);
  the_mesh.self_id = mesh::LocalIdentity(&true_rng);  // create new random identity

  nextAnnounce = 0;

  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor !");
  }

  Serial.print("Temperature : ");
  Serial.println(bme.readTemperature());


}

void loop() {
  if (the_mesh.millisHasNowPassed(nextAnnounce)) {
    char data [30];
    float temp = bme.readTemperature();
    float hum = bme.readHumidity();
    float pres = bme.readPressure();
    Serial.print("Temp : ");
    Serial.print(temp);
    Serial.print(" Humidity : ");
    Serial.print(hum);
    Serial.print(" Pressure : ");
    Serial.println(pres);

    sprintf(data, "SENS %.2f %.2f %.2f", temp, hum, pres);

    Serial.print("Sent packet : ");
    Serial.println((char*) data );

    mesh::Packet* pkt = the_mesh.createAdvert(the_mesh.self_id, (const uint8_t *)data, strlen(data));
    if (pkt) the_mesh.sendFlood(pkt);

    nextAnnounce = the_mesh.futureMillis(30000);  // announce every 30 seconds (test only, don't do in production!)
  }
  the_mesh.loop();


  // TODO: periodically check for OLD entries in known_clients[], and evict
}
