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

struct ClientInfo {
  mesh::Identity id;
  uint32_t last_timestamp;
  uint8_t secret[PUB_KEY_SIZE];
  int out_path_len;
  uint8_t out_path[MAX_PATH_SIZE];
};

#define MAX_CLIENTS   4

class MyMesh : public mesh::Mesh {
  int num_clients;
  ClientInfo known_clients[MAX_CLIENTS];

  ClientInfo* putClient(const mesh::Identity& id) {
    for (int i = 0; i < num_clients; i++) {
      if (id.matches(known_clients[i].id)) return &known_clients[i];  // already known
    }
    if (num_clients < MAX_CLIENTS) {
      auto newClient = &known_clients[num_clients++];
      newClient->id = id;
      newClient->out_path_len = -1;  // initially out_path is unknown
      newClient->last_timestamp = 0;
      self_id.calcSharedSecret(newClient->secret, id);   // calc ECDH shared secret
      return newClient;
    }
    return NULL;  // table is full
  }

protected:
  void onAnonDataRecv(mesh::Packet* packet, uint8_t type, const mesh::Identity& sender, uint8_t* data, size_t len) override {
    if (type == PAYLOAD_TYPE_ANON_REQ) {  // received a PING!
      uint32_t timestamp;
      memcpy(&timestamp, data, 4);

      auto client = putClient(sender);  // add to known clients (if not already known)
      if (client == NULL || timestamp <= client->last_timestamp) {
        return;  // FATAL: client table is full -OR- replay attack 
      }

      client->last_timestamp = timestamp;

      uint32_t now = getRTCClock()->getCurrentTime(); // response packets always prefixed with timestamp

      if (packet->isRouteFlood()) {
        // let this sender know path TO here, so they can use sendDirect(), and ALSO encode the Ping response
        mesh::Packet* path = createPathReturn(sender, client->secret, packet->path, packet->path_len,
                                              PAYLOAD_TYPE_RESPONSE, (uint8_t *) &now, sizeof(now));
        if (path) sendFlood(path);
      } else {
        mesh::Packet* reply = createDatagram(PAYLOAD_TYPE_RESPONSE, sender, client->secret, (uint8_t *) &now, sizeof(now));
        if (reply) {
          if (client->out_path_len >= 0) {  // we have an out_path, so send DIRECT
            sendDirect(reply, client->out_path, client->out_path_len);
          } else {
            sendFlood(reply);
          }
        }
      }
    }
  }

  int  matching_peer_indexes[MAX_CLIENTS];

  int searchPeersByHash(const uint8_t* hash) override {
    int n = 0;
    for (int i = 0; i < num_clients; i++) {
      if (known_clients[i].id.isHashMatch(hash)) {
        matching_peer_indexes[n++] = i;  // store the INDEXES of matching contacts (for subsequent 'peer' methods)
      }
    }
    return n;
  }

  // not needed for this example, but for sake of 'completeness' of Mesh impl
  void getPeerSharedSecret(uint8_t* dest_secret, int peer_idx) override {
    if (peer_idx >= 0 && peer_idx < MAX_CLIENTS) {
      // lookup pre-calculated shared_secret
      int i = matching_peer_indexes[peer_idx];
      memcpy(dest_secret, known_clients[i].secret, PUB_KEY_SIZE);
    } else {
      MESH_DEBUG_PRINTLN("Invalid peer_idx: %d", peer_idx);
    }
  }

  bool onPeerPathRecv(mesh::Packet* packet, int sender_idx, const uint8_t* secret, uint8_t* path, uint8_t path_len, uint8_t extra_type, uint8_t* extra, uint8_t extra_len) override {
    if (sender_idx >= 0 && sender_idx < MAX_CLIENTS) {
      Serial.printf("PATH to client, path_len=%d\n", (uint32_t) path_len);

      // TODO: prevent replay attacks

      int i = matching_peer_indexes[sender_idx];
      if (i >= 0 && i < num_clients) {
        auto client = &known_clients[i];  // get from our known_clients table (sender SHOULD already be known in this context)
        memcpy(client->out_path, path, client->out_path_len = path_len);  // store a copy of path, for sendDirect()
      }
    } else {
      MESH_DEBUG_PRINTLN("Invalid sender_idx: %d", sender_idx);
    }

    // NOTE: no reciprocal path send!!
    return false;
  }

public:
  MyMesh(mesh::Radio& radio, mesh::MillisecondClock& ms, mesh::RNG& rng, mesh::RTCClock& rtc, mesh::MeshTables& tables)
     : mesh::Mesh(radio, ms, rng, rtc, *new StaticPoolPacketManager(16), tables)
  {
    num_clients = 0;
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
