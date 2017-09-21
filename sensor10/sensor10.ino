/*
   sensor10
   originated @ cyclehack berlin 2017

*/

// libraries
// TTN, lmic and hal are available via Library management tool of the Arduino IDE
#include <TheThingsNetwork.h>
#include "SDS011.h" // SDS011 Particulate Matter Sensor
#include <lmic.h> // LoRaWAN
#include <hal/hal.h> // Harware Abstraction Library
#include <TinyGPS.h> // Download at http://arduiniana.org/libraries/tinygps/

// your The Things Network keys
#include "ttnKeys.h"

// PINS
#define RX 51
#define TX 50
const lmic_pinmap lmic_pins = {
  .nss = 10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 9,
  .dio = {2, 6, 7}, // Specify pin numbers for DIO0, 1, 2
  // connected to D2, D6, D7
};

// values for sensor data
float p10, p25;
int sds_error, nr_sat;
unsigned long lati, longi;
static uint8_t message[12];

const unsigned TX_INTERVAL = 10;
static osjob_t sendjob;

// define sensors
SDS011 sds;
TinyGPS gps;

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static void smartdelay(unsigned long ms)
{
  unsigned long start = millis();
  do {
    while (Serial1.available()) {
      //ss.print(Serial.read());
      gps.encode(Serial1.read());
    }
  } while (millis() - start < ms);
}

static void print_float(float val, float invalid, int len, int prec) {
  if (val == invalid) {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  } else {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i = flen; i < len; ++i)
      Serial.print(' ');
  }
  smartdelay(0);
}

static void print_int(unsigned long val, unsigned long invalid, int len)
{
  char sz[32];
  if (val == invalid)
    strcpy(sz, "*******");
  else
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i = strlen(sz); i < len; ++i)
    sz[i] = ' ';
  if (len > 0)
    sz[len - 1] = ' ';
  Serial.print(sz);
  smartdelay(0);
}

void onEvent (ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      break;
    case EV_RFU1:
      Serial.println(F("EV_RFU1"));
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.dataLen) {
        // data received in rx slot after tx
        Serial.print(F("Data Received: "));
        Serial.write(LMIC.frame + LMIC.dataBeg, LMIC.dataLen);
        Serial.println();
      }
      // Schedule next transmission
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    default:
      Serial.println(F("Unknown event"));
      break;
  }
}

void do_send(osjob_t* j) {
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    // Prepare upstream data transmission at the next possible time.
    LMIC_setTxData2(1, message, sizeof(message), 0);
    Serial.println(F("Packet queued"));
  }
  // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
  Serial.begin(9600);
  Serial.println(F("Starting"));
  Serial1.begin(9600);
/*  while (!Serial) {
    ;
  };*/
  Serial.println("TinyGPS library v. "); Serial.println(TinyGPS::library_version());
  // init SDS
  sds.begin(RX, TX);

  Serial.println("SDS initialized");
  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  // Set static session parameters. Instead of dynamically establishing a session
  // by joining the network, precomputed session parameters are be provided.
#ifdef PROGMEM
  // On AVR, these values are stored in flash and only copied to RAM
  // once. Copy them to a temporary buffer here, LMIC_setSession will
  // copy them into a buffer of its own again.
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
#else
  // If not running an AVR with PROGMEM, just use the arrays directly
  LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
#endif

  // Disable link check validation
  LMIC_setLinkCheckMode(0);

  // Set data rate and transmit power (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF7, 14);

  Serial.println("LoRa setup done");
}

void read_gps_data() {
  float flat, flon;
  unsigned long age, date, time, chars = 0;
  unsigned short sentences = 0, failed = 0;

  print_int(gps.satellites(), TinyGPS::GPS_INVALID_SATELLITES, 5);
  nr_sat = gps.satellites();
  print_int(gps.hdop(), TinyGPS::GPS_INVALID_HDOP, 5);
  gps.f_get_position(&flat, &flon, &age);

  lati = flat * 1000000;
  longi = flon * 1000000;
  print_float(flat, TinyGPS::GPS_INVALID_F_ANGLE, 10, 6);
  print_float(flon, TinyGPS::GPS_INVALID_F_ANGLE, 11, 6);

  gps.stats(&chars, &sentences, &failed);
}

void loop() {
  read_gps_data();
  sds_error = sds.read(&p25, &p10);
  Serial.print("pm10: ");
  Serial.print(p10);
  Serial.print(", pm2,5: ");
  Serial.println(p25);

  if (nr_sat >= 3 && lati != 1000000000) {
    if (!sds_error) {
      message[0] = (lati & 0xFF000000) >> 24;
      message[1] = (lati & 0x00FF0000) >> 16;
      message[2] = (lati & 0x0000FF00) >> 8;
      message[3] = (lati & 0x000000FF);
      message[4] = (longi & 0xFF000000) >> 24;
      message[5] = (longi & 0x00FF0000) >> 16;
      message[6] = (longi & 0x0000FF00) >> 8;
      message[7] = (longi & 0x000000FF);
      int p10_copy = (int) p10;
      message[8] = (p10_copy & 0xFF00) >> 8;
      message[9] = (p10_copy & 0x00FF);
      int p25_copy = (int) p25;
      message[10] = (p25_copy & 0xFF00) >> 8;
      message[11] = (p25_copy & 0x00FF);

      do_send(&sendjob);
    } else {
      Serial.println("error reading SDS");
    }
  } else {
    Serial.println("GPS accuracy too low!");
  }
  os_runloop_once(); // LMIC needs this
  smartdelay(1000);
}
