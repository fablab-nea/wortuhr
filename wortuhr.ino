#include <Adafruit_NeoPixel.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <TimeLib.h>
#include <Ticker.h>

#define LED_PIN 5 // GPIO5 = D1
#define LED_COUNT 114
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

char ssid[] = "neustadt-aisch.freifunk.net";
char pass[] = "";
IPAddress timeServer(132, 163, 4, 101); // time-a.timefreq.bldrdoc.gov
int timeZone = 1;

WiFiUDP Udp;

struct phrase {
  uint8_t row;
  uint8_t col;
  uint8_t len;
};

const uint8_t ledGrid[10][11] =  {
  { 0,    1,   2,   3,   4,  5,    6,   7,   8,   9,  10},
  { 21,  20,  19,  18,  17,  16,  15,  14,  13,  12,  11},
  { 22,  23,  24,  25,  26,  27,  28,  29,  30,  31,  32},
  { 43,  42,  41,  40,  39,  38,  37,  36,  35,  34,  33},
  { 44,  45,  46,  47,  48,  49,  50,  51,  52,  53,  54},
  { 65,  64,  63,  62,  61,  60,  59,  58,  57,  56,  55},
  { 66,  67,  68,  69,  70,  71,  72,  73,  74,  75,  76},
  { 87,  86,  85,  84,  83,  82,  81,  80,  79,  78,  77},
  { 88,  89,  90,  91,  92,  93,  94,  95,  96,  97,  98},
  {109, 108, 107, 106, 105, 104, 103, 102, 101, 100,  99}
};

uint8_t minutesIndex[] = {
  111, // 1
  112, // 2
  113, // 3
  110  // 4
};

const phrase hourTable[14] = {
  {8, 6, 5}, // zwölf
  {5, 0, 4}, // eins
  {5, 7, 4}, // zwei
  {6, 0, 4}, // drei
  {6, 7, 4}, // vier
  {4, 7, 4}, // fünf
  {7, 0, 5}, // sechs
  {8, 0, 6}, // sieben
  {7, 7, 4}, // acht
  {9, 3, 4}, // neun
  {9, 0, 4}, // zehn
  {4, 5, 3}, // elf
  {0, 0, 0}, // reserved
  {5, 0, 3}  // ein
};

const phrase phrase_es          = {0, 0,  2};
const phrase phrase_ist         = {0, 3,  3};

const phrase phrase_uhr         = {9, 8,  3};

const phrase phrase_fuenf       = {0, 7,  4};
const phrase phrase_zehn        = {1, 0,  4};
const phrase phrase_zwanzig     = {1, 4,  7};

const phrase phrase_viertel     = {2, 4,  7};
const phrase phrase_halb        = {4, 0,  4};
const phrase phrase_dreiviertel = {2, 0, 11};

const phrase phrase_vor         = {3, 0,  3};
const phrase phrase_nach        = {3, 7,  4};

void printWifiStatus() {
  Serial.print("SSID: ");
  Serial.print(WiFi.SSID());
  Serial.print("\r\n");

  IPAddress ip = WiFi.localIP();
  Serial.print("IP: ");
  Serial.print(ip);
  Serial.print("\r\n");

  long rssi = WiFi.RSSI();
  Serial.print("RSSI: ");
  Serial.print(rssi);
  Serial.print(" dBm");
}

/*-------- NTP code ----------*/
const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address)
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}

time_t getNtpTime()
{
  while (Udp.parsePacket() > 0) ; // discard any previously received packets
  Serial.println("Transmit NTP Request");
  sendNTPpacket(timeServer);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = Udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      Serial.println("Receive NTP Response");
      Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
    }
  }
  Serial.println("No NTP Response :-(");
  return 0; // return 0 if unable to get the time
}

void syncTime() {
  Udp.begin(8888);
  setSyncProvider(getNtpTime);
}

void clearLeds(bool sync) {
  for(int i=0;i<LED_COUNT;i++){
    pixels.setPixelColor(i, pixels.Color(0,0,0));
  }
  if (sync) {
    pixels.show();
  }
}

void putPhrase(const phrase *p, uint32_t c) {
  if (p->len > 0) {
    for (uint8_t i=0; i < p->len; i++) {
      pixels.setPixelColor(ledGrid[p->row][p->col+i], c);
    }
  }
}

void setup() {
  pixels.begin();
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    delay(500);
  }
  
  Serial.println("Connected to wifi");
  printWifiStatus();
 
  clearLeds(true);
  syncTime();
}

void loop() {
  uint8_t h;
  phrase (*pMinutes)[3];
  phrase *pHour;

  Serial.print("Time: ");
  Serial.print(hour());
  Serial.print(":");
  Serial.print(minute());
  Serial.print(":");
  Serial.println(second());
  
  clearLeds(false);
  
  uint32_t minutesColor = pixels.Color(0,0,16);
  uint32_t normalColor = pixels.Color(8,8,8);
  uint32_t hoursColor = pixels.Color(16,0,0);
  uint32_t exactMinutesColor = pixels.Color(0,16,0);
  
  h = hour()%12;

  putPhrase(&phrase_es, normalColor);
  putPhrase(&phrase_ist, normalColor);

  switch (minute()/5) {
    case 0:
      if (h == 1)
		h = 13; // "es ist ein uhr"
      putPhrase(&phrase_uhr, normalColor);
      break;
      
    case 1:
      putPhrase(&phrase_fuenf, minutesColor);
      putPhrase(&phrase_nach, normalColor);
      break;
      
    case 2:
      putPhrase(&phrase_zehn, minutesColor);
      putPhrase(&phrase_nach, normalColor);
      break;
      
    case 3:
      putPhrase(&phrase_viertel, minutesColor);
      
      //putPhrase(&phrase_nach, normalColor);

      h++;
      break;
      
    case 4:
      putPhrase(&phrase_zwanzig, minutesColor);
      putPhrase(&phrase_nach, normalColor);
      break;

    case 5:
      putPhrase(&phrase_fuenf, minutesColor);
      putPhrase(&phrase_vor, normalColor);
      putPhrase(&phrase_halb, minutesColor);
      h++;
      break;
      
    case 6:
      putPhrase(&phrase_halb, minutesColor);
      h++;
      break;
      
    case 7:
      putPhrase(&phrase_fuenf, minutesColor);
      putPhrase(&phrase_nach, normalColor);
      putPhrase(&phrase_halb, minutesColor);
      h++;
      break;
      
    case 8:
      putPhrase(&phrase_zwanzig, minutesColor);
      putPhrase(&phrase_vor, normalColor);
      h++;
      break;
      
    case 9:
      putPhrase(&phrase_dreiviertel, minutesColor);
      h++;
      break;

    case 10:
      putPhrase(&phrase_zehn, minutesColor);
      putPhrase(&phrase_vor, normalColor);
      h++;
      break;
      
    case 11:
      putPhrase(&phrase_fuenf, minutesColor);
      putPhrase(&phrase_vor, normalColor);
      h++;
      break;
  }

  if (h==12) h=0;
  
  // hour
  putPhrase(&hourTable[h], hoursColor);


  // single leds for exact minutes
  for(uint8_t i=0; i<minute()%5; i++) {
    pixels.setPixelColor(minutesIndex[i], exactMinutesColor);
  }

  pixels.show();
  yield();
  delay(1000);
}

