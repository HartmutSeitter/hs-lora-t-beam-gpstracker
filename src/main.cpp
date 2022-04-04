#include <Arduino.h>
// Fix Parameters
// Possible Values for Serial_Print_Mode  ! DONT TOUCH !
//
// This is a tesprogram for ESP32 boards with Lora
//
// It uses parts of the multigeiger program
//  I tested it in Jan 2020 successfully
// in the userdefines.h some selection must be made.
// Also check patformio.ini file to select the correct board



#define   Serial_None            0  // No Serial Printout
#define   Serial_Debug           1  // Only debug and error output will be printed via RS232(USB)
#define   Serial_Logging         2  // Log measurement as table via RS232(USB)
#define   Serial_One_Minute_Log  3  // One Minute logging will be printed via RS232(USB)
#define   Serial_Statistics_Log  4  // Lists time between two events in us via RS232(USB)

// Usable CPU-Types
// WIFI -> Heltev Wifi Kit 32
#define WIFI 0
// LORA  ->  Heltec Wifi Lora 32 (V2)
#define LORA 1
// STICK ->  Heltec Wireless Stick  (has LoRa on board)
#define STICK 2
//
// Includes
//====================================================================================================================================
#include "userdefines.h"

//====================================================================================================================================
#include <Arduino.h>
#include <U8x8lib.h>

char          ssid[30];

// T-Beam specific hardware
#define BUILTIN_LED 21

// this is my ttgo t-beam board
#define SERIAL1_RX 12  // GPS_RX -> 12
#define SERIAL1_TX 15  // GPS_TX -> 15
String read_sentence;

// buffer to send to ttn
uint8_t tx_payload[11];

double lat, lon, alt, kmph; // GPS data are saved here: Latitude, Longitude, Altitude, Speed in km/h
float tmp, hum, pressure, alt_barometric; // BME280 data are saved here: Temperature, Humidity, Pressure, Altitude calculated from atmospheric pressure
int sats; // GPS satellite count

char s[32]; // used to sprintf for Serial output

String gps_lat;
String gps_lon;
String gps_sat;
String gps_hgt;
String gps_lat_o;
String gps_lon_o;

// for LoRa

#include "loraWan.h"


unsigned long getESPchipID() {
  uint64_t espid = ESP.getEfuseMac();
  uint8_t *pespid = (uint8_t*)&espid;
  uint32_t id = 0;
  uint8_t *pid = (uint8_t *)&id;
  pid[0] = (uint8_t)pespid[5];
  pid[1] = (uint8_t)pespid[4];
  pid[2] = (uint8_t)pespid[3];
  Serial.printf("ID: %08X\n", id);
  Serial.printf("MAC: %04X%08X\n",(uint16_t)(espid>>32),(uint32_t)espid);
  return id;
}
// LoRa payload:
// to minimise airtime, we only send necessary bytes. We do NOT use Cayenne LPP.
// The payload will be translated via http integration and a small python program
// to be compatible with luftdaten.info. For byte definitions see ttn2luft.pdf in
// docs directory

// LoRa payload:
// send data to TTN
// the formatted GPS Data are 11 bytes 
void sendData2TTN(int sendwhat,  uint8_t * ttndata) {
  Serial.println("sendData2TTN");
  int cnt;
  cnt = 11;
  lorawan_send(1,ttndata,cnt,false,NULL,NULL,NULL);
}
String sentence_sep(String input, int index) {
  int finder = 0;
  int strIndex[] = { 0, -1 };
  int maxIndex = input.length() - 1;

  for (int i = 0; i <= maxIndex && finder <= index; i++) {
    if (input.charAt(i) == ',' || i == maxIndex) {  //',' = separator
      finder++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }
  
  return finder > index ? input.substring(strIndex[0], strIndex[1]) : "";
}

//
// convert the gps coordinates
//
float convert_gps_coord(float deg_min, String orientation) {
  double gps_min = fmod((double)deg_min, 100.0);
  int gps_deg = deg_min / 100;
  double dec_deg = gps_deg + ( gps_min / 60 );
  if (orientation == "W" || orientation == "S") {
    dec_deg = 0 - dec_deg;
  }
  return dec_deg;
}

//
// generate the payload to be sent
//
void generate_payload(double lat, double lon, int alt, int sat) {
  uint32_t LatitudeBinary = ((lat + 90) * 1000000);
  uint32_t LongitudeBinary = ((lon + 180) * 1000000);

  uint8_t payload[11];

  payload[0] = ( LatitudeBinary >> 24 ) & 0xFF;
  payload[1] = ( LatitudeBinary >> 16 ) & 0xFF;
  payload[2] = ( LatitudeBinary >> 8 ) & 0xFF;
  payload[3] = LatitudeBinary & 0xFF;

  payload[4] = ( LongitudeBinary >> 24 ) & 0xFF;
  payload[5] = ( LongitudeBinary >> 16 ) & 0xFF;
  payload[6] = ( LongitudeBinary >> 8 ) & 0xFF;
  payload[7] = LongitudeBinary & 0xFF;

  payload[8] = ( alt >> 8 ) & 0xFF;
  payload[9] = alt & 0xFF;

  payload[10] = sat & 0xFF;
  
  int i = 0;
  while (i < sizeof(payload)) {
    tx_payload[i] = payload[i];
    i++;
  }
}




void setup()
{ 
  Serial.begin(115200);
  delay(1000);
  Serial.println("Let's go!");
  Serial.println(F("TTN Mapper"));
  // setup 2nd serial communication for GPS module on ttgo t-team
  Serial1.begin(9600, SERIAL_8N1, SERIAL1_RX, SERIAL1_TX);

  // read the chip id
  uint32_t xx = getESPchipID();

  // build SSID
  sprintf(ssid,"ESP32-%d",xx);
  Serial.println(ssid);
  
  // init LoRa
     lorawan_setup();
} // end of setup


void loop()
{


   // read the serial data from GPS Module
  read_sentence = Serial1.readStringUntil(13); //13 = return (ASCII)
  read_sentence.trim();

  if (read_sentence.startsWith("$GPGGA")) {
    String gps_lat = sentence_sep(read_sentence, 2); //Latitude in degrees & minutes
    String gps_lon = sentence_sep(read_sentence, 4); //Longitude in degrees & minutes
    String gps_sat = sentence_sep(read_sentence, 7);
    String gps_hgt = sentence_sep(read_sentence, 9);
    String gps_lat_o = sentence_sep(read_sentence, 3); //Orientation (N or S)
    String gps_lon_o = sentence_sep(read_sentence, 5); //Orientation (E or W

    // print the data on debugging monitor
    Serial.print("LAT: ");
    Serial.print(gps_lat);
    Serial.print(" LON: ");
    Serial.print(gps_lon);
    Serial.print(" HEIGHT: ");
    Serial.print(gps_hgt);
    Serial.print(" SAT: ");
    Serial.println(gps_sat);

  } else {
    // in the real app - just send data only GPS is available
    // no gps signal - for test purpose send dummy values
    gps_lat = "4840.74971";
    gps_lon = "00854.16609";
    gps_sat = "05";
    gps_hgt = "453.2";
    gps_lat_o = "N";
    gps_lon_o = "E";

    Serial.print("LAT dummy: ");
    Serial.print(gps_lat);
    Serial.print(" LON dummy: ");
    Serial.print(gps_lon);
    Serial.print(" HEIGHT dummy: ");
    Serial.print(gps_hgt);
    Serial.print(" SAT dummy: ");
    Serial.println(gps_sat);
  }
  float Latitude = convert_gps_coord(gps_lat.toFloat(), gps_lat_o);
  float Longitude = convert_gps_coord(gps_lon.toFloat(), gps_lon_o);
  float Altitude = gps_hgt.toFloat();
  int Sat = gps_sat.toInt();
  generate_payload(Latitude, Longitude, Altitude, Sat);

  Serial.println("Sending to TTN ...");
    
  sendData2TTN(1,tx_payload);
  delay(100);

}