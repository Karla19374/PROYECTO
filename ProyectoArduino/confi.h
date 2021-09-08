/************************ Adafruit IO Config *******************************/

// visit io.adafruit.com if you need to create an account,
// or if you need your Adafruit IO key.
#define IO_USERNAME "Karla19374"
#define IO_KEY "aio_bpju64QkQTAZFNDmax159fN0D1p8"
/******************************* WIFI **************************************/
#define WIFI_SSID "TIGO-3281"
#define WIFI_PASS "2NB144207118"

// comment out the following lines if you are using fona or ethernet
#include "AdafruitIO_WiFi.h"
AdafruitIO_WiFi io(IO_USERNAME, IO_KEY, WIFI_SSID, WIFI_PASS);
