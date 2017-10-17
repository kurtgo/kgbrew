#include <TimerOne.h>

/***************************************************
  Adafruit MQTT Library WINC1500 Example

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/
#include <SPI.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <Adafruit_WINC1500.h>
#include <Adafruit_MAX31865.h>

// The value of the Rref resistor. Use 430.0!
#define RREF 430.0
#define PUBLISH_MS 60000


/************************* WiFI Setup *****************************/
#define WINC_CS_PIN   8
#define WINC_IRQ  7
#define WINC_RST  4
#define WINC_EN   5     // or, tie EN to VCC

#define SSR_PIN   9
#define MAX31865_CS_PIN 10
#define MCP4132_CS_PIN 6

#include "/kghome.h"
int keyIndex = 0;
#define POLL_TIME 2000  // poll MQTT every 2 seconds



class MCP4132 {
public:
	int pot_cs;
	const byte write = 0b00000000;
	const byte read  = 0b00000100;
	MCP4132(int cs) { pot_cs = cs; };

	void digitalPotWrite(int value)
	{
		digitalWrite(pot_cs, LOW);
		SPI.transfer(write+((value >> 8) & 0x3));
		SPI.transfer((byte)value);
		digitalWrite(pot_cs, HIGH);
	}
	void test() {
		digitalPotWrite(0x00);
		delay(5000);

		// adjust  wiper in the  Mid point  .
		digitalPotWrite(0x40);
		delay(5000);

		// adjust Lowest Resistance .
		digitalPotWrite(0x80);
		delay(5000);

		// adjust Lowest Resistance .
		digitalPotWrite(0xd0);
		delay(5000);

		// adjust Lowest Resistance .
		digitalPotWrite(0x100);
		delay(5000);

		digitalPotWrite(0x140);
		delay(5000);

		// adjust Lowest Resistance .
		digitalPotWrite(0x0);
		delay(5000);
	}
};

float CtoF(float celsius)
{
	float fahrenheit = (1.8 * celsius) + 32;
	return fahrenheit;
}


Adafruit_WINC1500 WiFi(WINC_CS_PIN, WINC_IRQ, WINC_RST);
Adafruit_MAX31865 max31865 = Adafruit_MAX31865(MAX31865_CS_PIN);
MCP4132 mcp4132 = MCP4132(MCP4132_CS_PIN);

int status = WL_IDLE_STATUS;

/************************* Adafruit.io Setup *********************************/



/************ Global State (you don't need to change this!) ******************/

//Set up the wifi client
Adafruit_WINC1500Client client;

Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

// You don't need to change anything below this line!
#define halt(s) { Serial.println(F( s )); while(1);  }

/****************************** Feeds ***************************************/

// Setup a feed called 'photocell' for publishing.
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
Adafruit_MQTT_Publish temp = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/kg_brewtemp");

// Setup a feed called 'onoff' for subscribing to changes.
Adafruit_MQTT_Subscribe onoffbutton = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/kg_brewonoff");
Adafruit_MQTT_Subscribe settemp = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/kg_brewrtmp");
Adafruit_MQTT_Subscribe setpower = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/kg_brewpwr");

/*************************** Sketch Code ************************************/

#define LEDPIN 13
void setup() {
  while (!Serial);
  Serial.begin(115200);

  Serial.println(F("KG Brewer"));


  // set the chip selects for the SPI bus to all off
  pinMode(MAX31865_CS_PIN, OUTPUT);
  digitalWrite(MAX31865_CS_PIN, HIGH);

  pinMode(MCP4132_CS_PIN, OUTPUT);
  digitalWrite(MCP4132_CS_PIN, HIGH);

  pinMode(WINC_CS_PIN, OUTPUT);
  digitalWrite(WINC_CS_PIN, HIGH);

#ifdef WINC_EN
  pinMode(WINC_EN, OUTPUT);
  digitalWrite(WINC_EN, HIGH);
#endif

  max31865.begin(MAX31865_3WIRE);  // set to 2WIRE or 4WIRE as necessary
  if (max31865.readFault() != 0) {
	  Serial.println(F("MAX31865 failed to init"));
  } else {
	  Serial.println(F("MAX31865 functioning!!!"));
  }
  Serial.print(F("temp="));
  Serial.println(CtoF(max31865.temperature(100,RREF)));
  getTemp();


  Timer1.initialize(166666); // set 30hz period

  pinMode(SSR_PIN, OUTPUT);
  digitalWrite(SSR_PIN, LOW);
  Timer1.pwm(SSR_PIN, 0);



  // Initialize the Client
  Serial.print(F("\nInit the WiFi module..."));
  // check for the presence of the breakout
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WINC1500 not present");
    // don't continue:
    while (true);
  }
  Serial.println(F("ATWINC OK!"));

  pinMode(LEDPIN, OUTPUT);
  mqtt.subscribe(&onoffbutton);
  mqtt.subscribe(&setpower);
  mqtt.subscribe(&settemp);
}


uint32_t power = 0;
uint32_t pwm_on = 0;
uint32_t target_temp = 0;
float cur_temp = 0;
uint32_t lastms = 0;

float getTemp()
{
  uint16_t rtd = max31865.readRTD();
  float tmp = max31865.temperature(100, RREF);

//  Serial.print(F("RTD value: ")); Serial.println(rtd);
  float ratio = rtd;
  ratio /= 32768;
//  Serial.print(F("Ratio = ")); Serial.println(ratio,8);
//  Serial.print(F("Resistance = ")); Serial.println(RREF*ratio,8);
//  Serial.print(F("Temperature = ")); Serial.println(tmp);

  // Check and print any faults
  uint8_t fault = max31865.readFault();
  if (fault) {
    Serial.print(F("Fault 0x")); Serial.println(fault, HEX);
    if (fault & MAX31865_FAULT_HIGHTHRESH) {
      Serial.println(F("RTD High Threshold"));
    }
    if (fault & MAX31865_FAULT_LOWTHRESH) {
      Serial.println(F("RTD Low Threshold"));
    }
    if (fault & MAX31865_FAULT_REFINLOW) {
      Serial.println(F("REFIN- > 0.85 x Bias"));
    }
    if (fault & MAX31865_FAULT_REFINHIGH) {
      Serial.println(F("REFIN- < 0.85 x Bias - FORCE- open"));
    }
    if (fault & MAX31865_FAULT_RTDINLOW) {
      Serial.println(F("RTDIN- < 0.85 x Bias - FORCE- open"));
    }
    if (fault & MAX31865_FAULT_OVUV) {
      Serial.println(F("Under/Over voltage"));
    }
    max31865.clearFault();
  }
  Serial.println();
  return tmp;
}
void loop() {
  uint32_t on = pwm_on;
  uint32_t newpower = power;
  uint32_t newtemp = target_temp;

  // Ensure the connection to the MQTT server is alive (this will make the first
  // connection and automatically reconnect when disconnected).  See the MQTT_connect
  // function definition further below.

  MQTT_connect();

  // this is our 'wait for incoming subscription packets' busy subloop
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(POLL_TIME))) {
    if (subscription == &onoffbutton) {

      if (0 == strcmp((char *)onoffbutton.lastread, "OFF")) {
        on = 0;
      } else {
        on = 1;
      }
    } else
    if (subscription == &setpower) {
      char *value = (char *)setpower.lastread;
      newpower  = atoi(value);
      Serial.print(F("\nsetpower="));
      Serial.print(newpower);
    } else
    if (subscription == &settemp) {
      char *value = (char *)setpower.lastread;
      newtemp  = atoi(value);
    }
  }

  float tmp = getTemp();
  Serial.print(F("Temp: "));
  Serial.println(tmp);

  if (power > 100) power = 100;

  target_temp = newtemp;

  if (on != pwm_on || power != newpower) {
      power = newpower;
      if (on) {
          Serial.print(F("\non: power="));
          Serial.print(power);
          Timer1.pwm(SSR_PIN, power*1024/100);
      } else {
          Serial.print(F("\nOFF"));
          Timer1.pwm(SSR_PIN, 0);
      }
      pwm_on = on;
  }

  cur_temp = tmp;

  // Now we can publish stuff!
  uint32_t x = millis();

  if (x-lastms > PUBLISH_MS) {
    if (! temp.publish(CtoF(cur_temp))) {
      Serial.println(F("Failed"));
    } else {
      Serial.print(F("."));
    }
    lastms = x;
  }
}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;

  // attempt to connect to Wifi network:
  while (WiFi.status() != WL_CONNECTED) {

    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    uint8_t timeout = 10;
    while (timeout && (WiFi.status() != WL_CONNECTED)) {
      timeout--;
      delay(1000);
    }
  }

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 5 seconds...");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds
  }
  Serial.println("MQTT Connected!");
}
