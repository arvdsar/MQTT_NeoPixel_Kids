/*
MQTT NeoPixel Kids
Written by: Alexander van der Sar

Based on the Build status light for Devops, but now with two buttons for kids to have some fun communicating with each other in
kind of secret lights language. Current version only uses the 'pattern button'.

website: https://www.vdsar.net/build-status-light-for-devops/
Repository: https://github.com/arvdsar/MQTT_NeoPixel_Status_Multiple_Improved
3D Print design: https://www.thingiverse.com/thing:4665511

Depending on IotWebConf library v3.x
This library keeps the configuration portal available so you don't have to reflash to change settings.

IMPORTANT: To reduce NeoPixel burnout risk, add 1000 uF capacitor across
pixel power leads, add 300 - 500 Ohm resistor on first pixel's data input
and minimize distance between Arduino and first pixel.  Avoid connecting
on a live circuit...if you must, connect GND first.


v0.1 Initial version and equal to MQTT_NeoPixel_status_Multiple_Improved v1.2. Ready to be changed for the kids :-)
v0.3 working version using half of the pixels to receive and the other half for sending. Left button (Pattern) switches the color.
v0.4 After a reboot of the device you want to see the status of before the reboot. This is accomplished
by using node-red to receive and forward the messages to another topic which does retain the status.
Additionally the send color is also forwared to another topic so at boot it can get this status.
*/

#define VERSIONNUMBER "v0.4 - 14-05-2021"

#include <ESP8266WiFi.h>        //https://github.com/esp8266/Arduino
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <PubSubClient.h>
#include <IotWebConf.h>         // https://github.com/prampec/IotWebConf
#include <IotWebConfUsing.h>    // This loads aliases for easier class names.
#ifdef ESP8266
# include <ESP8266HTTPUpdateServer.h>
#elif defined(ESP32)
# include <IotWebConfESP32HTTPUpdateServer.h>
#endif

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

// Define the button pins. 

const int interruptPinColor = D7; //GPIO 6 (Select Button)
const int interruptPinPattern = D6; //GPIO 7 (Commit Button)

// -- Initial name of the Thing. Used e.g. as SSID of the own Access Point.
// -- Keep it under 16 characters to handle MQTT compatiblity (thingName+ChipID = unique mqttClientID)
const char thingName[] = "NeoPxKids";

// -- Initial password to connect to the Thing, when it creates an own Access Point.
const char wifiInitialApPassword[] = "password";

#define STRING_LEN 128
#define NUMBER_LEN 32
// -- Configuration specific key. The value should be modified if config structure was changed.
#define CONFIG_VERSION "npxk3"

// -- When CONFIG_PIN is pulled to ground on startup, the Thing will use the initial
//      password to buld an AP. (E.g. in case of lost password)
#define CONFIG_PIN D3


// -- Status indicator pin.
//      First it will light up (kept LOW), on Wifi connection it will blink,
//      when connected to the Wifi it will turn off (kept HIGH).
#define STATUS_PIN LED_BUILTIN

// -- Callback method declarations.
void wifiConnected();
void configSaved();
bool formValidator(iotwebconf::WebRequestWrapper* webRequestWrapper);
void handleRoot();
void showLedOffset();
void mqttCallback(char* topic, byte* payload, unsigned int length);

void colorWipe(uint32_t c, uint8_t wait);
void colorWipeIn(uint32_t c, uint8_t wait);
void colorWipeOut(uint32_t c, uint8_t wait);

void theaterChase(uint32_t c, uint8_t wait);
void ICACHE_RAM_ATTR ColorISR();
void ICACHE_RAM_ATTR PatternISR();

DNSServer dnsServer;
WebServer server(80);
#ifdef ESP8266
ESP8266HTTPUpdateServer httpUpdater;
#elif defined(ESP32)
HTTPUpdateServer httpUpdater;
#endif

WiFiClient espClient;
PubSubClient client(espClient); //MQTT

char mqttServerValue[STRING_LEN];
char mqttUserNameValue[STRING_LEN];
char mqttUserPasswordValue[STRING_LEN];
char mqttTopicSendValue[STRING_LEN];
char mqttTopicReceiveValue[STRING_LEN];

char ledOffsetValue[NUMBER_LEN];
char ledBrightnessValue[NUMBER_LEN];

char mqttClientId[STRING_LEN]; //automatically created. not via config!

IotWebConf iotWebConf(thingName, &dnsServer, &server, wifiInitialApPassword, CONFIG_VERSION);
IotWebConfTextParameter mqttServerParam = IotWebConfTextParameter("MQTT server", "mqttServer", mqttServerValue, STRING_LEN);
IotWebConfTextParameter mqttUserNameParam = IotWebConfTextParameter("MQTT user", "mqttUser", mqttUserNameValue, STRING_LEN);
IotWebConfPasswordParameter mqttUserPasswordParam = IotWebConfPasswordParameter("MQTT password", "mqttPass", mqttUserPasswordValue, STRING_LEN);
IotWebConfTextParameter mqttTopicSendParam = IotWebConfTextParameter("MQTT Topic Send", "mqttTopicSend", mqttTopicSendValue, STRING_LEN,NULL,"some/thing/#");  
IotWebConfTextParameter mqttTopicReceiveParam = IotWebConfTextParameter("MQTT Topic Receive", "mqttTopicReceive", mqttTopicReceiveValue, STRING_LEN,NULL,"some/thing/#");
IotWebConfNumberParameter ledOffsetParam = IotWebConfNumberParameter("Led Offset", "ledOffset", ledOffsetValue, NUMBER_LEN, "0");

//LedBrightness: 255 is the max brightness. It will draw to much current if you turn on all leds on white color (12 leds x 20 milliAmps x 3 colors (to make white) = 720 mA. Wemos can handle 500 mA)
//White means all leds Red/Green/Blue on so 3 x 20 mA per pixel. Just to be sure limited the Max setting to 200 instead of 255. No exact science though.
IotWebConfNumberParameter ledBrightnessParam = IotWebConfNumberParameter("Led Brightness", "ledBrightness", ledBrightnessValue, NUMBER_LEN, "60","5..200", "min='5' max='200' step='5'"); //Limited to 200 (out of 255)


#define PIN 4 //Neo pixel data pin (GPIO4 / D2)
#define NUMBEROFLEDS 12 //the amount of Leds on the strip

// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUMBEROFLEDS, PIN, NEO_GRB + NEO_KHZ400);

// IMPORTANT: To reduce NeoPixel burnout risk, add 1000 uF capacitor across
// pixel power leads, add 300 - 500 Ohm resistor on first pixel's data input
// and minimize distance between Arduino and first pixel.  Avoid connecting
// on a live circuit...if you must, connect GND first.

int pixel = 0;      //Indicate which Pixel to light
int inConfig = 0;  //Indicator if you are on config portal or not (for blocking Led Pattern)
long lastMsg = 0;   //timestamp of last MQTT Publish

//long previous_time = 0;
//long current_time = 0;
bool needReset = false;

bool patternInterrupt = false;
bool colorInterrupt = false;
long patternTime = 0;
long colorTime = 0;

bool updateLedsIn = false;
bool updateLedsOut = false;
bool bootup = true;

/*
Assume NUMBEROFLEDS is 12, so using a 12 pixel led ring (or strip)
ledStateArr[] stores the state (color/blinking) of each Led Pixel. Leds start at 1 and count up.
ledStateArr[1] contains the state of Led 1
ledStateArr[12] contains the state of Led 12 
ledStateArr[0] contains 'garbage'. If you would send a MQTT topic like: some/thing/13 which is a not existing led
                                   then that will be captured and stored in ledStateArr[0] (only if content is valid)
                                   The same applies if you would have some/thing/wrong. That would also go to [0] (only if content is valid)
So this is a bit different than usual where Array position 0 is the first position. 
From MQTT I want to drive Led 1 to 12. Not led 0 to 11.
*/
int ledStateArr[NUMBEROFLEDS+1]; //Store state of each led (where Led 1 = ledStateArr[1] and not ledStateArr[0])


//***************************** SETUP ***************************************************
void setup() {

  Serial.begin(115200);
  Serial.println();
  Serial.println("Starting up...");


  iotWebConf.setStatusPin(STATUS_PIN);
  iotWebConf.setConfigPin(CONFIG_PIN);
  iotWebConf.addSystemParameter(&mqttServerParam);
  iotWebConf.addSystemParameter(&mqttUserNameParam);
  iotWebConf.addSystemParameter(&mqttUserPasswordParam);
  iotWebConf.addSystemParameter(&mqttTopicSendParam);
  iotWebConf.addSystemParameter(&mqttTopicReceiveParam);
  iotWebConf.addSystemParameter(&ledOffsetParam);
  iotWebConf.addSystemParameter(&ledBrightnessParam);
 // iotWebConf.addSystemParameter(&singleStatusParam);
  iotWebConf.setConfigSavedCallback(&configSaved);
  iotWebConf.setFormValidator(&formValidator);
  iotWebConf.getApTimeoutParameter()->visible = false; //set to true if you want to specify the timeout in portal

  iotWebConf.setWifiConnectionCallback(&wifiConnected);
  iotWebConf.setupUpdateServer(
    [](const char* updatePath) { httpUpdater.setup(&server, updatePath); },
    [](const char* userName, char* password) { httpUpdater.updateCredentials(userName, password); });

  // -- Initializing the configuration.
  boolean validConfig = iotWebConf.init();
  if (!validConfig)
  {
    mqttServerValue[0] = '\0';
    mqttUserNameValue[0] = '\0';
    mqttUserPasswordValue[0] = '\0';
    mqttTopicSendValue[0] ='\0';
    mqttTopicReceiveValue[0] ='\0';
    ledOffsetValue[0] = '\0';
    ledBrightnessValue[0] = '\0';
  }
  
  //Setup Ledstrip
  strip.begin();
  strip.setBrightness(atoi(ledBrightnessValue));
  strip.show(); // Initialize all pixels to 'off'
  
  strip.setPixelColor(0,strip.Color(255 ,0, 0)); //Set the first led of the LedRing to Red; 
  for(int x=1; x<NUMBEROFLEDS;x++){
      strip.setPixelColor(x,strip.Color(0 ,0, 200)); //Set the remaining led to blue; 
  }
  strip.show(); 

  // -- Set up required URL handlers on the web server.
  server.on("/", handleRoot);
  server.on("/config", []{ iotWebConf.handleConfig(); });
  server.onNotFound([](){ iotWebConf.handleNotFound(); });

  Serial.println("local ip");
  Serial.println(WiFi.localIP());

  //Set MQTT Server and port 
  client.setServer(mqttServerValue, 1883);
  client.setCallback(mqttCallback);

  showLedOffset(); //Display real Led 1 and the Led 1 after offset
  delay(5000); // so you have time to check if the green led is at the right spot.

//Select Buttons for Interrupt (select color and select pattern)
  pinMode(interruptPinColor, INPUT_PULLUP); 
  attachInterrupt(digitalPinToInterrupt(interruptPinColor), ColorISR, CHANGE); 

  pinMode(interruptPinPattern, INPUT_PULLUP); 
  attachInterrupt(digitalPinToInterrupt(interruptPinPattern), PatternISR, CHANGE);

  //add random string to mqttClientId to make it Unique
   //mqttClientId += String(ESP.getChipId(), HEX); //ChipId seems to be part of Mac Address 

//Create UNIQUE MQTT ClientId - When not unique on the same MQTT server, you'll get strange behaviour
//It uses the unique chipID of the ESP.
sprintf(mqttClientId, "%s%u", thingName,ESP.getChipId()); 
Serial.print("mqttclientid: ");
Serial.println(mqttClientId);
}
//************************ END OF SETUP ********************************************


/*
MQTT Callback function
Determine Topic number and store the payload in ledStateArr (Array)
*/
void mqttCallback(char* topic, byte* payload, unsigned int length) {

  int LedId = 0;

  //you should subscribe to topics like topic/# or topic/subtopic/#
  //This will result in topics like: topic/subtopic/0, topic/subtopic/1 where the number corresponds with the LED
  //ledStateArr[LedId] will contain the led status (what color you want) per led.
  
  char *token = strtok(topic, "/"); //split on /
    // Keep printing tokens while one of the 
    // delimiters present in str[]. 
    while (token != NULL) 
    { 
        LedId = atoi(token); 
        token = strtok(NULL, "/"); //break the while. LedId contains the last token
    } 
  //check if LedId is within the range of available leds (for incoming messages we use half of the pixels. 
  //half of the pixels + 1 is the LedId that is used to show the previously send item of the device itself
  //this is used to restore the display in case of a reboot.

    if(LedId > (NUMBEROFLEDS/2)+1 ) 
    LedId = 0;             //Send the value to index 0 which is not used (ledStateArr[0] is not used)

  //Serial.print("Token: ");
  //Serial.println(LedId);

  payload[length] = '\0';
  
  //Print payload to Serial for debugging
  //for (unsigned int i=0;i<length;i++) { 
  //  Serial.print((char)payload[i]);
  //}


  /*
  Define color codes and with or without blink:
  green (1)
  red (2)
  yellow (3)
  purple (4)
  blue (5)
  white (6)
  off (0)
  
  */
  //check for possible topics
  if(strcmp((char*)payload,"green") == 0){ 
      ledStateArr[LedId] = 1;
    }
  else if(strcmp((char*)payload,"red") == 0){
          ledStateArr[LedId] = 2;
    }
  else if(strcmp((char*)payload,"yellow") == 0){
          ledStateArr[LedId] = 3; 
  }
  else if(strcmp((char*)payload,"purple") == 0){
        ledStateArr[LedId] = 4;
    }
  else if(strcmp((char*)payload,"blue") == 0){
          ledStateArr[LedId] = 5;
          }
  else if(strcmp((char*)payload,"white") == 0){
          ledStateArr[LedId] = 6; 
  }
    else if(strcmp((char*)payload,"off") == 0){
          ledStateArr[LedId] = 0; 
  }

  if(LedId == 7 && bootup == true){
    updateLedsOut = true;
    bootup = false;
}
  else
    updateLedsIn = true;
}
//**************** END OF MQTT CALLBACK FUNCTION *********************************


void reconnect() {
  // Loop until we're reconnected
  while ((iotWebConf.getState() == IOTWEBCONF_STATE_ONLINE) && !client.connected()) { 
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    // If you do not want to use a username and password, change next line to
    // if (client.connect("ESP8266Client")) {
    if (client.connect(mqttClientId, mqttUserNameValue, mqttUserPasswordValue)) { //mqtt_user, mqtt_pass
      Serial.println("connected");
      Serial.println(mqttTopicReceiveValue);
      
      client.subscribe(mqttTopicReceiveValue); //subscribe to topic
    } 
    else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }

    
  }
}

//******************** START OF LOOP () *****************************************************

void loop() {

  iotWebConf.doLoop();
  client.loop(); //make sure MQTT Keeps running (hopefully prevents watchdog from kicking in)
  delay(10);
 
  //Handle Interrupt button press of pattern button 
  if((patternInterrupt == true) && (millis() > (patternTime+200U))){ 
    Serial.println("pattern interrupt");
    patternInterrupt = false;
    updateLedsOut = true;
    }

  //Handle Interrupt button press of color button
  if((colorInterrupt == true) && (millis() > (colorTime+200U))){
  
    int LedId = (NUMBEROFLEDS/2)+1; //in case of 12 leds, divide by 2 = 6. Add 1 --> LedId = 7. So in the ledStateArr on position 7 we will have the color stored.
    
    if(ledStateArr[LedId] < 6) //max 6 colors + off combinations
      ledStateArr[LedId] = ledStateArr[LedId]+1;
    else  
      ledStateArr[LedId] = 0;

    Serial.print("LedStateArr: ");
    Serial.println(ledStateArr[LedId]);
    colorInterrupt = false;
    updateLedsOut = true;
  }
 /*
  DRIVE THE LEDS
  green (1)
  red (2)
  yellow (3)
  purple (4)
  blue (5)
  white (6)
  off (0)
  */
 
 

if(updateLedsIn == true){ //true means we want to only show one status in total on all leds where all leds are 50% of them.
  int x = 1; 
  if(ledStateArr[x] == 1) //GREEN
      colorWipeIn(strip.Color(0, 255, 0), 100); // Green

  //Check for 2nd topic
  else if(ledStateArr[x] == 2) //RED
    colorWipeIn(strip.Color(255, 0, 0), 100); // Red 
    
  else if(ledStateArr[x] == 3) //YELLOW
    colorWipeIn(strip.Color(128, 128, 0), 100); // Red
  
  //Check for 4nd topic
  else if(ledStateArr[x] == 4) //PURPLE
    colorWipeIn(strip.Color(128, 0, 128), 100); // Purple

  //BLUE SINGLE STATUS
  else if(ledStateArr[x] == 5) //BLUE
    colorWipeIn(strip.Color(0, 0, 255), 100); // Blue

  //WHITE SINGLE STATUS  
  else if(ledStateArr[x] == 6) //WHITE
     colorWipeIn(strip.Color(200, 200, 200), 100); 
  
  //OFF
  else if(ledStateArr[x] == 0) //LED OFF
      colorWipeIn(strip.Color(0,0,0),0);

  updateLedsIn = false;
}

if(updateLedsOut == true){
   int x = (NUMBEROFLEDS/2)+1; //in case of 12 pixels, number 7 will contain the status for sending the data.

  //GREEN SINGLE STATUS
  if(ledStateArr[x] == 1){ //GREEN
      colorWipeOut(strip.Color(0, 255, 0), 100); // Green
      client.publish(mqttTopicSendValue, "green"); //publish 'color' message to topic.
  }

 //RED SINGLE STATUS
  else if(ledStateArr[x] == 2){ //RED
    colorWipeOut(strip.Color(255, 0, 0), 100); // Red
    client.publish(mqttTopicSendValue, "red"); //publish 'color' message to topic.
}
  //YELLOW SINGLE STATUS
  else if(ledStateArr[x] == 3){ //YELLOW
    colorWipeOut(strip.Color(128, 128, 0), 100); // yellow
    client.publish(mqttTopicSendValue, "yellow"); //publish 'color' message to topic.
  }

  //PURPLE SINGLE STATUS
  else if(ledStateArr[x] == 4){ //PURPLE
    colorWipeOut(strip.Color(128, 0, 128), 100); // Purple
    client.publish(mqttTopicSendValue, "purple"); //publish 'color' message to topic.
  }
  //BLUE SINGLE STATUS
  else if(ledStateArr[x] == 5){ //BLUE
    colorWipeOut(strip.Color(0, 0, 255), 100); // Blue
    client.publish(mqttTopicSendValue, "blue"); //publish 'color' message to topic.
  }

  //WHITE SINGLE STATUS  
  else if(ledStateArr[x] == 6){ //WHITE
          colorWipeOut(strip.Color(200, 200, 200), 100); 
          client.publish(mqttTopicSendValue, "white"); //publish 'color' message to topic.
  }
  //OFF
   else if(ledStateArr[x] == 0){ //LED OFF
        colorWipeOut(strip.Color(0, 0, 0), 0); 
        client.publish(mqttTopicSendValue, "off"); //publish 'color' message to topic.
    }
  updateLedsOut = false;

}

//Block updating the LEDs while in Configuration portal (inConfig)

if(inConfig == 0) 
  strip.show(); //set all pixels  
 

   
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  delay(10);

  if (needReset)
  {
    Serial.println("Rebooting after 1 second.");
    iotWebConf.delay(1000);
    ESP.restart();
  }
/* 
// Publish 'ONLINE' Message to Topic. Uncomment if you want to use this.
  long now = millis();
  if (now - lastMsg > 10000) {
    lastMsg = now;
  client.publish("build/test", "ONLINE"); //publish 'ONLINE' message to topic.
  }
  */
}
//******************** END OF LOOP () *****************************************


/**
 * Handle web requests to "/" path.
 */
void handleRoot()
{
  // -- Let IotWebConf test and handle captive portal requests.
  if (iotWebConf.handleCaptivePortal())
  {
    // -- Captive portal request were already served.
    return;
  }
  inConfig = 1; //You are in the Config Portal
  showLedOffset(); //Show real LED1 and your Led 1 at offset

  String s = F("<!DOCTYPE html><html lang=\"en\"><head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1, user-scalable=no\"/>");
  s += iotWebConf.getHtmlFormatProvider()->getStyle();
  s += "<title>MQTT NeoPixel Kids Light</title></head><body>";
  s += "<H1>";
  s += iotWebConf.getThingName();
  s+= "</H1>";
  s += "<div>MQTT ClientId: ";
  s += mqttClientId;
  s += "</div>";
  s += "<div>MAC address: ";
  s += WiFi.macAddress();
  s += "</div>";
  s += "<div>MQTT Server: ";
  s += mqttServerValue;
  s += "</div>";
  s += "<div>MQTT Send Topic: ";
  s += mqttTopicSendValue;
  s += "</div>";
  s += "<div>MQTT Receive Topic: ";
  s += mqttTopicReceiveValue;
  s += "</div>";
  s += "<div>LED Offset: ";
  s += ledOffsetValue;
  s += "</div>";
  s += "<div>LED Brightness: ";
  s += ledBrightnessValue;
  s += "</div>";
  s += "<button type='button' onclick=\"location.href='';\" >Refresh</button>";
  s += "<div>Go to <a href='config'>configure page</a> to change values.</div>";
  s +="<div><small>MQTT NeoPixel Kids - Version: ";
  s += VERSIONNUMBER;
  s += " - Get latest version on <a href='https://github.com/arvdsar/MQTT_NeoPixel_Kids' target='_blank'>Github</a>.";
  s += "</small></div>";

  s += "</body></html>\n";
  server.send(200, "text/html", s);
}

void wifiConnected()
{
  //needMqttConnect = true; //not using this.
}

void configSaved()
{
  Serial.println("Configuration was updated.");
  showLedOffset(); //Show real LED1 and your Led 1 at offset so you can check the offset
  delay(5000);
  inConfig = 0; // Enable Led Pattern again
  needReset = true; 
}

bool formValidator(iotwebconf::WebRequestWrapper* webRequestWrapper)
{
  Serial.println("Validating form.");
  boolean valid = true;

  int l = server.arg(mqttServerParam.getId()).length();
  if (l < 3)
  {
    mqttServerParam.errorMessage = "Please provide at least 3 characters!";
    valid = false;
  }

  return valid;
}


/*
 Handle led_offset
 Set all leds to blue, next make the original first led Red and then set the 
 led with offset to green. The Green Led should be where YOU want to see LED 1.
*/
void showLedOffset(){

  for(pixel =0;pixel < NUMBEROFLEDS;pixel++)
      strip.setPixelColor(pixel,strip.Color(0 ,0, 255)); //Set all leds to Blue
  strip.setPixelColor(0,strip.Color(255 ,0, 0)); //Set the offical first led to Red.

  pixel = 0 + atoi(ledOffsetValue);
  if(pixel > (NUMBEROFLEDS-1)){
    pixel = pixel - NUMBEROFLEDS;
  }  
  strip.setPixelColor(pixel,strip.Color(0 ,255, 0)); //Set the first led with offset to Green. Ready to go.
  strip.show(); 

}


// update pixels (updated color wipe) for incoming messages
void colorWipeIn(uint32_t c, uint8_t wait) {

  for(uint16_t i=1; i<=NUMBEROFLEDS/2; i++) { 
         //Handle led_offset
        pixel = (i-1) + atoi(ledOffsetValue);
        if(pixel > (NUMBEROFLEDS-1)){
            pixel = pixel - NUMBEROFLEDS;
        }
    strip.setPixelColor(pixel, c);
  }
  strip.show();
}

// update pixels (updated color wipe) for outcoming messages
void colorWipeOut(uint32_t c, uint8_t wait) {

  for(uint16_t i=(NUMBEROFLEDS/2)+1; i<=NUMBEROFLEDS; i++) { 
         //Handle led_offset
        pixel = (i-1) + atoi(ledOffsetValue);
        if(pixel > (NUMBEROFLEDS-1)){
            pixel = pixel - NUMBEROFLEDS;
        }
    strip.setPixelColor(pixel, c);
  }
  strip.show();
}

void ICACHE_RAM_ATTR ColorISR(){
//What to do when select button is pushed?
 // Serial.println("ColorISR");
  colorInterrupt = true;
  colorTime = millis();
}

void ICACHE_RAM_ATTR PatternISR(){
//To commit the selected state to the other device
 //Serial.println("PatternISR");
 patternInterrupt = true;
 patternTime = millis();
}