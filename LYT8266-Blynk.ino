/*
    1MB flash size

    gpio 15 - power
    gpio 2 - white intensity - 0 - 255
    gpio 13 - red intensity - 0 - 255
    gpio 12 - green intensity - 0 - 255
    gpio 14 - blue intensity - 0 - 255

*/

#define HOSTNAME "sauron"

//comment out to completly disable respective technology
#define INCLUDE_BLYNK_SUPPORT
//#define INCLUDE_MQTT_SUPPORT

int presets [20] [4] = {
  {  0,  80,  30,   0}, // night light
  {  80,   0,   0,  0}, // feeding
  {  0,   0,   0,   0}, // 3
  {  0,   0,   0,   0}, // 4
  {  0,   0,   0,   0}, // 5
  {  0,   0,   0,   0}, // 6
  { 10,   0,   0,   0}, // LOW
  { 60,   0,   0,   0}, // MED
  {255,   0,   0,   0}, // HIGH
  {  0,   0,   0,   0}
};

/********************************************
   Should not need to edit below this line *
 * *****************************************/
#define POWER 15
#define WHITE 2
#define RED 13
#define GREEN 12
#define BLUE 14

#include <ESP8266WiFi.h>

#ifdef INCLUDE_BLYNK_SUPPORT
#define BLYNK_PRINT Serial    // Comment this out to disable prints and save space
#include <BlynkSimpleEsp8266.h>

static bool BLYNK_ENABLED = true;
#endif

#ifdef INCLUDE_MQTT_SUPPORT
#include <PubSubClient.h>        //https://github.com/Imroy/pubsubclient

WiFiClient wclient;
PubSubClient mqttClient(wclient);

static bool MQTT_ENABLED              = true;
int         lastMQTTConnectionAttempt = 0;
#endif

#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager

#include <EEPROM.h>

#define EEPROM_SALT 12661
typedef struct {
  char  bootState[4]      = "on";
  char  blynkToken[33]    = "blynk-token";
  char  blynkServer[33]   = "blynk-cloud.com";
  char  blynkPort[6]      = "80";
  char  mqttHostname[33]  = "iot.eclipse.org";
  char  mqttPort[6]       = "1883";
  char  mqttClientID[24]  = "smqclient";
  char  mqttTopic[33]     = HOSTNAME;
  int   salt              = EEPROM_SALT;
} WMSettings;

WMSettings settings;

typedef struct {
  int  white  = 10;
  int  red    = 0;
  int  green  = 0;
  int  blue   = 0;
  int  salt   = EEPROM_SALT;
} LightSettings;

LightSettings light;

unsigned int lightSettingsLastUpdate = 0;

#include <ArduinoOTA.h>

//http://stackoverflow.com/questions/9072320/split-string-into-string-array
String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length()-1;

  for(int i=0; i<=maxIndex && found<=index; i++){
    if(data.charAt(i)==separator || i==maxIndex){
        found++;
        strIndex[0] = strIndex[1]+1;
        strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }

  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}

//gets called when WiFiManager enters configuration mode
void configModeCallback (WiFiManager *myWiFiManager) {
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());
  //if you used auto generated SSID, print it
  Serial.println(myWiFiManager->getConfigPortalSSID());
}

void updateBlynk(int channel) {
#ifdef INCLUDE_BLYNK_SUPPORT
/*  int state = digitalRead(SONOFF_RELAY_PINS[channel]);
  Blynk.virtualWrite(channel * 5 + 4, state * 255);*/
#endif
}

void updateMQTT(int channel) {
#ifdef INCLUDE_MQTT_SUPPORT
  /*int state = digitalRead(SONOFF_RELAY_PINS[channel]);
  char topic[50];
  sprintf(topic, "%s/channel-%d/status", settings.mqttTopic, channel);
  String stateString = state == 0 ? "off" : "on";
  if ( channel >= SONOFF_AVAILABLE_CHANNELS) {
    stateString = "disabled";
  }
  mqttClient.publish(topic, stateString);*/
#endif
}

//flag for saving data
bool shouldSaveConfig = false;

//callback notifying us of the need to save config
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

void restart() {
  //TODO turn off relays before restarting
  saveLastLightSettings();
  ESP.reset();
  delay(1000);
}

void reset() {
  //reset settings to defaults
  //TODO turn off relays before restarting
  /*
    WMSettings defaults;
    settings = defaults;
    EEPROM.begin(1024);
    EEPROM.put(0, settings);
    EEPROM.end();
  */
  //reset wifi credentials
  WiFi.disconnect();
  delay(1000);
  ESP.reset();
  delay(1000);
}

void saveLastLightSettings() {
  EEPROM.begin(1024);
  EEPROM.put(768, light);
  EEPROM.end();
}

void setWRGB(int white, int red, int green, int blue) {
  if (digitalRead(POWER) == LOW) {
    digitalWrite(POWER, HIGH);    
  }
  analogWrite(WHITE, white);    
  analogWrite(RED,   red);    
  analogWrite(GREEN, green);    
  analogWrite(BLUE,  blue);   

  light.white = white;
  light.red = red;
  light.green = green;
  light.blue = blue;

  if(lightSettingsLastUpdate == 0) {
    lightSettingsLastUpdate = millis();
  }
}

#ifdef INCLUDE_BLYNK_SUPPORT
/**********
 ***********/

BLYNK_WRITE_DEFAULT() {
  int pin = request.pin;
  if (pin >= 10 && pin <= 19) {
    pin -= 10;
    int a = param.asInt();
    if (a != 0) {
      setWRGB( presets[pin][0],  presets[pin][1],  presets[pin][2],  presets[pin][3]);       
    }
  }
}

// dimmable white
BLYNK_WRITE(20) {
  int a = param.asInt();
  setWRGB( a,  0,  0,  0); 
}

BLYNK_WRITE(21) {
  int a = param.asInt();
  setWRGB( a,  0,  0,  0); 
}

// zeRGBa
BLYNK_WRITE(22) {
  int red = param[0].asInt(); 
  int green = param[1].asInt(); 
  int blue = param[2].asInt(); 
  setWRGB( 0,  red,  green,  blue); 
}

//restart - button
BLYNK_WRITE(40) {
  digitalWrite(POWER, LOW);
  int a = param.asInt();
  if (a != 0) {
    restart();
  }
}

//reset - button
BLYNK_WRITE(41) {
  int a = param.asInt();
  if (a != 0) {
    reset();
  }
}

#endif

#ifdef INCLUDE_MQTT_SUPPORT
void mqttCallback(const MQTT::Publish& pub) {
  Serial.print(pub.topic());
  Serial.print(" => ");
  if (pub.has_stream()) {
    int BUFFER_SIZE = 100;
    uint8_t buf[BUFFER_SIZE];
    int read;
    while (read = pub.payload_stream()->read(buf, BUFFER_SIZE)) {
      Serial.write(buf, read);
    }
    pub.payload_stream()->stop();
    Serial.println("had buffer");
  } else {
    Serial.println(pub.payload_string());
    String topic = pub.topic();
    String payload = pub.payload_string();
    
    if (topic == settings.mqttTopic) {
      Serial.println("exact match");
      return;
    }
    
    if (topic.startsWith(settings.mqttTopic)) {
      Serial.println("for this device");
      topic = topic.substring(strlen(settings.mqttTopic) + 1);
      String channelString = getValue(topic, '/', 0);
      if(!channelString.startsWith("channel-")) {
        Serial.println("no channel");
        return;
      }
      channelString.replace("channel-", "");
      int channel = channelString.toInt();
      Serial.println(channel);
      if (payload == "on") {
        turnOn(channel);
      }
      if (payload == "off") {
        turnOff(channel);
      }
      if (payload == "toggle") {
        //toggle(channel);
      }
      if(payload == "") {
        updateMQTT(channel);
      }
      
    }
  }
}
    
#endif

void setup()
{
  Serial.begin(115200);

  //set led pin as output
  pinMode(POWER, OUTPUT);
  pinMode(WHITE, OUTPUT);
  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(BLUE, OUTPUT);

  EEPROM.begin(1024);
  EEPROM.get(768, light);
  EEPROM.end();
  if (light.salt != EEPROM_SALT) {
    Serial.println("Invalid settings in EEPROM, trying with defaults");
    LightSettings defaults;
    light = defaults;
  }
  digitalWrite(POWER, HIGH);    
  analogWrite(WHITE, light.white);    
  analogWrite(RED,   light.red);    
  analogWrite(GREEN, light.green);    
  analogWrite(BLUE,  light.blue);    

  

  const char *hostname = HOSTNAME;

  WiFiManager wifiManager;
  //set callback that gets called when connecting to previous WiFi fails, and enters Access Point mode
  wifiManager.setAPCallback(configModeCallback);

  //timeout - this will quit WiFiManager if it's not configured in 3 minutes, causing a restart
  wifiManager.setConfigPortalTimeout(180);

  //custom params
  EEPROM.begin(1024);
  EEPROM.get(0, settings);
  EEPROM.end();

  if (settings.salt != EEPROM_SALT) {
    Serial.println("Invalid settings in EEPROM, trying with defaults");
    WMSettings defaults;
    settings = defaults;
  }


  WiFiManagerParameter custom_boot_state("boot-state", "on/off on boot", settings.bootState, 33);
  wifiManager.addParameter(&custom_boot_state);


  Serial.println(settings.bootState);

#ifdef INCLUDE_BLYNK_SUPPORT
  Serial.println(settings.blynkToken);
  Serial.println(settings.blynkServer);
  Serial.println(settings.blynkPort);

  WiFiManagerParameter custom_blynk_text("<br/>Blynk config. <br/> No token to disable.<br/>");
  wifiManager.addParameter(&custom_blynk_text);

  WiFiManagerParameter custom_blynk_token("blynk-token", "blynk token", settings.blynkToken, 33);
  wifiManager.addParameter(&custom_blynk_token);

  WiFiManagerParameter custom_blynk_server("blynk-server", "blynk server", settings.blynkServer, 33);
  wifiManager.addParameter(&custom_blynk_server);

  WiFiManagerParameter custom_blynk_port("blynk-port", "port", settings.blynkPort, 6);
  wifiManager.addParameter(&custom_blynk_port);
#endif


#ifdef INCLUDE_MQTT_SUPPORT
  Serial.println(settings.mqttHostname);
  Serial.println(settings.mqttPort);
  Serial.println(settings.mqttClientID);
  Serial.println(settings.mqttTopic);
  
  WiFiManagerParameter custom_mqtt_text("<br/>MQTT config. <br/> No url to disable.<br/>");
  wifiManager.addParameter(&custom_mqtt_text);

  WiFiManagerParameter custom_mqtt_hostname("mqtt-hostname", "Hostname", settings.mqttHostname, 33);
  wifiManager.addParameter(&custom_mqtt_hostname);

  WiFiManagerParameter custom_mqtt_port("mqtt-port", "port", settings.mqttPort, 6);
  wifiManager.addParameter(&custom_mqtt_port);

  WiFiManagerParameter custom_mqtt_client_id("mqtt-client-id", "Client ID", settings.mqttClientID, 24);
  wifiManager.addParameter(&custom_mqtt_client_id);

  WiFiManagerParameter custom_mqtt_topic("mqtt-topic", "Topic", settings.mqttTopic, 33);
  wifiManager.addParameter(&custom_mqtt_topic);
#endif

  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  if (!wifiManager.autoConnect(hostname)) {
    Serial.println("failed to connect and hit timeout");
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(1000);
  }

  //Serial.println(custom_blynk_token.getValue());
  //save the custom parameters to FS
  if (shouldSaveConfig) {
    Serial.println("Saving config");

    strcpy(settings.bootState, custom_boot_state.getValue());

#ifdef INCLUDE_BLYNK_SUPPORT
    strcpy(settings.blynkToken, custom_blynk_token.getValue());
    strcpy(settings.blynkServer, custom_blynk_server.getValue());
    strcpy(settings.blynkPort, custom_blynk_port.getValue());
#endif

#ifdef INCLUDE_MQTT_SUPPORT
    strcpy(settings.mqttHostname, custom_mqtt_hostname.getValue());
    strcpy(settings.mqttPort, custom_mqtt_port.getValue());
    strcpy(settings.mqttClientID, custom_mqtt_client_id.getValue());
    strcpy(settings.mqttTopic, custom_mqtt_topic.getValue());
#endif

    Serial.println(settings.bootState);
    Serial.println(settings.blynkToken);
    Serial.println(settings.blynkServer);
    Serial.println(settings.blynkPort);

    EEPROM.begin(1024);
    EEPROM.put(0, settings);
    EEPROM.end();
  }

#ifdef INCLUDE_BLYNK_SUPPORT
  //config blynk
  if (strlen(settings.blynkToken) == 0) {
    BLYNK_ENABLED = false;
  }
  if (BLYNK_ENABLED) {
    Blynk.config(settings.blynkToken, settings.blynkServer, atoi(settings.blynkPort));
  }
#endif


#ifdef INCLUDE_MQTT_SUPPORT
  //config mqtt
  if (strlen(settings.mqttHostname) == 0) {
    MQTT_ENABLED = false;
  }
  if (MQTT_ENABLED) {
    mqttClient.set_server(settings.mqttHostname, atoi(settings.mqttPort));
  }
#endif

  //OTA
  ArduinoOTA.onStart([]() {
    Serial.println("Start OTA");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.setHostname(hostname);
  ArduinoOTA.begin();

  //if you get here you have connected to the WiFi
  Serial.println("connected...yeey :)");

  Serial.println("done setup");
}


void loop()
{

  //ota loop
  ArduinoOTA.handle();

#ifdef INCLUDE_BLYNK_SUPPORT
  //blynk connect and run loop
  if (BLYNK_ENABLED) {
    Blynk.run();
  }
#endif


#ifdef INCLUDE_MQTT_SUPPORT
  //mqtt loop
  if (MQTT_ENABLED) {
    if (!mqttClient.connected()) {
      if(lastMQTTConnectionAttempt == 0 || millis() > lastMQTTConnectionAttempt + 3 * 60 * 1000) {
        lastMQTTConnectionAttempt = millis();
        Serial.println(millis());
        Serial.println("Trying to connect to mqtt");
        if (mqttClient.connect(settings.mqttClientID)) {
          mqttClient.set_callback(mqttCallback);
          char topic[50];
          //sprintf(topic, "%s/+/+", settings.mqttTopic);
          //mqttClient.subscribe(topic);
          sprintf(topic, "%s/+", settings.mqttTopic);
          mqttClient.subscribe(topic);

          //TODO multiple relays
          updateMQTT(0);
        } else {
          Serial.println("failed");
        }
      }
    } else {
      mqttClient.loop();
    }
  }
#endif
  // cooloff so we don t write to eeprom causing unnecessary wear
  if (lightSettingsLastUpdate != 0 && lightSettingsLastUpdate + 30 * 1000 < millis() ) {
    saveLastLightSettings();
    lightSettingsLastUpdate = 0;
  }
}




