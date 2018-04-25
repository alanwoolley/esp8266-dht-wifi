#include <FS.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <ESP8266WiFi.h>          //ESP8266 Core WiFi Library
#include <DNSServer.h>            //Local DNS Server used for redirecting all requests to the configuration portal
#include <ESP8266WebServer.h>     //Local WebServer used to serve the configuration portal
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager WiFi Configuration Magic
#include <ArduinoJson.h>
#include <DoubleResetDetector.h>

String IDENT = String(ESP.getChipId());

#define SERIAL_BAUD 115200
#define WIFI_PASSWORD "12345678"

/* Double reset detection (DRD)
   Allows the resetting of the WiFi configuration by resetting the board twice in quick succession
*/
#define DRD_TIMEOUT 10 // Number of seconds after reset during which a
                       // subseqent reset will be considered a double reset
#define DRD_ADDRESS 0 // RTC Memory Address for the DoubleResetDetector to use

#define DHT_PIN 0 // PIN connected to DHT

#define SENSOR_DELAY 30000 // Sensor read interval

#define CONFIG_FILE_NAME "/config.json"

DHT dht(DHT_PIN, DHT22);
DoubleResetDetector drd(DRD_TIMEOUT, DRD_ADDRESS);

char mqtt_server[40];
char mqtt_port[6] = "1883";

WiFiClient espClient;
PubSubClient mqttClient(espClient);

//flag for saving data
bool shouldSaveConfig = false;

//callback notifying us of the need to save config
void saveConfigCallback () {
  shouldSaveConfig = true;
}

void configModeCallback (WiFiManager *myWiFiManager) {
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());
  Serial.println(myWiFiManager->getConfigPortalSSID());
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  pinMode(DHT_PIN,INPUT);

  String wifiName = "SENSOR_" + IDENT;

  WiFiManager wifiManager;

  // Erase all the settings if a double reset has been detected
  if (drd.detectDoubleReset()) {
    SPIFFS.format();
    wifiManager.resetSettings();
  }

  // Read configuration from the FS

  if (SPIFFS.begin()) {
    if (SPIFFS.exists(CONFIG_FILE_NAME)) {
      File configFile = SPIFFS.open(CONFIG_FILE_NAME, "r");
      if (configFile) {
        size_t size = configFile.size();

        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);
        if (json.success()) {
          strcpy(mqtt_server, json["mqtt_server"]);
          strcpy(mqtt_port, json["mqtt_port"]);
        } else {
          Serial.println("failed to load JSON config");
        }
      }
    }
  } else {
    Serial.println("Failed to mount FS");
  }

  // The extra parameters to be configured (can be either global or just in the setup)
  // After connecting, parameter.getValue() will get you the configured value
  // id/name placeholder/prompt default length
  WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqtt_server, 40);
  WiFiManagerParameter custom_mqtt_port("port", "mqtt port", mqtt_port, 6);
  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);

  wifiManager.setSaveConfigCallback(saveConfigCallback);
  wifiManager.setAPCallback(configModeCallback);

  wifiManager.setConfigPortalTimeout(180);

  if (!wifiManager.autoConnect(wifiName.c_str(), WIFI_PASSWORD)) {
    // if (!wifiManager.startConfigPortal(WIFI_NAME, WIFI_PASSWORD)) {
    Serial.println("failed to connect and hit timeout");
    delay(3000);

    ESP.reset();
    delay(5000);
  }

  Serial.println("Connected to WiFi");

  //read updated parameters
  strcpy(mqtt_server, custom_mqtt_server.getValue());
  strcpy(mqtt_port, custom_mqtt_port.getValue());

  //save the custom parameters to FS
  if (shouldSaveConfig) {

    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    json["mqtt_server"] = mqtt_server;
    json["mqtt_port"] = mqtt_port;

    File configFile = SPIFFS.open(CONFIG_FILE_NAME, "w");
    if (configFile) {
      json.printTo(Serial);
      json.printTo(configFile);
      configFile.close();
    } else {
      Serial.println("failed to open config file for writing");
    }
  }

  Serial.print("Local IP: ");
  Serial.println(WiFi.localIP());

  dht.begin();
  mqttClient.setServer(mqtt_server, atoi(mqtt_port));

  Serial.println("Running");
}

void getTopic(char* type, char* out) {
    (String("sensors/") + IDENT + String("/") + String(type)).toCharArray(out, 255);
}

void loop() {
  drd.loop();

  if (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (mqttClient.connect(IDENT.c_str())) {
      Serial.println("MQTT connected");
    } else {
      Serial.println("Unable to establish MQTT connection");
      Serial.print("failed, rc=");
      Serial.println(mqttClient.state());
    }
  }

  if (mqttClient.connected()) {
    char topic[255];
    float temperature = dht.readTemperature(false, false);
    if (isnan(temperature)) {
      Serial.println("Error reading temperature!");
    } else {
      Serial.print("Temperature: ");
      Serial.print(temperature);
      Serial.println(" *C");
      getTopic("temperature", topic);
      mqttClient.publish(topic, String(temperature,2).c_str());
    }
    // Get humidity event and print its value.
    float humidity = dht.readHumidity(false);

    if (isnan(humidity)) {
      Serial.println("Error reading humidity!");
    } else {
      Serial.print("Humidity: ");
      Serial.print(humidity);
      Serial.println("%");
      getTopic("humidity", topic);
      mqttClient.publish(topic, String(humidity,2).c_str());
    }

    if (!isnan(humidity) && !isnan(temperature) ) {
      float hi = dht.computeHeatIndex(temperature,humidity,false);
      if (!isnan(hi)) {
        Serial.print("Heat Index: ");
        Serial.print(hi);
        Serial.println(" *C");
        getTopic("heatindex", topic);
        mqttClient.publish(topic, String(hi,2).c_str());
      }
    }
  }

  for (int i=0; i<SENSOR_DELAY; i=i+1000) {
    mqttClient.loop();
    delay(1000);
  }
}
