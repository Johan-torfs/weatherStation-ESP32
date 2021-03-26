#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <BH1750.h>

/* Temperature and pressure sensors */
Adafruit_BMP280 bmp; // use I2C interface
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();

/* Light sensor */
BH1750 bh;

/* Last time sensors were read */
long lastSensorRead = 0;
int sensorReadInterval = 1800000; // Every 30 minutes

/* Variables to store last read values */
float temperature = 0.0;
float pressure = 0.0;
float lightlvl = 0.0;

#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <PubSubClient.h>
#include <SPIFFS.h>

/* Web server */
WebServer server(80);

/* Set these to your desired softAP credentials. They are not configurable at runtime */
const char *softAP_ssid = "ESP32-Weather-Station";
const char *softAP_password = "12345678";

/* hostname for mDNS. Should work at least on windows. Try http://esp32.local */
const char *myHostname = "esp32";

/* Don't set this wifi credentials. They are configurated at runtime and stored on EEPROM */
char ssid[32] = "";
char password[32] = "";
char home[32] = "WiFi-2.4-f0b2";

/* DNS server */
const byte DNS_PORT = 53;
DNSServer dnsServer;

/* Storage for SSID and password */
Preferences preferences;

/* Soft AP network parameters */
IPAddress apIP(192, 168, 4, 1);
IPAddress netMsk(255, 255, 255, 0);


/* Should I connect to WLAN asap? */
boolean connect;

/* Last time I tried to connect to WLAN */
long lastConnectTry = 0;

/* Last time I tried to connect to MQTT */
long lastMQTTConnectTry = 0;

/* Current WLAN status */
int status = WL_IDLE_STATUS;

/* Has mDNS been started arleady? */
bool mDNSStarted = false;

// Add your MQTT Broker domain/address:
const char* mqtt_server = "mqtt.johantorfs.tk";
const char* mqtt_server_local = "192.168.1.13";

WiFiClient espClient;
PubSubClient client(espClient);
char msg[50];
int value = 0;

void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();
}

/* Define file system */
#define FILESYSTEM SPIFFS
#define FORMAT_FILESYSTEM false

/* Format bytes */
String formatBytes(size_t bytes) {
  if (bytes < 1024) {
    return String(bytes) + "B";
  } else if (bytes < (1024 * 1024)) {
    return String(bytes / 1024.0) + "KB";
  } else if (bytes < (1024 * 1024 * 1024)) {
    return String(bytes / 1024.0 / 1024.0) + "MB";
  } else {
    return String(bytes / 1024.0 / 1024.0 / 1024.0) + "GB";
  }
}

/* Get type of file */
String getContentType(String filename) {
  if (server.hasArg("download")) {
    return "application/octet-stream";
  } else if (filename.endsWith(".htm")) {
    return "text/html";
  } else if (filename.endsWith(".html")) {
    return "text/html";
  } else if (filename.endsWith(".css")) {
    return "text/css";
  } else if (filename.endsWith(".js")) {
    return "application/javascript";
  } else if (filename.endsWith(".png")) {
    return "image/png";
  } else if (filename.endsWith(".gif")) {
    return "image/gif";
  } else if (filename.endsWith(".jpg")) {
    return "image/jpeg";
  } else if (filename.endsWith(".ico")) {
    return "image/x-icon";
  } else if (filename.endsWith(".xml")) {
    return "text/xml";
  } else if (filename.endsWith(".pdf")) {
    return "application/x-pdf";
  } else if (filename.endsWith(".zip")) {
    return "application/x-zip";
  } else if (filename.endsWith(".gz")) {
    return "application/x-gzip";
  }
  return "text/plain";
}

/* Does the path exist? */
bool exists(String path){
  bool yes = false;
  File file = FILESYSTEM.open(path, "r");
  if(!file.isDirectory()){
    yes = true;
  }
  file.close();
  return yes;
}

/* Is this an IP? */
boolean isIp(String str) {
  for (int i = 0; i < str.length(); i++) {
    int c = str.charAt(i);
    if (c != '.' && (c < '0' || c > '9')) {
      return false;
    }
  }
  return true;
}

/* IP to String? */
String toStringIp(IPAddress ip) {
  String res = "";
  for (int i = 0; i < 3; i++) {
    res += String((ip >> (8 * i)) & 0xFF) + ".";
  }
  res += String(((ip >> 8 * 3)) & 0xFF);
  return res;
}

/* Redirect to captive portal if we got a request for another domain. Return true in that case so the page handler do not try to handle the request again. */
boolean captivePortal() {
  if (!isIp(server.hostHeader()) && server.hostHeader() != (String(myHostname)+".local")) {
    Serial.print("Request redirected to captive portal");
    server.sendHeader("Location", String("http://") + toStringIp(server.client().localIP()), true);
    server.send ( 302, "text/plain", ""); // Empty content inhibits Content-length header so we have to close the socket ourselves.
    server.client().stop(); // Stop is needed because we sent no content length
    return true;
  }
  return false;
}

/* Read file and stream to webserver */
/* Returns true if succesfull */
bool handleFileRead(String path) {
  if (captivePortal()) { // If caprive portal redirect instead of displaying the error page.
    return false;
  }
  Serial.println("handleFileRead: " + path);
  if (path.endsWith("/")) {
    path += "index.htm";
  }
  String contentType = getContentType(path);
  String pathWithGz = path + ".gz";
  if (exists(pathWithGz) || exists(path)) {
    if (exists(pathWithGz)) {
      path += ".gz";
    }
    File file = FILESYSTEM.open(path, "r");
    server.streamFile(file, contentType);
    file.close();
    return true;
  }
  return false;
}

/* Load WLAN credentials from Preferences */
void loadCredentials() {
  preferences.getString("ssid", ssid, sizeof(ssid));
  preferences.getString("password", password, sizeof(password));
  Serial.println("Recovered credentials:");
  Serial.println(ssid);
  Serial.println(strlen(password)>0?"********":"<no password>");
}

/* Store WLAN credentials to Preference */
void saveCredentials() {
  preferences.putString("ssid", ssid);
  preferences.putString("password", password);
  Serial.println("Saved credentials:");
  Serial.println(ssid);
  Serial.println(strlen(password)>0?"********":"<no password>");
}

void connectWifi() {
  Serial.println("Connecting as wifi client...");
  WiFi.disconnect();
  WiFi.begin ( ssid, password );
  int connRes = WiFi.waitForConnectResult();
  Serial.print ( "connRes: " );
  Serial.println ( connRes );
}

/* Tries to reconnect to MQTT server */
void reconnect() {
  if (String(ssid) == String(home)) {
    client.setServer(mqtt_server_local, 1883);
  } else {
    client.setServer(mqtt_server, 1883);
  }
  Serial.print("Attempting MQTT connection...");
  // Attempt to connect
  if (client.connect("ESP32")) {
    Serial.println("connected");
    Serial.println();
  } else  {
    Serial.print("failed, rc=");
    Serial.println(client.state());
    Serial.println();
  }
  lastMQTTConnectTry = millis();
}

/* Sends data to MQTT server in the form of a json */
void sendMQTT(String subtopic, float val) {
  String json = "{";
  json += "\"Location\":\"" + String(ssid) + "\"";
  json += ", \"" + subtopic + "\":" + String(val);
  json += "}";

  int json_length = json.length();
  char payload[json_length + 1];
  strcpy(payload, json.c_str());

  String top = "esp32WeatherStation/";
  top += subtopic;
  int top_length = top.length();
  char topic[top_length + 1];
  strcpy(topic, top.c_str());
  client.publish(topic, payload);
}

/* Reads sensor data and publishes though MQTT if connected */
void readSensors () {
  sensors_event_t temp_event, pressure_event;
  bmp_temp->getEvent(&temp_event);
  bmp_pressure->getEvent(&pressure_event);

  temperature = temp_event.temperature;
  pressure = pressure_event.pressure;
  lightlvl = bh.readLightLevel();

  Serial.println("---------------------");
  Serial.println("-- Sensor readings --");
  Serial.println("---------------------");
  
  Serial.print(F("Temperature = "));
  Serial.print(temperature);
  Serial.println(" *C");

  Serial.print(F("Pressure = "));
  Serial.print(pressure);
  Serial.println(" hPa");

  Serial.print(F("Light level = "));
  Serial.print(lightlvl);
  Serial.println(" lux");

  if (client.connected()) {
      sendMQTT("Temperature", temperature);
      sendMQTT("Pressure", pressure);
      sendMQTT("Light_Level", lightlvl);
      Serial.println("Values send over MQTT");
      Serial.println();
    }

  Serial.println();
  lastSensorRead = millis();
}

void connectEverything() {
  Serial.println ( "Connect requested" );
  connect = false;
  connectWifi();
  lastConnectTry = millis();

  int s = WiFi.status();
  Serial.print ( "Status: " );
  Serial.println ( s );
  Serial.println ( "" );
  status = s;
  if (s == WL_CONNECTED) {
    /* Just connected to WLAN */
    Serial.print ( "Connected to " );
    Serial.println ( ssid );
    Serial.print ( "IP address: " );
    Serial.println ( WiFi.localIP() );
    saveCredentials();

    /* Setup MDNS responder */
    if (!MDNS.begin(myHostname)) {
      Serial.println("Error setting up MDNS responder!");
    } else if (!mDNSStarted) {
      Serial.println("mDNS responder started");
      /* Add service to MDNS-SD */
      MDNS.addService("http", "tcp", 80);
      mDNSStarted = true;
    } else {
      Serial.println("mDNS responder started");
    }
    /* Connect to MQTT Server */
    if (!client.connected()) {
      reconnect();
    }
  }
}

void setup() {
  delay(1000);
  Serial.begin(9600);
  Serial.println();

  /* Setup filesystem  and print out files*/
  if (FORMAT_FILESYSTEM) FILESYSTEM.format();
  FILESYSTEM.begin();
  {
      File root = FILESYSTEM.open("/");
      File file = root.openNextFile();
      while(file){
          String fileName = file.name();
          size_t fileSize = file.size();
          Serial.printf("FS File: %s, size: %s\n", fileName.c_str(), formatBytes(fileSize).c_str());
          file = root.openNextFile();
      }
      Serial.printf("\n");
  }

  /* Setup access point */
  preferences.begin("CapPortAdv", false);
  Serial.print("Configuring access point...");
  /* You can remove the password parameter if you want the AP to be open. */
  WiFi.softAPConfig(apIP, apIP, netMsk);
  WiFi.softAP(softAP_ssid, softAP_password);
  delay(500); // Without delay I've seen the IP address blank
  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());

  /* Setup the DNS server redirecting all the domains to the apIP */  
  dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
  dnsServer.start(DNS_PORT, "*", apIP);

  server.on("/", []() {
    if (!handleFileRead("/")) {
      server.send(404, "text/plain", "FileNotFound");
    }
  });
  server.on("/wifi.htm", []() {
    if (!handleFileRead("/wifi.htm")) {
      server.send(404, "text/plain", "FileNotFound");
    }
  });
  server.on("/wifisave", HTTP_POST, []() {
    Serial.println("WiFi save requested");
    server.arg("name").toCharArray(ssid, sizeof(ssid) - 1);
    server.arg("pass").toCharArray(password, sizeof(password) - 1);
    connect = ssid > 0;
    if (!handleFileRead("/wifi.htm")) {
      server.send(404, "text/plain", "FileNotFound");
    }
  });
  server.on("/wifisave", HTTP_GET, []() {
    if (!handleFileRead("/wifi.htm")) {
      server.send(404, "text/plain", "FileNotFound");
    }
  });
  server.on("/generate_204", []() {
    if (!handleFileRead("/")) {
      server.send(404, "text/plain", "FileNotFound");
    }
  });  //Android captive portal. Maybe not needed. Might be handled by notFound handler.
  server.on("/fwlink", []() {
    if (!handleFileRead("/")) {
      server.send(404, "text/plain", "FileNotFound");
    }
  });  //Microsoft captive portal. Maybe not needed. Might be handled by notFound handler.
  server.onNotFound([]() {
    if (!handleFileRead(server.uri())) {
      server.send(404, "text/plain", "FileNotFound");
    }
  });
  //Get sensor values in one json call
  server.on("/sensorvalues", HTTP_GET, []() {
    String json = "{";
    json += "\"temperature\":" + String(temperature);
    json += ", \"pressure\":" + String(pressure);
    json += ", \"lightlvl\":" + String(lightlvl);
    json += "}";
    server.send(200, "text/json", json);
    json = String();
  });
  //Get all available WiFi connections in one json call
  server.on("/wificonnections", HTTP_GET, []() {
    String json = "{";
    if (server.client().localIP() == apIP) {
      json += "\"local\":1";
    } else {
      json += "\"local\":0";
    }

    json += ", \"ssidcurrent\":\"" + String(ssid) + "\"";
    json += ", \"ipcurrent\":\"" + toStringIp(WiFi.localIP()) + "\"";
    json += ", \"connected\":" + String((status == WL_CONNECTED));

    Serial.println("scan start");
    int amount = WiFi.scanNetworks();
    Serial.println("scan done");
    json += ", \"amount\":" + String(amount);

    if (amount > 0) {
      json += ", \"availableconnections\":[";
      for (int i = 0; i < amount; i++) {
        if (i != 0) {
          json += ",";
        }
        json += "{";
        json += "\"ssid\":\"" + WiFi.SSID(i) + "\"";
        json += ", \"encrypted\":" + String((WiFi.encryptionType(i) != WIFI_AUTH_OPEN));
        json += "}";
      }
      json += "]";
    }
    json += "}";
    server.send(200, "text/json", json);
    json = String();
  });
  server.begin(); // Web server start
  Serial.println("HTTP server started");
  loadCredentials(); // Load WLAN credentials from network
  /*String home ="WiFi-2.4-f0b2";
  String homepass = "EVa79nSAs7y2";
  home.toCharArray(ssid, sizeof(ssid) - 1);
  homepass.toCharArray(password, sizeof(password) - 1);
  saveCredentials();*/
  connect = ssid > 0;
  if (connect) {
    connectEverything();
  }

  /* Start Sensors */
  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1) delay(10);
  }

  if (!bh.begin()) {
    Serial.println(F("Could not find a valid BH1750 sensor, check wiring!"));
    while (1) delay(10);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  bmp_temp->printSensorDetails();
  client.setCallback(callback);
  readSensors();
}

void loop() {
  if (connect) {
    connectEverything();
  }

  if (status != WL_CONNECTED) {
      // If WLAN disconnected
      // Don't set retry time too low as retry interfere the softAP operation
      if (millis() > (lastConnectTry + 60000))
      {
        connect = true;
      }
  } else if (status != WiFi.status()) {
    connect = true;
  } else if (millis() > (lastMQTTConnectTry + 60000)) {
    if (!client.connected()) {
      reconnect();
    }
  }

  client.loop();
  if (millis() > (lastSensorRead + sensorReadInterval) ) {
    readSensors();
  }
  // Do work:
  //DNS
  dnsServer.processNextRequest();
  //HTTP
  server.handleClient();
}
