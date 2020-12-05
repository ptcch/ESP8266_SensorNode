#include <Arduino.h>
#include <FS.h>          // this needs to be first, or it all crashes and burns...
#include <ArduinoJson.h> // https://github.com/bblanchon/ArduinoJson
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h> 
#include <DNSServer.h>
#include <ESP8266mDNS.h>

#define MDNS_NAME "esp8266"
#define NODE_DEFAULT_ADDRESS ((uint32_t) 211)

#define TIMEOUT_STEP_MS 500
#define TIMEOUT_MS 30000

float param1=1000;
float param2=2000;

const char SEND_page_OK[] PROGMEM = "<body style=\"background-color:lightgreen\"><a><center><h1> <br><br><br><font size=\"10\">New settings acquired successfully!</font></h1></center> </a></body>";

const char SEND_page_FAILURE[] PROGMEM = "<body style=\"background-color:rgba(255,20,0,0.6)\"><a><center><h1><p> <br><br><br><font size=\"10\">Oops! Something went wrong.</font></p></h1></center> </a></body>";

const char MAIN_page[] PROGMEM = R"=====(
<!DOCTYPE html>
<html>
  <head> 
    <title>Change Placeholder alignment</title> 
    <style> 
      
      input[type="text"]::placeholder { 
        text-align: center; 
      } 

      body { 
        text-align:center; 
      }
    </style> 
  </head> 
    
<body style="background-color:powderblue;">

<h1><font size="10">NODE SETTING</font></h1>

<form action="/action_page">
 <b> <font size="4">parameter 1:</font></b>
 <br>
  <input type="number" name="param1"  min="1" max="5" style="cursor:pointer;"  title="meaning of param1"> 
 <br>
 <i>actual value %.2f</i>
  <br>
  <br>
  <br>
  <b><font size="4">parameter 2:</font></b><br>
  <input type="number" name="param2"  min="-10" max="40" style="cursor:pointer;"  title="meaning of param2">
  <br>
  <i>actual value %.2f</i>
  <br>
  <br>
  <br>
  <input type="submit" value="Submit">
  
</form> 
</body>
</html>
)=====";


ESP8266WebServer server(80); //Server on port 80

//===============================================================
// This routine is executed when you open its IP in browser
//===============================================================
void handleRoot() {
  
  char html[sizeof(MAIN_page)+100]; //NOTE: sizeof(html) > sizeof(MAIN_page)
  
  //Serial.println(sizeof(MAIN_page));
  
  snprintf_P(html, sizeof(html), MAIN_page, param1,param2);

   server.send_P(200, "text/html", html);
}
//===============================================================
// This routine is executed when you press submit
//===============================================================
void handleForm() {
  String param1 = server.arg("param1"); 
  String param2 = server.arg("param2"); 

 Serial.print("Parameter 1: ");
 Serial.println(param1.toFloat());

 Serial.print("parameter 2: ");
 Serial.println(param2.toFloat());
 
 server.send_P(200, "text/html", SEND_page_OK); 

//if (wrong)
// server.send_P(200, "text/html", SEND_page_FAILURE); 
}

//==============================================================

struct ConfigSetting{
  IPAddress ip, gateway, netmask, dns1, dns2;
  String ssid, pwd;
};

bool shouldSaveConfig = false;

bool setupSpiffs(struct ConfigSetting *conf) {
  //clean FS, for testing
  // SPIFFS.format();

  //read configuration from FS json
  Serial.println("mounting FS...");

  if (SPIFFS.begin()) {
    Serial.println("mounted file system");
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonDocument jsonDoc(1024);
        auto error = deserializeJson(jsonDoc, buf.get());
        serializeJson(jsonDoc, Serial);
        if (!error) {
          Serial.println("\nparsed json");

           if(jsonDoc["ip"]) {            
             Serial.println("setting custom ip from config");

             if(!(conf->ip.fromString((const char*) jsonDoc["ip"]))) {
               return false;
             }
             if(!(conf->gateway.fromString((const char*) jsonDoc["gateway"]))) {
               return false;
             }
             if(!(conf->netmask.fromString((const char*) jsonDoc["subnet"]))) {
               return false;
             }
             if(!(conf->dns1.fromString((const char*) jsonDoc["dns1"]))) {
               return false;
             }
             if(!(conf->dns2.fromString((const char*) jsonDoc["dns2"]))) {
               if(strcmp_P((const char*) jsonDoc["dns2"], PSTR("(IP unset)")) == 0) conf->dns2 = conf->dns1;
               else return false;
             }

             conf->ssid = String((const char*) jsonDoc["ssid"]);
             conf->pwd = String((const char*) jsonDoc["pwd"]);

             Serial.println(conf->ip);

             return true;
           } else {
             Serial.println("no custom ip in config");
           }

        } else {
          Serial.println("failed to load json config");
        }
      }
    }
  } else {
    Serial.println("failed to mount FS");
  }
  //end read

  return false;
}

void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

void routineWM() {
  //wifi connection setup: load the sketch and then connect to Node_Setup_WiFi network
    WiFiManager wifiManager;
    
    wifiManager.setSaveConfigCallback(saveConfigCallback);
    wifiManager.resetSettings();
    wifiManager.autoConnect("Node_Setup_WiFi");

    uint32_t defaultIP = uint32_t(WiFi.gatewayIP())&uint32_t(WiFi.subnetMask());
    defaultIP = defaultIP + (NODE_DEFAULT_ADDRESS<<__builtin_popcount(uint32_t(WiFi.subnetMask())));
  
    WiFi.config(IPAddress(defaultIP), WiFi.gatewayIP(), WiFi.subnetMask(),WiFi.dnsIP(0),WiFi.dnsIP(1));
    WiFi.reconnect();

    //save the custom parameters to FS
    if (shouldSaveConfig) {
      Serial.println("saving config");
      DynamicJsonDocument jsonDoc(1024);
  
      jsonDoc["ip"] = WiFi.localIP().toString();
      jsonDoc["gateway"] = WiFi.gatewayIP().toString();
      jsonDoc["subnet"] = WiFi.subnetMask().toString();
      jsonDoc["dns1"] = WiFi.dnsIP(0).toString();
      jsonDoc["dns2"] = WiFi.dnsIP(1).toString();
      jsonDoc["ssid"] = WiFi.SSID();
      jsonDoc["pwd"] = WiFi.psk();
  
      File configFile = SPIFFS.open("/config.json", "w");
      if (!configFile) {
        Serial.println("failed to open config file for writing");
      }

      serializeJsonPretty(jsonDoc, Serial);
      serializeJson(jsonDoc, configFile);
      configFile.close();
      //end save
      shouldSaveConfig = false;
    }
}

//==============================================================
//                  SETUP
//==============================================================
void setup(void){
  Serial.begin(115200);

  struct ConfigSetting conf;

  if(setupSpiffs(&conf) == false) {
    routineWM();
  } else {
    WiFi.disconnect();

    WiFi.mode(WIFI_STA);

    WiFi.config(conf.ip, conf.gateway, conf.netmask, conf.dns1, conf.dns2);
    WiFi.begin(conf.ssid,conf.pwd);

    uint32_t cnt = 0;
    while (WiFi.status() != WL_CONNECTED/* && cnt < TIMEOUT_MS*/)
    {
      cnt += TIMEOUT_STEP_MS;
      delay(TIMEOUT_STEP_MS);
      Serial.print(".");
    }

    if(WiFi.status() != WL_CONNECTED) {
      routineWM();
    }
  }
  
  //If connection successful show IP address in serial monitor
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println("WiFi");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());  //IP address assigned to your ESP

  MDNS.begin(MDNS_NAME);
 
  server.on("/", handleRoot);      //Which routine to handle at root location
  server.on("/action_page", handleForm); //form action is handled here

  server.begin();                  //Start server
  Serial.println("HTTP server started");

  MDNS.addService("http", "tcp", 80);
  Serial.println("MDNS started");
}
//==============================================================
//                     LOOP
//==============================================================
void loop(void){
  server.handleClient();          //Handle client requests
  MDNS.update();
}