#include <LittleFS.h>
#include <ArduinoJson.h>
#include "data.h"

#include "auth.h" // здесь токен, имя сети и пароль

#define JBUFSIZE 512

Cfg cfg = {
  MQTT_SERVER, // MQTT сервер
  SSID,        // Имя сети WiFi
  PASSW,       // Пароль сети WiFi
  MQTT_ID,
  15,          //
  877/4.035,   //
  10,          // timeout
  0.0,         // T comp
  1833         // mqtt_port
};

// Loads the configuration from a file
void loadConfiguration(const char *filename) 
{
  // Open file for reading
  File file = LittleFS.open(filename,"r");

  // Allocate a temporary JsonDocument
  // Don't forget to change the capacity to match your requirements.
  // Use arduinojson.org/v6/assistant to compute the capacity.
  StaticJsonDocument<JBUFSIZE> doc;

  // Deserialize the JSON document
  DeserializationError error = deserializeJson(doc, file);
  if (error)
    Serial.println(F("Failed to read file, using default configuration"));

  // Copy values from the JsonDocument to the Config
  memset(cfg.ssid,0,sizeof(cfg.ssid));
  strcpy(cfg.ssid, (const char*)doc["SSID"]);
  memset(cfg.pass,0,sizeof(cfg.pass));
  strcpy(cfg.pass, (const char*)doc["Passw"]);
  memset(cfg.mqtt_server,0,sizeof(cfg.mqtt_server));
  strcpy(cfg.mqtt_server, (const char*)doc["MQTT_server"]);
  memset(cfg.mqtt_id,0,sizeof(cfg.mqtt_id));
  strcpy(cfg.mqtt_id, (const char*)doc["MQTT_ID"]);
  cfg.period  = doc["Period"];
  cfg.coeff_v = doc["Coeff_V"];
  cfg.timeout = doc["Timeout"];
  cfg.tcomp   = doc["T-comp"];
  cfg.mqtt_port = doc["Port"];
  
  // Close the file (Curiously, File's destructor doesn't close the file)
  file.close();
}



// Saves the configuration to a file
void saveConfiguration(const char *filename) 
{
  // Open file for writing
  File file = LittleFS.open(filename, "w");
  if (!file) 
  {
    Serial.println(F("Failed to create file"));
    return;
  }

  // Allocate a temporary JsonDocument
  // Don't forget to change the capacity to match your requirements.
  // Use arduinojson.org/assistant to compute the capacity.
  StaticJsonDocument<JBUFSIZE> doc;

  // Set the values in the document
  doc["MQTT_server"] = cfg.mqtt_server;
  doc["MQTT_ID"]     = cfg.mqtt_id;
  doc["SSID"]        = cfg.ssid;
  doc["Passw"]       = cfg.pass;
  doc["Period"]      = cfg.period;
  doc["Coeff_V"]     = cfg.coeff_v;
  doc["Timeout"]     = cfg.timeout;
  doc["T-comp"]      = cfg.tcomp;
  doc["Port"]        = cfg.mqtt_port;

  // Serialize JSON to file
  if (serializeJson(doc, file) == 0) 
  {
    Serial.println(F("Failed to write to file"));
  }

  // Close the file
  file.close();
}

// Prints the content of a file to the Serial
void printFile(const char *filename, Stream& S) 
{
  // Open file for reading
  File file = LittleFS.open(filename,"r");
  if (!file) 
  {
    S.println(F("Failed to read file"));
    return;
  }

  // Extract each characters by one by one
  while (file.available()) 
  {
    S.print((char)file.read());
  }
  S.println();

  // Close the file
  file.close();
}

float Filter1::Filter(float A)
{
  v = k*A + (1-k)*v;
  return v;
}

