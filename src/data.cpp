#include <FS.h>
#include <ArduinoJson.h>
#include "data.h"

#include "auth.h" // здесь токен, имя сети и пароль

Cfg cfg = {
  BLYNK_TOKEN, // Токен проекта 6351a704ef664d43ba2c308c6b7b1515
  SSID,        // Имя сети WiFi
  PASSW,       // Пароль сети WiFi
  15,          //
  877/4.035,   //
  10           // timeout
};

// Loads the configuration from a file
void loadConfiguration(const char *filename) 
{
  // Open file for reading
  File file = SPIFFS.open(filename,"r");

  // Allocate a temporary JsonDocument
  // Don't forget to change the capacity to match your requirements.
  // Use arduinojson.org/v6/assistant to compute the capacity.
  StaticJsonDocument<512> doc;

  // Deserialize the JSON document
  DeserializationError error = deserializeJson(doc, file);
  if (error)
    Serial.println(F("Failed to read file, using default configuration"));

  // Copy values from the JsonDocument to the Config
  memset(cfg.ssid,0,sizeof(cfg.ssid));
  memset(cfg.pass,0,sizeof(cfg.pass));
  memset(cfg.auth,0,sizeof(cfg.auth));
  strcpy(cfg.ssid, (const char*)doc["SSID"]);
  strcpy(cfg.pass, (const char*)doc["Passw"]);
  strcpy(cfg.auth, (const char*)doc["Auth"]);
  cfg.Period  = doc["Period"];
  cfg.Coeff_V = doc["Coeff_V"];
  cfg.Timeout = doc["Timeout"];
  
  // Close the file (Curiously, File's destructor doesn't close the file)
  file.close();
}



// Saves the configuration to a file
void saveConfiguration(const char *filename) 
{
  // Open file for writing
  File file = SPIFFS.open(filename, "w");
  if (!file) 
  {
    Serial.println(F("Failed to create file"));
    return;
  }

  // Allocate a temporary JsonDocument
  // Don't forget to change the capacity to match your requirements.
  // Use arduinojson.org/assistant to compute the capacity.
  StaticJsonDocument<256> doc;

  // Set the values in the document
  doc["Auth"]    = cfg.auth;
  doc["SSID"]    = cfg.ssid;
  doc["Passw"]   = cfg.pass;
  doc["Period"]  = cfg.Period;
  doc["Coeff_V"] = cfg.Coeff_V;
  doc["Timeout"] = cfg.Timeout;

  // Serialize JSON to file
  if (serializeJson(doc, file) == 0) 
  {
    Serial.println(F("Failed to write to file"));
  }

  // Close the file
  file.close();
}

// Prints the content of a file to the Serial
void printFile(const char *filename) 
{
  // Open file for reading
  File file = SPIFFS.open(filename,"r");
  if (!file) 
  {
    Serial.println(F("Failed to read file"));
    return;
  }

  // Extract each characters by one by one
  while (file.available()) 
  {
    Serial.print((char)file.read());
  }
  Serial.println();

  // Close the file
  file.close();
}

float Kalman::Filter(float A)
{
  v = k*A + (1-k)*v;
  return v;
}

