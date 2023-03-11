#pragma once

struct Cfg
{
  char mqtt_server[40];
  char ssid[32];
  char pass[32];
  char mqtt_id[40];
  int period;
  float coeff_v;
  int timeout;
  float tcomp;
  uint16_t mqtt_port;
};

extern Cfg cfg;

void loadConfiguration(const char *filename);
void saveConfiguration(const char *filename);
void printFile(const char *filename, Stream& S);

class Filter1
{
  private:
    float k;
    float v;
  public:
    Filter1(float V = 0, float K = 0.9){v=V; k=K;}
    void SetV(float V){v=V;}
    void SetK(float K){k=K;}
    float Filter(float A);
};

class TDelta
{
  private:
    static const int CNT = 60;
    float data[CNT];
    int idx;
  public:
    TDelta();
    float update(float V);
    void print(){for(int i=0; i<CNT; i++) Serial.printf("%0.2f ", data[i]); Serial.printf("\r\n");}
};
