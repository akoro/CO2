#pragma once

struct Cfg
{
  char auth[36];
  char ssid[32];
  char pass[32];
  int Period;
  float Coeff_V;
  int Timeout;
  float TComp;
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
