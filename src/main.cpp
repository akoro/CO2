/* 
Измеритель параметров атмосферы
- содержание CO2,
- температура,
- относительная влажность,
- давление.
На основе технологии Blynk. (http://blynk.cc)
Версия 0.1 2019.11.04
           2019.12.01
           2019.12.17 updates
*/

#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <SimpleTimer.h>
#include <SoftwareSerial.h>
#include <BlynkSimpleEsp8266.h>
#include <FS.h>
#include <ArduinoJson.h>
#include <Bounce2.h>
#include <console.h>

#include "data.h"

const char *cfg_name = "/config";

// GPIO Defines
#define I2C_SDA  4  
#define I2C_SCL  5 
#define CO2_POW 12 
#define CO2_RX  15
#define CO2_TX  13
#define BUT1     0
#define BUT2    14

U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, I2C_SCL, I2C_SDA, U8X8_PIN_NONE);
SoftwareSerial swSer(CO2_TX, CO2_RX, false);
Adafruit_BME280 bme;
SimpleTimer timer;

// Sensors data
int   co2         = -1;
float Pressure    = 0;
float Humidity    = 0;
float Temperature = 0;
float Voltage     = 0;

Kalman FPressure(750);
Kalman FTemperature(25);
Kalman FHumidity(30);
Kalman FVoltage(4.0);

Bounce Button1, Button2;

static bool  Indication    = true;
static const float Coeff_P = 760.0/101325.0; // mm Hg
static int IndicationPeriod = 3; // сколько периодов измерения включён индикатор
static String Line = "";
bool isFirstConnect = true; // Keep this flag not to re-sync on every reconnection

void draw(int);

void Indicator(const bool E)
{
  Indication = E;
  Blynk.virtualWrite(V5, Indication);
  if(Indication)
  {
    u8g2.setPowerSave(0);
    Serial.println(F("display ON"));
    IndicationPeriod = 3;
  }
  else
  {
    u8g2.setPowerSave(1);
    Serial.println(F("display OFF"));
    IndicationPeriod = 0;
    draw(0);
  }
}

BLYNK_WRITE(V5)
{
  Indicator(param.asInt());
}

BLYNK_WRITE(V7)
{
  if (param.asInt()) ESP.restart();
}

// This function will run every time Blynk connection is established
BLYNK_CONNECTED() 
{
  if (isFirstConnect) 
  {
    // Request Blynk server to re-send latest values for all pins
    Blynk.syncAll();
    isFirstConnect = false;
  }
}

void readCO2() 
{
  const byte cmd[9] = {0xFF,0x01,0x86,0x00,0x00,0x00,0x00,0x00,0x79};
  byte response[9] = {0,0,0,0,0,0,0,0,0};
  bool header_found = false;

  co2 = -1;
  swSer.write(cmd, 9);

  // Looking for packet start
  while(swSer.available() && (!header_found)) 
  {
    if(swSer.read() == 0xff ) 
    {
      if(swSer.read() == 0x86 ) header_found = true;
    }
  }

  if (header_found) 
  {
    swSer.readBytes(response, 7);

    byte crc = 0x86;
    for (int i = 0; i < 6; i++)
    {
      crc+= (byte)response[i];
    }
    crc = 0xff - crc;
    crc++;

    if (response[6] != crc)
    {
      Serial.printf("CO2: CRC error: %d/%d\r\n", crc, response[6]);
    } 
    else 
    {
      unsigned int responseHigh = (unsigned int) response[0];
      unsigned int responseLow = (unsigned int) response[1];
      unsigned int ppm = (256*responseHigh) + responseLow;
      co2 = ppm;
    }
  } 
  else 
  {
    Serial.println(F("CO2: Header not found"));
  }
}

void readBME280(void)
{
  bme.takeForcedMeasurement();
  Humidity    = FHumidity.Filter(bme.readHumidity());
  Pressure    = FPressure.Filter(bme.readPressure());
  Temperature = FTemperature.Filter(bme.readTemperature());
}

void CheckPower(void)
{
  Voltage = FVoltage.Filter(analogRead(A0) / cfg.Coeff_V);
  if(Voltage > 3.7)
  {
    digitalWrite(CO2_POW, HIGH);
  }
  else if(Voltage < 3.2)
  {
    digitalWrite(CO2_POW, LOW);
    u8g2.setPowerSave(1);
    char buff[30];
    sprintf(buff, "V=%0.2fV", Voltage);
    Blynk.email("Voltage low", buff);
    ESP.deepSleep(600000000); // засыпаем на 10 минут
  }
}

void draw(int Progress) 
{
  byte x,y,h;
  char buff[30];
  
  u8g2.clearBuffer();
  
  // содержание CO2
  if (co2 > 0) 
  {
    sprintf(buff, "%i", co2);
  } 
  else 
  {
    memset(buff,'.',4); buff[4]='\0'; 
  }
  u8g2.setFont(u8g2_font_inb19_mf);
  x = (128 - u8g2.getStrWidth(buff))/2;
  y = u8g2.getAscent() - u8g2.getDescent();
  u8g2.drawStr(x, y, buff);
  
  // влажность, давление, температура, напряжение
  u8g2.setFont(u8g2_font_6x12_mf);
  y = y + 2 + u8g2.getAscent() - u8g2.getDescent();
  x = 0;
  sprintf(buff, "H=%0.0f P=%0.2f T=%0.1f", Humidity, Pressure*Coeff_P, Temperature);
  u8g2.drawStr(x, y, buff);
  y = y + 2 + u8g2.getAscent() - u8g2.getDescent();
  x = 0;
  sprintf(buff, "V=%0.2f", Voltage);
  
  // счётчик времени
  u8g2.drawStr(x, y, buff);
  y = 60;
  x = 0;
  u8g2.drawBox(x,y,128*Progress/100,y+3);
  
  // уровень заряда
  x=0; 
  y=0;
  if(Voltage > 4.25) 
  {
    const byte bm[16] = {0x3C,0x24,0xE7,0x89,0x91,0x91,0xA1,0xA1,0xBD,0x85,0x85,0x89,0x89,0x91,0x81,0xFF};
    u8g2.drawBitmap(x,y,1,16,bm); // внешнее питание и зарядка
  }
  else
  { 
    if(Voltage <= 3.00)            
      h = 0;                      // батарея разряжена
    else if(Voltage <= 4.20)
      h = 12*(Voltage-3.00)/1.20;
    else
      h = 12;                     // полный заряд
    const byte bm[16] = {0x3C,0x24,0xE7,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0xFF};
    u8g2.drawBitmap(x,y,1,16,bm);
    u8g2.drawBox(x,y+15-h,8,h);
  }
  
  // Соединение
  x=119;
  y=0;
  if(Blynk.connected())
  {
    const byte bm[8] = {0xFF,0x00,0x7E,0x00,0x3C,0x00,0x18,0x18};
    u8g2.drawBitmap(x,y,1,8,bm);
  }

  u8g2.sendBuffer();
}

void DoMeasurements() 
{
  static int Counter = 0;
  if(++Counter == cfg.Period)
  {
    Counter = 0;
    CheckPower();
    readCO2();
    readBME280();
    
    if(Line.length()==0)
    {
      Serial.printf("CO2: %d\r\n", co2);
      Serial.printf("BME280: h=%0.0f p=%0.1f t=%0.1f\r\n", Humidity,Pressure,Temperature);
    }

    if(Blynk.connected())
    {
      Blynk.virtualWrite(V1, Humidity);
      Blynk.virtualWrite(V2, Temperature);
      Blynk.virtualWrite(V3, Pressure*Coeff_P);
      if(co2 > 0)  
        Blynk.virtualWrite(V4, co2);
      Blynk.virtualWrite(V6, Voltage);
    }
 
    if(IndicationPeriod)
    {
      IndicationPeriod--;
      if(IndicationPeriod == 0)
        Indicator(false);
    }
  }

  if(Indication)
  {
    draw(Counter*100/cfg.Period);
  }

}

void drawBoot(char* msg) 
{
  static byte x,y;
  if(*msg=='\0')
  {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x12_mf);
    x = 0;
    y = u8g2.getAscent() - u8g2.getDescent();
  }
  else
  {
    bool f = *msg == '+';
    if(f) msg++;
    u8g2.drawStr(x, y, msg);
    if(f) y = y + 2 + u8g2.getAscent() - u8g2.getDescent();
  }
  u8g2.sendBuffer();
} 

void setup() 
{
  // Init serial ports
  Serial.begin(115200);
  swSer.begin(9600);

  // Power ON of CO2 sensor
  pinMode(CO2_POW, OUTPUT);
  digitalWrite(CO2_POW, LOW);

  // Init file system and load configuration
  if(!SPIFFS.begin()) 
  {
    Serial.println(F("Failed to mount SPIFFS. Formatting..."));
    SPIFFS.format();
  }
  Serial.println(F("SPIFFS is ready."));
  if(SPIFFS.exists(cfg_name))
  {
    loadConfiguration(cfg_name);
    Serial.println(F("Config loaded"));
  }
  else
    Serial.println(F("Default config"));
  
  // Check power
  Voltage = analogRead(A0) / cfg.Coeff_V;
  if(Voltage < 3.2)
  {
    ESP.deepSleep(600000000); // засыпаем на 10 минут
  }
  FVoltage.SetV(Voltage);

  // Init I2C interface
  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.println(F("\r\nI2C OK"));
  
  // Init display
  u8g2.begin();
  drawBoot("");
  delay(10);
  Serial.println(F("Display OK"));

  // Init sensor
  Serial.print(F("BME280 "));
  if(bme.begin(0x76))
  {
    Serial.println(F("OK"));
  }
  else
    Serial.println(F("fail"));
  drawBoot("+BME280");

  // buttons
  Button1.attach(BUT1);
  Button1.interval(30);
  Button2.attach(BUT2);
  Button2.interval(30);

  // Init Blynk
  Blynk.config(cfg.auth);


  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
    if (WiFi.status() != WL_CONNECTED) 
  {
    drawBoot("+WiFi");
    Serial.print("Connecting WiFi ");
    WiFi.begin(cfg.ssid, cfg.pass);
    int timeout = cfg.Timeout;
    while (WiFi.status() != WL_CONNECTED && timeout) 
    {
      delay(1000);
      timeout--;
      Serial.print('.');
      drawBoot(".");
    }
    if(timeout == 0)
    {
      WiFi.disconnect(true);
      Serial.println(F(" not connected"));
      drawBoot("+not connected");
    }
    else
    {
      Serial.print(F(" connected ")); Serial.println(WiFi.localIP());
      Blynk.connect(5000);
      drawBoot("+connected");
    }
  }

  if(Blynk.connected())
  {
    Serial.println(F("Blynk connected"));
    drawBoot("Blynk");
    delay(1000);
  } 
  // Setup a function to be called every 1 second
  timer.setInterval(1000L, DoMeasurements);

  DoMeasurements();
  
  Serial.print('>');  // консоль
}

// проверка команды и выделение параметров
static bool command(const String& T, String& L, String& P)
{
  if(L.startsWith(T))
  {
    P = L.substring(L.indexOf(' '));
    P.trim();
    return true;
  }
  return false;
}

// разбор консольных команд
static void Parsing(String& L)
{
  String Par;
  
  // форматирование файловой системы
  if(command("format", L, Par))
  {
    Serial.print(F("Formatting..."));
    SPIFFS.format();
    Serial.println(F(" done"));
  }
  
  // тест
  else if(command("test", L, Par))
  {
    if(Par.length())
    {
      Serial.print('[');Serial.print(Par);Serial.println(']');
    }
  }

  // задать SSID сети
  else if(command("ssid", L, Par))
  {
    if(Par.length())
    {
      memset(cfg.ssid, 0, sizeof(cfg.ssid));
      strlcpy(cfg.ssid, Par.c_str(), Par.length()+1);
    }
    Serial.printf("SSID: \"%s\"\r\n", cfg.ssid);
  }
  
  // задать пароль сети
  else if(command("pass", L, Par))
  {
    if(Par.length())
    {
      memset(cfg.pass, 0, sizeof(cfg.pass));
      strlcpy(cfg.pass, Par.c_str(), Par.length()+1);
    }
    Serial.printf("Password: \"%s\"\r\n", cfg.pass);
  }
  
  // задать идентификатор проекта (токен)
  else if(command("token", L, Par))
  {
    if(Par.length()==32)
    {
      memset(cfg.auth, 0, sizeof(cfg.auth));
      strlcpy(cfg.auth, Par.c_str(), 33);
    }
    else
    {
      Serial.println(F("token must have 32 simbols"));
    }
    Serial.printf("token: \"%s\"\r\n", cfg.auth);
  }
  
  // задать период измерений
  else if(command("period", L, Par))
  {
    if(Par.length())
    {
      int i = Par.toInt();
      if(i>=10 && i<=60)
        cfg.Period = i;
      else
        Serial.println(F("! Period should be in range 10...60"));
    }
    Serial.printf("Period = %ds\r\n", cfg.Period);
  }
  
  // задать время соединения к WiFi
  else if(command("timeout", L, Par))
  {
    if(Par.length())
    {
      int i = Par.toInt();
      if(i>=1 && i<=20)
        cfg.Timeout = i;
      else
        Serial.println(F("! Timeout should be in range 1...20"));
    }
    Serial.printf("Timeout = %ds\r\n", cfg.Timeout);
  }
  
  // калибровка АЦП
  else if(command("volt", L, Par))
  {
    if(Par.length())
    {
      float v = Par.toFloat();
      if(v>=2.9 && v<=5.0)
      {
        int s = 0;
        const int n=16;
        for(int i=0; i<n; i++)
          s += analogRead(A0);
        cfg.Coeff_V = (float)s/n/v;
      }
      else
        Serial.println(F("! Voltage should be in range 2.9...5.0V"));
    }
    Serial.printf("Coeff_V=%0.5f\r\n", cfg.Coeff_V);
  }
  
  // сохранить параметры
  else if(command("save", L, Par))
  {
    saveConfiguration(cfg_name);
    Serial.println(F("Configuration saved"));
  }
  // вывести конфигурационный файл
  else if(command("type", L, Par))
  {
    printFile(cfg_name);
  }
  // сброс
  else if(command("reset", L, Par))
  {
    ESP.restart();
  }
  
  // информация о системе
  else if(command("info", L, Par))
  {
    Serial.printf("ID: Chip:%08X, Flash:%08X CPU f=%dMHz\r\n", ESP.getChipId(), ESP.getFlashChipId(), ESP.getCpuFreqMHz());
    Serial.printf("SDK: %s, Core: %s\r\n",ESP.getSdkVersion(), ESP.getCoreVersion().c_str());
    Serial.printf("IP: %s\r\n", WiFi.localIP().toString().c_str());
    Serial.print(F("Compiled at ")); Serial.println(F(__TIMESTAMP__));
  }
  
  // вкл./выкл. индикатор
  else if(command("ind", L, Par))
  {
    Indicator(Par == "1");
  }
  else
  {
    Serial.print("? ");
    Serial.println(L);
  }
}

void loop() 
{
  Button1.update();
  Button2.update();
  Blynk.run(); 
  timer.run();
  if(Button1.fell())
  {
    Indicator(true);
  }

  if(Serial.available())
  {
    char c = Serial.read();
    if((c>=' ') && (c<127)) // ввод команды
    {
      Line += c;
      Serial.write(c);
    }
    else if(c == '\r') // команда принята,
    {
      Serial.write('\r'); Serial.write('\n');
      Line.trim();
      if(Line != "")
        Parsing(Line); // разбор команды
      Line = "";
      Serial.print(">");
    }
    else if(Line.length()!=0 && ((c == '\b')||(c == 127))) // забой
    {
      Line.remove(Line.length()-1);
      Serial.write("\b \b");
    }
  }
}
