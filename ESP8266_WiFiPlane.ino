//**************************************************
// 物聯網WIFI手機遠端遙控飛機
// 軟實力精進聯盟 SoftSkillsUnion 黃建彰
// http://www.s2u4o.com
//***************************************************

#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager
#include <Bounce2.h>     // https://github.com/thomasfredericks/Bounce2
#include <WiFiUdp.h>
#include <ArduinoJson.h> // https://github.com/bblanchon/ArduinoJson

char *strtochar(String x)
{
  static char c[50];
  x.toCharArray(c, 50);
  return c;
}

char *floattochar(float i)
{
  static char c[5];
  sprintf(c, "%3.1f", i);
  return c;
}

const char *apName = strtochar("SSU WifiPlane " + String(ESP.getChipId()));

/***************↓↓↓↓↓ 更改設定值 ↓↓↓↓↓***************/

const char *apPwd = "12345678"; // 基地台模式登入密碼

/***************↑↑↑↑↑ 更改設定值 ↑↑↑↑↑***************/

#define motorA D1
#define motorB D5
#define pinReset D3 // GPIO0

int receivePort = 8888; // 接收UDP封包的通訊埠
int remotPort = 2390;   // 發佈數據報的通訊埠

IPAddress receiveIP;
IPAddress remotIP;

const int bufferSize = 50;     // 預設接收封包的緩衝區大小
char packetBuffer[bufferSize]; // 接收傳入的封包

unsigned long lastGetVccTime = 0; // 前次取得電池電壓時間
int intervalVcc = 3000;           // 傳輸電池電壓間隔

unsigned long lastGetDataTime = 0; // 前次取得數據時間
int intervalData = 5000;           // 判斷是否斷線間隔

Bounce debouncer = Bounce();
unsigned long btnPressTime = 0;        // 按下時間
unsigned long btnPressDuration = 3000; // 按下持續時間

int speedA = 0;
int speedB = 0;

WiFiUDP UDP;
WiFiManager wm;

void setup()
{
  Serial.begin(115200);
  Serial.println();
  analogWriteRange(255);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  pinMode(motorA, OUTPUT);
  pinMode(motorB, OUTPUT);
  analogWrite(motorA, 0);
  analogWrite(motorB, 0);
  pinMode(pinReset, INPUT);
  debouncer.attach(pinReset);
  debouncer.interval(5);

  WiFi.mode(WIFI_STA);
  bool res = wm.autoConnect(apName, apPwd);
  if (!res)
  {
    Serial.println("Failed to connect");
  }
  else
  {
    //if you get here you have connected to the WiFi
    Serial.println("connected)");
    Serial.print("STA IP address: ");
    Serial.println(WiFi.localIP());
    receiveIP = remotIP = WiFi.localIP();

    remotIP[3] = 255; // 設定發佈數據報的IP為廣播位址
    UDP.begin(receivePort);
    digitalWrite(LED_BUILTIN, HIGH);
  }
}

void loop()
{
  debouncer.update();

  if (debouncer.fell())
  {
    btnPressTime = millis();
  }

  if (debouncer.read() == LOW && millis() - btnPressTime >= btnPressDuration)
  {
    Serial.println("Reset");
    wm.resetSettings();
    ESP.restart();
  }

  // UDP接收
  int packetSize = UDP.parsePacket();
  if (packetSize)
  {
    // 讀取封包
    Serial.printf("Received %d bytes from %s, port %d\n", packetSize, UDP.remoteIP().toString().c_str(), UDP.remotePort());
    int len = UDP.read(packetBuffer, bufferSize);
    packetBuffer[len] = 0; // 清除前筆資料暫存
    Serial.println(packetBuffer);
    if (len)
    {
      // 分配空間大小至 https://arduinojson.org/v6/assistant/ 計算
      StaticJsonDocument<65> doc;
      deserializeJson(doc, packetBuffer);
      int maxSpeed = doc["maxSpeed"];
      speedA = map(doc["speedA"], 0, maxSpeed, 0, 255); // 將doc["speedA"]的值從0－100區間映射到0－255區間
      speedB = map(doc["speedB"], 0, maxSpeed, 0, 255);
      Serial.printf("MotorA speed %d； MotorB speed %d\n", speedA, speedB);
      analogWrite(motorA, speedA);
      analogWrite(motorB, speedB);
      lastGetDataTime = millis();
    }
  }

  // 發送電壓、IP位置資訊、訊號強度資料
  if (millis() - lastGetVccTime > intervalVcc)
  {
    int rssi = WiFi.RSSI();

    lastGetVccTime = millis();
    int getAnalog = analogRead(A0);
    float getVoltage = getAnalog * (4.2 * 0.9767) / 1024;

    // Serial.printf("Now Voltage is %f V\n", getVoltage);

    char json[60];
    StaticJsonDocument<100> doc;

    doc["receiveip"] = receiveIP.toString();
    doc["voltage"] = floattochar(getVoltage);
    doc["rssi"] = rssi;
    serializeJson(doc, json);

    UDP.beginPacket(remotIP, remotPort);
    UDP.write(json);
    UDP.endPacket();
  }

  // 控制器連線中斷，關閉馬達
  if (millis() - lastGetDataTime > intervalData)
  {
    analogWrite(motorA, 0);
    analogWrite(motorB, 0);
    Serial.println("Disconnected");
  }
}