#include <Arduino.h>
#include "ADS1X15.h"
#include "esp_task_wdt.h"
#include <ArduinoJson.h>

#define RXD2 36
#define TXD2 37

// IO điều khiển
#define IO_MOTOR_PH     4
#define IO_MOTOR_AIR_1  5
#define IO_AIR_2        6
#define IO_LIGHT        7

#define VREF 5.0
#define SCOUNT 30

ADS1115 ADS(0x48);
byte ByteArrayDO[250];
byte MSG_READ[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x2A, 0xC4, 0x15};

// Biến cảm biến
float oxi_ = 0.0;
float temperature = 0.0;
float decimalValue = 0.0;
float tdsValue = 0;
float adc1 = 0, f = 1.0;

TaskHandle_t sensorTaskHandle;

String inputString = "";

// Gửi chuỗi JSON dữ liệu cảm biến
void sendJSON()
{
  char jsonBuffer[256];
  snprintf(jsonBuffer, sizeof(jsonBuffer),
           "{\"oxi\":%.2f,\"ph\":%.3f,\"temp\":%.2f,\"tss\":%.3f}",
           oxi_, decimalValue, temperature, tdsValue);
  Serial.println(jsonBuffer);
}

// Gửi yêu cầu đến cảm biến DO (MODBUS)
void sendRequest(const byte *request, size_t len)
{
  for (size_t i = 0; i < len; ++i)
  {
    Serial2.write(MSG_READ[i]);
    Serial2.flush();
  }
}

// Nhận phản hồi từ cảm biến DO
void receiveResponse(byte *byteArray, int maxLen)
{
  int index = 0;
  unsigned long start = millis();
  while ((millis() - start < 1000) && index < maxLen)
  {
    if (Serial2.available())
    {
      byteArray[index++] = Serial2.read();
    }
  }
}

// Xử lý dữ liệu DO nhận được
bool processDOData()
{
  for (int i = 0; i < 5; i++)
  {
    sendRequest(MSG_READ, sizeof(MSG_READ));
    receiveResponse(ByteArrayDO, sizeof(ByteArrayDO));
    if (ByteArrayDO[0] == 0x01 && ByteArrayDO[1] == 0x03 && ByteArrayDO[2] == 0x54)
    {
      word oxi = ((ByteArrayDO[43] << 8) & 0xFF00) | ByteArrayDO[44];
      word temp = (ByteArrayDO[37] << 8) | ByteArrayDO[38];
      word phval = (ByteArrayDO[79] << 8) | ByteArrayDO[80];

      temperature = float(temp) * 0.01;
      decimalValue = float(phval) * 0.001;
      oxi_ = oxi * 0.01;
      return true;
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
  return false;
}

// Task đo cảm biến định kỳ
void sensor_task(void *param)
{
  for (;;)
  {
    esp_task_wdt_reset();

    digitalWrite(16, 1);
    vTaskDelay(30000 / portTICK_PERIOD_MS); // bật nguồn cảm biến

    bool ok = processDOData();
    digitalWrite(16, 0);

    int16_t val_1 = ADS.readADC(1);
    adc1 = val_1 * f;
    float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0);
    float compensationVolatge = adc1 / compensationCoefficient;
    tdsValue = (133.42 * pow(compensationVolatge, 3) - 255.86 * pow(compensationVolatge, 2) + 857.39 * compensationVolatge) * 0.5;

    if (ok)
    {
      sendJSON();
    }

    vTaskDelay(30000 / portTICK_PERIOD_MS); // chờ 30s nữa trước lần đo tiếp theo
  }
}

// Xử lý lệnh JSON từ Serial và phản hồi IO đang bật
void handleJsonCommand(const String &input)
{
  StaticJsonDocument<128> doc;
  DeserializationError err = deserializeJson(doc, input);

  if (err)
  {
    Serial.print("[LỖI] JSON không hợp lệ: ");
    Serial.println(err.c_str());
    return;
  }

  String ioOn = "";

  if (doc.containsKey("STATUS_MOTOR_PH"))
  {
    bool on = doc["STATUS_MOTOR_PH"].as<String>() == "1";
    digitalWrite(IO_MOTOR_PH, on ? LOW : HIGH);
    if (on) ioOn += "IO_MOTOR_PH, ";
  }

  if (doc.containsKey("STATUS_MOTOR_AIR_1"))
  {
    bool on = doc["STATUS_MOTOR_AIR_1"].as<String>() == "1";
    digitalWrite(IO_MOTOR_AIR_1, on ? LOW : HIGH);
    if (on) ioOn += "IO_MOTOR_AIR_1, ";
  }

  if (doc.containsKey("STATUS_AIR_2"))
  {
    bool on = doc["STATUS_AIR_2"].as<String>() == "1";
    digitalWrite(IO_AIR_2, on ? LOW : HIGH);
    if (on) ioOn += "IO_AIR_2, ";
  }

  if (doc.containsKey("STATUS_LIGH"))
  {
    bool on = doc["STATUS_LIGH"].as<String>() == "1";
    digitalWrite(IO_LIGHT, on ? LOW : HIGH);
    if (on) ioOn += "IO_LIGHT, ";
  }

  Serial.println("[INFO] Đã xử lý lệnh điều khiển IO.");

  if (ioOn.length() > 0)
  {
    ioOn.remove(ioOn.length() - 2); // xóa dấu ", " cuối
    Serial.print("[TRẠNG THÁI IO] IO đã bật: ");
    Serial.println(ioOn);
  }
  else
  {
    Serial.println("[TRẠNG THÁI IO] Không có IO nào đang bật.");
  }
}

void setup()
{
  Serial.begin(9600);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);

  // GPIO cấp nguồn cho cảm biến DO
  pinMode(16, OUTPUT);
  digitalWrite(16, 1);

  // IO điều khiển
  pinMode(IO_MOTOR_PH, OUTPUT);
  pinMode(IO_MOTOR_AIR_1, OUTPUT);
  pinMode(IO_AIR_2, OUTPUT);
  pinMode(IO_LIGHT, OUTPUT);

  // Mặc định tất cả IO tắt (HIGH)
  digitalWrite(IO_MOTOR_PH, HIGH);
  digitalWrite(IO_MOTOR_AIR_1, HIGH);
  digitalWrite(IO_AIR_2, HIGH);
  digitalWrite(IO_LIGHT, HIGH);

  Wire.begin(42, 2, 100000);
  ADS.begin();
  ADS.setGain(0);
  f = ADS.toVoltage(1);

  esp_task_wdt_init(10, true);
  esp_task_wdt_add(NULL);

  xTaskCreatePinnedToCore(
      sensor_task,
      "SensorTask",
      4096,
      NULL,
      1,
      &sensorTaskHandle,
      1);
}

void loop()
{
  esp_task_wdt_reset();

  // Đọc chuỗi JSON từ Serial
  while (Serial.available())
  {
    char c = Serial.read();
    if (c == '\n')
    {
      inputString.trim();
      if (!inputString.isEmpty())
        handleJsonCommand(inputString);
      inputString = "";
    }
    else
    {
      inputString += c;
    }
  }

  delay(100);
}
