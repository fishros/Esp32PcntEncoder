#include <Arduino.h>
#include <WiFi.h>
#include <Esp32PcntEncoder.h>

Esp32PcntEncoder encoders[2];

void setup()
{
  // 1.初始化串口
  Serial.begin(115200);
  // 2.设置编码器
  encoders[0].init(32, 33, 0);
  encoders[1].init(26, 25, 1);
}

void loop()
{
  delay(10);
  Serial.printf("tick1=%d,tick2=%d\n", encoders[0].getTicks(), encoders[1].getTicks());
}