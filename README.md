# Esp32McpwmMotor
EncoderWith ESP32 PCNT.

Example Encoder With ESP32 PCNT.

```
#include <Arduino.h>
#include <Esp32McpwmMotor.h>

Esp32McpwmMotor motor;

void setup()
{
  Serial.begin(115200);
  motor.attachMotors(22, 23, 12, 13);
}

void loop()
{
  motor.updateMotorSpeed(0, 50);
  motor.updateMotorSpeed(1, -50);
}
```