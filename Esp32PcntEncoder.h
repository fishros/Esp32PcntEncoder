#ifndef __ESP32PCNTENCODER_H__
#define __ESP32PCNTENCODER_H__
#include "Arduino.h"
#include <driver/pcnt.h>
#include <hal/pcnt_hal.h>
#include <soc/pcnt_struct.h>

class Esp32PcntEncoder
{
public:
	Esp32PcntEncoder() = default;
	bool init(int pcntUnit, int pinA, int pinB);
	bool setGlitchFilter(uint32_t max_glitch_us);
	bool start();
	bool reset();
	bool stop();

	int32_t getTicks();
	// 速度计算
	bool setPeerPcntDistance(float peerPcntDistance);
	float getSpeed();

private:
	int _pinA;
	int _pinB;
	int32_t _ticks{0};
	float _peerPcntDistance{0.1};
	uint32_t _lastUpdateSpeedTime;
	int32_t _lastUpdateSpeedTick;

public:
	pcnt_unit_t pcntUnit;
	int accumu_count{0};
};

#endif // __ESP32PCNTENCODER_H__
