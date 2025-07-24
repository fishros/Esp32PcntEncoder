#include "Arduino.h"
#include <Esp32PcntEncoder.h>

static const char *TAG = "rotary_encoder";
#define EC11_PCNT_DEFAULT_HIGH_LIMIT (100)
#define EC11_PCNT_DEFAULT_LOW_LIMIT (-100)
#define ROTARY_CHECK(a, msg, tag, ret, ...)                                       \
	do                                                                            \
	{                                                                             \
		if (unlikely(!(a)))                                                       \
		{                                                                         \
			ESP_LOGE(TAG, "%s(%d): " msg, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
			return false;                                                         \
		}                                                                         \
	} while (0)

static void pcnt_overflow_handler(void *arg)
{
	Esp32PcntEncoder *encoder = (Esp32PcntEncoder *)arg;
	uint32_t status = 0;
	pcnt_get_event_status(encoder->pcntUnit, &status);

	if (status & PCNT_EVT_H_LIM)
	{
		encoder->accumu_count += EC11_PCNT_DEFAULT_HIGH_LIMIT;
	}
	else if (status & PCNT_EVT_L_LIM)
	{
		encoder->accumu_count += EC11_PCNT_DEFAULT_LOW_LIMIT;
	}
}

bool Esp32PcntEncoder::init( int _pcntUnit, int pinA, int pinB)
{
	this->_pinA = pinA;
	this->_pinB = pinB;
	this->pcntUnit = (pcnt_unit_t)_pcntUnit;
	// 1.set pin mode
	pinMode(_pinA, INPUT_PULLUP);
	pinMode(_pinB, INPUT_PULLUP);
	// 2.config pcnt
	pcnt_config_t dev_config = {
		.pulse_gpio_num = (gpio_num_t)_pinA,
		.ctrl_gpio_num = (gpio_num_t)_pinB,
		.lctrl_mode = PCNT_MODE_REVERSE,
		.hctrl_mode = PCNT_MODE_KEEP,
		.pos_mode = PCNT_COUNT_DEC,
		.neg_mode = PCNT_COUNT_INC,
		.counter_h_lim = EC11_PCNT_DEFAULT_HIGH_LIMIT,
		.counter_l_lim = EC11_PCNT_DEFAULT_LOW_LIMIT,
		.unit = pcntUnit,
		.channel = PCNT_CHANNEL_0,
	};
	ROTARY_CHECK(pcnt_unit_config(&dev_config) == ESP_OK, "config pcnt channel 0 failed", err, ESP_FAIL);

	dev_config.pulse_gpio_num = (gpio_num_t)_pinB;
	dev_config.ctrl_gpio_num = (gpio_num_t)_pinA;
	dev_config.channel = PCNT_CHANNEL_1;
	dev_config.pos_mode = PCNT_COUNT_INC;
	dev_config.neg_mode = PCNT_COUNT_DEC;
	ROTARY_CHECK(pcnt_unit_config(&dev_config) == ESP_OK, "config pcnt channel 0 failed", err, ESP_FAIL);

	// 3. PCNT pause and reset value
	pcnt_counter_pause(pcntUnit);
	pcnt_counter_clear(pcntUnit);

	// if(ec11->pcnt_unit==0)
	if (pcntUnit == PCNT_UNIT_0)
	{
		ROTARY_CHECK(pcnt_isr_service_install(0) == ESP_OK, "install isr service failed", err, ESP_FAIL);
	}

	pcnt_isr_handler_add(pcntUnit, pcnt_overflow_handler, this);

	pcnt_event_enable(pcntUnit, PCNT_EVT_H_LIM);
	pcnt_event_enable(pcntUnit, PCNT_EVT_L_LIM);

	setGlitchFilter(1000);
	start();

	return true;
}

bool Esp32PcntEncoder::setGlitchFilter(uint32_t max_glitch_us)
{
	ROTARY_CHECK(pcnt_set_filter_value(pcntUnit, max_glitch_us * 1) == ESP_OK, "set glitch filter failed", err, ESP_FAIL);
	if (max_glitch_us)
	{
		pcnt_filter_enable(pcntUnit);
	}
	else
	{
		pcnt_filter_disable(pcntUnit);
	}
	return true;
}
bool Esp32PcntEncoder::start()
{
	pcnt_counter_resume(pcntUnit);
	pcnt_counter_clear(pcntUnit);
	return true;
}

bool Esp32PcntEncoder::reset()
{
	_ticks = 0;
	return true;
}

bool Esp32PcntEncoder::stop()
{
	pcnt_counter_pause(pcntUnit);
	pcnt_counter_clear(pcntUnit);
	return true;
}

int32_t Esp32PcntEncoder::getTicks()
{
	int16_t val = 0;
	pcnt_get_counter_value(pcntUnit, &val);
	_ticks = val + accumu_count;
	return _ticks;
}

float Esp32PcntEncoder::getSpeed()
{
	uint64_t dt = micros() - _lastUpdateSpeedTime;
	float speed = (float)(getTicks() - _lastUpdateSpeedTick) * _peerPcntDistance / dt;
	_lastUpdateSpeedTick = getTicks();
	_lastUpdateSpeedTime = micros();
	return speed;
}

bool Esp32PcntEncoder::setPeerPcntDistance(float peerPcntDistance)
{
	_peerPcntDistance = peerPcntDistance * 1000 * 1000;
	return true;
}
