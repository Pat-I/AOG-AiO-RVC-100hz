#ifndef _WT901_H_
#define _WT901_H_
#include <Arduino.h>
#include <string.h>

class CWT901
{
  public:
	CWT901::CWT901()
{
	lastTime = millis();
}

void CWT901::attach(Stream & Serial_temp)
{
	Serial_ = &Serial_temp;
}

bool CWT901::readSerialData(uint8_t data)
{
	rxBuffer[rxCnt] = data;
	rxCnt++;
	if (rxBuffer[0] != 0x55) { 
		rxCnt = 0;
		return false;
	}
	if (rxCnt<11) {
		return false;
	}
	rxCnt = 0;  
	uint8_t sum = 0;
	for (uint8_t cnt = 0; cnt<10; cnt++) {
		sum += rxBuffer[cnt];
	}
	if (sum != rxBuffer[10]) {
		return false;
	}
	switch (rxBuffer[1])
	{
		case 0x51:  memcpy(&WT901_data.acc,     &rxBuffer[2], 8); break;
		case 0x52:  memcpy(&WT901_data.gyro,    &rxBuffer[2], 8); break;
		case 0x53:  memcpy(&WT901_data.angle,   &rxBuffer[2], 8); break;
	}
	lastTime = millis();
	return true;
}

bool CWT901::receiveSerialData(void)
{
	bool status = false;
	while (Serial_->available()) {
		status = CWT901::readSerialData(Serial_->read());
	}
	return status;
}

double CWT901::getAccX()
{
	return WT901_data.acc.x / (32768.0/16.0);
}

double CWT901::getAccY()
{
	return WT901_data.acc.y / (32768.0/16.0);
}

double CWT901::getAccZ()
{
	return WT901_data.acc.z / (32768.0/16.0);
}

double CWT901::getGyroX()
{
	return WT901_data.gyro.x / (32768.0/2000.0);
}

double CWT901::getGyroY()
{
	return WT901_data.gyro.y / (32768.0/2000.0);
}

double CWT901::getGyroZ()
{
	return WT901_data.gyro.z / (32768.0/2000.0);
}

int16_t CWT901::getAccRawX()
{
	return WT901_data.acc.x;
}

int16_t CWT901::getAccRawY()
{
	return WT901_data.acc.y;
}

int16_t CWT901::getAccRawZ()
{
	return WT901_data.acc.z;
}

int16_t CWT901::getGyroRawX()
{
	return WT901_data.gyro.x;
}

int16_t CWT901::getGyroRawY()
{
	return WT901_data.gyro.y;
}

int16_t CWT901::getGyroRawZ()
{
	return WT901_data.gyro.z;
}

double CWT901::getRoll()
{
	return WT901_data.angle.roll / (32768.0/180.0);
}

double CWT901::getPitch()
{
	return WT901_data.angle.pitch / (32768.0/180.0);
}
double CWT901::getYaw()
{
	return WT901_data.angle.yaw / (32768.0/180.0);
}

unsigned long CWT901::getLastTime(void)
{
	return lastTime;
}



  private:
	Stream * Serial_ = NULL;
	unsigned long lastTime;
	uint8_t rxBuffer[12]={0};
	uint8_t rxCnt = 0;
	struct
	{
		struct
		{
			int16_t x;
			int16_t y;
			int16_t z;
			int16_t temperature;
		}acc;
		struct
		{
			int16_t x;
			int16_t y;
			int16_t z;
			int16_t temperature;
		}gyro;
		struct
		{
			int16_t roll;
			int16_t pitch;
			int16_t yaw;
			int16_t temperature;
		}angle;
	}WT901_data;
};
extern CWT901 WT901;
#endif
