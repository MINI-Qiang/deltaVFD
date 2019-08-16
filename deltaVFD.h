#ifndef deltaVFD_h
#define deltaVFD_h


#if ARDUINO >= 100
#include "Arduino.h"
#include "VariableConversion.h"
#else
#include "WProgram.h"
#include "VariableConversion.h"
#endif

/* 模式选择
	SERIAL_5N1
	SERIAL_6N1
	SERIAL_7N1
	SERIAL_8N1 (the default)
	SERIAL_5N2
	SERIAL_6N2
	SERIAL_7N2
	SERIAL_8N2
	SERIAL_5E1
	SERIAL_6E1
	SERIAL_7E1
	SERIAL_8E1
	SERIAL_5E2
	SERIAL_6E2
	SERIAL_7E2
	SERIAL_8E2
	SERIAL_5O1
	SERIAL_6O1
	SERIAL_7O1
	SERIAL_8O1
	SERIAL_5O2
	SERIAL_6O2
	SERIAL_7O2
	*/


class deltaVFD
{
	public:
		deltaVFD(byte deviceID);
		void begin(HardwareSerial& SerialData );
		void SetF(uint16_t Frequency);
		void run(bool direction);
		void stop();
		
		uint16_t readDate(uint16_t DateAddr);   //读取数据请求
		void writeCoil(uint16_t DateAddr,uint16_t Date);   // 写线圈
		void writeDate(uint16_t DateAddr,uint16_t Date);    //写MODUBS
		void writeDate(uint16_t DateAddr,uint16_t AddrNum,uint16_t *Date);
		
	private:
		byte DeviceID;
		HardwareSerial *SerialID;
		
};


#endif