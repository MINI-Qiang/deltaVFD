#include "deltaVFD.h"

//CRC校验
unsigned int crc_chk(unsigned char* data, unsigned char length) 
{ 
  int j;  
  unsigned int reg_crc = 0xFFFF;   
  while (length--) 
  { 
    reg_crc ^= *data++;
    for (j = 0;j<8;j++) 
    { 
      if (reg_crc & 0x01) 
      {  /* LSB(b0)=1 */         
        reg_crc = (reg_crc >> 1) ^ 0xA001; 
      } 
      else 
      { 
        reg_crc = reg_crc >> 1; 
      } 
    } 
  }   
  return reg_crc; 
}


deltaVFD::deltaVFD(byte deviceID)
{
	DeviceID = deviceID;
}

//初始化modbus串口
void deltaVFD::begin(HardwareSerial& SerialData)
{
	//SerialID->begin(portRate,_Mode);
	SerialID = &SerialData; 

}


//设置变频器频率

void deltaVFD::SetF(uint16_t Frequency)
{
	writeDate(0x2001, Frequency);    //设置运行频率
	delay(50);
}


void deltaVFD::run(bool direction)
{
	if (direction == 0)
	{
		writeDate(0x2000, 0x12);  //地址0x2000 ，指令0x12 正向运行
		delay(50);
	}
	else
	{
		writeDate(0x2000, 0x22);  //地址0x2000 ，指令0x22 反向运行
		delay(50);
	}
	
}


void deltaVFD::stop()
{
	writeDate(0x2000, 0x01);  //地址0x2000 ，停机指令 0x01
	delay(50);
}







//私有函数部分



uint16_t deltaVFD::readDate(uint16_t DateAddr)  //地址，数据地址，数据长度，
{
  byte DateBuff[8];  //改成10

  DateBuff[0] = DeviceID;  //设备地址
  DateBuff[1] = 0x03;   //操作功能（读取）
  /*拆数据地址*/
  byte tempDateAddr[2];
  UintToByte(DateAddr, tempDateAddr);
  DateBuff[2] = tempDateAddr[1];
  DateBuff[3] = tempDateAddr[0];
  /*拆数据长度*/
  byte tempDate[2];
  UintToByte(0x01, tempDate);
  DateBuff[4] = tempDate[1];
  DateBuff[5] = tempDate[0];
  /*拆CRC校验*/
  uint16_t crc = crc_chk(DateBuff, 6);
  byte tempcrc[2];
  UintToByte(crc, tempcrc);
  DateBuff[6] = tempcrc[0];
  DateBuff[7] = tempcrc[1];
  /*串口发送指令*/
  SerialID->write(DateBuff, 8);
  delay(11);  //手册要求发送数据后要保持10ms以上时间作为结束消息

  /*串口监听，等待消息*/
  uint32_t Times = millis();  //记录进入函数的时间
  char SerialBuff[64];
  while (1)  //首先用死循环锁死函数
  {
    if (SerialID->available() > 0) //检查串口是否有数据，并且缓冲区是否可用
    {
      delay(50);  //等待串口消息结束
      byte SeriaDataLen = SerialID->available();
      for (int a = 0; a < SeriaDataLen; a++)
      {
        SerialBuff[a] =  SerialID->read();
      }

      byte tempByte[2];
      uint16_t Date;
      tempByte[0] = SerialBuff[4];
      tempByte[1] = SerialBuff[3];
      ByteToUint(Date,tempByte);
      return Date;     //返回收到的数据
    }
    //超时处理
    uint32_t ThisTime = millis();   //记录当前时间，判断是否超时，超时则跳出循环
    if ((ThisTime - Times) >= 1000)
    {
      break;
    }
    else if (ThisTime - Times < 0)  //防止溢出
    {
      break;  //数据溢出依旧跳出循环
    }
  }
}

void deltaVFD::writeCoil(uint16_t DateAddr,uint16_t Date)
{
	byte DateBuff[8];
	DateBuff[0] = DeviceID;  //设备地址
	DateBuff[1] = 0x05;   //操作功能（线圈写入）
	/*拆数据地址*/
	byte tempDateAddr[2];
	UintToByte(DateAddr,tempDateAddr);
	DateBuff[2] = tempDateAddr[1];
	DateBuff[3] = tempDateAddr[0];
	/*拆数据内容*/
	byte tempDate[2];
	UintToByte(Date,tempDate);
	DateBuff[4] = tempDate[1];
	DateBuff[5] = tempDate[0];
	/*拆CRC校验*/
	uint16_t crc = crc_chk(DateBuff,6);
	byte tempcrc[2];
	UintToByte(crc,tempcrc);
	DateBuff[6] = tempcrc[0];
	DateBuff[7] = tempcrc[1];
	/*串口发送指令*/
	SerialID->write(DateBuff,8);
	delay(11);  //手册要求发送数据后要保持10ms以上时间作为结束消息 
	while(SerialID->read() >= 0);  //清空缓冲区
	
}




void deltaVFD::writeDate(uint16_t DateAddr,uint16_t Date)
{
	byte DateBuff[8];
	
	DateBuff[0] = DeviceID;  //设备地址
	DateBuff[1] = 0x06;   //操作功能（写入）
	/*拆数据地址*/
	byte tempDateAddr[2];
	UintToByte(DateAddr,tempDateAddr);
	DateBuff[2] = tempDateAddr[1];
	DateBuff[3] = tempDateAddr[0];
	/*拆数据内容*/
	byte tempDate[2];
	UintToByte(Date,tempDate);
	DateBuff[4] = tempDate[1];
	DateBuff[5] = tempDate[0];
	/*拆CRC校验*/
	uint16_t crc = crc_chk(DateBuff,6);
	byte tempcrc[2];
	UintToByte(crc,tempcrc);
	DateBuff[6] = tempcrc[0];
	DateBuff[7] = tempcrc[1];
	/*串口发送指令*/
	SerialID->write(DateBuff,8);
	delay(11);  //手册要求发送数据后要保持10ms以上时间作为结束消息 
	while(SerialID->read() >= 0);  //清空缓冲区
}



void deltaVFD::writeDate(uint16_t DateAddr,uint16_t AddrNum,uint16_t *Date)
{
	byte DateBuff[12];
	
	DateBuff[0] = DeviceID;  //设备地址
	DateBuff[1] = 0x10;   //操作功能（写入）
	/*拆数据地址*/
	byte tempDateAddr[2];
	UintToByte(DateAddr,tempDateAddr);
	DateBuff[2] = tempDateAddr[1];
	DateBuff[3] = tempDateAddr[0];
	//数据长度
	byte tempAddrNum[2];
	UintToByte(AddrNum,tempAddrNum);
	DateBuff[4] = tempAddrNum[0];
	DateBuff[5] = tempAddrNum[1];
	
	
	/*拆数据内容*/
	byte tempDate[2];
	UintToByte(Date[0],tempDate);
	DateBuff[6] = tempDate[1];
	DateBuff[7] = tempDate[0];
	
	UintToByte(Date[1],tempDate);
	DateBuff[8] = tempDate[1];
	DateBuff[9] = tempDate[0];
	
	/*拆CRC校验*/
	uint16_t crc = crc_chk(DateBuff,6);
	byte tempcrc[2];
	UintToByte(crc,tempcrc);
	DateBuff[10] = tempcrc[0];
	DateBuff[11] = tempcrc[1];
	/*串口发送指令*/
	SerialID->write(DateBuff,12);
	delay(11);  //手册要求发送数据后要保持10ms以上时间作为结束消息 
	while(SerialID->read() >= 0);  //清空缓冲区
}