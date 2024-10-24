#ifndef SYSTEM_CRC_TEST_HPP
#define SYSTEM_CRC_TEST_HPP

class SystemCRCTest
{
public:
	SystemCRCTest();
	virtual ~SystemCRCTest() {};

	//calculate CRC16 of device's reply
	unsigned int calculateCRC16Test(const char* reply, int replyLength) const;

private:
	unsigned int calcValues(unsigned int crc, int data) const;

	static unsigned int crcTable_[256];
};
#endif // SYSTEM_CRC_TEST_HPP