#ifndef _UDP_READER_HPP_
#define _UDP_READER_HPP_

#include "UdpConnection.h"

class UdpReader
{
public:
	UdpReader();
	UdpReader(UdpConnection* UdpCon);

	//reset UDP
	void resetUDP();

	//clear UDP buffer
	void clearBuffer();

	//analysis device's reply information
	int getBufferSize();
	std::string getData(size_t start, size_t length) const;
	int readBytes();
	void skipBytes(int numBytes);
	byte_t get_byte();
	uint16_t get_uint16();
	uint32_t get_uint32();
	double get_double();

	//device's reply
	std::byte buffer[1500];
	std::byte buffer2[1500];
	int port;

private:
	UdpConnection* UdpCon_;

	int buffersize = 1500;
	int currentIndex_ = 0;
};
#endif
#pragma once