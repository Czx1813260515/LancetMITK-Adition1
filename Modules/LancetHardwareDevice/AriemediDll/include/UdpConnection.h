#ifndef UDP_CONNECTION_HPP
#define UDP_CONNECTION_HPP

#include <string>
#include <iostream>
#include <ws2tcpip.h>
#include <vector>

#pragma comment(lib,"ws2_32.lib")
#pragma comment(lib, "iphlpapi.lib")

#include <stdint.h>
typedef uint8_t byte_t;

#define DEVICE_PORT 8080
#define SIO_UDP_CONNRESET _WSAIOW(IOC_VENDOR,12)

class UdpConnection
{
public:
	UdpConnection(std::string hostname, int port, bool errorPrint);
	~UdpConnection();

	//read and write information from device
	int read(byte_t* buffer, byte_t* buffer2, int length, int& port) const;
	int write(byte_t* buffer, int length) const;

	//clean buffer
	void cleanBuff();

	//get connection status
	int isConnected() const;

	//close UDP connection
	int closeUDP();

	//get connection IP
	std::vector<std::string> getConnectionIP();

	enum ConnectionError
	{
		nonError = 0,
		resloveError = -1,
		initialUDPError = -2,
		portConflicts = -3,
		invalidAddr = -4,
		deviceInexist = -5,
		unknownError = -10,
	};

private:
	int connectionInfo;
	
	SOCKET deviceSocket;
	sockaddr_in addrSrv;
	
	std::string deviceIP;
	std::string localIP;
	int portNumber;

	bool errorShow;

	//initialize UDP connection
	int init(std::string LocalIP, std::string DeviceIP);

	//get device and local IP
	std::string getDevicIP(std::string hostname);
	std::string getLocalIP(std::string DeviceIPv4);
	int getInfo(std::string text, std::vector<std::string>& Info);
};
#endif
#pragma once