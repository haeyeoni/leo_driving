/**
 * @file serial.cpp
 * @author JSon (junwooson@postech.ac.kr)
 * @brief This is a modified version of the serial communication library for RoboClaw Motor Controller.
 * @version 1.0
 * @date 2018-10-22
 * 
 * @copyright Copyright (c) 2018. Junwoo Son. All rights reserved. All contents cannot be copied without permission.
 */

#include "serial.hpp"

#include <iostream>

using namespace std;


Serial::Serial(int portNum, int baudRate, int byteSize, int parityBits, int stopBits)
{
	this->portNum = portNum;
	this->baudRate = baudRate;
	this->byteSize = byteSize;
	this->parityBits = parityBits;
	this->stopBits = stopBits;

	this->isOpen = false;
	this->handle = 0;

	// Start mutex
	int result = pthread_mutex_init(&lock_, NULL);
	if ( result != 0 )
	{
//    		LOG(WARNING) << this->portNum << " pn mutex init failed";
		cout << this->portNum << " pn mutex init failed"<<endl;
	}
}

Serial::~Serial()
{
	this->portClose();
	// destroy mutex
	pthread_mutex_destroy(&lock_);
}

int Serial::portOpen(void)
{
	if (this->isOpen == true)
	{
		return (-1);    // The port is already open
	}

	char fileName[32];
	if (this->portNum == 0) sprintf(fileName, "/dev/ttyACM0");
	else sprintf(fileName, "/dev/ttyACM%d", this->portNum);
	//if (this->portNum == 0) sprintf(fileName, "/dev/ttyACM0");
	//if (this->portNum == 0) sprintf(fileName, "/dev/ttyS0");
	//if (this->portNum == 0) sprintf(fileName, "/dev/ttyUSB0");
	//else sprintf(fileName, "/dev/ttyTHS%d", this->portNum);
	
	this->handle = open(fileName, O_RDWR | O_NOCTTY | O_SYNC);// | O_NONBLOCK);

	if (this->handle < 0)
	{
		return (-2);    // Fail to open the handle
	}

  struct termios tty;
  if (tcgetattr(this->handle, &tty) < 0)
  {
      return (-3);    // Fail to get the tty
  }
  if (cfsetospeed(&tty, this->getBaudRate(this->baudRate)) != 0)
  {
      return (-4);    // Fail to set the tty
  }
  if (cfsetispeed(&tty, this->getBaudRate(this->baudRate)) != 0)
  {
      return (-4);    // Fail to set the tty
  }

  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
  tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
  tty.c_oflag &= ~OPOST;

  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(this->handle, TCSANOW, &tty) != 0)
  {
      return (-4);    // Fail to set the tty
  }

	this->isOpen = true;
	return 0;
}
int Serial::portOpen_arduino(void)
{
	if (this->isOpen == true)
	{
		return (-1);    // The port is already open
	}

	char fileName[32];
	//if (this->portNum == 0) sprintf(fileName, "/dev/ttyS0");
	if (this->portNum == 0) sprintf(fileName, "/dev/ttyACM0");
	else sprintf(fileName, "/dev/ttyACM0%d", this->portNum);
	
	this->handle = open(fileName, O_RDWR | O_NOCTTY | O_SYNC);// | O_NONBLOCK);

	if (this->handle < 0)
	{
		return (-2);    // Fail to open the handle
	}

  struct termios tty;
  if (tcgetattr(this->handle, &tty) < 0)
  {
      return (-3);    // Fail to get the tty
  }
  if (cfsetospeed(&tty, this->getBaudRate(this->baudRate)) != 0)
  {
      return (-4);    // Fail to set the tty
  }
  if (cfsetispeed(&tty, this->getBaudRate(this->baudRate)) != 0)
  {
      return (-4);    // Fail to set the tty
  }

  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
  tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
  tty.c_oflag &= ~OPOST;

  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(this->handle, TCSANOW, &tty) != 0)
  {
      return (-4);    // Fail to set the tty
  }

	this->isOpen = true;
	return 0;
}

serialSpeed_t Serial::getBaudRate(int baudRate)
{
    switch (baudRate)
    {
        default:         return (-1);
        case 9600:      return B9600;
				case 38400:    	 return B38400;
				case 57600:    	 return B57600;
        case 115200:    return B115200;
        case 230400:    return B230400;
        case 460800:    return B460800;
        case 921600:    return B921600;
    }
}

int Serial::portClose(void)
{
	if (this->isOpen == false)
	{
		return (-1);    // The port is already closed
	}

	close(this->handle);
	this->isOpen = false;
	return 0;
}

int Serial::bytesToRead(void)
{
	if (this->isOpen == false)
	{
		return (-1);    // The port is not open
	}

	int bytes;
	if (ioctl(this->handle, FIONREAD, &bytes) == 0) {
		return bytes;
	}
	else {
//		LOG(WARNING) << this->portNum <<  " pn Error in ioctl (bytesToRead), errno: " << errno;
//		printf("%d pn Error in ioctl (bytesToRead), errno: %d",this->portNum,errno);
		cout << this->portNum <<  " pn Error in ioctl (bytesToRead), errno: " << errno<<endl;

		return 0;
	}
}

int Serial::readBytes(unsigned char *dst, int length)
{

	if (this->isOpen == false)
	{
//		LOG(WARNING) << this->portNum << " pn not opend (readBytes)";
		cout << this->portNum << " pn not opend (readBytes)"<<endl;
		return (-1);    // The port is not open
	}	
	// Lock
//	pthread_mutex_lock(&lock_);
	
	int bytes, numberOfBytesRead = 0;
	if (ioctl(this->handle, FIONREAD, &bytes) != 0)
	{
		//TODO - check this isOpen=false is right???
		this->isOpen=false;//test
	//this->portNum=MAXnum_portNum+1;
		//printf("Error in ioctl, %d\n", errno);
	//		LOG(WARNING) << this->portNum << " pn Error in ioctl (readBytes), errno: " << errno;
		cout << this->portNum << " pn Error in ioctl (readBytes), errno: " << errno<<endl;

		return 0;
	}

	if (bytes == 0)
	{
		// bytes can be read 0
		//printf("bytes is 0\n");
		//TODO - check why bytes is zero???
    //LOG_EVERY_N(WARNING, 1) << this->portNum << " pn bytes is 0 after ioctl (readBytes)";
//		LOG_EVERY_N(WARNING, 1500) << this->portNum << " pn bytes is 0 after ioctl (readBytes)";
//		cout << this->portNum << " pn bytes is 0 after ioctl (readBytes)"<<endl;//TODO JIN on
		
		//TODO - ***check return 0 is true?***
		return 0;
	}

	if (bytes > length)
	{
		bytes = length;
	}
  //LOG_EVERY_N(INFO, 1) << this->portNum << " pn bytes: " << bytes << " length: " << length << " (readBytes)";
  
  // select function
	// example code Blocked until message arrived (select).
	//fd_set read_fs;
	//FD_ZERO(&read_fs);
	//FD_SET(this->handle, &read_fs);
	//select(this->handle+1, &read_fs, NULL, NULL, NULL);
	fd_set set;
	FD_ZERO(&set);
	FD_CLR(this->handle, &set);
	FD_SET(this->handle, &set);
	struct timeval tm;
	tm.tv_sec = 0;
	tm.tv_usec = 1000;
	// select has three state 1, 0, -1
	// 1 means available, 0 means timeout, -1 means cannot access
	if (int state = select(this->handle + 1, &set, NULL, NULL, &tm) > 0) { 
		if (FD_ISSET(this->handle, &set)) {
			//LOG(INFO) << "from motor (" << this->portNum << ") before_read";
	  	numberOfBytesRead = read(this->handle, dst, bytes);
			//LOG(INFO) << "from motor (" << this->portNum << ") after_read";
		}
		else {
//			LOG(INFO) << "from motor (" << this->portNum << ") read FD_ISSET not pass";
			cout << "from motor (" << this->portNum << ") read FD_ISSET not pass"<<endl;
		}
	}
	else {
//    LOG_EVERY_N(WARNING, 500) << this->portNum <<  " pn select state (readBytes) state:" << state << " errno: " << errno;
//		LOG_EVERY_N(WARNING, 500) << "above logging EVERY_500";
	cout << this->portNum <<  " pn select state (readBytes) state:" << state << " errno: " << errno<<endl;


	}

  //LOG_EVERY_N(INFO, 1) << this->portNum << " nOBR: " << numberOfBytesRead;
	//printf("numberOfBytesRead: %d\n", numberOfBytesRead);
	//	printf("read %d bytes:  %s\n", bytes, dst);
	// Unlock
	//	pthread_mutex_unlock(&lock_);
	
	if(numberOfBytesRead < 0){
		//printf("Error in read, %d\n", errno);
//    LOG_EVERY_N(WARNING, 500) << this->portNum << " pn numberOfBytesRead < 0 (readBytes)";
//		LOG_EVERY_N(WARNING, 500) << "above logging EVERY_500";
		cout << this->portNum << " pn numberOfBytesRead < 0 (readBytes)"<<endl;
	}
  return numberOfBytesRead;
}

int Serial::writeBytes(unsigned char *src, int length)
{
	if (this->isOpen == false)
	{
		return (-1);    // The port is not open
	}

	int nwrites = -1;
	fd_set setwrite;
	FD_ZERO(&setwrite);
  	FD_CLR(this->handle, &setwrite);
	FD_SET(this->handle, &setwrite);
	struct timeval tm;
	tm.tv_sec = 0;
	tm.tv_usec = 1000;
	if (int state = select(this->handle + 1, NULL, &setwrite, NULL, &tm) > 0) { 
		if (FD_ISSET(this->handle, &setwrite)) {
			//	pthread_mutex_lock(&lock_);
			//LOG(INFO) << "from motor (" << this->portNum << ") before_write";
			nwrites = write(this->handle, src, length);
			//LOG(INFO) << "from motor (" << this->portNum << ") after_write: nwrites: " << nwrites;
			// Wait until all data has been written
			//	tcdrain(this->handle);
			//	pthread_mutex_unlock(&lock_);	
		}
		else {
//			LOG_EVERY_N(WARNING, 500) << "from motor (" << this->portNum << ") write FD_ISSET not pass";
			cout << "from motor (" << this->portNum << ") write FD_ISSET not pass"<<endl;
		}
	}
	else {
	//    LOG_EVERY_N(WARNING, 500) << this->portNum <<  " pn select state (writeBytes) state:" << state << " errno: " << errno;
		cout << this->portNum <<  " pn select state (writeBytes) state:" << state << " errno: " << errno<<endl;
		
	}
	
	return nwrites;
}



int Serial::flush(void)
{
	if (this->isOpen == false)
	{
		return (-1);    // The port is not open
	}

	tcflush(this->handle, TCIOFLUSH);
	return 0;
}
