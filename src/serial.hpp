/**
 * @file serial.hpp
 * @author JSon (junwooson@postech.ac.kr)
 * @brief This is a modified version of the serial communication library for RoboClaw Motor Controller.
 * @version 1.0
 * @date 2018-10-22
 * 
 * @copyright Copyright (c) 2018. Junwoo Son. All rights reserved. All contents cannot be copied without permission.
 */


#ifndef SERIAL_H
#define SERIAL_H

#include <iostream>
#include <memory>

#include <unistd.h>
#include <termios.h>
#include <errno.h>
#include <stdint.h>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <pthread.h> // This uses POSIX Threads

#include <sys/select.h>
#include <sys/time.h>

typedef int			serialHandle_t;
typedef speed_t		serialSpeed_t;


//---- DUE TEST
#define char_data 0
#define long_data 1
#define float_data 2

typedef struct {
  void * DataPtr;
  void * DataPtr2;
  void * DataPtr3;
  void * DataPtr4;
  uint8_t data[256];
  unsigned long indx;
} ToHost;

typedef struct {
	float Motor1;
	float Motor2;
} Motor_data;

typedef struct {
	float Enc1_theta;
	float Enc1_dot_theta;
	float Enc2_theta;
	float Enc2_dot_theta;
} Encoder_data;

class Serial
{
public:
	Serial(int portNum, int baudRate, int byteSize = 8, int parityBits = 0, int stopBits = 1);
	~Serial();
	
	

	serialHandle_t	handle;
	ToHost toSimulink;

	bool			isOpen;
	int				portNum;
	int				baudRate;
	int				byteSize;
	int				parityBits;
	int				stopBits;

	int				portOpen(void);
	int				portOpen_arduino(void);
	int				portClose(void);

    serialSpeed_t	getBaudRate(int baudRate);

	int				bytesToRead(void);

	int				readBytes(unsigned char *pDst, int length);
	int				writeBytes(unsigned char *pSrc, int length);

	int				flush(void);


private:
	pthread_mutex_t  lock_;
};

#endif // !SERIAL_H
