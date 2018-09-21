# MW-AHRSv1-RS232-communicate
MW-AHRSv1 about RS232 communicate on qt c++, ros kinetic

### 소개
안녕하세요. MW-AHRSv1 imu센서를 RS232 텍스트로 받아오는 것을 해보았습니다. 추가로 ROS에서 메세지를 보내는 것 까지 확인하였습니다.
웹상에서 RS232를 통해서 데이터 값을 받아오는 자료가 많이 없어서 제 깃 허브에 공유합니다.
코드는 QT로 C++언어로 구현했고 Ubuntu 16.04, ROS Kinetic버전입니다.

운영체제 : 우분투 16.04
소프트웨어 : ROS Kinetic
컴파일러 : QT

## 구성
총 두 개의 파일로 나뉘어져 있습니다.
1. mw_ahrsv1.cpp (헤더파일 x)
2. Serial.cpp (헤더파일 o)

mw_ahrsv1.cpp 파일은 MW_AHRS클래스가 있는 파일이고 통신을 하는 기본적인 코드가 구현이 되어있습니다.
Serial.cpp 파일은 C코드로 구현된 시리얼 통신을 할 수 있는 기본적인 코드가 구현이 되어있습니다.

### ROS 관련 내용
노드 이름은 my_ahrsv1이고, 패키지 이름 또한 my_ahrsv1으로 같습니다.
실행하실 때 roscore를 먼저 켜주시고, rosrun my_ahrsv1 my_ahrsv1 명령어를 통해서 노드를 켜주시길 바랍니다.


### Introduce
Good morning. I tried to import the MW-AHRSv1 imu sensor as RS232 text. We have also confirmed that we are sending messages from ROS.
There is not much data to receive data value through RS232 on the web, so it is shared with the github.
The code is implemented in the C ++ language as QT and is Ubuntu 16.04, ROS Kinetic version.

Operating System: Ubuntu 16.04
Software: ROS Kinetic
Compiler: QT

## Configuration
It is divided into two files in total.
1. mw_ahrsv1.cpp (header file x)
2. Serial.cpp (header file o)

The mw_ahrsv1.cpp file is a file with the MW_AHRS class and has implemented basic code for communication.
The serial.cpp file is implemented as basic code that enables serial communication implemented with C code.

### About ROS
The node name is my_ahrsv1, and the package name is also equal to my_ahrsv1.
Please turn on roscore first and rosrun my_ahrsv1 my_ahrsv1 command.


## mw_ahrsv1.cpp
```c++
#include "ros/ros.h"
#include "Serial.h"
#include <iostream>
#include <../include/mw_ahrsv1/imu.h>


typedef struct
{
    // Euler
    float roll;
    float pitch;
    float yaw;

}Euler;

class MW_AHRS
{

public:
    MW_AHRS()
    {
        for(int i=0; i<100; i++)
            buffer[i] = 0;

        dev = open_serial((char*)"/dev/ttyUSB0", 115200, 0, 0);

        Tx[0] = A;     // a
        Tx[1] = N;     // n
        Tx[2] = G;     // g
        Tx[3] = CR;     // CR
        Tx[4] = LF;     // LF
    }
    ~MW_AHRS()
    {
        close_serial(dev);
    }

    Euler get_data(void)
    {
        write(dev,Tx,5);
        read(dev, &buffer, 100);

        if(buffer[0] == 'a' && buffer[1] == 'n' && buffer[2] == 'g')
        {
            char *ptr = strtok(buffer, " ");

            ang_count=0;

            while(ptr != NULL)
            {
                ang_count++;

                ptr = strtok(NULL, " ");

                if(ang_count == 1)
                {
                    euler.roll = atof(ptr);
                }
                else if(ang_count == 2)
                {
                    euler.pitch = atof(ptr);
                }
                else if(ang_count == 3)
                {
                    euler.yaw = atof(ptr);
                }

            }
        }

        std::cout << "roll = " << euler.roll << std::endl;
        std::cout << "pitch = " << euler.pitch << std::endl;
        std::cout << "yaw = " << euler.yaw << std::endl;
        std::cout << std::endl;

        return euler;

    }


private:
    // Device Name
    int dev = 0;

    Euler euler;

    // Data buffer
    char buffer[100];
    unsigned char Tx[5];

    // Serperate Euler Angle Variable
    int ang_count = 0;

    // ASCII CODE
    const unsigned char A = 0x61;
    const unsigned char N = 0x6E;
    const unsigned char G = 0x67;
    const unsigned char CR = 0x0D;
    const unsigned char LF = 0x0A;

};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "mw_ahrsv1");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);

    ros::Publisher imu_pub = nh.advertise<mw_ahrsv1::imu>("imu", 100);
    mw_ahrsv1::imu msg;

    MW_AHRS ahrs_obj;
    Euler euler;

    while (ros::ok())
    {
        euler = ahrs_obj.get_data();


        msg.roll = euler.roll;
        msg.pitch = euler.pitch;
        msg.yaw = euler.yaw;

        imu_pub.publish(msg);

        loop_rate.sleep();

    }


    return 0;
}
```

## Serial.cpp
```c++
/*
 * Serial.cpp
 *
 *  Created on: Aug 12, 2015
 *      Author: odroid
 */
#include "Serial.h"
#include <iostream>
//#include <fstream>
#include <termios.h>
#include <fcntl.h>
using namespace std;

int open_serial(char *dev_name, int baud, int vtime, int vmin)
{
    int fd;
    struct termios newtio;

    //open serial port
    fd = open(dev_name, O_RDWR | O_NOCTTY);
    //cout << "hahah" << endl;
    if(fd < 0)
    {
        //fail open
    	cout << "fail" << endl;
        printf("fail open");
        return -1;
    }

    // port configure
    memset(&newtio, 0, sizeof(newtio));
    newtio.c_iflag = IGNPAR;    // no-parity
    newtio.c_oflag = 0;

    newtio.c_cflag = CS8 | CLOCAL | CREAD;  // no-rts & no-cts

    switch(baud)
    {
		case 500000 : newtio.c_cflag |= B500000; break;
		case 250000 : newtio.c_cflag |= B230400; break;
		case 115200 : newtio.c_cflag |= B115200; break;
		case 57600  : newtio.c_cflag |= B57600; break;
		case 9600   : newtio.c_cflag |= B9600; break;
		default     : newtio.c_cflag |= B115200; break;
    }

    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = vtime;     // timeout 0.1s
    newtio.c_cc[VMIN] = vmin;       // wait

    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW, &newtio);

    return fd;
}
void close_serial(int fd)
{
    close(fd);
}
void sendMessage(char* text, int cnt, int fd)
{
    char buf[128]={0};
   for(int i = 0; i < cnt; i ++)
    {
        memset(buf,*(text+i),1);
        write(fd, buf, 1);
    }
}





```

## Serial.h

```c++
/*
 * Serial.h
 *
 *  Created on: Aug 12, 2015
 *      Author: odroid
 */

#ifndef SERIAL_H_
#define SERIAL_H_

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <string.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/signal.h>
#include <sys/ioctl.h>
#include <sys/poll.h>

int open_serial(char *dev_name, int baud, int vtime, int vmin);
void close_serial(int fd);
void sendMessage(char* text, int cnt, int fd);


class xSerial
{
private:

public:
    int open_serial(char *dev_name, int baud, int vtime, int vmin);
    void close_serial(int fd);
    void sendMessage(char* text, int cnt, int fd);

};

#endif /* SERIAL_H_ */
```



