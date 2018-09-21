# MW-AHRSv1---RS232-communicate
MW-AHRSv1 about RS232 communicate on qt c++, ros kinetic

안녕하세요. MW-AHRSv1 imu센서를 RS232 텍스트로 받아오는 것을 해보았습니다. 추가로 ROS에서 메세지를 보내는 것 까지 확인하였습니다.
웹상에서 RS232를 통해서 데이터 값을 받아오는 자료가 많이 없어서 제 깃 허브에 공유합니다.
코드는 QT로 C++언어로 구현했고 Ubuntu 16.04, ROS Kinetic버전입니다.

Hello. MW-AHRSv1 imu sensor to RS232 Text-to-speech and hacker. More information is available through the ROS.
Use a hub to obtain data using RS232 on the web.
Ubuntu 16.04, which implements the C ++ language with QT, is the ROS kinetic version.

운영체제 : 우분투 16.04
소프트웨어 : ROS Kinetic
컴파일러 : QT

Operating System: Ubuntu 16.04
Software: ROS Kinetic
Compiler: QT

총 두 개의 파일로 나뉘어져 있습니다.
1. mw_ahrsv1.cpp (헤더파일 x)
2. Serial.cpp (헤더파일 o)

mw_ahrsv1.cpp 파일은 MW_AHRS클래스가 있는 파일이고 통신을 하는 기본적인 코드가 구현이 되어있습니다.
Serial.cpp 파일은 C코드로 구현된 시리얼 통신을 할 수 있는 기본적인 코드가 구현이 되어있습니다.
