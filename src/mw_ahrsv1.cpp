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



