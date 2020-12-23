/*
*  3iRoboticsLIDAR System II
*  Driver Interface
*
*  Copyright 2017 3iRobotics
*  All rights reserved.
*
*	Author: 3iRobotics, Data:2017-09-15
*
*/


#include "C3iroboticsLidar.h"
#include "CSerialConnection.h"
#include <unistd.h>
#include <time.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>


#define DEBUG_MAIN

#define DEG2RAD(x) ((x)*M_PI/180.)

typedef struct _rslidar_data
{
    _rslidar_data()
    {
        signal = 0;
        angle = 0.0;
        distance = 0.0;
    }
    uint8_t signal;
    float   angle;
    float   distance;
}RslidarDataComplete;

using namespace std;
using namespace everest::hwdrivers;


int main(int argc, char * argv[])
{
    int count = 0;
	int    opt_com_baudrate = 115200;//230400;
    string opt_com_path = "/dev/ttyS5";

    CSerialConnection serial_connect;
    C3iroboticsLidar robotics_lidar;

    robotics_lidar.PwmInit();//PWM init

    serial_connect.setBaud(opt_com_baudrate);
    serial_connect.setPort(opt_com_path.c_str());
    if(serial_connect.openSimple())
    {
        printf("[AuxCtrl] Open serail port sucessful!\n");
        printf("baud rate:%d\n",serial_connect.getBaud());
    }
    else
    {
        printf("[AuxCtrl] Open serail port %s failed! \n", opt_com_path.c_str());
        return -1;
    }

    printf("C3iroboticslidar connected\n");
    robotics_lidar.initilize(&serial_connect);
    
    if(robotics_lidar.GetDeviceInfo())//get device infomation
    {
        std::string str1 = robotics_lidar.GetLidarSNCode();
        std::string str2 = robotics_lidar.GetLidarType();
        std::string str3 = robotics_lidar.GetLidarFirmwareVersion();
        std::string str4 = robotics_lidar.GetLidarHardwareVersion();
        printf("SN:%s type:%s FW:%s HW:%s \n",str1.c_str(), str2.c_str(), str3.c_str(), str4.c_str());
    }
    while(1)
    {
        //usleep(100000);
		TLidarGrabResult result = robotics_lidar.getScanData();

        robotics_lidar.controlLidarSpeed();

        switch(result)
        {
            case LIDAR_GRAB_ING:
            {
                break;
            }
            case LIDAR_GRAB_SUCESS:
            {
                TLidarScan lidar_scan = robotics_lidar.getLidarScan();
                size_t lidar_scan_size = lidar_scan.getSize();

                if(lidar_scan_size > 0)
                {
                    std::vector<RslidarDataComplete> send_lidar_scan_data_part;
                    send_lidar_scan_data_part.resize(lidar_scan_size);
                    RslidarDataComplete one_lidar_data;
                    for(size_t i = 0; i < lidar_scan_size; i++)
                    {
                        one_lidar_data.signal = lidar_scan.signal[i];
                        one_lidar_data.angle = lidar_scan.angle[i];
                        one_lidar_data.distance = lidar_scan.distance[i];
                        send_lidar_scan_data_part[i] = one_lidar_data;
                    }
                    
                    //printf("[Main] Lidar count = %3d, start angle = %6.2f, current speed = %5.2lf r/s \n", (int)lidar_scan_size, lidar_scan.angle[0], robotics_lidar.getLidarCurrentSpeed());
                    robotics_lidar.m_lidar_scan.clear();
                }
                break;
            }
            
            case LIDAR_GRAB_ERRO:
            {
                
                printf("[Main] Lidar error code = %d \n", robotics_lidar.getLidarError());

                break;
            }
            case LIDAR_GRAB_ELSE:
            {
                printf("[Main] LIDAR_GRAB_ELSE!\n");
                break;
            }
        }
        //usleep(50);
    }

    return 0;
}