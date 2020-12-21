/*********************************************************************************
File name:	  C3iroboticsLidar.h
Author:       Kimbo
Version:      V1.7.1
Date:	 	  2017-02-03
Description:  3irobotics lidar sdk
Others:       None

History:
	1. Date:
	Author:
	Modification:
***********************************************************************************/
#define DEBUG
/********************************* File includes **********************************/
#include "C3iroboticsLidar.h"

/******************************* Current libs includes ****************************/
#include <iostream>
#include <unistd.h>
#include <time.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
//#include <sys/io.h>
/********************************** Name space ************************************/
using namespace everest;
using namespace everest::hwdrivers;

/***********************************************************************************
Function:     C3iroboticsLidar
Description:  The constructor of C3iroboticsLidar
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
C3iroboticsLidar::C3iroboticsLidar()
{
    m_device_connect = NULL;
    m_data_with_signal = true;
    m_receive_lidar_speed = false;
    m_current_lidar_speed = -1.0;
    Expectspeed = 6.0;
    resetScanGrab();
    Error_timeout.Shieldflag = FALSE;
    Error_timeout.speedflag = FALSE;
    m_Shield_count = 0;
    Node_num = 0;
    Lds_str = "LDS";
    //调整速度相关变量
    error = 0.0;
    last_error = 0;
    error_sum = 0.0;
    percent = 0;
    speedStableFlag = false;
    countSpeed = 0;
    memset(pProInfopBuf, 0, 128);
    memset(SNCode, 0, 16);
    memset(SoftwareV, 0, 16);
    memset(HardwareV, 0, 16);
    memset(Lidartype, 0, 8);
}

/***********************************************************************************
Function:     C3iroboticsLidar
Description:  The destructor of C3iroboticsLidar
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
C3iroboticsLidar::~C3iroboticsLidar()
{

}

/***********************************************************************************
Function:     initilize
Description:  Set device connect
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
bool C3iroboticsLidar::initilize(CDeviceConnection *device_connect)
{
    if(device_connect == NULL || device_connect->getStatus() != CDeviceConnection::STATUS_OPEN)
    {
        printf("[C3iroboticsLidar] Init failed Can not open device connect!\n");
        return false;
    }
    else
    {
        printf("[C3iroboticsLidar] Init device connect sucessful!\n");
        m_receiver.setDeviceConnection(device_connect);
        return true;
    }
}
/***********************************************************************************
Function:     ScanErrTimeOut
Description:   Scan Error TimeOut
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
int C3iroboticsLidar::ScanErrTimeOut(CLidarPacket *packet)
{

    if(LIDAR_ERROR_TIME_OVER == packet->m_lidar_erro)
        goto LOG;

    if(Error_timeout.speedflag)
    {
        
        if(m_speed_count_down.isEnd())
        {
            printf("Lidar speed lost\n");
            m_speed_count_down.setTime((double)m_params.speed_time_out);
            packet->m_lidar_erro = LIDAR_ERROR_LOST_SPEED;

            return -1;
        }
    }
    else if(Error_timeout.Shieldflag)
    {
        if(m_Shield_ocount_down.isEnd())
        {
            printf("Lidar shield\n");
            m_Shield_ocount_down.setTime((double)m_params.Shield_time_out);
            packet->m_lidar_erro = LIDAR_ERROR_SHIELD;
            return -1;
        }
    }
    else
    {
        //Nothing to do
    }
    LOG:
       return 0;
}
/***********************************************************************************
Function:     getScanData
Description:   Get scan data
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
TLidarGrabResult C3iroboticsLidar::getScanData()
{
    TLidarGrabResult grab_result;
    while(1)
    {
        

        if(m_remainder_flag)
        {
            printf("[C3iroboticsLidar] Handle remainer scan!\n");
            resetScanGrab();
            combineScan(m_remainder_tooth_scan);
        }
        else 
        {
            if(m_receiver.receivePacket(&m_packet))
                grab_result = analysisPacket(m_packet);
            else
                grab_result = LIDAR_GRAB_ERRO;

            if(-1 == ScanErrTimeOut(&m_packet))
            {
                grab_result = LIDAR_GRAB_ERRO;
            }
            break;
        }

    }
    return grab_result;
}

/***********************************************************************************
Function:     analysisPacket
Description:  Analysis packet
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
TLidarGrabResult C3iroboticsLidar::analysisPacket(CLidarPacket &lidar_packet)
{
    TLidarCommandID command_id = TLidarCommandID(lidar_packet.getCommandID());
    switch(command_id)
    {
        case I3LIDAR_DISTANCE: return analysisToothScan(lidar_packet);
        case I3LIDAR_HEALTH: return analysisHealthInfo(lidar_packet);
        case I3LIDAR_LIDAR_SPEED: return analysisLidarSpeed(lidar_packet);
        case I3LIDAR_NEW_DISTANCE : return analysisNewToothScan(lidar_packet);
        default:
            printf("[C3iroboticsLidar] Special command id %d!\n", command_id);
        return LIDAR_GRAB_ELSE;
    }
}

/***********************************************************************************
Function:     analysisToothScan
Description:  Analysis tooth scan
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
TLidarGrabResult C3iroboticsLidar::analysisToothScan(CLidarPacket &lidar_packet)
{
    TToothScan tooth_scan;
    if(m_data_with_signal)
    {
        tooth_scan = CLidarUnpacket::unpacketLidarScan2(lidar_packet);
    }
    else
    {
        tooth_scan = CLidarUnpacket::unpacketLidarScan(lidar_packet);
    }
    return combineScan(tooth_scan);
}

/***********************************************************************************
Function:     analysisToothScan
Description:  Analysis tooth scan
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
TLidarGrabResult C3iroboticsLidar::analysisNewToothScan(CLidarPacket &lidar_packet)
{
    TToothScan tooth_scan;
    if(Error_timeout.speedflag)
        Error_timeout.speedflag = false;
    
    if(m_data_with_signal)
    {
        tooth_scan = CLidarUnpacket::unpacketNewLidarScanHasSingal(lidar_packet);
    }
    else
    {
        tooth_scan = CLidarUnpacket::unpacketNewLidarScanNoSingal(lidar_packet);
    }

    m_current_lidar_speed = tooth_scan.lidar_speed;
    m_receive_lidar_speed = true;
    m_Shield_count += tooth_scan.Shield_count;

    return combineScan(tooth_scan);
}

/***********************************************************************************
Function:     combineScan
Description:  Combine scan
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
TLidarGrabResult C3iroboticsLidar::combineScan(TToothScan &tooth_scan)
{
    //printf("[C3iroboticsLidar] m_grab_scan_state %d, m_grab_scan_count %d!\n", m_grab_scan_state, m_grab_scan_count);
    switch(m_grab_scan_state)
    {
        case GRAB_SCAN_FIRST:
        {
            m_data_count_down.setTime(m_params.scan_time_out_ms);

            /* First scan come */
            if(isFirstScan(tooth_scan))
            {
                resetScanGrab();
                grabFirstScan(tooth_scan);
                int count = m_Shield_count;
                m_Shield_count = 0;
                if(count < 6)
                {
                    Error_timeout.Shieldflag = TRUE;
                }
                else
                {
                    Error_timeout.Shieldflag = FALSE;
                }

            }
            else
            {
                printf("[C3iroboticsLidar] GRAB_SCAN_FIRST tooth scan angle %5.2f!\n",
                          tooth_scan.angle);
            }
            return LIDAR_GRAB_ING;
        }
        case GRAB_SCAN_ELSE_DATA:
        {
            if(m_data_count_down.isEnd())
            {
                printf("[C3iroboticsLidar] grab scan is time out %d ms! Reset grab scan state, current is %5.2f, last is %5.2f! \n",
                          m_params.scan_time_out_ms, tooth_scan.angle, m_last_scan_angle);
                m_grab_scan_state = GRAB_SCAN_FIRST;
                return LIDAR_GRAB_ING;
            }
            m_data_count_down.setTime(m_params.scan_time_out_ms);
//            printf("[C3iroboticsLidar] tooth_scan.angle %5.2f, m_last_scan_angle %5.2f!\n",
//                      tooth_scan.angle, m_last_scan_angle);
            /* Handle angle suddenly reduces */
            if(tooth_scan.angle < m_last_scan_angle)
            {
                printf("[C3iroboticsLidar] may receive next scan, current %5.2f, last %5.2f\n",
                          tooth_scan.angle, m_last_scan_angle);
                if(isFirstScan(tooth_scan))
                {
                    m_remainder_flag = true;
                    m_remainder_tooth_scan = tooth_scan;

                }
                return LIDAR_GRAB_SUCESS;
            }

            /* Insert tooth scan in scan */
            TLidarScan part_scan_data;
            toothScan2LidarScan(tooth_scan, part_scan_data);
            m_lidar_scan.insert(part_scan_data);
            m_grab_scan_count++;
            m_last_scan_angle = tooth_scan.angle;

            /* Judge whether finish grab one compelte scan */
            if(m_grab_scan_count == m_params.tooth_number)
            {
                m_grab_scan_state = GRAB_SCAN_FIRST;
                return LIDAR_GRAB_SUCESS;
            }
            else
            {
                return LIDAR_GRAB_ING;
            }
        }
        default:
            printf("[C3iroboticsLidar] Uknow grab scan data state %d!\n",
                      m_grab_scan_state);
        break;
    }
    printf("[C3iroboticsLidar] combineScan should not come to here!\n");
    return LIDAR_GRAB_ERRO;
}

/***********************************************************************************
Function:     grabFirstScan
Description:  Grab first scan
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
TLidarGrabResult C3iroboticsLidar::grabFirstScan(TToothScan &tooth_scan)
{
//    printf("[C3iroboticsLidar] Enter first grab scan data 1 !\n");

    // Change grab state
    m_grab_scan_state = GRAB_SCAN_ELSE_DATA;
    m_grab_scan_count = 1;
    m_last_scan_angle = tooth_scan.angle;


    // Insert scan data
    TLidarScan part_scan_data;

    toothScan2LidarScan(tooth_scan, part_scan_data);

    m_lidar_scan.insert(part_scan_data);

    return LIDAR_GRAB_ING;
}

/***********************************************************************************
Function:     isFirstScan
Description:  Return true if tooth scan is first
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
bool C3iroboticsLidar::isFirstScan(TToothScan &tooth_scan)
{
    if(tooth_scan.angle >= 0 && tooth_scan.angleEnd <= 22.5)
	return true;
    else
	return false;
    //return tooth_scan.angle < 0.0001? true: false;
}

/***********************************************************************************
Function:     toothScan2LidarScan
Description:  Change tooth scan to lidar scan
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
void C3iroboticsLidar::toothScan2LidarScan(TToothScan &tooth_scan, TLidarScan &lidar_scan)
{
    size_t size = tooth_scan.getSize();
    float  first_angle = tooth_scan.getAngle();
    float  last_angle = tooth_scan.getAngleEnd();
    bool has_signal = !tooth_scan.signal.empty();

//    printf()
    /*
                              last_angle - first_angle
        angle_step =  ---------------------------------
                                  tooth_scan_size
    */
    float  angle_step = float(last_angle - first_angle) / (float(size));
    lidar_scan.angle.resize(size);
    lidar_scan.distance.resize(size);

    for(size_t i = 0; i < size; i++)
    {
        lidar_scan.angle[i] = first_angle + i * angle_step;
        lidar_scan.distance[i] = tooth_scan.distance[i];
    }
    if(has_signal)
    {
       lidar_scan.signal.resize(size);
       for(size_t i = 0; i < size; i++)
        lidar_scan.signal[i] = tooth_scan.signal[i];
    }
    else
    {
        lidar_scan.signal.clear();
    }
}

/***********************************************************************************
Function:     analysisHealthInfo
Description:  Analysis health info
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
TLidarGrabResult C3iroboticsLidar::analysisHealthInfo(CLidarPacket &lidar_packet)
{
    TLidarError lidar_error = CLidarUnpacket::unpacketHealthInfo(lidar_packet);
    printf("[C3iroboticsLidar] Lidar error is %5.5f!\n", lidar_error );
    return LIDAR_GRAB_ERRO;
}

/***********************************************************************************
Function:     analysisLidarSpeed
Description:  Analysis health info
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
TLidarGrabResult C3iroboticsLidar::analysisLidarSpeed(CLidarPacket &lidar_packet)
{
    
    char *pTemp = (char*)CLidarUnpacket::unpacketLidarInformation(lidar_packet);
    if(NULL == pTemp)
        return LIDAR_GRAB_ING;

    double lidar_erro_speed = (*pTemp) * 0.05f;

    if(!Error_timeout.speedflag)
        Error_timeout.speedflag = true;
    
    pTemp++;
    std::string str_Sn = pProInfopBuf;
    if(str_Sn.npos == str_Sn.find(Lds_str)&&(NULL != pTemp))
        strncpy(pProInfopBuf, pTemp, 56);
        

    m_current_lidar_speed = lidar_erro_speed;
    m_receive_lidar_speed = true;

    return LIDAR_GRAB_ING;
}

/***********************************************************************************
Function:     resetScanGrab
Description:  Reset scan grab
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
void C3iroboticsLidar::resetScanGrab()
{
    m_lidar_scan.clear();
    m_grab_scan_state = GRAB_SCAN_FIRST;
    m_grab_scan_count = 0;
    m_last_scan_angle = 0.0;
    m_remainder_flag = false;;
}




/***********************************************************************************
Function:     adbWrite
Description:  adb Wripte
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
void C3iroboticsLidar::PwmWriteData(const char *file_name, int64_t data)
{
    int fp = 0;
    char pbuf[20] = "0";

    sprintf(pbuf, "%d", data);
    fp = open(file_name, O_WRONLY);
    int fd = write(fp, pbuf, strlen(pbuf));
    close(fp);
}


/***********************************************************************************
Function:     adbWrite
Description:  adb Write
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
void C3iroboticsLidar::PwmWriteData(const char *file_name, const char *data)
{
    int fp = 0;
    fp = open(file_name, O_WRONLY);
    int fd = write(fp, data, strlen(data));
    close(fp);
}

/***********************************************************************************
Function:     adbInit
Description:  adb Init
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
void C3iroboticsLidar::PwmInit()
{
    int num = GetDeviceNodeID();
    std::string str = "/sys/class/pwm/pwmchip" + std::to_string(num)+"/pwm0";
    std::string str_export = "/sys/class/pwm/pwmchip" + std::to_string(num) + "/export";
    std::string str_period =  "/sys/class/pwm/pwmchip" + std::to_string(num)+"/pwm0/period";
    std::string str_duty_cycle =  "/sys/class/pwm/pwmchip" + std::to_string(num)+"/pwm0/duty_cycle";
    std::string str_polarity =  "/sys/class/pwm/pwmchip" + std::to_string(num)+"/pwm0/polarity";
    std::string str_enable =  "/sys/class/pwm/pwmchip" + std::to_string(num)+"/pwm0/enable";
   if(access(str.c_str(), 0) == -1)
   {
       PwmWriteData(str_export.c_str(), (int64_t)0);
       usleep(20);
   }
    PwmWriteData(str_period.c_str(), 20000000);
    PwmWriteData(str_duty_cycle.c_str(), 15000000);
    PwmWriteData(str_polarity.c_str(), "inversed");//normal 
    PwmWriteData(str_enable.c_str(), (int64_t)1);
}
/***********************************************************************************
Function:     ControlLidarPause
Description:  Control Lidar pause
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
void C3iroboticsLidar::ControlLidarPause()
{
    int num = GetDeviceNodeID();
    std::string str =  "/sys/class/pwm/pwmchip" + std::to_string(num)+"/pwm0/enable";
    PwmWriteData(str.c_str(), (int64_t)0);
}
/***********************************************************************************
Function:     ControlLidarStart
Description:  Control Lidar Start
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
void C3iroboticsLidar::ConnectLidarStart()
{
    int num = GetDeviceNodeID();
    std::string str =  "/sys/class/pwm/pwmchip" + std::to_string(num)+"/pwm0/enable";
    PwmWriteData(str.c_str(), (int64_t)1);
}
/***********************************************************************************
Function:     SetDeviceNodeID
Description:  set device node id
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
void C3iroboticsLidar::SetDeviceNodeID(u8 num)
{
    Node_num = num;
}
/***********************************************************************************
Function:     GetDeviceNodeID
Description:  get device node id
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
u8 C3iroboticsLidar::GetDeviceNodeID()
{
    return Node_num;
}
/***********************************************************************************
Function:     controlLidarPWM
Description:  Control Lidar PWM
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
void C3iroboticsLidar::controlLidarPWM(int8_t percent)
{
    uint32_t duty_cycle = 0;
    int num = GetDeviceNodeID();

    std::string str =  "/sys/class/pwm/pwmchip" + std::to_string(num)+"/pwm0/duty_cycle";

    if((percent < 0)||(percent > 100))
    {
        percent = 40;
    }
    percent = percent % 100;

    duty_cycle = percent * 20000000 / 100;
    
    PwmWriteData(str.c_str(), duty_cycle);
}


/***********************************************************************************
Function:     controlLidarSpeed
Description:  control Lidar Speed
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
void C3iroboticsLidar::controlLidarSpeed()
{

    double currentSpeed = getLidarCurrentSpeed();
    double expectspeed = GetLidarExpectspeed();
    
    double speed = currentSpeed;
    last_error = error;   
    error = expectspeed - speed;           
    error_sum += error;          
    if(error_sum > 20)
    {
        error_sum = 20;
    }
    else if(error_sum < -20)
    {
        error_sum = -20;
    }
    if(error < 1 && error > -1)
        percent = 40 + error_sum*1.0 + 0.1*error + 0.5*(error - last_error);
    else
        percent = 40 + error_sum*1.0 + 5*error + 5*(error - last_error);

    controlLidarPWM(percent);

    
}
/***********************************************************************************
Function:     SetLidarExpectSpeed
Description:  Set Lidar Expect Speed
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
int C3iroboticsLidar::SetLidarExpectSpeed(double speed)
{
    if((speed > 8.0)||(speed < 4.0))
        return -1;
    else
    {
        Expectspeed = speed;
    }
    return 0;
}
/***********************************************************************************
Function:     GetLidarSNCode
Description:  get lidar SN code
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
u8 * C3iroboticsLidar::GetLidarSNCode()
{

    u8 pTmpbuf[32] = {0};
    u8 *temp = pTmpbuf;
    char *tmp = pProInfopBuf;

    for(int i = 0;i < 24;i++)
    {
        char str = *tmp;
        if(str < 0x41)
            str = str - 0x30;
        else
            str = str - 65 + 10;
        *temp = (u8)str;
        tmp++;
        temp++;
    }
   
    for(int i = 0;i < 12; i++)
    {
        SNCode[i] = (pTmpbuf[2*i] << 4) + pTmpbuf[2*i+1];
        printf("%02x ", SNCode[i]);
    }
    return SNCode;

}
/***********************************************************************************
Function:     GetLidarSoftwareVersion
Description:  get lidar SN code
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
char *C3iroboticsLidar::GetLidarSoftwareVersion()
{
    char *tmp = pProInfopBuf;
    char * temp = SoftwareV;
    tmp += 32;
    strncpy(temp, tmp, 11);
    
    return SoftwareV;
}
/***********************************************************************************
Function:     GetLidarHardwareVersion
Description:  get lidar SN code
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
char *C3iroboticsLidar::GetLidarHardwareVersion()
{
    char *tmp = pProInfopBuf;
    char * temp = HardwareV;
    tmp += 44; 
    strncpy(temp, tmp, 11);
    
    return HardwareV;
}
/***********************************************************************************
Function:     GetLidarType
Description:  get lidar SN code
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
char *C3iroboticsLidar::GetLidarType()
{
    char *tmp = pProInfopBuf;
    char *temp = Lidartype;
    tmp += 25;
    strncpy(temp, tmp, 6);
    return Lidartype;
}