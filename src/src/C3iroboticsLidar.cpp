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
#include <cmath>
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
    Node_num = 1;
    Pwm_num = 0;
    Lds_str = "LDS";
    m_pwm_polarity_state = NORMAL;
    //调整速度相关变量
    error = 0.0;
    MaxPwm = 85;
    last_error = 0;
    error_sum = 0.0;
    percent = 0;
    speedStableFlag = false;
    countSpeed = 0;
    LidarV = TLidarVersion::LIDAR_NONE;
    memset(pProInfopBuf, 0, 128);
    memset(SNCode, 0, 64);
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
    printf("~C3iroboticsLidar\n");
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
   if(packet->m_error_Data_Wrong)//data wrong
    {
        if(packet->Timeout_Data_Wrong.isEnd())
        {
            printf("(LINE:%d)-<FUNCTOIN:%s> Lidar Data Wrong\n",__LINE__,__FUNCTION__);
            packet->m_error_Data_Wrong = false;
            packet->m_lidar_erro = LIDAR_ERROR_DATA_WRONG;
            return -1;
        }
    }
    if(packet->m_error_crc)//crc
    {
        if(packet->Timeout_CRC.isEnd())
        {
            printf("(LINE:%d)-<FUNCTOIN:%s> Lidar CRC timeout\n",__LINE__,__FUNCTION__);
            packet->m_error_crc = false;
            packet->m_lidar_erro = LIDAR_ERROR_CRC;
            return -1;
        }
    } 
    else if(Error_timeout.speedflag)//speed
    {
        
        if(m_speed_count_down.isEnd())
        {
            printf("(LINE:%d)-<FUNCTOIN:%s> Lidar speed lost\n",__LINE__,__FUNCTION__);
            packet->m_lidar_erro = LIDAR_ERROR_LOST_SPEED;
            Error_timeout.speedflag = false;
            return -1;
        }
    }
    else if(Error_timeout.Shieldflag)
    {
        if(m_Shield_count_down.isEnd())
        {
            printf("(LINE:%d)-<FUNCTION:%s> Lidar shield\n",__LINE__,__FUNCTION__);
            packet->m_lidar_erro = LIDAR_ERROR_SHIELD;
            Error_timeout.Shieldflag = false;
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
                    if(!Error_timeout.Shieldflag)
                    {
                        Error_timeout.Shieldflag = TRUE;
                        m_Shield_count_down.setTime((double)m_params.Shield_time_out);
                    }
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
Function:     LidarTranform
Description:  None
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
void C3iroboticsLidar::LidarTranform(float &Angle, float &Dis)
{
    const float DIS_OFFEST = 18.3;
    const float LIDAR_PAI = 3.1415926;
    float angle_temp = Angle;
    float dis_temp = Dis*1000.0;
    static float angelComp= 0.0;
    if(dis_temp > 0.0)
    {
        float angle1 = LIDAR_PAI/2.0 - std::atan(dis_temp/DIS_OFFEST);
        angelComp = angle1/LIDAR_PAI*180;
        angle_temp = angle_temp + angle1/LIDAR_PAI*180;
        if(angle_temp > 360.0)
        {
            angle_temp -= 360.0;
        }
        else if(angle_temp < 0.0)
        {
            angle_temp += 360.0;
        }
        dis_temp = std::sqrt(DIS_OFFEST*DIS_OFFEST+dis_temp*dis_temp);
        Angle = angle_temp;
        Dis = dis_temp/1000.0f;
    }
    else
    {
        Angle = Angle + angelComp; 
    }
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
        //printf("yuanshi:angle:%6.3f, dis:%6.3f\n\r",lidar_scan.angle[i], lidar_scan.distance[i]);
        LidarTranform(lidar_scan.angle[i], lidar_scan.distance[i]);
        //printf("swap:angle:%6.3f, dis:%6.3f\n\r",lidar_scan.angle[i], lidar_scan.distance[i]);
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
    uint tmp = CLidarUnpacket::UnpackerLidarVersion(lidar_packet);
    if(NULL == pTemp)
        return LIDAR_GRAB_ING;
        
    if(0x10 == tmp)
    {
        SetLidarversion(LIDAR_2_6_K);
    }
    else if(0x20 == tmp)
    {
        SetLidarversion(LIDAR_2_1_K);
    }
    else if(0x21 == tmp)
    {
        SetLidarversion(LIDAR_2_6_K_SC02);
    }
    
    
    double lidar_erro_speed = (*pTemp) * 0.05f;
    //printf("curspeed:%5.2f\n", lidar_erro_speed);
    if(!Error_timeout.speedflag)
    {
        Error_timeout.speedflag = true;
         m_speed_count_down.setTime((double)m_params.speed_time_out);
         printf("curspeed:%5.2f\n", lidar_erro_speed);
    }
    pTemp++;
    std::string str_Sn = pProInfopBuf;
    if(str_Sn.npos == str_Sn.find(Lds_str)&&(NULL != pTemp))
        strncpy(pProInfopBuf, pTemp, 56);
        
    //printf("info:%s\n", pProInfopBuf);
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
void C3iroboticsLidar::PwmInit(PWMPolarityState state)
{
    int num = GetDeviceNodeID();
    std::string pwmid = GetDevicepwmID(); 
    std::string pwmnum = GetDevicepwmNum();
    std::string str = "/sys/class/pwm/pwmchip" + std::to_string(num)+"/"+ pwmid;
    std::string str_export = "/sys/class/pwm/pwmchip" + std::to_string(num) + "/export";
    std::string str_period =  "/sys/class/pwm/pwmchip" + std::to_string(num)+"/"+ pwmid + "/period";
    std::string str_duty_cycle =  "/sys/class/pwm/pwmchip" + std::to_string(num)+"/"+pwmid+"/duty_cycle";
    std::string str_polarity =  "/sys/class/pwm/pwmchip" + std::to_string(num)+"/"+ pwmid+"/polarity";
    std::string str_enable =  "/sys/class/pwm/pwmchip" + std::to_string(num)+"/"+ pwmid +"/enable";
    
    if(access(str.c_str(), 0) == -1)
   {
       PwmWriteData(str_export.c_str(), pwmnum.c_str());
       usleep(20);
   }
    PwmWriteData(str_period.c_str(), 50000);
    PwmWriteData(str_duty_cycle.c_str(), 37500);

    if(INVERSED == state)
    {
        m_pwm_polarity_state=INVERSED;
        PwmWriteData(str_polarity.c_str(), "inversed");//normal
    }
    else
    {
        m_pwm_polarity_state=NORMAL;
        PwmWriteData(str_polarity.c_str(), "normal");
    }
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
    std::string pwmid = GetDevicepwmID(); 
    std::string str =  "/sys/class/pwm/pwmchip" + std::to_string(num)+"/"+pwmid+"/enable";
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
    std::string pwmid = GetDevicepwmID(); 
    std::string str =  "/sys/class/pwm/pwmchip" + std::to_string(num)+"/"+pwmid+"/enable";
    
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
Function:     GetDevicepwmID
Description:  Get  Device  pwm  ID
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
std::string C3iroboticsLidar::GetDevicepwmID()
{
    std::string pwm;
    pwm = "pwm" + std::to_string(Pwm_num);
    return pwm;
}
/***********************************************************************************
Function:     GetDevicepwmNum
Description:  Get  Device  pwm  Num
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
std::string C3iroboticsLidar::GetDevicepwmNum()
{
    std::string Num;
    Num = std::to_string(Pwm_num);
    return Num;
}
/***********************************************************************************
Function:     SetDevicepwmID
Description:  Set  Device  pwm  ID
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
void C3iroboticsLidar::SetDevicepwmID(u8 pwmnum)
{
    Pwm_num = pwmnum;
}
/***********************************************************************************
Function:     GetPWMMaxLimit
Description:  get max pwm limit
Input:        None
Output:       None
Return:       None
Others:       None
/***********************************************************************************/
u8 C3iroboticsLidar::GetPWMMaxLimit()
{
    return MaxPwm;
}
/***********************************************************************************
Function:     GetPWMMaxLimit
Description:  get max pwm limit
Input:        None
Output:       None
Return:       None
Others:       None
/***********************************************************************************/
int C3iroboticsLidar::SetPWMMaxLimit(u8 limit)
{
    if((limit < 10)||(limit > 85))
    {
        printf("pwm max limited is 10%~85%!");
        return -1;
    }
    MaxPwm = limit;
    return 0;
}
/***********************************************************************************
Function:     controlLidarPWM
Description:  Control Lidar PWM
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
void C3iroboticsLidar::controlLidarPWM(int32_t percent_num)
{
    uint32_t duty_cycle = 0;
    int num = GetDeviceNodeID();
    u8 limit = GetPWMMaxLimit();
    std::string pwmid = GetDevicepwmID();
    std::string str =  "/sys/class/pwm/pwmchip" + std::to_string(num)+"/"+pwmid+"/duty_cycle";
    if(percent_num > limit)
        percent_num = limit;
    else if(percent_num < 0)
        percent_num = 0;
    percent = percent_num;
    duty_cycle = percent_num * 50000 / 100;
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
    int32_t percent_num;
    double currentSpeed = getLidarCurrentSpeed();
    double expectspeed = GetLidarExpectspeed();
    //printf("curr speed %5.2f\n, currentSpeed");
    double speed = currentSpeed;
    error = expectspeed - speed;           
    error_sum += error;          
    if(error_sum > 40)
        error_sum = 40;
    else if(error_sum < -40)
        error_sum = -40;
    percent_num = 45 + error_sum*1.0 + error * 1.0;
    controlLidarPWM(percent_num);

    
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
std::string C3iroboticsLidar::GetLidarSNCode()
{

    for(int i = 0;i < 8;i++)
    {
        SNCode[i] = '0';
    }
    strncpy(&SNCode[8], pProInfopBuf, 24);
    SNCode[32] = '\0';
    std::string string = SNCode;
    return string;

}
/***********************************************************************************
Function:     SetLidarversion
Description:  Set lidar version
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
void C3iroboticsLidar::SetLidarversion(TLidarVersion ver)
{
    LidarV = ver;
}
/***********************************************************************************
Function:     GetLidarversion
Description:  get lidar version
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
TLidarVersion C3iroboticsLidar::GetLidarversion()
{
    return LidarV;
}
/***********************************************************************************
Function:     GetLidarSoftwareVersion
Description:  get lidar SN code
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
std::string C3iroboticsLidar::GetLidarFirmwareVersion()
{
    strncpy(SoftwareV, &pProInfopBuf[32], 11);
    SoftwareV[11] = '\0';

    std::string string = SoftwareV;
    return string;
}
/***********************************************************************************
Function:     GetLidarHardwareVersion
Description:  get lidar SN code
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
std::string C3iroboticsLidar::GetLidarHardwareVersion()
{
    strncpy(HardwareV, &pProInfopBuf[44], 11);
    HardwareV[11] = '\0';

    std::string string = HardwareV;
    return string;
}
/***********************************************************************************
Function:     GetLidarType
Description:  get lidar SN code
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
std::string C3iroboticsLidar::GetLidarType()
{

    strncpy(Lidartype, &pProInfopBuf[25], 6);
    Lidartype[6] = '\0';
    
    std::string string = Lidartype;
    return string;
}
/***********************************************************************************
Function:     GetDeviceInfo
Description:  get lidar infomation
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
bool C3iroboticsLidar::GetDeviceInfo()
{
    bool flut = FALSE;
    std::string str_Info;

	m_receiver.SetSNFlag(m_receiver.GetSN_ING);
    m_GetSNcount_down.setTime(m_params.GetSN_time_out);
    
    while(1)
    {
        getScanData();
        str_Info = pProInfopBuf;

        if(str_Info.npos != str_Info.find(Lds_str))
        {
            flut = TRUE;
            m_receiver.SetSNFlag(m_receiver.GetSN_SUCCESS);
            //printf("get SN flag success:%d\n", m_receiver.GetSNFlag());
            break;
        }
        else if(m_GetSNcount_down.isEnd())
        {
            flut = FALSE;
            m_receiver.SetSNFlag(m_receiver.GetSN_FAILED);
            m_packet.m_lidar_erro = LIDAR_ERROR_DATA_INIT_DATA_WRONG;
            //printf("get SN flag failed:%d\n", m_receiver.GetSNFlag());
            break;
        }
        

    }
    return flut;

    
}
/***********************************************************************************
Function:     SetPwmpolarity
Description:  set pwm polarity
Input:        pwm polarity
Output:       None
Return:       success return 0,or return -1 
Others:       None
***********************************************************************************/
bool C3iroboticsLidar::SetPwmpolarity(PWMPolarityState state)
{
    switch (state)
    {
        case INVERSED:
        {
            m_pwm_polarity_state = INVERSED;
            return 0;
        }    
        case NORMAL:
        {
            m_pwm_polarity_state = NORMAL;
            return 0;
        }
        default:
        {
            printf("please set correct pwm polarity!");
            break;
        }
        
    }
    return -1;
}
/***********************************************************************************
Function:     getpwm
Description:  get pwm
Input:        None
Output:       None
Return:       None
Others:       None
***********************************************************************************/
int32_t C3iroboticsLidar::GetPwm()
{
    return percent;
}
