/*********************************************************************************
File name:	  CLidarUnpacket.h
Author:       Kimbo
Version:      V1.7.1
Date:	 	  2017-02-06
Description:  3irobotics Lidar unpacket
Others:       None

History:
	1. Date:
	Author:
	Modification:
***********************************************************************************/

#ifndef EVEREST_LIDAR_CLIDARUNPACKET_H
#define EVEREST_LIDAR_CLIDARUNPACKET_H

/******************************* Current libs includes ****************************/
#include "CLidarPacket.h"

/******************************* System libs includes *****************************/
#include <vector>

/******************************* Other libs includes ******************************/

namespace everest
{
	namespace hwdrivers
	{
	    struct TToothScan
	    {
            TToothScan() : offset_valid(false), Shield_count(0), offset_angle(0.0), lidar_speed(-1.0), angle(0.0), angleEnd(0.0), distance(), signal(){ }

            float getAngle() 	{ return angle; }
	        float getAngleEnd() { return angleEnd; }
            size_t getSize() 	{ return distance.size(); }

            bool                  offset_valid;
            int                  Shield_count;
            float                 offset_angle;     // unit: degree
            float                 lidar_speed;      // unit: lidar speed
            float                 angle;            // unit: degree start
	        float                 angleEnd;         // unit: degree end
            std::vector<float>    distance;         // unit: meter
            std::vector<int>      signal;           // range: 0 - 255
	    };

        

		class CLidarUnpacket
		{
            public:

                /* Constructor */
                CLidarUnpacket();

                /* Destructor */
                ~CLidarUnpacket();

                /* Lidar unpacket */
                static TToothScan unpacketLidarScan(CLidarPacket &packet);

                /* Lidar unpacket */
                static TToothScan unpacketLidarScan2(CLidarPacket &packet);

                /* Lidar unpacket */
                static TToothScan unpacketNewLidarScanHasSingal(CLidarPacket &packet);

                /* Lidar unpacket */
                static TToothScan unpacketNewLidarScanNoSingal(CLidarPacket &packet);

                /* Health info unpacket */
                static TLidarError unpacketHealthInfo(CLidarPacket &packet);

                /* Lidar speed */
                static int unpacketLidarSpeed(CLidarPacket &packet);
                /* Lidar Information */
                static u8 *unpacketLidarInformation(CLidarPacket &packet);

                /*get lidar version*/                
                static u8 UnpackerLidarVersion(CLidarPacket &packet); 



		};
	}
}

#endif


