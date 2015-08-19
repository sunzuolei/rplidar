
/*
 * Copyright (C) 2014  RoboPeak
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
/*
 *  RoboPeak Lidar System
 *  Simple Data Grabber Demo App
 *
 *  Copyright 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *
 *  An ultra simple app to fetech RPLIDAR data continuously....
 *
 */

/*
 *此代码功能：把从rplidar采回来的数据（数据格式 时间+角度+距离）保存为二进制文件timedata
 *			  供Matlab仿真用
 *
 *		作者：		刘德志
 *		时间：2014年11月7日 19:35:11
 */

#include <CoreWindow.h>
#include <stdio.h>
#include <stdlib.h>

#include "rplidar.h" //RPLIDAR standard sdk, all-in-one header
#include <time.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include <string>
using namespace std;




#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

using namespace rp::standalone::rplidar;


bool checkRPLIDARHealth(RPlidarDriver * drv)
{
    u_result     op_result;
    rplidar_response_device_health_t healthinfo;


    op_result = drv->getHealth(healthinfo);
    if (IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
        printf("RPLidar health status : %d\n", healthinfo.status);
        if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
            fprintf(stderr, "Error, rplidar internal error detected. Please reboot the device to retry.\n");
            // enable the following code if you want rplidar to be reboot by software
            // drv->reset();
            return false;
        } else {
            return true;
        }

    } else {
        fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
        return false;
    }
}

int main(int argc, const char * argv[]) {
    const char * opt_com_path = NULL;
    _u32         opt_com_baudrate = 115200;
    u_result     op_result;

struct MYTIME {
    float wYear;
    float wMonth;
    float wDayOfWeek;
    float wDay;
    float wHour;
    float wMinute;
    float wSecond;
    float wMilliseconds;
}; 
	MYTIME  iitime;
	SYSTEMTIME sys; //系统时间
	unsigned short lastSecond;
	unsigned short lastMinute;
	unsigned short diff;

//////////////////////////////////////////////////
struct tm *ptr;
time_t now;
time(&now); //等同于now = time（NULL）
ptr = localtime(&now);
stringstream ss; //把int型转换为字符串型
ss <<ptr->tm_year+1900 //tm_year 是今年与1900年的偏差
	<<ptr->tm_mon+1
	<<ptr->tm_mday
	<<ptr->tm_hour 
	<<ptr->tm_min
	<<ptr->tm_sec
	<<".dat";
string s = ss.str();
//cout << s.c_str() <<endl; //得到的字符串的调用方法
//////////////////////////////////////////////////
	ofstream outfile(s.c_str(),ios::binary);
	if(!outfile)
	{
		cerr << "open error!"<<endl;
		abort();
	}
    // read serial port from the command line...
    if (argc>1) opt_com_path = argv[1]; // or set to a fixed value: e.g. "com3" 

    // read baud rate from the command line if specified...
    if (argc>2) opt_com_baudrate = strtoul(argv[2], NULL, 10);

    if (!opt_com_path) {
#ifdef _WIN32
        // use default com port
        opt_com_path = "\\\\.\\com7";
#else
        opt_com_path = "/dev/ttyUSB0";
#endif
    }

    // create the driver instance
    RPlidarDriver * drv = RPlidarDriver::CreateDriver(RPlidarDriver::DRIVER_TYPE_SERIALPORT);
    
    if (!drv) {
        fprintf(stderr, "insufficent memory, exit\n");
        exit(-2);
    }

    // make connection...
    if (IS_FAIL(drv->connect(opt_com_path, opt_com_baudrate))) {
        fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
            , opt_com_path);
  //      goto on_finished;
    }

    // check health...
    if (!checkRPLIDARHealth(drv)) {
//        goto on_finished;
    }
/*	      //测试延时用
//	Sleep(10000);		
//	cout << "\a";
//	cout << "响铃"<<endl;

	GetLocalTime(&sys);
	lastMinute = sys.wMinute;
	lastSecond = sys.wSecond;
	cout << "now:"<< lastSecond<<endl;
	do
	{
		Sleep(1000);
		GetLocalTime(&sys);		
		cout << "new:"<<sys.wSecond<< endl;
		if(sys.wMinute != lastMinute)
		{
			diff = sys.wSecond + 60 - lastSecond;
		}
		else
		{
			diff = sys.wSecond-lastSecond;
		}
	}
	while(diff <= 10);

	cout << "\a";
	cout << "响铃again"<<endl;
*/

    // start scan...
    drv->startScan();

		rplidar_response_measurement_node_t nodes[360*2];
        size_t   count = _countof(nodes);

		GetLocalTime(&sys);//测量 1 Min 看二进制文件大小
		lastMinute = sys.wHour; 
		lastSecond = sys.wMinute;
		cout << "lastS:"<<lastSecond<<endl;
		do
		{
			op_result = drv->grabScanData(nodes, count);
			if (IS_OK(op_result)) 
			{
				GetLocalTime(&sys);

				iitime.wYear = sys.wYear;	
				iitime.wMonth = sys.wMonth;
				iitime.wDayOfWeek = sys.wDayOfWeek;
				iitime.wDay = sys.wDay;
				iitime.wHour = sys.wHour;
				iitime.wMinute = sys.wMinute;
				iitime.wSecond = sys.wSecond;
				iitime.wMilliseconds = sys.wMilliseconds;

				outfile.write((char*)&iitime,sizeof(iitime));
				int pos = 0;
				for(pos=0; pos<(int)count; pos++)
				{
					float angle = (nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f;
					float distance = nodes[pos].distance_q2/4.0f;
					outfile.write((char*)&angle,sizeof(angle));
					outfile.write((char*)&distance,sizeof(distance));
				}
				for(; pos <320; pos++)
				{
					float zero = 0.0f;
					outfile.write((char*)&zero,sizeof(float));
					outfile.write((char*)&zero,sizeof(float));
				}

/*				GetLocalTime(&sys); //当前时间
				fout << sys.wYear<<sys.wMonth << sys.wDay << sys.wHour <<
				sys.wMinute << sys.wSecond << sys.wMilliseconds << endl; //时间戳
				for (int pos = 0; pos < (int)count ; ++pos) 
				 {
					fout <<" "<< pos;
					fout << ((nodes[pos].sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) ?"S":" ")
						<< ((nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f)<< " " 
						<< ( nodes[pos].distance_q2/4.0f)<< " "
						<< (nodes[pos].sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT) <<endl;
				 }	
*/
			}
			cout << "nowM:"<<sys.wMinute<<endl;

			if(sys.wHour != lastMinute)
			{
				diff = sys.wMinute + 60 - lastSecond;
			}
			else
			{
				diff = sys.wMinute - lastSecond;
			} 

        }while(0);//while(diff <= 1);//1Min

		cout <<"\a"<< "OK !Very GOOD!"<<endl;
		
    // done!
on_finished:
	outfile.close( );
    RPlidarDriver::DisposeDriver(drv);
    return 0;
}