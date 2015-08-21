
#pragma once

namespace rp { namespace standalone{ namespace rplidar {

class RPlidarDriverMpig : public RPlidarDriverSerialImpl
{
public:
		string fileName;
		string fileAddr;
		
		string renameFileName(const fileAddr = NULL);//以当前时间+.dat命名文件夹名,例如"2015420104403.dat
		bool saveScanData(const fileName = "000.dat"); //向fileName文件名中写入数据。
}

}}}