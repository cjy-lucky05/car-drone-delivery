#pragma once

#include "nvilidar_def.h"
#include "nvilidar_protocol.h"

//---visual studio include lib file 
#ifdef WIN32
	#define NVILIDAR_FILTER_API __declspec(dllexport)
#else
	#define NVILIDAR_FILTER_API
#endif // ifdef WIN32


namespace nvilidar
{
    //lidar driver 
	class  NVILIDAR_FILTER_API LidarFilter
    {
		public:
			LidarFilter();		
			~LidarFilter();

			void LidarFilterLoadPara(Nvilidar_UserConfigTypeDef cfg);		//load fit para 
			void LidarJumpFilter(std::vector<Nvilidar_Node_Info> &in);		//jump data  filter 

		private:
			Nvilidar_UserConfigTypeDef     lidar_cfg;				//lidar model 
    };
}
