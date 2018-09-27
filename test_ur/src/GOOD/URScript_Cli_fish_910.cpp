/*******************************
FileName: ur_control.cpp
Time: 2018年 08月 26日 星期日 19:33:14 CST
Author: fish
Description:
	1.目的是不通过moveit来对机器臂简单控制
	2.简单的URScript 发布movej 给ur_driver/URScript  可以移动,在按ctr-c 才能执行,有人采用服务消息类型
	3.  fatal error: ur_msgs/urscript.h: No such file or directory  这个消息是 服务消息类型生成,添加find_package (ur_msgs )
	4. 2018年 09月 19日 星期三 18:28:58 CST  需要改进一下 选择性 执行 输入参数 来执行
other:
	vim 快捷键: %s/^\s\+
******************************/

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <vector>
#include <mutex>
#include <math.h>
#include <condition_variable>
#include <vector>
#include <cstdlib>
#include <iostream>

#include "ur_msgs/urscript.h"

class URScript
{

public:
	URScript();
	~URScript();
private:

};
URScript::URScript()
{
}
URScript::~URScript()
{

}
int main(int argc, char **argv)
{

	ros::init(argc, argv, "ur_pub_");
	ROS_INFO(" URScript_Cli %d", argc );
	
	if (argc != 2) {
		ROS_INFO(" Parameter Error ");
        	return 1;	
	}
	
	ros::NodeHandle nh;
	ros::ServiceClient joint_state_client = nh.serviceClient<ur_msgs::urscript>("ur_driver/URScript_srv");
	ur_msgs::urscript srv;
	
	
	int count;
	std_msgs::String msg;
	std::stringstream cmd_str;
//	cmd_str << "movej([-1.5704,-1.5706,-1.5717, -1.5697,1.5701, -0.0001],1.4,1.05,5)\n";
	//选择 1
	if( atoll(argv[1]) == 1 ) {
		ROS_INFO(" mode 1 ");  //初始位置
		cmd_str << "movej([-1.3704,-1.5706,-1.5717, -1.5697,1.5701, -0.0001],1.4,1.05,5)\n";
	}

	//选择 2
	if( atoll(argv[1]) == 2 ) {
		ROS_INFO(" mode 2 ");
		cmd_str << "movep([0.109,0.486,0.432, -0.50,0.50, 0.50],a=1.4,v=1.05)\n";
	}

  	cmd_str <<"rq_close()\n";
	msg.data = cmd_str.str();	
	srv.request.script = msg.data;
	while(!joint_state_client.call(srv)) //这个使得一直调用服务，直到成功call到服务.  
	{
		count++;
		std::cout<<"joint_state_client.call(srv)  "<< joint_state_client.call(srv)<<"count = " <<count <<std::endl;
	}
	/*
	ros::Rate loop_rate(125);
	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
*/
}

