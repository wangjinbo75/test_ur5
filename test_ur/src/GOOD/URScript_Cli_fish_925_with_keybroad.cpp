/*******************************
FileName: ur_control.cpp
Time: 2018年 08月 26日 星期日 19:33:14 CST
Author: fish
Description:
	1.目的是不通过moveit来对机器臂简单控制
	2.简单的URScript 发布movej 给ur_driver/URScript  可以移动,在按ctr-c 才能执行,有人采用服务消息类型
	3.  fatal error: ur_msgs/urscript.h: No such file or directory  这个消息是 服务消息类型生成,添加find_package (ur_msgs )
	4. 2018年 09月 19日 星期三 18:28:58 CST  需要改进一下 选择性 执行 输入参数 来执行
	5. 末端位置 的输入
	6. 写成成员函数
	7. 2018年 09月 25日 星期二 14:39:47 CST 字符串中包含双引号 解决方案，在双引号前加一个反斜杠。 
	8. 把 z = 400cm 作为 零点
other:
	vim 快捷键: %s/^\s\+
	//	ur.cmd_str << "stopl(5.0)\n";
		//ur.cmd_str << "movel(get_inverse_kin(p[0.109,-0.487,0.431,0.0,3.141,0.0]),a=0.5, v=0.3)\n";

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
#include <string>
#include "ur_msgs/urscript.h"
using namespace std;

float pt_x;
float pt_y;
float pt_z;

const std::string HEADER="def gripper_action():\n socket_close(\"1\")\n sync()\n socket_open(\"127.0.0.1\",63352,\"1\")\n sync()\n";
const std::string ENDING=" socket_close(\"G\")\nend\n";

#define  SID          "socket_set_var(\"SID\", 9, \"1\")\n sync()\n"  
#define  GTO_0        "socket_set_var(\"GTO\", 0, \"1\")\n sync()\n"  
#define  POS_0        "socket_set_var(\"POS\"," 
#define  POS_1        ",\"1\")\n sync()\n" 
#define  GTO_1	      "socket_set_var(\"GTO\", 1, \"1\")\n sync()\n" 

class URScript
{

public:
	URScript();
	~URScript();

	ros::NodeHandle nh;
	ros::ServiceClient joint_state_client;
	
	ur_msgs::urscript srv;
	std_msgs::String msg;
	std::stringstream cmd_str;
	std::string command;
	
	string movej_joint(double joint_angle[6],float a=1.3,float v=0.3,float t=0,float r=0);//移动到基座标系下指定位置,参数为关节角
	string movej_pose(double tcp_pose[6],float a=1.3,float v=0.3,float t=0,float r=0); //移动到基座标系下指定位置,参数为TCP位置
	string movel_pose(double tcp_pose[6],float a=1.3,float v=0.3,float t=0,float r=0); //移动到基座标系下指定位置,线性插值方式
private:


};
URScript::URScript()
{
	ROS_INFO("URScript");
	joint_state_client = nh.serviceClient<ur_msgs::urscript>("ur_driver/URScript_srv");
}
URScript::~URScript()
{
	ROS_INFO("~URScript");

}
string URScript::movej_joint(double joint_angle[6],float a, float v, float t, float r) //注意：带默认参数的函数,声明和定义两者默认值只能在一个地方出现，不能同时出现。
{
	stringstream temp;   //创建一个流
	string cmd;
	temp<<"movej(["<<joint_angle[0]<<","<<joint_angle[1]<<","<<joint_angle[2]<<","<<joint_angle[3]<<","<<joint_angle[4]<<","<<joint_angle[5]<<"],"
		<<a<<","<<v<<","<<t<<","<<r<<")";
	cmd = temp.str();

	return cmd;
}
string URScript::movej_pose(double tcp_pose[6],float a, float v, float t, float r) 
{
	stringstream temp;   
	string cmd;
	temp<<"movej(p["<<tcp_pose[0]<<","<<tcp_pose[1]<<","<<tcp_pose[2]<<","<<tcp_pose[3]<<","<<tcp_pose[4]<<","<<tcp_pose[5]<<"],"
		<<a<<","<<v<<","<<t<<","<<r<<")";
	cmd = temp.str();
	return cmd;
}

string URScript::movel_pose(double tcp_pose[6],float a, float v, float t, float r) 
{
	stringstream temp;   
	string cmd;
	temp<<"movej(p["<<tcp_pose[0]<<","<<tcp_pose[1]<<","<<tcp_pose[2]<<","<<tcp_pose[3]<<","<<tcp_pose[4]<<","<<tcp_pose[5]<<"],"
		<<a<<","<<v<<","<<t<<","<<r<<")";
	cmd = temp.str();
	return cmd;
}
int main(int argc, char **argv)
{

	ros::init(argc, argv, "ur_pub_");

	cout << "---------------------\n";
	cout << "mode 1  :homepose    \n";
	cout << "mode 2  :readypose   \n";
	cout << "mode 3  :test_pose    \n";
	cout << "mode 4  :test_gripper\n";
	cout << "mode 5  :homepose    \n";
	cout << "---------------------\n";

	cout << " please choose a mode : \n";
	int  mode_x;
	cin >> mode_x;
	cout << "mode numder  =" << mode_x << "\n";

	URScript ur;
	
	if( mode_x == 1 ) {
		ROS_INFO(" mode 1 : homepose");  //初始位置 关节控制
		ur.cmd_str << "movej([-1.571,-0.314,-2.618,0.0,1.571, 0.0],1.4,1.05,5)\n";
	}
	
	if( mode_x == 2 ) {
		ROS_INFO(" mode 2 : readyhose");
		ur.cmd_str  << "movej([-1.571,-1.571,-1.571,-1.571, 1.571, 0.0], 1.4 , 1.05 ,5)\n"; 
		//ur.cmd_str << "movel(get_inverse_kin(p[0.109,-0.487,0.431,0.0,3.141,0.0]),a=0.5, v=0.3)\n";
	}
	
	if( mode_x == 3 ) {
		pt_x = -0.109  ;
		pt_y = -0.487 ;
		pt_z =  0.382  ;  // 400mm 
		ROS_INFO(" mode 3 : test_pose "); //目标点 位置 
		ur.cmd_str << "movej(get_inverse_kin(p["<< pt_x<< ","<<pt_y<< ","<< pt_z<< "," << "0.0,3.141,0.0]),a=0.5, v=0.3)\n";
	}

	if( mode_x == 4 ) {
		ROS_INFO(" mode 4 : test_gripper"); //目标点 位置
		int gripper_open ;
		cout << "please input angle(0,255) :  ";
		cin  >> gripper_open;
		ur.cmd_str << HEADER<<SID <<GTO_0 << POS_0 << gripper_open <<POS_1 << GTO_1 <<ENDING;
	}
	
	
	ur.msg.data = ur.cmd_str.str();	
	ur.srv.request.script = ur.msg.data;
	int count;
	while(!ur.joint_state_client.call(ur.srv) && ros::ok() ) //这个使得一直调用服务，直到成功call到服务.  
	{
		count++;
		std::cout<<"joint_state_client.call(srv)  "<< ur.joint_state_client.call(ur.srv)<<"count = " <<count <<std::endl;
	}
	/*
	ros::Rate loop_rate(125);
	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
	*/
	return 0;
}

