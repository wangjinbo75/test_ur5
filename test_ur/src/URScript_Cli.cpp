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
	8. 把 z = 400mm 作为 零点
	9. 2018年 09月 27日 星期四 09:07:53 CST  把成员函数 写在里面 希望把二维码识别 重新做一遍  获取当前的位置 ur里面设置机座 为参考系
other:
	vim 快捷键: %s/^\s\+ 去掉整个行首的空格
	59,79/^\+\s
	缩进
	按v进入visual状态，选择多行，用>或<缩进或缩出
	2. 通常根据语言特征使用自动缩进排版：在命令状态下对当前行用== （连按=两次）
	ur.cmd_str << "stopl(5.0)\n";
	ur.cmd_str << "movel(get_inverse_kin(p[0.109,-0.487,0.431,0.0,3.141,0.0]),a=0.5, v=0.3)\n";
	ros::Rate loop_rate(125);
	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

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
#include <thread>

#include "ur_msgs/urscript.h"
#include "ar_track_alvar_msgs/AlvarMarkers.h"
#include "sensor_msgs/JointState.h"

using namespace std;

class URScript
{
public:

	URScript()
	{
		ROS_INFO("URScript");
		joint_state_client = nh_.serviceClient<ur_msgs::urscript>("ur_driver/URScript_srv");
		ar_sub = nh_.subscribe("/ar_pose_marker", 1, &URScript::AR_Callback,this);
		tcp_pose_sub =nh_.subscribe("/joint_states",1,&URScript::Tcp_Pose_Callback,this);
	}
	~URScript()
	{
		ROS_INFO("~URScript");
	}
	string movej_joint(double joint_angle[6],float a=1.3,float v=0.3,float t=0,float r=0)//移动到基座标系下指定位置,参数为关节角
	{
		stringstream temp;   
		string cmd;
		temp<<"movej(["<<joint_angle[0]<<","<<joint_angle[1]<<","<<joint_angle[2]<<","<<joint_angle[3]<<","<<joint_angle[4]<<","<<joint_angle[5]<<"],"
			<<a<<","<<v<<","<<t<<","<<r<<")";
		cmd = temp.str();
		return cmd;
	}
	string movej_pose(double tcp_pose[6],float a=1.3,float v=0.3,float t=0,float r=0)  //移动到基座标系下指定位置,参数为TCP位置
	{
		stringstream temp;   
		string cmd;
		temp<<"movej(p["<<tcp_pose[0]<<","<<tcp_pose[1]<<","<<tcp_pose[2]<<","<<tcp_pose[3]<<","<<tcp_pose[4]<<","<<tcp_pose[5]<<"],"
			<<a<<","<<v<<","<<t<<","<<r<<")";
		cmd = temp.str();
		return cmd;
	}
	string movel_pose(double tcp_pose[6],float a=1.3,float v=0.3,float t=0,float r=0) //移动到基座标系下指定位置,线性插值方式
	{
		stringstream temp;   
		string cmd;
		temp<<"movej(p["<<tcp_pose[0]<<","<<tcp_pose[1]<<","<<tcp_pose[2]<<","<<tcp_pose[3]<<","<<tcp_pose[4]<<","<<tcp_pose[5]<<"],"
			<<a<<","<<v<<","<<t<<","<<r<<")";
		cmd = temp.str();
		return cmd;
	}
	void get_actual_tcp_pose() 
	{
		const std::string POSE="def read():\n a=get_actual_tcp_pose()\n end\n read()\n";
		stringstream temp;   
		string cmd;
		temp << POSE;
		
		cmd = temp.str();
	        Send_Script_Commands( cmd );
		cout<<"Send_Script_Commands( pose );"<<endl;
	}
	string test_gripper(int gripper_open)
	{
		const std::string HEADER="def gripper_action():\n socket_close(\"1\")\n sync()\n socket_open(\"127.0.0.1\",63352,\"1\")\n sync()\n";
		const std::string ENDING="socket_close(\"G\")\nend\n";
		const std::string SID   ="socket_set_var(\"SID\", 9, \"1\")\n sync()\n" ; 
		const std::string GTO_0 ="socket_set_var(\"GTO\", 0, \"1\")\n sync()\n" ; 
		const std::string POS_0 ="socket_set_var(\"POS\"," ;
		const std::string POS_1 =",\"1\")\n sync()\n" ;
		const std::string GTO_1	="socket_set_var(\"GTO\", 1, \"1\")\n sync()\n" ;

		stringstream temp;   
		string cmd;

		temp << HEADER<<SID <<GTO_0 << POS_0 << gripper_open <<POS_1 << GTO_1 <<ENDING;
		cmd = temp.str();
		return cmd;
	}
	void Send_Script_Commands(string cmd)
	{
		srv.request.script = cmd ;	
		while(!joint_state_client.call(srv) && ros::ok() ) //这个使得一直调用服务，直到成功call到服务.  
		{
			cout<<"joint_state_client.call(srv)  "<< joint_state_client.call(srv)<<endl;
		}
	}
	string Selection_Mode(int mode_x)
	{
		string str;
		switch ( mode_x )  {
			case 1 :{
					ROS_INFO(" mode 1 : homepose"); 
					double joint_angle[6] = {-1.571,-0.314,-2.618,0.0,1.571, 0.0};
					str = movej_joint( joint_angle , 1.4 , 1.05, 6.0, 0.0);
					break;
				}
			case 2 :{
					ROS_INFO(" mode 2 : readyhose");
					double joint_angle[6] = {-1.571,-1.571,-1.571,-1.571, 1.571, 0.0};
					str = movej_joint( joint_angle , 1.4 , 1.05, 6.0, 0.0);
					break;
				}
			case 3 :{
					ROS_INFO(" mode 3 : test_pose "); 
					double tcp_pose[6] = {-0.109, -0.487, 0.432, 0.0, 3.141, 0.0};
					str = movej_pose(tcp_pose , 1.4 , 1.05, 6.0, 0.0);
					break;
				}
			case 4 :{
					ROS_INFO(" mode 4 : test_gripper"); 
					int gripper_open ;
					cout << "please input angle(0,255) :  ";
					cin  >> gripper_open;
					str = test_gripper( gripper_open );
					break;
				}
			case 5 :{
					Movement_To_The_Target_Point();
			        }
		}
		return str;
	}
	void Movement_To_The_Target_Point( )
	{
			cout <<"joint position : ="<< joint_pose[0] << "\n";
			cout <<"tcp position : ="<< tcp_pose[0] << "\n";
	}
	void Execute_Selection_Mode()
	{
		cout << "---------------------\n";
		cout << "mode 1  :homepose    \n";
		cout << "mode 2  :readypose   \n";
		cout << "mode 3  :test_pose   \n";
		cout << "mode 4  :test_gripper\n";
		cout << "mode 5  :Print coordinate position \n";
		cout << "---------------------\n";
		while(ros::ok()) {
			cout << " please choose a mode : \n";
			int  mode_x ;
			cin >> mode_x;
			cout << "mode numder  = " << mode_x << "\n";
			Send_Script_Commands( Selection_Mode (mode_x) ) ;	
		}
	}
private:
	void AR_Callback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg)
	{	
		if( msg->markers.size() > 0  ){
			pt0[0] = msg->markers[0].pose.pose.position.x;
			pt0[1] = msg->markers[0].pose.pose.position.y;
			pt0[2] = msg->markers[0].pose.pose.position.z;
			pt0[3] = msg->markers[0].pose.pose.orientation.z ;
			
			ROS_INFO("p_x = %f,p_y = %f,p_z = %f,p_rz = %f",pt0[0],pt0[1],pt0[2],pt0[3]);
		}
	}
	void Tcp_Pose_Callback(const sensor_msgs::JointState::ConstPtr& msg)
	{
		if( msg->position.size() > 0 )
		{
			for(int i = 0; i < 6 ; i++) {
			  	joint_pose[i] = msg->position[i];
			  	tcp_pose[i] = msg->effort[i];
			}
			cout <<"joint position : ="<< joint_pose << "\n";
			cout <<"tcp position : ="<< tcp_pose << "\n";
		}
	}
protected:
	ros::NodeHandle nh_;
	ros::ServiceClient joint_state_client;
	ros::Subscriber ar_sub;
	ros::Subscriber tcp_pose_sub;
	ur_msgs::urscript srv;
	double pt0[4];
	double tcp_pose[6];
	double joint_pose[6];
};
int main(int argc, char **argv)
{
	ros::init(argc, argv, "ur_pub_");
	ros::NodeHandle nh;

	URScript ur;
	ur.Execute_Selection_Mode();

	ros::waitForShutdown();
	return 0;
}

