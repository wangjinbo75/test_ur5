/*******************************
FileName: ur_control.cpp
Time: 2018年 08月 21日 星期二 09:34:24 CST
Author: fish
Description:
	1.目的是socket 通信部分
	2.const void* 指针
	3.c++ 与 socket 通信部分调好了 2018年 08月 28日 星期二 00:49:38 CST
	4.二维码的坐标信息  c_str()  函数要清楚会用 	ss怎么清空　,怎么只取前几位
	5.把通信写成成员函数 成员变量　　进行处理
	6.视觉抓取搞定 　2018年 08月 28日 星期二 11:42:29 CST
	7.订阅go goal 目标点
other:
	vim 快捷键: %s/^\s\+ 去掉空格
//	const char *s0 = msg.data.c_str();
//	std::string s1 ("(1.2,1.3)");
	char s1[] = "(1.2,1.3)";
//	s1 << "(1.2,1.3)";
	//msg.data = ss.str();
//      std::cout<<msg.markers.size()<< std::endl;
//    ROS_INFO("target_position = [ %1.2f, %1.2f, %1.3f ]", msg.markers[0].pose.pose.position.x,msg.markers[0].pose.pose.position.y,msg.markers[0].pose.pose.position.z);
//    ROS_INFO("target_orientation = [ %1.3f, %1.2f, %1.3f ]", msg.markers[0].pose.pose.orientation.x,msg.markers[0].pose.pose.orientation.y,msg.markers[0].pose.pose.orientation.z);
******************************/
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include<actionlib_msgs/GoalStatusArray.h>

#include <iostream>
#include <sstream>
#include <string>

#include <sys/types.h> /* See NOTES */
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

float pt0[4]={0,0,0,0};
int on_goal = 55; //false 111 true 55
bool  send_flag = false;
class URScript
{
	public:
		URScript();
	        ~URScript();
		void  AR_Callback(ar_track_alvar_msgs::AlvarMarkers msg);
		void  status_cb(const actionlib_msgs::GoalStatusArray::ConstPtr & msg);
		void  socket_init();
		
		std_msgs::String msg;
		std::stringstream ss;
		char s0[30];
		const int PORT = 6000;
		int QUEUE = 20;
		int tcpCliSock;	
		int tcpSerSock;
		int go_goal = 11; //

	private:
		ros::NodeHandle n;
		ros::Subscriber ar_sub;
		ros::Subscriber status_sub ;
};
URScript::URScript()
{
 	ROS_INFO("vison_fetch");
	ar_sub   = n.subscribe("/ar_pose_marker", 10, &URScript::AR_Callback,this);
	status_sub = n.subscribe<actionlib_msgs::GoalStatusArray>("/move_base/status",10,&URScript::status_cb,this);
}
URScript::~URScript()
{ 

 	ROS_INFO("~vison_fetch");
}
void URScript::AR_Callback(ar_track_alvar_msgs::AlvarMarkers msg)
{
	if(msg.markers.size() > 0  ){
	   pt0[0] = msg.markers[0].pose.pose.position.x;
           pt0[1] = msg.markers[0].pose.pose.position.y;
           pt0[2] = msg.markers[0].pose.pose.position.z * 2 ;
           pt0[3] = msg.markers[0].pose.pose.orientation.z ;
         
	   int p_x =(int)(pt0[0]*1000);  
	   int p_y =(int)(pt0[1]*1000);  
	   int p_z =(int)(pt0[2]*1000);  
	   int p_rz =(int)(pt0[3]*1000);  
	 
	   pt0[0]= -(float)p_x/1000+0.095;//.108;
	   pt0[1]= (float)p_y/1000-0.635;//-0.480;/
	   pt0[2]= -0.345; //(float)p_z/1000;
	   pt0[3]=(float)p_rz/1000;

	   ROS_INFO("p_x = %f,p_y = %f,p_z = %f,p_rz = %f",pt0[0],pt0[1],pt0[2],pt0[3]);
	 }
}
void URScript::status_cb(const actionlib_msgs::GoalStatusArray::ConstPtr & msg)
{
	if (!msg->status_list.empty()) {
	 //      actionlib_msgs::GoalStatus status_list_entry = msg->status_list[1]; 
		 ROS_INFO("mag->status_list =  %d",(msg->status_list[0]).status) ;
	         if( (msg -> status_list[0]).status == 3 ) {
                              	on_goal  =  55 ;
		 }
	}
}

void URScript::socket_init(){
	

	tcpSerSock = socket(AF_INET, SOCK_STREAM,0);
	//AF_INET指明使用TCP/IP协议族；	//SOCK_STREAM, IPPROTO_TCP具体指明使用TCP协议 
	//保存远程服务器的地址信息        
        //SOCKADDR_IN server;	
	//memset(&server, 0, sizeof(SOCKADDR_IN)); //先将保存地址的server置为全0   //9.20 fish
	
	if(tcpSerSock < 0) {
                 ROS_INFO("ERROR opening socket");
		 close(tcpSerSock);
         }
	struct sockaddr_in serverAddr ;
	serverAddr.sin_family = AF_INET;
	serverAddr.sin_port = htons(PORT);           
	serverAddr.sin_addr.s_addr = inet_addr("192.168.0.10"); 	
        if(bind(tcpSerSock, (struct sockaddr* ) &serverAddr, sizeof(serverAddr))==-1) {
		ROS_INFO("bind");
		close(tcpSerSock);
	}
    	if(listen(tcpSerSock, QUEUE) == -1) {
	        ROS_INFO("listen");
		close(tcpSerSock);
	}
	ROS_INFO ("waiting for connection ...");
	struct sockaddr_in clientAdd;
	socklen_t length = sizeof(clientAdd);  ///成功返回非负描述字，出错返回-1
	tcpCliSock = accept(tcpSerSock, (struct sockaddr*)&clientAdd, &length);
	if( tcpCliSock < 0 ) {
		ROS_INFO("connect");
		close(tcpSerSock);
		close(tcpCliSock);
	}
	else {
		ROS_INFO("socket connect !!!");
	}
	
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "ur5_fetch");
	
	URScript ar;
	ar.socket_init();
	
	ar.ss << "("<<pt0[0]<<","<<pt0[1]<<","<<pt0[2]<<","<<pt0[3]<<","<<on_goal <<")";
	ar.msg.data= ar.ss.str();
	strcpy(ar.s0,ar.msg.data.c_str());
	send(ar.tcpCliSock,ar.s0,sizeof(ar.s0), 0);
		
	ros::Rate loop_rate(60);	
	while(ros::ok())
	{
		ar.ss << "("<<pt0[0]<<","<<pt0[1]<<","<<pt0[2]<<","<<pt0[3]<<","<<on_goal <<")";
		ar.msg.data= ar.ss.str();
		strcpy(ar.s0,ar.msg.data.c_str());
		std::cout<<"send_flag =" << send_flag<<std::endl;
		send(ar.tcpCliSock,ar.s0,sizeof(ar.s0), 0);
	 	ROS_INFO("send : %s msg= len  %ld",ar.s0 ,sizeof(ar.s0));
		memset(ar.s0,0,sizeof(ar.s0));
		memset(pt0,0,sizeof(pt0));
		ar.ss.str("");
		ros::spinOnce();
		loop_rate.sleep();
	}
	close(ar.tcpSerSock);
	close(ar.tcpCliSock);
	return 0;
}
