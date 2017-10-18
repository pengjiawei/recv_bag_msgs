#include <iostream>
#include<string.h>
#include<fcntl.h>
#include<sys/types.h>
#include<sys/socket.h>
#include<stdlib.h>
#include<stdio.h>
#include <netinet/in.h>
#include<arpa/inet.h>
#include <unistd.h>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/tfMessage.h"

//#include "DataSet/DataType/LaserScan.h"

using namespace std;
int times = 0;
std::string ip;
std::string base_scan_port;
std::string tf_port;
typedef struct{
	float angle_min;
	float angle_max;
	float angle_increment;
	float time_increment;
	float scan_time;
	float range_min;
	float range_max;
	int ranges_size;
	float range_array[2048];
}LaserScan_Msg;

LaserScan_Msg laserData;


typedef struct{
	double r_x;
	double r_y;
	double r_z;
	double r_w;

	double t_x;
	double t_y;
	double t_z;
}Transform_Msg;

Transform_Msg transform_data;

void createSocketAndSendLaser(const std::string& port){
	int sock_id;
		struct sockaddr_in serv_addr;
		memset(&serv_addr, 0, sizeof(serv_addr));
		serv_addr.sin_family = AF_INET;
		serv_addr.sin_port = htons(atoi(port.c_str()));

		inet_pton(AF_INET, ip.c_str(), &serv_addr.sin_addr);

		//connect socket
		if ((sock_id = socket(AF_INET, SOCK_STREAM, 0)) < 0) {

			printf("Create socket failed %d\n", sock_id);

		}

		int reuseflag = 1;
		setsockopt(sock_id, SOL_SOCKET, SO_REUSEADDR, (const char*) &reuseflag,
				sizeof(int)); //set reused

		struct linger clsflag;
		clsflag.l_onoff = 1;
		clsflag.l_linger = 2000;
		setsockopt(sock_id, SOL_SOCKET, SO_LINGER, (const struct linger*) &clsflag,
				sizeof(struct linger)); //set closed

		int i_ret = connect(sock_id, (struct sockaddr *) &serv_addr,
				sizeof(serv_addr));

		printf("finish connect\n");
		if (i_ret == -1) {
			printf("Connect socket failed\n");
			close(sock_id);
		}
		char buf[8192];
		memset(buf, 0, 8192);
		printf("laserData angle_min = %.4f\n",laserData.angle_min);
		memcpy(buf, &laserData, sizeof(laserData));
		send(sock_id, buf, sizeof(buf), 0);

		close(sock_id);
}

void createSocketAndSendTF(const std::string& port){
	int sock_id;
		struct sockaddr_in serv_addr;
		memset(&serv_addr, 0, sizeof(serv_addr));
		serv_addr.sin_family = AF_INET;
		serv_addr.sin_port = htons(atoi(port.c_str()));

		inet_pton(AF_INET, ip.c_str(), &serv_addr.sin_addr);

		//connect socket
		if ((sock_id = socket(AF_INET, SOCK_STREAM, 0)) < 0) {

			printf("Create socket failed %d\n", sock_id);

		}

		int reuseflag = 1;
		setsockopt(sock_id, SOL_SOCKET, SO_REUSEADDR, (const char*) &reuseflag,
				sizeof(int)); //set reused

		struct linger clsflag;
		clsflag.l_onoff = 1;
		clsflag.l_linger = 2000;
		setsockopt(sock_id, SOL_SOCKET, SO_LINGER, (const struct linger*) &clsflag,
				sizeof(struct linger)); //set closed

		int i_ret = connect(sock_id, (struct sockaddr *) &serv_addr,
				sizeof(serv_addr));

		printf("finish connect\n");
		if (i_ret == -1) {
			printf("Connect socket failed\n");
			close(sock_id);
		}
		char buf[1024];
		memset(buf, 0, 1024);
		printf("one of the send transform_data t_x = %.4f\n",transform_data.t_x);
		memcpy(buf, &transform_data, sizeof(transform_data));
		send(sock_id, buf, sizeof(buf), 0);

		close(sock_id);
}

void base_scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg) {
	ROS_INFO("times = %d\n",++times);
	ROS_INFO("laser frame id = %s",msg->header.frame_id.c_str());

	int ranges_size = (int)msg->ranges.size();

	laserData.angle_min = msg->angle_min;
	laserData.angle_max = msg->angle_max;
	laserData.angle_increment = msg->angle_increment;
	laserData.time_increment = msg->time_increment;
	laserData.scan_time = msg->scan_time;
	laserData.range_min = msg->range_min;
	laserData.range_max = msg->range_max;
	laserData.ranges_size = ranges_size;

	memcpy(laserData.range_array,&msg->ranges[0],ranges_size * sizeof(float));

	ROS_INFO("laserScan ranges size = %d",sizeof(laserData.range_array)/sizeof(laserData.range_array[0]));
	ROS_INFO("recv angle_min : [%f]", msg->angle_min);
	ROS_INFO("recv angle_max : [%f]", msg->angle_max);
	ROS_INFO("recv ranges size = %d",ranges_size);
	ROS_INFO("recv ranges index 0 = %.4f",msg->ranges[0]);
	ROS_INFO("recv intensity size  = %d",msg->intensities.size());
	ROS_INFO("header = %s", msg->header.frame_id.c_str());
	printf("\n");

	createSocketAndSendLaser(base_scan_port);
}

void tf_callback(const tf::tfMessage::ConstPtr& msg) {

	ROS_INFO("tf transforms size = %d",msg->transforms.size());
	std::string frame_id = msg->transforms[0].header.frame_id;
	std:string child_frame_id = msg->transforms[0].child_frame_id.c_str();
	ROS_INFO("tf child frame id = %s",child_frame_id.c_str());
	ROS_INFO("tf frame id = %s",frame_id.c_str());
	if(strcmp(child_frame_id.c_str(),"base_laser") == 0 && strcmp(frame_id.c_str(),"base_link") == 0){
		double rotation_x = msg->transforms[0].transform.rotation.x;
			double rotation_y = msg->transforms[0].transform.rotation.y;
			double rotation_z = msg->transforms[0].transform.rotation.z;
			double rotation_w = msg->transforms[0].transform.rotation.w;

			double trans_x = msg->transforms[0].transform.translation.x;
			double trans_y = msg->transforms[0].transform.translation.y;
			double trans_z = msg->transforms[0].transform.translation.z;

			ROS_INFO("r x = %.4f",rotation_x);
			ROS_INFO("r y = %.4f",rotation_y);
			ROS_INFO("r z = %.4f",rotation_z);
			ROS_INFO("r w = %.4f",rotation_w);

			ROS_INFO("t x = %.4f",trans_x);
			ROS_INFO("t y = %.4f",trans_y);
			ROS_INFO("t z = %.4f",trans_z);

			transform_data.r_x = rotation_x;
			transform_data.r_y = rotation_y;
			transform_data.r_z = rotation_z;
			transform_data.r_w = rotation_w;

			transform_data.t_x = trans_x;
			transform_data.t_y = trans_y;
			transform_data.t_z = trans_z;

			createSocketAndSendTF(tf_port);
	}else{
		printf("frame id is not ok \n");
	}


	printf("\n");
}


int main(int argc, char* argv[]) {
	ros::init(argc, argv, "listener");
	ros::NodeHandle n;
	bzero(&laserData,sizeof(LaserScan_Msg));
	ros::Subscriber sub = n.subscribe("/base_scan", 1000, base_scan_callback);
	ros::Subscriber sub2 = n.subscribe("tf", 1000, tf_callback);

	ip = argv[1];
	base_scan_port = argv[2];
	tf_port = argv[3];
	printf("ip = %s\n", ip.c_str());
	printf("base_scan_port = %s\n",base_scan_port.c_str());
	printf("tf_port = %s\n",tf_port.c_str());
	ros::spin();
	return 0;
}
