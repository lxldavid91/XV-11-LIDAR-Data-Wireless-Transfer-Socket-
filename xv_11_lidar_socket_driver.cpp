#include <stdlib.h>
#include <unistd.h>
#include <errno.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/UInt16.h>

using namespace std;

struct lidar_data {

	int32_t rpm[360];
	int32_t ranges[360];
	int32_t intensities[360];

};

void laser_poll(int sockfd, sensor_msgs::LaserScan *scan, std_msgs::UInt16 *rpms) {

	int ret;
	int i;

	int offset = 0;
	struct lidar_data data;

	memset(&data, 0, sizeof(struct lidar_data));

	while(offset < sizeof(struct lidar_data)) {

		ret = recv(sockfd, (char *)&data + offset, sizeof(struct lidar_data) - offset, 0);
		if(ret <= 0) {
			break;
		} else {
			offset += ret;
		}
	}


	int32_t rpm = data.rpm[0];
	rpms->data = rpm;

    scan->angle_min = 0.0;
    scan->angle_max = 2.0*M_PI;
    scan->angle_increment = (2.0*M_PI/360.0);
    scan->time_increment = (rpm == 0 ? (1.0 / 360.0) : (60.0 / rpm / 360.0));
    scan->scan_time = (rpm == 0 ? 1 : 60.0 / rpm);
    scan->range_min = 0.06;
    scan->range_max = 5.0;
    scan->ranges.reserve(360);
    scan->intensities.reserve(360);

    for(i = 0; i < 360; i++) {
    	float range = (data.ranges[i] < 0 ? 0 : data.ranges[i] / 1000.0);
    	scan->ranges.push_back(range);
    	scan->intensities.push_back(data.intensities[i]);
    }

//    printf("Scan received!\n");
}


int main(int argc, char **argv) {

	// roscpp init

	ros::init(argc, argv, "xv_11_lidar_socket_driver");
	ros::NodeHandle n;
	ros::NodeHandle priv_nh("~");

	printf("xv_11_lidar_socket_driver started.\n");

	// configuration parameter
	string address;
	int port;
	string frame_id;

	priv_nh.param("address", address, string("192.168.1.2"));
	priv_nh.param("port", port, 5000);
	priv_nh.param("frame_id", frame_id, string("xv_11_lidar"));


	// connect socket
	int sockfd = -1;
	struct sockaddr_in serv_addr;

	if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		printf("Unable to create socket.\n");
		return -1;
	}

	memset(&serv_addr, 0, sizeof(serv_addr));
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_port = htons(5000);

	if (inet_pton(AF_INET, address.c_str(), &serv_addr.sin_addr) <= 0) {
		printf("Invalid server address\n");
		return -1;
	}

	if (connect(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr))
			< 0) {
		printf("Connect failed.\n");
		return -1;
	}

	printf("Connected to %s:%d\n", address.c_str(), port);


	// publisher
	ros::Publisher laser_pub = n.advertise<sensor_msgs::LaserScan>("scan",
			1000);
	ros::Publisher motor_pub = n.advertise<std_msgs::UInt16>("rpms", 1000);


    while (n.ok()) {
      sensor_msgs::LaserScan scan;
      std_msgs::UInt16 rpms;
      scan.header.frame_id = frame_id;
      scan.header.stamp = ros::Time::now();
      laser_poll(sockfd, &scan, &rpms);
      laser_pub.publish(scan);
      motor_pub.publish(rpms);

    }



	return 0;
}
