#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float64.h>

#include <math.h>

#include <f1tenth_simulator/scan_range.h>

class Laserscan {
// The class that handles publish a simple lidar node
private:
	ros::NodeHandle n;
	// TODO: create ROS subscribers and publishers
	ros::Subscriber sub_1;

	ros::Publisher pub_1;
	ros::Publisher pub_2;
	ros::Publisher pub_3;

	//sensor_msgs::LaserScan _scan;

public:
	Laserscan () {
		n = ros::NodeHandle("~");

		// TODO: create ROS subscribers and publishers
		sub_1 = n.subscribe("/scan", 1000, &Laserscan::scan_callback, this);

		pub_1 = n.advertise<std_msgs::Float64>("/laser_scan/closest_point", 1000);
		pub_2 = n.advertise<std_msgs::Float64>("/laser_scan/farthest_point", 1000);
		pub_3 = n.advertise<f1tenth_simulator::scan_range>("/scan_range", 1000);
	}

	void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
		double a, b ;

		std_msgs::Float64 _min, _max;
		f1tenth_simulator::scan_range _scan;

		a = scan_msg->ranges[0];
		b = scan_msg->ranges[0];
		for (size_t i = 0; i < scan_msg->ranges.size(); i++) {
			if (!std::isinf(scan_msg->ranges[i]) && !std::isnan(scan_msg->ranges[i])) {
				if( a > scan_msg->ranges[i] )
					a = scan_msg->ranges[i];
				if( b < scan_msg->ranges[i] )
					b = scan_msg->ranges[i];
			}
		}
		_min.data = a;
		_max.data = b;

		_scan.header = scan_msg->header;
		_scan.closest_point.data = a;
		_scan.farthest_point.data = b;

		pub_1.publish(_min);
		pub_2.publish(_max);
		pub_3.publish(_scan);
	}
};


int main(int argc, char ** argv) {
    ros::init(argc, argv, "Laser scan");
    Laserscan in;
    ros::spin();
    return 0;
}
