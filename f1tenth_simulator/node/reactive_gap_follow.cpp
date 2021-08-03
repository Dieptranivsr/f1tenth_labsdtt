#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ackermann_msgs/AckermannDrive.h>

#define PI 3.1415927

class Reactive_follow_gap{
private:
	ros::NodeHandle n;

	double angle;
	std::vector<double> ranges;
	unsigned int min_index, max_index;

	unsigned int start, end;

	double min_angle = -70 / 180.0 * PI;
	double max_angle = 70 / 180.0 * PI;

	// TODO: create ROS subscribers and publishers
	ros::Subscriber sub_1;		// Subscribe to LIDAR

	ros::Publisher pub_1;		// Publish to drive
public:
	Reactive_follow_gap(){


        sub_1 = n.subscribe("/scan", 1000, &Reactive_follow_gap::lidar_callback, this);

        pub_1 = n.advertise<ackermann_msgs::AckermannDriveStamped>("/nav", 1000);
	}

	void preprocess_lidar(sensor_msgs::LaserScan _scan_msg){
		// Preprocess the LiDAR scan array. Expert implementation includes:
		// 1. Setting each value to the mean over some window
		// 2. Rejecting high values (eg. > 3m)

		ranges = std::vector<double>(std::begin(_scan_msg.ranges), std::end(_scan_msg.ranges));
		min_index = (unsigned int)(std::floor((min_angle - _scan_msg.angle_min) / _scan_msg.angle_increment));
		max_index = (unsigned int)(std::ceil((max_angle - _scan_msg.angle_min) / _scan_msg.angle_increment));

		for (unsigned int i = min_index; i <= max_index; i++){
			if (std::isinf(_scan_msg.ranges[i]) || std::isnan(_scan_msg.ranges[i]))
				ranges[i] = 0.0;
			else if (_scan_msg.ranges[i] > _scan_msg.range_max)
				ranges[i] = _scan_msg.range_max;
		}
	}

	void find_max_gap(){
		// Return the start index & end index of the max gap in free_space_ranges
		start = min_index;
		end = min_index;
		unsigned int current_start = min_index - 1;
		unsigned int duration = 0;
		unsigned int longest_duration = 0;

		for (unsigned int i = min_index; i <= max_index; i++){
			if (current_start < min_index){
				if (ranges[i] > 0.0)
					current_start = i;
			}
			else if (ranges[i] <=0 ){
				duration = i - current_start;
				if (duration > longest_duration){
					longest_duration = duration;
					start = current_start;
					end = i - 1;
				}
				current_start = min_index - 1;
			}
		}
		if (current_start >= min_index){
			duration = max_index + 1- current_start;
			if (duration > longest_duration){
				longest_duration = duration;
				start = current_start;
				end = max_index;
			}
		}
	}

	void find_best_gap(sensor_msgs::LaserScan _scan_msg){
		// Start_i & end_i are start and end indicies of max-gap range, respectively
		// Return index of best point in ranges
		// Naive: Choose the furthest point within ranges and go there
		double current_max = 0.0;
		for (unsigned int i = start; i <= end; i++){
			if (ranges[i] > current_max){
				current_max = ranges[i];
				angle = _scan_msg.angle_min + i * _scan_msg.angle_increment;
			}
			else if (ranges[i] == current_max){
				if (std::abs(_scan_msg.angle_min + i * _scan_msg.angle_increment) < std::abs(angle))
					angle = _scan_msg.angle_min + i * _scan_msg.angle_increment;
			}
		}
	}

	void lidar_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg){
		// Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
		sensor_msgs::LaserScan _scan = *scan_msg;

		preprocess_lidar(_scan);

		// Find closest point to LiDAR
		unsigned int closest_index = min_index;
		double closest_distance = _scan.range_max * 5;
		for (unsigned int i = min_index; i <= max_index; i++){
			double distance = ranges[i - 2] + ranges[i - 1] + ranges[i] + ranges[i + 1] + ranges[i + 2];
			if (distance < closest_distance){
				closest_distance = distance;
				closest_index = i;
			}
		}

		// Eliminate all points inside 'bubble' (set them to zero)
		unsigned int radius = 150;
		for (unsigned int i = closest_index - radius; i < closest_index + radius + 1; i++)
			ranges[i] = 0;
		// Find max length gap
		find_max_gap();

		// Find the best point in the gap
		find_best_gap(_scan);

		// Publish Drive message
        ackermann_msgs::AckermannDriveStamped car;
        car.drive.steering_angle = angle;
        if (std::abs(angle) > 20.0 / 180.0 * PI) {
        	car.drive.speed = 0.5;
        } else if (std::abs(angle) > 10.0 / 180.0 * PI) {
        	car.drive.speed = 1.0;
        } else {
        	car.drive.speed = 1.5;
        }
        pub_1.publish(car);
	}
};

int main(int argc, char ** argv) {
	ros::init(argc, argv, "Reactive follow gap");
	Reactive_follow_gap rfg;
	ros::spin();
	return 0;
}
