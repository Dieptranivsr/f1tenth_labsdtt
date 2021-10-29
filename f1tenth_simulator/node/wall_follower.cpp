#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ackermann_msgs/AckermannDrive.h>

#include <math.h>


// #WALL FOLLOW PARAMS
#define ANGLE_RANGE 270 			// Hokuyo 10LX has 270 degrees scan
#define DESIRED_DISTANCE_RIGHT 0.9 	// meters
#define DESIRED_DISTANCE_LEFT 1		//0.55   //1.00   //1.2
#define VELOCITY 2.00 			// meters per second
#define CAR_LENGTH 0.50 			// Traxxas Rally is 20 inches or 0.5 meters
#define PI 3.1415927

class WallFollower {
// The class that handles publish a wall follower node
// Implement Wall Following on the car
private:
	ros::NodeHandle n;

	double kp, ki, kd;
	double servo_offset;
	double prev_error, error, integral ;
	double velocity, delta_t;
	double pre_t = ros::Time::now().toSec();

	double b_angle = 90.0 / 180.0 * PI;			//90 bad  // 105 good
	double a_angle = 45.0 / 180.0 * PI;

	double a = 0.0;
	double b = 0.0;

	// TODO: create ROS subscribers and publishers
	ros::Subscriber sub_1;		// Subscribe to LIDAR

	ros::Publisher pub_1;		// Publish to drive

public:
	WallFollower() {
		n = ros::NodeHandle("~");

		kp = 1.5;		// 0.4;   // 1.5
		ki = 0.05;
		kd = 0.25;		// 0.12;  // 0.25
		servo_offset = 0.0;
		prev_error = 0.0;
		error = 0.0;
		integral = 0.0;

		velocity = 0.0;
		delta_t = 0.0;

        sub_1 = n.subscribe("/scan", 1000, &WallFollower::lidar_callback, this);

        pub_1 = n.advertise<ackermann_msgs::AckermannDriveStamped>("/nav", 1000);
	}

	double bound(double angle, double minmax){
		if (angle < -minmax)
			return -minmax;
		if (angle > minmax)
			return minmax;
		return minmax;
	}

	void getRange(sensor_msgs::LaserScan _scan_msg){
		// data: single message from topic /scan
		// angle: between -45 to 225 degrees, where 0 degrees is directly to the right
		// Outputs length in meters to object with angle in lidar scan field of view
		// make sure to take care of nans etc.
		// TODO: implement

		std::cout << "MIN ANGLE : " << _scan_msg.angle_min << " - MAX ANGLE : " << _scan_msg.angle_max << std::endl;

		unsigned int b_index = (unsigned int)(floor((90.0 / 180.0 * PI - _scan_msg.angle_min) / _scan_msg.angle_increment));
		unsigned int a_index;

		if (_scan_msg.angle_min > 45.0 / 180.0 * PI){
			a_angle = _scan_msg.angle_min;
			a_index = 0;
		}
		else
		{
			a_index = (unsigned int)(floor((45.0 / 180.0 * PI - _scan_msg.angle_min) / _scan_msg.angle_increment));
		}

		if ( !std::isinf(_scan_msg.ranges[a_index]) && !std::isnan(_scan_msg.ranges[a_index])){
			a = _scan_msg.ranges[a_index];
		}
		else
		{
			a = 100.0;
		}

		if ( !std::isinf(_scan_msg.ranges[b_index]) && !std::isnan(_scan_msg.ranges[b_index])){
			b = _scan_msg.ranges[b_index];
		}
		else
		{
			b = 100.0;
		}
	}

	void pid_control(){
		double angle = 0.0, _angle = 0.0;
		// TODO: Use kp, ki & kd to implement a PID controller for

		ackermann_msgs::AckermannDriveStamped drive_msg;
		double t_moment = ros::Time::now().toSec();

		delta_t = t_moment - pre_t;
		integral += prev_error * delta_t;
		_angle = -(kp * error + kd * (error - prev_error) / delta_t + ki * integral);

		//angle = bound(_angle, PI);
		angle = _angle;

		pre_t = t_moment;

        if (abs(angle) > 0 && abs(angle) < 10)
            velocity = 1.5;
        else if (abs(angle) > 10 && abs(angle) < 20)
            velocity = 1;
        else
            velocity = 0.5;

        drive_msg.header.stamp = ros::Time::now();
        drive_msg.header.frame_id = "laser";
        drive_msg.drive.steering_angle = angle / 180 * PI; //angle / 180 * PI;
        drive_msg.drive.speed = velocity;
        pub_1.publish(drive_msg);

		std::cout << "Running ... " << angle << std::endl;
	}

	void followLeft(){
		// Follow left wall as per the algorithm
		// TODO:implement

		double t_angle = b_angle - a_angle;

		double alpha = atan((a * cos(t_angle) - b ) / (a* sin(t_angle)));

		double D_t = b * cos(alpha);

		// propose distance which car moved
		double D_t1 = D_t + DESIRED_DISTANCE_RIGHT * sin(alpha);

		error = DESIRED_DISTANCE_LEFT - D_t1;		// TODO: replace with error returned by followLeft
	}

	void lidar_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
		sensor_msgs::LaserScan _scan = *scan_msg;

		getRange(_scan);

		followLeft();

		// send error to pid_control
		pid_control();
	}
};


int main(int argc, char ** argv) {
	ros::init(argc, argv, "Wall Follower");
	WallFollower wf;
	ros::spin();
	return 0;
}
