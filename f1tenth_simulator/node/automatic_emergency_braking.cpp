#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include <ackermann_msgs/AckermannDriveStamped.h>
#include <std_msgs/Bool.h>

#include "f1tenth_simulator/ackermann_kinematics.hpp"
// TODO: include ROS msg type headers and libraries

#include "f1tenth_simulator/precompute.hpp"

using namespace racecar_simulator;

class Safety {
// The class that handles emergency braking
private:
    ros::NodeHandle n;
    double speed;
    // TODO: create ROS subscribers and publishers
    ros::Subscriber sub_1;
    ros::Subscriber sub_2;

    ros::Publisher pub_1;
    ros::Publisher pub_2;

    int _scan_beams;
	double _scan_fov, _scan_ang_incr;
    double _wheelbase, _width, _scan_distance_to_base_link;

    // precompute cosines of scan angles
    std::vector<double> _cosines;

    // precompute distance from lidar to edge of car for each beam
    std::vector<double> _car_distances;

    // for collision check
    bool TTC = false;

public:
    Safety() {
        n = ros::NodeHandle("~");
        speed = 0.0;

        /*
         * GET PARAMS
         */
        n.getParam("scan_beams", _scan_beams);
        n.getParam("wheelbase", _wheelbase);
        n.getParam("scan_field_of_view", _scan_fov);
        n.getParam("scan_distance_to_base_link", _scan_distance_to_base_link);
        n.getParam("width", _width);
        n.getParam("wheelbase", _wheelbase);
        _scan_ang_incr = _scan_fov/(_scan_beams-1);

        _cosines = Precompute::get_cosines(_scan_beams, -_scan_fov/2.0, _scan_ang_incr);
        _car_distances = Precompute::get_car_distances(_scan_beams, _wheelbase, _width, _scan_distance_to_base_link, -_scan_fov/2.0, _scan_ang_incr);

        /*
        One publisherd should publish to the /brake topic with an
        ackermann_msgs/AckermannDriveStamped brake message.

        One publisher should publish to the /brake_bool topic with a
        std_msgs/Bool message.

        You should also subscribe to the /scan topic to get the
        sensor_msgs/LaserScan messages and the /odom topic to get
        the nav_msgs/Odometry messages

        The subscribers should use the provided odom_callback and
        scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        */

        // TODO: create ROS subscribers and publishers
        sub_1 = n.subscribe("/scan", 1000, &Safety::scan_callback, this);
        sub_2 = n.subscribe("/odom", 1000, &Safety::odom_callback, this);

        pub_1 = n.advertise<std_msgs::Bool>("/brake_bool", 1000);
        pub_2 = n.advertise<ackermann_msgs::AckermannDriveStamped>("/brake", 1000);
    }

    /// ---------------------- CALLBACK FUNCTIONS ----------------------

    void odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg) {
        // TODO: update current speed
    	speed = odom_msg->twist.twist.linear.x;
    }

    void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
        double ttc_threshold = 0.5;

        // TTC Calculations are done here so the car can be halted in the simulator:
        // to reset TTC
        bool no_collision = true;
        if (speed != 0) {
            for (size_t i = 0; i < scan_msg->ranges.size(); i++) {
                // TTC calculations

                // calculate projected velocity
                double _proj_velocity = speed * _cosines[i];
                double ttc = (scan_msg->ranges[i] - _car_distances[i]) / _proj_velocity;
                //double ttc = scan_msg->ranges[i] / _proj_velocity;
                // if it's small enough to count as a collision
                if ((ttc < ttc_threshold) && (ttc >= 0.0)) {
                    if (!TTC) {
                        //first_ttc_actions();
                    	ROS_INFO("AEB brake has been implemented");
                    	std_msgs::Bool msg_bool;
                    	msg_bool.data = true;
                    	ackermann_msgs::AckermannDriveStamped msg_brake;
                    	msg_brake.drive.speed = 0.0;

                    	msg_brake.drive.steering_angle = 0.0;
                    	msg_brake.drive.steering_angle_velocity = 0.0;
                    	//msg_brake.drive.acceleration = 0.0;
                    	//msg_brake.drive.jerk = 0.0;
                    	pub_1.publish(msg_bool);
                    	pub_2.publish(msg_brake);
                    }

                    no_collision = false;
                    TTC = true;
                }
            }
        }

        // reset TTC
        if (no_collision)
            TTC = false;

        // TODO: publish drive/brake message
    }
};

int main(int argc, char ** argv) {
    ros::init(argc, argv, "AEB");
    Safety sn;
    ros::spin();
    return 0;
}
