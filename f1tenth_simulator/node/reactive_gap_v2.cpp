#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ackermann_msgs/AckermannDrive.h>

#include <eigen3/Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>

#include <algorithm>

#define PI 3.1415927

double prev_err = 0.0;
double integral = 0.0;

class Reactive_follow_gap{
private:
	ros::NodeHandle n;

	// Topics & Subscriptions,Publishers
	ros::Subscriber sub_1;		// Subscribe to LIDAR

	ros::Publisher drive_pub;	//pub_1;		// Publish to drive

	sensor_msgs::LaserScan _scan;

	// Lidar Parameters
	double angle_min = 0.0;
	double angle_max = 0.0;
	double angle_increment = 0.0;
	std::vector<double> Ld_angle;

	// Extracting Parameters form the YAML parameter file
	double rb = 0.4;		// get_param("/reactive/bubble_radius")
	double view_ang = 60;	// get_param("/reactive/view_angle")
	double thresh = 1.8;	// get_param("/reactive/threshold")
	double cp_blo = 1.5;	// get_param("/reactive/cp_threshold")
	double d_st_ang = 0.0;	// get_param("/reactive/desired_steering_angle")
	Eigen::Vector3d velocity = Eigen::Vector3d(1,2,3);
							// get_param("/reactive/velocity")
	Eigen::Vector2d steer_lim = Eigen::Vector2d(15,25);
							// get_param("/reactive/steer_angle_limits")

	double kp = 0.7;		// get_param("/reactive/kp")
	double kd = 0.0;		// get_param("/reactive/kd")
	double ki = 0.0;		// get_param("/reactive/ki")

public:
	Reactive_follow_gap(){
        sub_1 = n.subscribe("/scan", 1000, &Reactive_follow_gap::lidar_callback, this);

        // pub_1 = n.advertise<ackermann_msgs::AckermannDriveStamped>("/nav", 1000);
        drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/drive", 1000);
	}

	// Call back for Lidar
	void lidar_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg){
		// Topics & Subscriptions,Publishers
		_scan = *scan_msg;

		// Lidar parameters update according to the Lidar
		angle_min = _scan.angle_min;
		angle_max = _scan.angle_max;
		angle_increment = _scan.angle_increment;

		// Extracting the ranges matrix
		std::vector<double> ranges;
		for(size_t i=0; i<_scan.ranges.size();i++)
			ranges.push_back( _scan.ranges[i]);

		// Preprocessing the range matrix
		std::vector<double> proc_ranges;
		proc_ranges = preprocess_lidar(ranges);

		// Getting the index of the closest point in the processed array
		int cp_idx = std::min_element(proc_ranges.begin(),proc_ranges.end()) - proc_ranges.begin();

		// Creating the Bubble
		proc_ranges = CreateBubble(proc_ranges,cp_idx);

		// Refilter with a certain threshold for a better value
		proc_ranges = ThreshFilter(proc_ranges, thresh);

		// Find max length gap indices
		Eigen::Vector2d mg_idx = find_max_gap(proc_ranges);

		// Find the index of the best point in the gap
		int M_idx = find_best_point(mg_idx);

		// Getting the new current steering angle
		double M_ang = Ld_angle.at(M_idx);

		// Sending the current angle to the PID controller (Desired is always set to zero)
		double st_ang = PIDControl(M_ang);

		// Getting the Velocity according to the steering angle
		double _velocity = getVel(st_ang);

		// Publish Drive message
		publish_val(st_ang,_velocity);
	}

	std::vector<double> preprocess_lidar(std::vector<double> ranges){
		// Getting the index corresponding to the viewing angle parameter
		int p_ind = getAngleIndex(view_ang);
		int n_ind = getAngleIndex(-view_ang);

		// Creating the whole range matrix to keep track of the correct indices
		RealAngleMatrix(n_ind,p_ind);

		// Convertinf ranges to array for ease of operations
		// Processing range to limit the view of the lidar
		std::vector<double> proc_ranges;
		for (size_t i = 0; i < ranges.size(); i++)
		{
			if (i >= n_ind && i < p_ind)
				proc_ranges.push_back(ranges.at(i));
		}

		// Filtering Nan and Inf
		proc_ranges = NanInfFilter(proc_ranges);

		return proc_ranges;//proc_ranges;
	}

	int getAngleIndex(double angle){
		// Final angle of the new matrix converted to radians
		double new_angle = (angle/360.0)*2*PI;

		// Array Created
		int ind = (int)(std::floor((new_angle - _scan.angle_min) / _scan.angle_increment))+1;

		return ind;
	}

	void RealAngleMatrix(int n_ind, int p_ind){
		// Angle matrix for the whole lidar range
		std::vector<double> R_an_array;

		for (double i = angle_min; i < angle_max; i+=angle_increment)
		{
			R_an_array.push_back(i);
		}

		for (size_t i = 0; i < R_an_array.size(); i++)
			// Updating the general Matrix
			if (i >= n_ind && i < p_ind)
				Ld_angle.push_back(R_an_array.at(i));
	}

	std::vector<double> NanInfFilter(std::vector<double> proc_ranges){
		// Filtering Nan and Inf for any array
		std::vector<double> filter;

		for (auto i : proc_ranges){
			if (std::isinf(i) || std::isnan(i))
			{
				//
			}
			else
				filter.push_back(i);
		}
		return filter;
	}

	std::vector<double> CreateBubble(std::vector<double> proc_ranges, int cp_idx){
		// Getting the closest point
		double cp = proc_ranges.at(cp_idx);

		// Calculating the angle for bubble
		double ang = atan2(rb,cp);

		// If the value is Nan, look for more data
		if (std::isinf(ang) || std::isnan(ang))
			sub_1 = n.subscribe("/scan", 1000, &Reactive_follow_gap::lidar_callback, this);

		// Calculating the index value from angle
		int num_idx = int(ang / angle_increment);

		// Calculating the left and right angle
		int l_idx = (cp_idx - num_idx);
		int r_idx = (cp_idx + num_idx);

		// Taking care of negative indices
		if ( l_idx <= 0 )
			l_idx = 0;

		if ( r_idx <= 0 )
			r_idx = 0;
		else if ( r_idx > proc_ranges.size() - 1)
			r_idx = proc_ranges.size() - 1;

		// Changing the lidar data to represent the bubble
		for ( int i = l_idx; i < r_idx; i++)
			proc_ranges.at(i) = 0;

		// Closest point blowup bubble
		double t1 = cp_blo * cp;
		proc_ranges = ThreshFilter(proc_ranges, t1);

		return proc_ranges;
	}

	std::vector<double> ThreshFilter(std::vector<double> proc_ranges, double th){
		// Filtering the value with respect to the threshold in the YAML file
		for (unsigned int i = 0; i < proc_ranges.size(); i++)
			if (proc_ranges.at(i) < th)
				proc_ranges.at(i) = 0;

		return proc_ranges;
	}

	Eigen::Vector2d find_max_gap(std::vector<double> proc_ranges){
		// Detecting where the zeroes are in the matrix
		std::vector<size_t> zero_mat;
		for (size_t i = 0; i < proc_ranges.size(); i++)
			if (proc_ranges.at(i) == 0)
				zero_mat.push_back(i);

		// Getting an array of arrays showing the zero start and end indices
		std::vector<std::vector<size_t>> bound_mat;
		std::vector<size_t> count;
		for (size_t i = 1; i < zero_mat.size(); i++)
		{
			count.push_back(zero_mat.at(i-1));
			if(zero_mat.at(i) - zero_mat.at(i-1) != 1)
			{
				bound_mat.push_back(count);
				count.clear();
			}

			if(i == zero_mat.size() - 1)
			{
				count.push_back(zero_mat.at(i));
				bound_mat.push_back(count);
			}
		}

		// Start and End matrix declarations
		std::vector<int> strt; 	// start is a collection of all indices where the zeroes start
		std::vector<int> end; 	// end is a collection of all indices where the zeroes end

		for( auto k : bound_mat){
			strt.push_back(k.at(0));
			end.push_back(k.at(k.size()-1));
		}

		strt.push_back(proc_ranges.size());		// Putting length of ranges to the end
		end.insert(end.begin(),-1);			// Adding -1 to the start for proper gap length calculation

		// Array for all gap lengths
		std::vector<int> comp;
		comp.push_back(0);

		// Getting the indices of the max gap
		int s_idx = 0, e_idx = 0;

		for( size_t j = 0; j < strt.size(); j++)
		{
			int sz = strt[j] - end[j] - 1;
			comp.push_back(sz);

			if (comp.at(j+1) > comp.at(j))
			{
				s_idx = end.at(j);
				e_idx = strt.at(j);
			}
		}
		return Eigen::Vector2d(s_idx,e_idx);
	}

	int find_best_point(Eigen::Vector2d mg_idx){
		// Getting the middle index
		int start_i = mg_idx.x();
		int end_i = mg_idx.y();
		int M_idx = (int)((start_i + end_i)/2);

		return M_idx;
	}

	double PIDControl(double M_ang){
		double err, diff, sign;

		// Error calculation
		err = M_ang - d_st_ang;

		// Differential term
		diff = err - prev_err;

		// Integral term
		integral = integral + err;

		// Making the intergral term zero at the crossover
		sign = err*prev_err;

		if (sign <= 0)
			integral = 0;

		// Storing error for next loop
		prev_err = err;

		return (kp * err + kd * diff + ki * integral);
	}

	double getVel(double angle){
		// Conversion to dergees for ease of classification
		double deg = (180*angle)/PI;
		double vel;

		// Choosing velocities
		if (std::abs(deg) <= steer_lim.x())
			vel = velocity.z();
		else if (std::abs(deg) > steer_lim.x() && std::abs(deg) <= steer_lim.y())
			vel = velocity.y();
		else
			vel = velocity.x();

		return vel;
	}

	void publish_val(double angle, double _velocity){
		ackermann_msgs::AckermannDriveStamped drive_msg;
		drive_msg.header.stamp = ros::Time::now();
		drive_msg.header.frame_id = "laser";
		drive_msg.drive.steering_angle = angle;
		drive_msg.drive.speed = _velocity;
		drive_pub.publish(drive_msg);
	}
};

int main(int argc, char ** argv) {
	ros::init(argc, argv, "FollowGap_node");
	Reactive_follow_gap rfg;
	//ros::Duration(0.1).sleep();
	ros::spin();
	return 0;
}
