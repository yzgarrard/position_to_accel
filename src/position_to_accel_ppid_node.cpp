#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_local_position_setpoint.hpp>

#include <chrono>

using namespace std::chrono;
using namespace std::chrono_literals;

class PPID : public rclcpp::Node {
public:
	PPID() : Node("garrard_position_to_acceleration") {

		acceleration_setpoint_publisher_ =
			this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("garrard_accelerationsetpoint_pubsub", 1);
		// acceleration_setpoint_publisher_ =
		// 	this->create_publisher<px4_msgs::msg::VehicleLocalPositionSetpoint>("VehicleLocalPositionSetpoint_PubSubTopic", 10);
		offboard_control_mode_publisher_ =
			this->create_publisher<px4_msgs::msg::OffboardControlMode>("OffboardControlMode_PubSubTopic", 1);

		// Get FCU timestamp
		timesync_sub_ =
			this->create_subscription<px4_msgs::msg::Timesync>("Timesync_PubSubTopic", 1,
				[this](const px4_msgs::msg::Timesync::UniquePtr msg) 
				{
					if (msg->sys_id == 1)
					{
                        fcu_timestamp = msg->tc1;
					}
				});

        // Trigger controller update when new information comes from mocap
        vehicle_pose_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
			"VehicleLocalPosition_PubSubTopic", 1, 
            std::bind(&PPID::update_controller, this, std::placeholders::_1));

		// Update trajectory setpoint whenever it becomes available
		trajectory_sp_sub_ = 
			this->create_subscription<px4_msgs::msg::TrajectorySetpoint>("garrard_trajectorysetpoint_pub", 10, 
					[this](const px4_msgs::msg::TrajectorySetpoint::UniquePtr traj_cur) 
					{
						sp_current = *(traj_cur.get());
					});
    }

private:
	rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr timesync_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_pose_sub_;
	rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_sp_sub_;

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr acceleration_setpoint_publisher_;
	// rclcpp::Publisher<px4_msgs::msg::VehicleLocalPositionSetpoint>::SharedPtr acceleration_setpoint_publisher_;
	
    px4_msgs::msg::VehicleLocalPosition vehicle_pose_history[3];
	px4_msgs::msg::VehicleLocalPosition filtered_vehicle_pose_history[3];	// After filtering velocity
	px4_msgs::msg::TrajectorySetpoint sp_current;

	std::atomic<int64_t> fcu_timestamp;

	// Gains for P-PID
	float kpp_xy = 0.95, kpv_xy = 1.8, kiv_xy = 0.4, kdv_xy = 0.2;	// xy position and velocity PID gains
	float kpp_z = 1, kpv_z = 5, kiv_z = 2, kdv_z = 0;	// z position and velocity PID gains

	// Max velocities and acceleration
	float max_vxy = 12;	// max horizontal velocity in m/s
	float max_vz = 1;	// max vertical velocity in m/s
	float max_axy = 5;	// max horizontal acceleration in m/s^2
	float max_az = 4;	// max vertical acceleration in m/s^2

	// Bounds on dt
	float min_dt = 0.00694;
	float max_dt = 0.01042;

	// Integrator accumulators
	float accumulator_x = 0;
	float accumulator_y = 0;
	float accumulator_z = 0;

    void update_controller(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) 
	{
		RCLCPP_INFO(get_logger(), "Updating controller");

        // Have the last three pose information available for later use
        vehicle_pose_history[2] = vehicle_pose_history[1];
        vehicle_pose_history[1] = vehicle_pose_history[0];
        vehicle_pose_history[0] = *msg;

		// Have the last three pose information with filtered velocity for later use
		filtered_vehicle_pose_history[2] = filtered_vehicle_pose_history[1];
        filtered_vehicle_pose_history[1] = filtered_vehicle_pose_history[0];
        filtered_vehicle_pose_history[0] = *msg;

		//=====Calculate dt in seconds===============
		float dt = vehicle_pose_history[0].timestamp_sample - vehicle_pose_history[1].timestamp_sample;
		dt /= 1000000;
		dt = bound(dt, min_dt, max_dt);

		//=====Calculate Position Errors======================

		float err_x = sp_current.x - vehicle_pose_history[0].x;
		float err_y = sp_current.y - vehicle_pose_history[0].y;
		float err_z = sp_current.z - vehicle_pose_history[0].z;

		RCLCPP_INFO(get_logger(), "\nerr_x:\t%4.8f\nerr_y:\t%4.8f\nerr_z:\t%4.8f", err_x, err_y, err_z);
		
        //=====Calculate Velocity Setpoint==========
        
		float sp_vx = err_x * kpp_xy;
		float sp_vy = err_y * kpp_xy;
		float sp_vz = err_z * kpp_z;

		sp_vx = bound(sp_vx, -max_vxy, max_vxy);
		sp_vy = bound(sp_vy, -max_vxy, max_vxy);
		sp_vz = bound(sp_vz, -max_vz, max_vz);

		RCLCPP_INFO(get_logger(),
					"\nsp_vx:\t%4.8f\nsp_vy:\t%4.8f\nsp_vz:\t%4.8f\n",
					sp_vx, sp_vy, sp_vz);

		//=====Filter Velocity================================

		filtered_vehicle_pose_history[0].vx = LPF(	vehicle_pose_history[0].vx,
													vehicle_pose_history[1].vx,
													vehicle_pose_history[2].vx,
													filtered_vehicle_pose_history[1].vx,
													filtered_vehicle_pose_history[2].vx,
													10, dt, 1
												);
		filtered_vehicle_pose_history[0].vy = LPF(	vehicle_pose_history[0].vy,
													vehicle_pose_history[1].vy,
													vehicle_pose_history[2].vy,
													filtered_vehicle_pose_history[1].vy,
													filtered_vehicle_pose_history[2].vy,
													10, dt, 1
												);
		filtered_vehicle_pose_history[0].vz = LPF(	vehicle_pose_history[0].vz,
													vehicle_pose_history[1].vz,
													vehicle_pose_history[2].vz,
													filtered_vehicle_pose_history[1].vz,
													filtered_vehicle_pose_history[2].vz,
													10, dt, 1
												);

		//=====Calculate Velocity Error (UNFILTERED)=============

		float err_vx = sp_vx - vehicle_pose_history[0].vx;
		float err_vy = sp_vy - vehicle_pose_history[0].vy;
		float err_vz = sp_vz - vehicle_pose_history[0].vz;

		RCLCPP_INFO(get_logger(),
					"\nerr_vx:\t%4.8f\nerr_vy:\t%4.8f\nerr_vz:\t%4.8f",
					err_vx, err_vy, err_vz);

        //=====Calculate Acceleration Setpoint======

		// Compute proportional action
		float sp_ax_kpv_action = err_vx * kpv_xy;
		float sp_ay_kpv_action = err_vy * kpv_xy;
		float sp_az_kpv_action = err_vz * kpv_z;

		RCLCPP_INFO(get_logger(),
					"\np_action_ax:\t%4.8f\np_action_ay:\t%4.8f\np_action_az:\t%4.8f",
					sp_ax_kpv_action, sp_ay_kpv_action, sp_az_kpv_action);

		// Compute derivative action
		float vx_delta = (filtered_vehicle_pose_history[0].vx - filtered_vehicle_pose_history[1].vx) * dt;
		float vy_delta = (filtered_vehicle_pose_history[0].vy - filtered_vehicle_pose_history[1].vy) * dt;
		float vz_delta = (filtered_vehicle_pose_history[0].vz - filtered_vehicle_pose_history[1].vz) * dt;
		float sp_ax_kdv_action = vx_delta * kdv_xy;
		float sp_ay_kdv_action = vy_delta * kdv_xy;
		float sp_az_kdv_action = vz_delta * kdv_z;

		RCLCPP_INFO(get_logger(),
					"\nd_action_ax:\t%4.8f\nd_action_ay:\t%4.8f\nd_action_az:\t%4.8f",
					sp_ax_kdv_action, sp_ay_kdv_action, sp_az_kdv_action);


		// Refer to PX4 controller diagram to see why these are subtracted rather than added 
		float sp_ax = sp_ax_kpv_action - sp_ax_kdv_action;
		float sp_ay = sp_ay_kpv_action - sp_ay_kdv_action;
		float sp_az = sp_az_kpv_action - sp_az_kdv_action;

		// Check if current control is within bounds
		float sp_axy = sqrt(sp_ax*sp_ax+sp_ay*sp_ay);
		int sp_axy_sign = isInBound(sp_axy, -max_axy, max_axy);
		int sp_az_sign = isInBound(sp_az, -max_az, max_az);


		// Compute integral action
		float sp_ax_kiv_action = accumulator_x * kiv_xy;;
		float sp_ay_kiv_action = accumulator_y * kiv_xy;;
		float sp_az_kiv_action = accumulator_z * kiv_z;;

		// If adding integral action exceeds maximum, don't add to accumulators
		if (sp_axy_sign == 0 
			&& abs(sp_axy + sqrt(sp_ax_kiv_action*sp_ax_kiv_action+sp_ay_kiv_action*sp_ay_kiv_action) < max_axy))
		{
			RCLCPP_INFO(get_logger(), "Adding to xy accumulator");
			accumulator_x += err_vx * dt;
			accumulator_y += err_vy * dt;
		} 
		if (sp_az_sign == 0 && abs(sp_az + sp_az_kiv_action) < max_az)
		{
			RCLCPP_INFO(get_logger(), "Adding to z accumulator");
			accumulator_z += err_vz * dt;
		}
		sp_ax_kiv_action = accumulator_x * kiv_xy;
		sp_ay_kiv_action = accumulator_y * kiv_xy;
		sp_az_kiv_action = accumulator_z * kiv_z;

		RCLCPP_INFO(get_logger(),
					"\ni_action_ax:\t%4.8f\ni_action_ay:\t%4.8f\ni_action_az:\t%4.8f",
					sp_ax_kiv_action, sp_ay_kiv_action, sp_az_kiv_action);

		// Add integral action to acceleration setpoint
		sp_ax += sp_ax_kiv_action;
		sp_ay += sp_ay_kiv_action;
		sp_az += sp_az_kiv_action;

		RCLCPP_INFO(get_logger(),
					"\nsp_ax:\t%4.8f\nsp_ay:\t%4.8f\nsp_az:\t%4.8f",
					sp_ax, sp_ay, sp_az);

		//========Scale acceleration to thrust=====

		float g = 9.81;
		float scaled_hover_thrust = 0.7;	// Obtained from acuator_controls.csv control[3] under hover conditions
		float scaled_x_thrust = (sp_ax / g) * scaled_hover_thrust;
		float scaled_y_thrust = (sp_ay / g) * scaled_hover_thrust;
		float scaled_z_thrust = (sp_az / g) * scaled_hover_thrust;// - scaled_hover_thrust;

		// Make sure values are scaled between -1...1 (altitude only since it is most important)
		if (abs(scaled_z_thrust) >= 1) 
		{
			scaled_x_thrust /= abs(scaled_z_thrust);
			scaled_y_thrust /= abs(scaled_z_thrust);
			scaled_z_thrust /= abs(scaled_z_thrust);
		}

		//======Set offboard control==============
		px4_msgs::msg::OffboardControlMode offboard_msg{};
		offboard_msg.timestamp = fcu_timestamp;
		offboard_msg.position = false;
		offboard_msg.velocity = false;
		offboard_msg.acceleration = true;
		offboard_msg.attitude = false;
		offboard_msg.body_rate = false;

		offboard_control_mode_publisher_->publish(offboard_msg);

		//=====Create msg and publish============
		px4_msgs::msg::TrajectorySetpoint sp;
		// px4_msgs::msg::VehicleLocalPositionSetpoint sp;
		sp.timestamp = 0;
		sp.x = NAN;
		sp.y = NAN;
		sp.z = NAN;
		// sp.x = 0;
		// sp.y = 0;
		// sp.z = -2;
		sp.yaw = 0;
		sp.yawspeed = NAN;
		sp.vx = NAN;
		sp.vy = NAN;
		sp.vz = NAN;
		// sp.acceleration = {NAN, NAN, NAN};
		sp.acceleration[0] = scaled_x_thrust;
		sp.acceleration[1] = scaled_y_thrust;
		sp.acceleration[2] = scaled_z_thrust;
		sp.jerk[0] = NAN;
		sp.jerk[1] = NAN;
		sp.jerk[2] = NAN;
		sp.thrust = {NAN, NAN, NAN};
		// sp.thrust[0] = scaled_x_thrust;
		// sp.thrust[1] = scaled_y_thrust;
		// sp.thrust[2] = scaled_z_thrust;
		RCLCPP_INFO(get_logger(),
					"\nsp.acc.x:\t%4.8f\nsp.acc.y:\t%4.8f\nsp.acc.z:\t%4.8f",
					sp.acceleration[0], sp.acceleration[1], sp.acceleration[2]);
		RCLCPP_INFO(get_logger(),
					"\nsp.thrust.x:\t%4.8f\nsp.thrust.y:\t%4.8f\nsp.thrust.z:\t%4.8f",
					sp.thrust[0], sp.thrust[1], sp.thrust[2]);
		acceleration_setpoint_publisher_->publish(sp);
		RCLCPP_INFO(get_logger(), "Publishing setpoint");
    }

	/**
	 * @brief Pass a signal through a second-order low-pass filter
	 * 
	 * @param u0 Current input signal value
	 * @param u1 Input signal value at step k-1
	 * @param u2 Input signal value at step k-2
	 * @param y1 Output signal value at step k-1
	 * @param y2 Output signal value at step k-2
	 * @param w Cutoff frequency in rad/s
	 * @param T Time between samples
	 * @param z Damping ratio
	 * @return float Filtered Signal
	 */
	float LPF(float u0, float u1, float u2, float y1, float y2, float w, float T, float z)
	{
		float denominator = T*T*w*w+4*z*T*w+4;
		float u0_coeff = T*T*w*w;
		float u1_coeff = 2*T*T*w*w;
		float u2_coeff = T*T*w*w;
		float y1_coeff = -(2*T*T*w*w - 8);
		float y2_coeff = -(T*T*w*w-4*z*T*w+4);
		float y0 = (1/denominator)*(u0_coeff*u0+u1_coeff*u1+u2_coeff*u2+y1_coeff*y1+y2_coeff*y2);
		return y0;
	}

	/**
	 * @brief If a signal is not within bounds, modify it to the closest valid point
	 * 
	 * @param val Value to bound
	 * @param min_val Minimum permissible value
	 * @param max_val Maximum permissible value
	 * @return float Bounded value
	 */
	float bound(float val, float min_val, float max_val)
	{
		if (val > max_val) val = max_val;
		if (val < min_val) val = min_val;
		return val;
	}

	/**
	 * @brief Checks if a value is below, in, or above permissible bounds
	 * 
	 * @param val Value to check
	 * @param min_val Minimum permissible value
	 * @param max_val Maximum permissible sdf value permisssion 
	 * @return int -1 if below, 0 if in (inclusive), 1 if above
	 */
	int isInBound(float val, float min_val, float max_val)
	{
		int isInBound;
		if (val > max_val) isInBound = 1;
		else if (val < min_val) isInBound = -1;
		else isInBound = 0;

		RCLCPP_INFO(get_logger(), 
					"\nval:\t%4.8f\nmin_val:\t%4.8f\nmax_val:\t%4.8f\nisInBound:\t%d", 
					val, min_val, max_val, isInBound);
		return isInBound;
	}
};

int main(int argc, char* argv[]) {
	std::cout << "Starting garrard_position_to_acceleration node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<PPID>());

	rclcpp::shutdown();
	return 0;
}