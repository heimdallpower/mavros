/**
 * @brief SetpointAttitudeMix plugin
 * @file setpoint_attitude_mix.cpp
 * @author Mads Bornebusch <mads.bornebusch@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <mavros/setpoint_mixin.h>
#include <eigen_conversions/eigen_msg.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/Thrust.h>

namespace mavros {
namespace std_plugins {

using SyncPoseTwistThrustPolicy = message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, geometry_msgs::TwistStamped, mavros_msgs::Thrust>;
using SyncPoseTwistThrust = message_filters::Synchronizer<SyncPoseTwistThrustPolicy>;

/**
 * @brief Setpoint attitude mix plugin
 *
 * Send setpoint attitude/orientation/thrust to FCU controller.
 */
class SetpointAttitudeMixPlugin : public plugin::PluginBase,
	private plugin::SetAttitudeTargetMixin<SetpointAttitudeMixPlugin>,
	private plugin::TF2ListenerMixin<SetpointAttitudeMixPlugin> {
public:
	SetpointAttitudeMixPlugin() : PluginBase(),
		sp_nh("~setpoint_attitude_mix"),
		reverse_thrust(false)
	{ }

	void initialize(UAS &uas_) override
	{
		PluginBase::initialize(uas_);

		// main params
		sp_nh.param("reverse_thrust", reverse_thrust, false);

		// thrust msg subscriber to sync
		th_sub.subscribe(sp_nh, "thrust", 1);
		pose_sub.subscribe(sp_nh, "attitude", 1);
		twist_sub.subscribe(sp_nh, "angular_vel", 1);

		/**
		 * @brief Matches messages, even if they have different time stamps,
		 * by using an adaptative algorithm <http://wiki.ros.org/message_filters/ApproximateTime>
		 */
		sync_pose_twist.reset(new SyncPoseTwistThrust(SyncPoseTwistThrustPolicy(10), pose_sub, twist_sub, th_sub));
		sync_pose_twist->registerCallback(boost::bind(&SetpointAttitudeMixPlugin::attitude_pose_twist_cb, this, _1, _2));
	}

	Subscriptions get_subscriptions() override
	{
		return { /* Rx disabled */ };
	}

private:
	friend class SetAttitudeTargetMixin;
	// friend class TF2ListenerMixin;
	ros::NodeHandle sp_nh;

	message_filters::Subscriber<mavros_msgs::Thrust> th_sub;
	message_filters::Subscriber<geometry_msgs::PoseStamped> pose_sub;
	message_filters::Subscriber<geometry_msgs::TwistStamped> twist_sub;

	std::unique_ptr<SyncPoseTwistThrust> sync_pose_twist;

	bool reverse_thrust;
	float normalized_thrust;

	/**
	 * @brief Function to verify if the thrust values are normalized;
	 * considers also the reversed trust values
	 */
	inline bool is_normalized(float thrust){
		if (reverse_thrust) {
			if (thrust < -1.0) {
				ROS_WARN_NAMED("attitude", "Not normalized reversed thrust! Thd(%f) < Min(%f)", thrust, -1.0);
				return false;
			}
		}
		else {
			if (thrust < 0.0) {
				ROS_WARN_NAMED("attitude", "Not normalized thrust! Thd(%f) < Min(%f)", thrust, 0.0);
				return false;
			}
		}

		if (thrust > 1.0) {
			ROS_WARN_NAMED("attitude", "Not normalized thrust! Thd(%f) > Max(%f)", thrust, 1.0);
			return false;
		}
		return true;
	}

	/* -*- mid-level helpers -*- */

	/**
	 * @brief Send attitude setpoint and thrust to FCU attitude controller
	 */
	void send_attitude_quaternion(const ros::Time &stamp, const Eigen::Affine3d &tr, Eigen::Vector3d &ang_vel, const float thrust)
	{
		/**
		 * @note RPY, also bits numbering started from 1 in docs
		 */
		const uint8_t ignore_roll_and_pitch_rate = (3 << 0);

		auto q = ftf::transform_orientation_enu_ned(
					ftf::transform_orientation_baselink_aircraft(Eigen::Quaterniond(tr.rotation()))
					);

		auto av = ftf::transform_frame_ned_enu(ang_vel);


		set_attitude_target(stamp.toNSec() / 1000000,
					ignore_roll_and_pitch_rate,
					q,
					av,
					thrust);
	}

	/* -*- callbacks -*- */


	void attitude_pose_twist_cb(const geometry_msgs::PoseStamped::ConstPtr &pose_msg, const geometry_msgs::TwistStamped::ConstPtr &twist, const mavros_msgs::Thrust::ConstPtr &thrust_msg) 
{
		Eigen::Affine3d tr;
		tf::poseMsgToEigen(pose_msg->pose, tr);
		Eigen::Vector3d ang_vel;
		tf::vectorMsgToEigen(twist->twist.angular, ang_vel);

		if (is_normalized(thrust_msg->thrust))
			send_attitude_quaternion(pose_msg->header.stamp, tr, ang_vel, thrust_msg->thrust);
	}

};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::SetpointAttitudeMixPlugin, mavros::plugin::PluginBase)
