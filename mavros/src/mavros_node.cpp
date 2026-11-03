/**
 * @brief MAVROS Node
 * @file mavros_node.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 */
/*
 * Copyright 2013,2014,2015 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros.h>

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "mavros");

	// On simulated time ROS time is 0 until the first /clock, and timesync samples taken before it start the offset
	// filter far off. With the system clock (no /use_sim_time) time is always valid, so this returns at once.
	ros::start();
	ros::Time::waitForValid();

	mavros::MavRos mavros;
	mavros.spin();

	return 0;
}

