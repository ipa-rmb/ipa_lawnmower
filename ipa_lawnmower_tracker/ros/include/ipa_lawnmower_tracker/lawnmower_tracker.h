/*!
 *****************************************************************
 * \file
 *
 * \note
 * Copyright (c) 2015 \n
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA) \n\n
 *
 *****************************************************************
 *
* \note
* Repository name: ipa_lawnmower
* \note
* ROS package name: ipa_lawnmower_tracker
 *
 * \author
 * Author: Richard Bormann
 * \author
 * Supervised by:
 *
 * \date Date of creation: 12.03.2015
 *
 * \brief
 *
 *
 *****************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer. \n
 * - Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution. \n
 * - Neither the name of the Fraunhofer Institute for Manufacturing
 * Engineering and Automation (IPA) nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission. \n
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

#ifndef LAWNMOWER_TRACKER_H
#define LAWNMOWER_TRACKER_H

#include <iostream>

// ROS
#include "ros/ros.h"

// messages
#include "cob_object_detection_msgs/DetectionArray.h"
#include <image_transport/image_transport.h>

// tf
//#include <tf/tf.h>


class LawnmowerTracker
{
public:
	LawnmowerTracker(ros::NodeHandle& nh);
	~LawnmowerTracker();

private:

	void callback(const cob_object_detection_msgs::DetectionArray::ConstPtr& detections_msg);

	void publishVideoFile(const std::string& video_file);

	ros::NodeHandle node_handle_;
	ros::Subscriber detections_sub_;
	image_transport::ImageTransport it_;
	image_transport::CameraPublisher video_pub_;

	bool from_file_;
	std::string video_file_;
};

#endif // LAWNMOWER_TRACKER_H
