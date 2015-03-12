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

#include "ipa_lawnmower_tracker/lawnmower_tracker.h"

#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"

#include <opencv2/highgui/highgui.hpp>

LawnmowerTracker::LawnmowerTracker(ros::NodeHandle& nh)
		: node_handle_(nh), it_(nh)
{
	// parameters
	node_handle_.param("lawnmower_tracker/from_file", from_file_, false);
	std::cout << "Param: from_file=" << from_file_ << std::endl;
	node_handle_.param<std::string>("lawnmower_tracker/video_file", video_file_, "");
	std::cout << "Param: video_file_=" << video_file_ << std::endl;

	// subscribers
	detections_sub_ = node_handle_.subscribe("detections", 0, &LawnmowerTracker::callback, this);

	// publishers
	if (from_file_ == true)
	{
		video_pub_ = it_.advertiseCamera("video_frames", 1);
		publishVideoFile(video_file_);
	}
}

LawnmowerTracker::~LawnmowerTracker()
{

}

void LawnmowerTracker::callback(const cob_object_detection_msgs::DetectionArray::ConstPtr& detections_msg)
{
}

void LawnmowerTracker::publishVideoFile(const std::string& video_file)
{
	cv::VideoCapture cap(video_file);
	if (cap.isOpened() == false)
		return;

	std::cout << "Video opened." << std::endl;

	cv::Mat frame;
	int counter = 1;
	ros::Rate loop_rate(10);
	while (cap.read(frame)==true)
	{
		// prepare image message
		cv_bridge::CvImage cv_ptr;
		cv_ptr.encoding = sensor_msgs::image_encodings::BGR8;
		cv_ptr.header.frame_id = "camera_link";
		cv_ptr.header.seq = counter;
		cv_ptr.header.stamp = ros::Time::now();
		cv_ptr.image = frame;
		// prepare info message
		sensor_msgs::CameraInfoPtr info_msg(new sensor_msgs::CameraInfo);
		info_msg->header = cv_ptr.header;
		info_msg->height = frame.rows;
		info_msg->width = frame.cols;
		info_msg->distortion_model = "plumb_bob";
		info_msg->D.resize(5, 0.f);
		info_msg->K[0] = 1216.; info_msg->K[1] = 0.; info_msg->K[2] = 960.; info_msg->K[3] = 0.; info_msg->K[4] = 1216.; info_msg->K[5] = 540.; info_msg->K[6] = 0.; info_msg->K[7] = 0.; info_msg->K[8] = 1.;
		info_msg->R[0] = 1.; info_msg->R[1] = 0.; info_msg->R[2] = 0.; info_msg->R[3] = 0.; info_msg->R[4] = 1.; info_msg->R[5] = 0.; info_msg->R[6] = 0.; info_msg->R[7] = 0.; info_msg->R[8] = 1.;
		info_msg->P[0] = 1216.; info_msg->P[1] = 0.; info_msg->P[2] = 960.; info_msg->P[3] = 0.; info_msg->P[4] = 0.; info_msg->P[5] = 1216.; info_msg->P[6] = 540.; info_msg->P[7] = 0.; info_msg->P[8] = 0.; info_msg->P[9] = 0.; info_msg->P[10] = 0.; info_msg->P[11] = 1.;
		info_msg->binning_x = 0;
		info_msg->binning_y = 0;
		info_msg->roi.x_offset = 0; info_msg->roi.y_offset = 0; info_msg->roi.width = 0; info_msg->roi.height = 0; info_msg->roi.do_rectify = false;
		// publish frame
		video_pub_.publish(cv_ptr.toImageMsg(), info_msg);
		++counter;

		loop_rate.sleep();
	}
	std::cout << "Video finished." << std::endl;
	cap.release();
}
