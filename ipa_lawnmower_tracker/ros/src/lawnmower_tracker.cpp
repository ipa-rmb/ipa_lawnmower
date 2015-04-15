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

#include <fstream>
#include <sstream>

#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"

#include <opencv2/highgui/highgui.hpp>

#include <boost/thread.hpp>

LawnmowerTracker::LawnmowerTracker(ros::NodeHandle& nh)
		: node_handle_(nh), it_(nh), from_file_(false)
{
	// parameters
	node_handle_.param("lawnmower_tracker/other_device", other_device_, false);
	std::cout << "Param: other_device=" << other_device_ << std::endl;
	node_handle_.param<std::string>("lawnmower_tracker/video_device", video_device_, "");
	std::cout << "Param: video_device=" << video_device_ << std::endl;
	node_handle_.param("lawnmower_tracker/playback_frame_rate", playback_frame_rate_, 10.);
	std::cout << "Param: playback_frame_rate=" << playback_frame_rate_ << std::endl;

	// subscribers
	detections_sub_ = node_handle_.subscribe("detections", 0, &LawnmowerTracker::callback, this);

	trajectory_marker_array_publisher_ = node_handle_.advertise<visualization_msgs::Marker>( "trajectory_marker", 0);

	// publishers
	if (other_device_ == true)
	{
		video_pub_ = it_.advertiseCamera("video_frames", 1);
		boost::thread video_publisher_thread(boost::bind(&LawnmowerTracker::publishVideoFile, this, video_device_));
	}
}

LawnmowerTracker::~LawnmowerTracker()
{

}

void LawnmowerTracker::callback(const cob_object_detection_msgs::DetectionArray::ConstPtr& detections_msg)
{
	if (from_file_ == true && frame_received_ == true)
		return;

	for (size_t det_index=0; det_index<detections_msg->detections.size(); ++det_index)
	{
		const cob_object_detection_msgs::Detection& detection = detections_msg->detections[det_index];
		if (detection.label.compare("tag_341")!=0)
			continue;

		// collect trajectory
		trajectory_.push_back(detection.pose);

		// Rviz visualization
		publishMarkerArray();

		// output to file
		std::ofstream file("lawnmower/trajectory.txt", std::ios::out | std::ios::app);
		if (file.is_open())
		{
			file << detection.pose.header.stamp << "\t" << detection.pose.pose.position.x << "\t" << detection.pose.pose.position.y << "\t" << detection.pose.pose.position.z
					<< detection.pose.pose.orientation.w << "\t" << detection.pose.pose.orientation.x << "\t" << detection.pose.pose.orientation.y << "\t" << detection.pose.pose.orientation.z << std::endl;
		}
		file.close();
	}

	frame_received_ = true;
}

void LawnmowerTracker::publishMarkerArray()
{
	// create line strip
	marker_msg_.header = trajectory_.back().header;
	marker_msg_.ns = "trajectory";
	marker_msg_.id = 1;
	marker_msg_.type = visualization_msgs::Marker::LINE_STRIP;
	marker_msg_.action = visualization_msgs::Marker::ADD;
	marker_msg_.color.a = 0.85;
	marker_msg_.color.r = 0;
	marker_msg_.color.g = 1.0;
	marker_msg_.color.b = 0;
	const size_t number_poses = trajectory_.size();
	marker_msg_.points.resize(number_poses);
	for (size_t i=0; i<number_poses; ++i)
		marker_msg_.points[i] = trajectory_[i].pose.position;
	marker_msg_.lifetime = ros::Duration(3600); // 1 second
	marker_msg_.scale.x = 0.01; // line diameter

	trajectory_marker_array_publisher_.publish(marker_msg_);
}

void LawnmowerTracker::publishVideoFile(const std::string& video_device)
{
	cv::VideoCapture cap;
	if (video_device.compare("0")==0 || video_device.compare("1")==0 || video_device.compare("2")==0)
	{
		std::stringstream ss;
		ss << video_device;
		int device_id;
		ss >> device_id;
		cap.open(device_id);
		from_file_ = false;
		cap.set(CV_CAP_PROP_FRAME_WIDTH, 1600);
		cap.set(CV_CAP_PROP_FRAME_HEIGHT, 1200);
		cap.set(CV_CAP_PROP_FPS, playback_frame_rate_);
	}
	else
	{
		from_file_ = true;
		cap.open(video_device);
	}
	if (cap.isOpened() == false)
		return;

	std::cout << "Video opened." << std::endl;

	cv::Mat frame;
	int counter = 1;
	ros::Rate loop_rate(playback_frame_rate_);
	while (cap.read(frame)==true)
	{
		// prepare image message
		cv_bridge::CvImage cv_ptr;
		cv_ptr.encoding = sensor_msgs::image_encodings::BGR8;
		cv_ptr.header.frame_id = "camera_link";
		cv_ptr.header.seq = counter;
		if (from_file_ == true)
		{
			frame_received_ = false;
			cv_ptr.header.stamp = ros::Time(0.001*cap.get(CV_CAP_PROP_POS_MSEC));
		}
		else
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

		if (from_file_ == true)
		{
			while (frame_received_ == false)
			{
				video_pub_.publish(cv_ptr.toImageMsg(), info_msg);
				ros::Rate(10).sleep();
			}
		}
		else
			loop_rate.sleep();
	}
	std::cout << "Video finished." << std::endl;
	cap.release();
}
