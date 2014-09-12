/**
* @file single_board.cpp
* @author Hamdi Sahloul
* @date September 2014
* @version 0.1
* @brief ROS version of the example named "simple_board" in the Aruco software package.
*/

#include <iostream>
#include <fstream>
#include <sstream>
#include <aruco/aruco.h>
#include <aruco/boarddetector.h>
#include <aruco/cvdrawingutils.h>

#include <opencv2/core/core.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <ar_sys/utils.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

using namespace aruco;

class ArucoSimpleBoard
{
	private:
		cv::Mat inImage;
		aruco::CameraParameters camParam;
		bool useRectifiedImages;
		bool draw_markers;
		bool draw_markers_cube;
		bool draw_markers_axis;
		MarkerDetector mDetector;
		vector<Marker> markers;
		BoardConfiguration the_board_config;
		BoardDetector the_board_detector;
		Board the_board_detected;
		ros::Subscriber cam_info_sub;
		bool cam_info_received;
		image_transport::Publisher image_pub;
		image_transport::Publisher debug_pub;
		ros::Publisher pose_pub;
		ros::Publisher transform_pub; 
		ros::Publisher position_pub;
		std::string marker_frame;
		std::string camera_frame;
		std::string reference_frame;

		double marker_size;
		std::string board_config;

		ros::NodeHandle nh;
		image_transport::ImageTransport it;
		image_transport::Subscriber image_sub;

		tf::TransformListener _tfListener;

	public:
		ArucoSimpleBoard()
			: cam_info_received(false),
			nh("~"),
			it(nh)
		{
			image_sub = it.subscribe("/image", 1, &ArucoSimpleBoard::image_callback, this);
			cam_info_sub = nh.subscribe("/camera_info", 1, &ArucoSimpleBoard::cam_info_callback, this);

			image_pub = it.advertise("result", 1);
			debug_pub = it.advertise("debug", 1);
			pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 100);
			transform_pub = nh.advertise<geometry_msgs::TransformStamped>("transform", 100);
			position_pub = nh.advertise<geometry_msgs::Vector3Stamped>("position", 100);

			nh.param<double>("marker_size", marker_size, 0.05);
			nh.param<std::string>("board_config", board_config, "boardConfiguration.yml");
			nh.param<std::string>("reference_frame", reference_frame, "");
			nh.param<std::string>("camera_frame", camera_frame, "");
			nh.param<std::string>("marker_frame", marker_frame, "");
			nh.param<bool>("image_is_rectified", useRectifiedImages, true);
			nh.param<bool>("draw_markers", draw_markers, false);
			nh.param<bool>("draw_markers_cube", draw_markers_cube, false);
			nh.param<bool>("draw_markers_axis", draw_markers_axis, false);

			ROS_ASSERT(camera_frame != "" && marker_frame != "");

			the_board_config.readFromFile(board_config.c_str());
			if ( reference_frame.empty() )
				reference_frame = camera_frame;

			ROS_INFO("Aruco node started with marker size of %f m and board configuration: %s",
					 marker_size, board_config.c_str());
			ROS_INFO("Aruco node will publish pose to TF with %s as parent and %s as child.",
					 reference_frame.c_str(), marker_frame.c_str());
		}

		bool getTransform(const std::string& refFrame,
			const std::string& childFrame,
			tf::StampedTransform& transform)
		{
			std::string errMsg;

			if (!_tfListener.waitForTransform
				(refFrame,
					childFrame,
					ros::Time(0),
					ros::Duration(0.5),
					ros::Duration(0.01),
					&errMsg)
			 )
			{
				ROS_ERROR_STREAM("Unable to get pose from TF: " << errMsg);
				return false;
			}
			else
			{
				try
				{
					_tfListener.lookupTransform( refFrame, childFrame,
					ros::Time(0), //get latest available
					transform);
				}
				catch ( const tf::TransformException& e)
				{
					ROS_ERROR_STREAM("Error in lookupTransform of " << childFrame << " in " << refFrame);
					return false;
				}

			}
			return true;
		}


		void image_callback(const sensor_msgs::ImageConstPtr& msg)
		{
			static tf::TransformBroadcaster br;
			if(!cam_info_received) return;

			cv_bridge::CvImagePtr cv_ptr;
			try
			{
				cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
				inImage = cv_ptr->image;

				//detection results will go into "markers"
				markers.clear();
				//Ok, let's detect
				mDetector.detect(inImage, markers, camParam, marker_size, false);
				//Detection of the board
				float probDetect=the_board_detector.detect(markers, the_board_config, the_board_detected, camParam, marker_size);
				//for each marker, draw info and its boundaries in the image
				for(size_t i=0; i<markers.size(); ++i)
				{
					if (draw_markers) markers[i].draw(inImage,cv::Scalar(0,0,255),2);
				}


				if(camParam.isValid() && marker_size != -1)
				{
					//draw a 3d cube in each marker if there is 3d info
					for(size_t i=0; i<markers.size(); ++i)
					{
						if (draw_markers_cube) CvDrawingUtils::draw3dCube(inImage, markers[i], camParam);
						if (draw_markers_axis) CvDrawingUtils::draw3dAxis(inImage, markers[i], camParam);
					}
					if (probDetect > 0.0)
					{
						//draw board axis
						CvDrawingUtils::draw3dAxis(inImage, the_board_detected, camParam);

						tf::Transform transform = ar_sys::getTf(the_board_detected.Rvec, the_board_detected.Tvec);
						tf::StampedTransform cameraToReference;
						cameraToReference.setIdentity();

						if ( reference_frame != camera_frame )
						{
							getTransform(reference_frame,
										 camera_frame,
										 cameraToReference);
						}

						transform = static_cast<tf::Transform>(cameraToReference) * transform;

						tf::StampedTransform stampedTransform(transform, ros::Time::now(), reference_frame, marker_frame);
						br.sendTransform(stampedTransform);
						geometry_msgs::PoseStamped poseMsg;
						tf::poseTFToMsg(transform, poseMsg.pose);
						poseMsg.header.frame_id = reference_frame;
						poseMsg.header.stamp = ros::Time::now();
						pose_pub.publish(poseMsg);

						geometry_msgs::TransformStamped transformMsg;
						tf::transformStampedTFToMsg(stampedTransform, transformMsg);
						transform_pub.publish(transformMsg);

						geometry_msgs::Vector3Stamped positionMsg;
						positionMsg.header = transformMsg.header;
						positionMsg.vector = transformMsg.transform.translation;
						position_pub.publish(positionMsg);
					}
				}

				if(image_pub.getNumSubscribers() > 0)
				{
					//show input with augmented information
					cv_bridge::CvImage out_msg;
					out_msg.header.stamp = ros::Time::now();
					out_msg.encoding = sensor_msgs::image_encodings::RGB8;
					out_msg.image = inImage;
					image_pub.publish(out_msg.toImageMsg());
				}

				if(debug_pub.getNumSubscribers() > 0)
				{
					//show also the internal image resulting from the threshold operation
					cv_bridge::CvImage debug_msg;
					debug_msg.header.stamp = ros::Time::now();
					debug_msg.encoding = sensor_msgs::image_encodings::MONO8;
					debug_msg.image = mDetector.getThresholdedImage();
					debug_pub.publish(debug_msg.toImageMsg());
				}
			}
			catch (cv_bridge::Exception& e)
			{
				ROS_ERROR("cv_bridge exception: %s", e.what());
				return;
			}
		}

		// wait for one camerainfo, then shut down that subscriber
		void cam_info_callback(const sensor_msgs::CameraInfo &msg)
		{
			camParam = ar_sys::getCamParams(msg, useRectifiedImages);
			cam_info_received = true;
			cam_info_sub.shutdown();
		}
};


int main(int argc,char **argv)
{
	ros::init(argc, argv, "ar_single_board");

	ArucoSimpleBoard node;

	ros::spin();
}
