/**
* @file multi_boards.cpp
* @author Hamdi Sahloul
* @date September 2014
* @version 0.1
* @brief Detect multi-boards simultaneously.
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

class ArSysMultiBoards
{
	private:
		int boards_count;
		cv::Mat inImage;
		aruco::CameraParameters camParam;
		bool useRectifiedImages;
		bool draw_markers;
		bool draw_markers_cube;
		bool draw_markers_axis;
		MarkerDetector mDetector;
		vector<Marker> markers;
		BoardConfiguration* boards_config_array;
		BoardDetector the_board_detector;
		ros::Subscriber cam_info_sub;
		bool cam_info_received;
		image_transport::Publisher image_pub;
		image_transport::Publisher debug_pub;
		ros::Publisher pose_pub;
		ros::Publisher transform_pub; 
		ros::Publisher position_pub;
		std::string* boards_frame_array;

		double* markers_size_array;
		std::string boards_config;
		std::string data_directory;

		ros::NodeHandle nh;
		image_transport::ImageTransport it;
		image_transport::Subscriber image_sub;

		tf::TransformListener _tfListener;

	public:
		ArSysMultiBoards()
			: cam_info_received(false),
			nh("~"),
			it(nh)
		{
			image_sub = it.subscribe("/image", 1, &ArSysMultiBoards::image_callback, this);
			cam_info_sub = nh.subscribe("/camera_info", 1, &ArSysMultiBoards::cam_info_callback, this);

			image_pub = it.advertise("result", 1);
			debug_pub = it.advertise("debug", 1);
			pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 100);
			transform_pub = nh.advertise<geometry_msgs::TransformStamped>("transform", 100);
			position_pub = nh.advertise<geometry_msgs::Vector3Stamped>("position", 100);

			nh.param<std::string>("boards_config", boards_config, "boardConfiguration.yml");
			nh.param<std::string>("data_directory", data_directory, "./data");
			nh.param<bool>("image_is_rectified", useRectifiedImages, true);
			nh.param<bool>("draw_markers", draw_markers, false);
			nh.param<bool>("draw_markers_cube", draw_markers_cube, false);
			nh.param<bool>("draw_markers_axis", draw_markers_axis, false);

			readFromFile(boards_config.c_str());
			ROS_INFO("ArSys node started with boards configuration: %s", boards_config.c_str());
		}

		void readFromFile ( string sfile ) throw ( cv::Exception )
		{
			try
			{
				cv::FileStorage fs ( sfile,cv::FileStorage::READ );
				readFromFile ( fs );
			}
			catch (std::exception &ex)
			{
				throw	cv::Exception ( 81818,"ArSysMultiBoards::readFromFile",ex.what()+string(" file=)")+sfile ,__FILE__,__LINE__ );
			}
		}


		void readFromFile ( cv::FileStorage &fs ) throw ( cv::Exception )
		{
			boards_count = 0;
			//look for the nboards
			if (fs["ar_sys_nboards"].name() != "ar_sys_nboards")
				throw cv::Exception ( 81818,"ArSysMultiBoards::readFromFile","invalid file type" ,__FILE__,__LINE__ );
			fs["ar_sys_nboards"]>>boards_count;

			boards_config_array = new BoardConfiguration[boards_count];
			boards_frame_array = new std::string[boards_count];
			markers_size_array = new double[boards_count];

			cv::FileNode boards=fs["ar_sys_boards"];
			int board_index=0;
			char* path = new char[256];
			for (cv::FileNodeIterator it = boards.begin(); it!=boards.end(); ++it,board_index++ )
			{
				sprintf (path, "%s/%s", data_directory.c_str(), ((std::string)(*it)["path"]).c_str());

				boards_frame_array[board_index] = (std::string)(*it)["frame_id"];
				boards_config_array[board_index].readFromFile(path);
				markers_size_array[board_index] = (double)(*it)["marker_size"];
			}
			delete[] path;
		}

		void image_callback(const sensor_msgs::ImageConstPtr& msg)
		{
			static tf::TransformBroadcaster br;
			if(!cam_info_received) return;

			cv_bridge::CvImagePtr cv_ptr;
			Board board_detected;
			try
			{
				cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
				inImage = cv_ptr->image;

				for (int board_index = 0; board_index < boards_count; board_index++)
				{
					//detection results will go into "markers"
					markers.clear();

					//Ok, let's detect
					mDetector.detect(inImage, markers, camParam, markers_size_array[board_index], false);
					//Detection of the board
					float probDetect=the_board_detector.detect(markers, boards_config_array[board_index], board_detected, camParam, markers_size_array[board_index]);
					if (probDetect > 0.0)
					{
						tf::Transform transform = ar_sys::getTf(board_detected.Rvec, board_detected.Tvec);

						tf::StampedTransform stampedTransform(transform, ros::Time::now(), msg->header.frame_id, boards_frame_array[board_index]);
						br.sendTransform(stampedTransform);
						geometry_msgs::PoseStamped poseMsg;
						tf::poseTFToMsg(transform, poseMsg.pose);
						poseMsg.header.frame_id = msg->header.frame_id;
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
					//for each marker, draw info and its boundaries in the image
					for(size_t i=0; draw_markers && i < markers.size(); ++i)
					{
						markers[i].draw(inImage,cv::Scalar(0,0,255),2);
					}


					if(camParam.isValid() && markers_size_array[board_index] != -1)
					{
						//draw a 3d cube in each marker if there is 3d info
						for(size_t i=0; i<markers.size(); ++i)
						{
							if (draw_markers_cube) CvDrawingUtils::draw3dCube(inImage, markers[i], camParam);
							if (draw_markers_axis) CvDrawingUtils::draw3dAxis(inImage, markers[i], camParam);
						}
						//draw board axis
						if (probDetect > 0.0) CvDrawingUtils::draw3dAxis(inImage, board_detected, camParam);
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
	ros::init(argc, argv, "ar_multi_boards");

	ArSysMultiBoards node;

	ros::spin();
}
