#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/plugins/CameraPlugin.hh>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/fill_image.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/imgproc/imgproc.hpp>

namespace gazebo{
	class ros_camera_plugin : public CameraPlugin{
		public:
			//From https://github.com/lucasw/vimjay/blob/master/src/standalone/cv_distort_image.cpp
			// TODO(lucasw) mirror initUndistortRectifyMap as much as makes sense
			// Add m1type parameter
			// And R rectification matrix? 
			// Need to investigate non-floating point version.
			void initDistortMap(
				const float fx, const float fy, const float cx, const float cy,
				const cv::Mat distCoeffs,
				const cv::Size size,
				cv::Mat& map1, cv::Mat& map2
			){
				cv::Mat pixel_locations_src = cv::Mat(size.width * size.height, 1, CV_32FC2);

				int ind = 0;
				for (int i = 0; i < size.height; i++) {
					for (int j = 0; j < size.width; j++) {
						// TODO(lucasw) maybe would want x and y offsets here to make the output
						// image bigger or smaller than the input?
						pixel_locations_src.at<cv::Point2f>(ind, 0) = cv::Point2f(j,i);
						++ind;
					}
				}
				
				cv::Mat K = (cv::Mat1d(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
				cv::Mat fractional_locations_dst = cv::Mat(pixel_locations_src.size(), CV_32FC2);
				cv::undistortPoints(pixel_locations_src, fractional_locations_dst, K, distCoeffs);

				// TODO(lucasw) is there a faster way to do this?
				// A matrix operation?
				// If the x and y were separate matrices it would be possible,
				// and remap does take separate x and y maps.
				cv::Mat pixel_locations_dst = cv::Mat(size, CV_32FC2);
				ind = 0;
				for (int i = 0; i < size.height; i++) {
					for (int j = 0; j < size.width; j++) {
						const float x = fractional_locations_dst.at<cv::Point2f>(ind, 0).x * fx + cx;
						const float y = fractional_locations_dst.at<cv::Point2f>(ind, 0).y * fy + cy;
						pixel_locations_dst.at<cv::Point2f>(i,j) = cv::Point2f(x,y);
						// if ((i == 0) && (j == 0))
						//  ROS_INFO_STREAM(ind << ": " << i << " " << j << ", " << y << " " << x);
						++ind;
					}
				}

				map1 = pixel_locations_dst;
				map2 = cv::Mat();
			}
		
		
		
			sensors::SensorPtr sp;
		
			ros::NodeHandle nh;
			ros::Publisher img_pub;
			ros::Publisher ci_pub;
			std::string tf_parent;
			
			bool good;
			bool publish_ci;
			
			double k1, k2, k3, p1, p2;
			double fx, fy, cx, cy;
			
			cv::Mat map1, map2;
  
			void Load(sensors::SensorPtr s, sdf::ElementPtr e){
				CameraPlugin::Load(s, e);
				this->sp = s;
				
				if(!e->HasElement("ImageTopicName")){
					good = false;
					printf("\e[91mMissing required argument ImageTopicName\e[39m\n");
					return;
				}
				std::string topic_name = e->Get<std::string>("ImageTopicName");
				if(!e->HasElement("fx")){
					good = false;
					printf("\e[91mMissing required argument fx\e[39m\n");
					return;
				}
				fx = e->Get<double>("fx");
				if(!e->HasElement("fy")){
					good = false;
					printf("\e[91mMissing required argument fy\e[39m\n");
					return;
				}
				fy = e->Get<double>("fy");
				if(!e->HasElement("cx")){
					good = false;
					printf("\e[91mMissing required argument cx\e[39m\n");
					return;
				}
				cx = e->Get<double>("cx");
				if(!e->HasElement("cy")){
					good = false;
					printf("\e[91mMissing required argument cy\e[39m\n");
					return;
				}
				cy = e->Get<double>("cy");
				
				std::string ci_topic_name;
				publish_ci = false;
				if(e->HasElement("CITopicName")){
					publish_ci = true;
					ci_topic_name = e->Get<std::string>("CITopicName");
				}
				
				k1 = 0;
				k2 = 0;
				k3 = 0;
				p1 = 0;
				p2 = 0;
				
				if(e->HasElement("k1")){
					k1 = e->Get<double>("k1");
				}
				if(e->HasElement("k2")){
					k2 = e->Get<double>("k2");
				}
				if(e->HasElement("k3")){
					k3 = e->Get<double>("k3");
				}
				if(e->HasElement("p1")){
					p1 = e->Get<double>("p1");
				}
				if(e->HasElement("p2")){
					p2 = e->Get<double>("p2");
				}
				
				

				this->camera->UpdateCameraIntrinsics(fx, fy, cx, cy, 0);
			
				int a = 0;//No, it will NOT just accept an argument size of 0 without shenanigans. Annoying.
				ros::init(a, (char **) NULL, "~");
				
				img_pub = nh.advertise<sensor_msgs::Image>(topic_name,1);
				if(publish_ci){
					ci_pub = nh.advertise<sensor_msgs::CameraInfo>(ci_topic_name,1);
				}
				
				if(k1!=0 || k2!=0 || k3!=0 || p1!=0 || p2!=0){
					cv::Mat K = (cv::Mat1d(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
					cv::Mat D = (cv::Mat1d(1, 5) << k1, k2, p1, p2, k3);
					
					initDistortMap(
						fx, fy, cx, cy, D,
						cv::Size(this->camera->ImageWidth(), this->camera->ImageHeight()),
						this->map1, this->map2
					);
					/*printf("MAP 1\n");
					std::cout << map1 << "\n\n";
					printf("MAP 2\n");
					std::cout << map2 << "\n\n";*/
				}
				
				good = true;
			}
			
			void OnNewFrame(
				const unsigned char *_image,												   		
				unsigned int _width, unsigned int _height,
				unsigned int _depth, const std::string &_format
			){
				if(!good){
					return;
				}
																  		
				common::Time sensor_update_time = this->sp->LastMeasurementTime();
				
				//This is copied more or less in its entirety from
				//https://github.com/ros-simulation/gazebo_ros_pkgs/blob/kinetic-devel/gazebo_plugins/src/gazebo_ros_camera_utils.cpp
				//Line 438 on
				std::string type;
				int skip;
				if (_format == "L8" || _format == "L_INT8"){
					type = sensor_msgs::image_encodings::MONO8;
					skip = 1;
				} else if (_format == "L16" || _format == "L_INT16"){
					type = sensor_msgs::image_encodings::MONO16;
					skip = 2;
				} else if (_format == "R8G8B8" || _format == "RGB_INT8"){
					type = sensor_msgs::image_encodings::RGB8;
					skip = 3;
				} else if (_format == "B8G8R8" || _format == "BGR_INT8"){
					type = sensor_msgs::image_encodings::BGR8;
					skip = 3;
				} else if (_format == "R16G16B16" ||  _format == "RGB_INT16"){
					type = sensor_msgs::image_encodings::RGB16;
					skip = 6;
				} else if (_format == "BAYER_RGGB8"){
					ROS_INFO_NAMED("camera_utils", "bayer simulation maybe computationally expensive.");
					type = sensor_msgs::image_encodings::BAYER_RGGB8;
					skip = 1;
				} else if (_format == "BAYER_BGGR8"){
					ROS_INFO_NAMED("camera_utils", "bayer simulation maybe computationally expensive.");
					type = sensor_msgs::image_encodings::BAYER_BGGR8;
					skip = 1;
				} else if (_format == "BAYER_GBRG8"){
					ROS_INFO_NAMED("camera_utils", "bayer simulation maybe computationally expensive.");
					type = sensor_msgs::image_encodings::BAYER_GBRG8;
					skip = 1;
				} else if (_format == "BAYER_GRBG8"){
					ROS_INFO_NAMED("camera_utils", "bayer simulation maybe computationally expensive.");
					type = sensor_msgs::image_encodings::BAYER_GRBG8;
					skip = 1;
				} else{
					ROS_ERROR_NAMED("camera_utils", "Unsupported Gazebo ImageFormat\n");
					type = sensor_msgs::image_encodings::BGR8;
					skip = 3;
				}
																  		
				sensor_msgs::Image i;
				i.header.frame_id = this->tf_parent;
				i.header.stamp.sec = sensor_update_time.sec;
				i.header.stamp.nsec = sensor_update_time.nsec;
				fillImage(i, type, _height, _width, skip * _width, reinterpret_cast<const void*>(_image));
				
				if(k1!=0 || k2!=0 || k3!=0 || p1!=0 || p2!=0){
					cv::Mat im_original;
					cv::Mat im_distorted;
					cv_bridge::CvImagePtr cv_ptr;
					try {
						cv_ptr = cv_bridge::toCvCopy(i);
						im_original = cv_ptr->image;
					} catch (cv_bridge::Exception &e) {
						ROS_ERROR("Could not convert from encoding to 'bgr8'.");
						return;
					}
					
					cv::remap(im_original, im_distorted, this->map1, this->map2, cv::INTER_LINEAR);
					
					cv_ptr->image = im_distorted;
					i = *(cv_ptr->toImageMsg());
				}
				
				this->img_pub.publish(i);
				
				
				if(publish_ci){
					sensor_msgs::CameraInfo ci;
					
					ci.height = this->camera->ImageHeight();
					ci.width = this->camera->ImageWidth();
					
					ci.distortion_model = "plumb_bob";
					ci.D = {k1, k2, p1, p2, k3};
					
					//TODO Determine if this will ever actually be changed when alpha changes (when we implement alpha).
					ci.K = {
						fx,	0,	cx,
						0,	fy,	cy,
						0,	0,	1
					};
					
					//TODO Will we ever use a rectification matrix hard-determined?
					ci.R = {
						1, 0, 0,
						0, 1, 0,
						0, 0, 1
					};
					
					//TODO Will we ever use a projection matrix with baseline as opposed to manually calculating binocular constraints?
					ci.P = {
						fx,	0,	cx,	0,
						0,	fy,	cy,	0,
						0,	0,	1,	0
					};
					
					this->ci_pub.publish(ci);
				}
			}
	};
	
	GZ_REGISTER_SENSOR_PLUGIN(ros_camera_plugin)
}
