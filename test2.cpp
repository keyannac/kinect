#include "../opencv/modules/core/include/opencv2/core.hpp"
#include "../opencv/modules/videoio/include/opencv2/videoio/videoio.hpp"
#include "../opencv/modules/highgui/include/opencv2/highgui.hpp"
#include "../opencv/modules/imgproc/include/opencv2/imgproc.hpp"
#include "../opencv/include/opencv/cv.hpp"
#include "./libfreenect/wrappers/cpp/libfreenect.hpp"
#include<iostream>
#include<stdio.h>
#include "../Documents/pcl/io/pcd_io.h"
#include "../Documents/pcl/point_types.h"
#include "../Documents/pcl/visualization/cloud_viewer.h"

#include <vector>
#include <cmath>
#include <pthread.h>
#include <cv.h>
#include <cxcore.h>


using namespace cv;
using namespace std;

class myMutex {
	public:
		myMutex() {
			pthread_mutex_init( &m_mutex, NULL );
		}
		void lock() {
			pthread_mutex_lock( &m_mutex );
		}
		void unlock() {
			pthread_mutex_unlock( &m_mutex );
		}
	private:
		pthread_mutex_t m_mutex;
};


class MyFreenectDevice : public Freenect::FreenectDevice {
	public:
		MyFreenectDevice(freenect_context *_ctx, int _index)
	 		: Freenect::FreenectDevice(_ctx, _index), m_buffer_depth(FREENECT_DEPTH_11BIT),
			m_buffer_rgb(FREENECT_VIDEO_RGB), m_gamma(2048), m_new_rgb_frame(false),
			m_new_depth_frame(false), depthMat(Size(640,480),CV_16UC1),
			rgbMat(Size(640,480), CV_8UC3, Scalar(0)),
			ownMat(Size(640,480),CV_8UC3,Scalar(0)) {
			
			for( unsigned int i = 0 ; i < 2048 ; i++) {
				float v = i/2048.0;
				v = std::pow(v, 3)* 6;
				m_gamma[i] = v*6*256;
			}
		}
		
		// Do not call directly even in child
		void VideoCallback(void* _rgb, uint32_t timestamp) {
			std::cout << "RGB callback" << std::endl;
			m_rgb_mutex.lock();
			uint8_t* rgb = static_cast<uint8_t*>(_rgb);
			rgbMat.data = rgb;
			m_new_rgb_frame = true;
			m_rgb_mutex.unlock();
		};
		
		// Do not call directly even in child
		void DepthCallback(void* _depth, uint32_t timestamp) {
			std::cout << "Depth callback" << std::endl;
			m_depth_mutex.lock();
			uint16_t* depth = static_cast<uint16_t*>(_depth);
			depthMat.data = (uchar*) depth;
			m_new_depth_frame = true;
			m_depth_mutex.unlock();
		}
		
		bool getVideo(Mat& output) {
			m_rgb_mutex.lock();
			if(m_new_rgb_frame) {
				cv::cvtColor(rgbMat, output, CV_RGB2BGR);
				m_new_rgb_frame = false;
				m_rgb_mutex.unlock();
				return true;
			} else {
				m_rgb_mutex.unlock();
				return false;
			}
		}
		
		bool getDepth(Mat& output) {
				m_depth_mutex.lock();
				if(m_new_depth_frame) {
					depthMat.copyTo(output);
					m_new_depth_frame = false;
					m_depth_mutex.unlock();
					return true;
				} else {
					m_depth_mutex.unlock();
					return false;
				}
			}
	private:
		std::vector<uint8_t> m_buffer_depth;
		std::vector<uint8_t> m_buffer_rgb;
		std::vector<uint16_t> m_gamma;
		Mat depthMat;
		Mat rgbMat;
		Mat ownMat;
		myMutex m_rgb_mutex;
		myMutex m_depth_mutex;
		bool m_new_rgb_frame;
		bool m_new_depth_frame;
};


int main(int argc, char **argv) {
	bool die(false);
	string filename("snapshot");
	string suffix(".png");
	int i_snap(0),iter(0);
	
	Mat depthMat(Size(640,480),CV_16UC1);
	Mat depthf (Size(640,480),CV_8UC1);
	Mat rgbMat(Size(640,480),CV_8UC3,Scalar(0));
	Mat ownMat(Size(640,480),CV_8UC3,Scalar(0));
	Mat pcMap(Size(640,480),CV_32FC3);
/*	
VideoCapture videoReader;
	videoReader.open( CV_CAP_OPENNI );
	videoReader.set( CV_CAP_OPENNI_IMAGE_GENERATOR_OUTPUT_MODE, CV_CAP_OPENNI_VGA_30HZ );
	videoReader.set(CV_CAP_OPENNI_DEPTH_GENERATOR_REGISTRATION,1); 
	//grab a frame
	if (videoReader.grab()){
    		videoReader.retrieve(rgbMat,CV_CAP_OPENNI_BGR_IMAGE);
    		videoReader.retrieve(depthMat,CV_CAP_OPENNI_POINT_CLOUD_MAP);
   		//Here you get the depth
   		int y = 0;
   		int x = 0;
  		cv::Vec3f pt_3d = rgbMat.at<cv::Vec3f>(y,x); //0 - X, 1 - Y, 2 - Z
		for (x = 0; x <480 ;x++){	
			for (y = 0; y<640; y++){
				std::cout<< rgbMat.at<cv::Vec3f>(y,x);
			}
		}
	}else{
  	std::cerr << "Error capturing frame !" << std::endl;
	}		

	// The next two lines must be changed as Freenect::Freenect
	// isn't a template but the method createDevice:
	// Freenect::Freenect<MyFreenectDevice> freenect;
	// MyFreenectDevice& device = freenect.createDevice(0);
	// by these two lines:
*/	
	Freenect::Freenect freenect;
	MyFreenectDevice& device = freenect.createDevice<MyFreenectDevice>(0);
	
	namedWindow("COLORIZED",CV_WINDOW_AUTOSIZE);
	namedWindow("GREYSCALE DEPTH",CV_WINDOW_AUTOSIZE);
	device.startVideo();
	device.startDepth();
	while (!die) {
		device.getVideo(rgbMat);
		device.getDepth(depthMat);
		cv::imshow("COLORIZED", rgbMat);
		depthMat.convertTo(depthf, CV_8UC1, 255.0/2048.0);
		cv::imshow("GREYSCALE DEPTH",depthf);
		char k = cvWaitKey(5);
		if( k == 27 ){
			cvDestroyWindow("COLORIZED");
			cvDestroyWindow("GREYSCALE DEPTH");
			break;
		}
		if( k == 8 ) {
			std::ostringstream file;
			file << filename << i_snap << suffix;
			cv::imwrite(file.str(),rgbMat);
			i_snap++;
		}
		if(iter >= 1000) break;
		iter++;
	}
	
	device.stopVideo();
	device.stopDepth();
	return 0;
pcl::PointCloud<pcl::PointXYZ> cloud;
cv::Mat1s depth_image;
const float dc1= -0.0030711016;
const float dc2=3.3309495161;
float fx_d,fy_d,px_d,py_d;
//From calibration
cloud.width = depth_image.cols; //Dimensions must be initialized to use 2-D indexing
cloud.height = depth_image.rows;
cloud.resize(cloud.width*cloud.height);
for(int v=0; v< depth_image.rows; v++)
//2-D indexing
for(int u=0; u< depth_image.cols; u++) {

float z = 1.0f / (depth_image(v,u)*dc1+dc2);
cloud(u,v).x = z*(u-px_d)/fx_d;
cloud(u,v).y = z*(v-py_d)/fy_d;
cloud(u,v).z = z;
}

}

