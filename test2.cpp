#include "../opencv/modules/core/include/opencv2/core.hpp"
#include "../opencv/modules/videoio/include/opencv2/videoio/videoio.hpp"
#include "../opencv/modules/highgui/include/opencv2/highgui.hpp"
#include "../opencv/modules/imgproc/include/opencv2/imgproc.hpp"
#include "../opencv/include/opencv/cv.hpp"

#include<iostream>
#include<stdio.h>

#include <vector>
#include <cmath>
#include <pthread.h>
#include <cv.hpp>
#include <cxcore.hpp>


using namespace cv;
using namespace std;

struct pixelData{
        uchar red;
        uchar green;
        uchar blue;
        unsigned short depthmm;
};


int main(int argc, char **argv) {
	bool die(false);
	string filename("snapshot");
	string suffix(".png");

        VideoCapture videoReader;
        videoReader.open( CV_CAP_OPENNI );
        videoReader.set( CV_CAP_OPENNI_IMAGE_GENERATOR_OUTPUT_MODE, CV_CAP_OPENNI_VGA_30HZ );
        videoReader.set(CV_CAP_OPENNI_DEPTH_GENERATOR_REGISTRATION,1);
        int width = videoReader.get(CV_CAP_PROP_FRAME_WIDTH);
        int height = videoReader.get(CV_CAP_PROP_FRAME_HEIGHT);
        cout << "Width is: " << width << "\n";
        cout <<"Height is: " << height<< "\n";

        pixelData points [width][height];

        Mat depthMat(Size(width,height),CV_16UC1);
        Mat depthf (Size(width,height),CV_8UC1);
        Mat rgbMat(Size(width,height),CV_8UC3,Scalar(0));
        Mat ownMat(Size(width,height),CV_8UC3,Scalar(0));
        Mat pcMap(Size(width,height),CV_32FC3);

        //grab a frame
	if (videoReader.grab()){

                videoReader.retrieve(rgbMat,CV_CAP_OPENNI_BGR_IMAGE);
                videoReader.retrieve(depthMat,CV_CAP_OPENNI_DEPTH_MAP);
   		//Here you get the depth
                for (int x = 0; x < width ;x++){
                        for (int y = 0; y< height; y++){
                            unsigned short pt_3d = depthMat.at<unsigned short>(y,x); //0 - X, 1 - Y, 2 - Z
                            Vec3b intensity = rgbMat.at<Vec3b>(y,x);

                            //uchar xpix = pt_3d.val[0];
                            //uchar ypix = pt_3d.val[1];
                            unsigned short zpix = (pt_3d);//depth in mm

                            unsigned char blue = intensity.val[0];
                            unsigned char green = intensity.val[1];
                            uchar red = intensity.val[2];
                            points[x][y].blue = blue;
                            points[x][y].green = green;
                            points[x][y].red = red;
                            points[x][y].depthmm = zpix;
                            cout << static_cast<unsigned>(blue) << " ";
                            cout << static_cast<unsigned>(green) << " ";
                            cout << static_cast<unsigned>(red) << " ";
                            cout << static_cast<unsigned>(zpix)<< "\n";
			}
		}
	}else{
  	std::cerr << "Error capturing frame !" << std::endl;
        }
        //for testing to confirm accuracy of data
        for (int y = 0; y < 20; y ++){
                for(int x = 0; x < 20; x++){
                //cout << static_cast<unsigned>(points[x][y].red)<<" ";
		}
                cout <<"\n";
        }
        videoReader.release();

}

