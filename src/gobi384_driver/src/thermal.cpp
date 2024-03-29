/*//NBL: ROS Compliance
#include "ros/ros.h"
#include "std_msgs/String.h"*/

#include "stdio.h" // C Standard Input/Output library.
#include "XCamera.h" // Xeneth SDK main header.
#include <iostream>   // std::cout
#include <string>     // std::string, std::to_string
#include <chrono>
#include <ctime>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
// #include <opencv2/core/eigen.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <dynamic_reconfigure/server.h>
#include <ros/package.h>
#include <thermal_camera/TutorialsConfig.h>

//---------Xeneth Camera ------
#include "XCamera.h"
#include "XFilters.h"

using namespace std;
using namespace cv;
using namespace std::chrono;

system_clock::time_point now;
time_t tt;
tm utc_tm;
tm local_tm;
tm last_tm;
int timestamp_modifier_i;
char timestamp_modifier_c;
cv::Mat thermal_img, thermal_img_inv, color_img;
ros::Publisher pub_image,pub_image16;

/*
//NBL: ROS Compliance
std_msgs::String record;*/
string recordData = "1";
bool camerasInitialized;

unsigned int imageCnt;
/*
//NBL: ROS Compliance
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
    record = *msg;
 
    //ROS_INFO("I heard: [%s]", msg->data.c_str());
}
*/
/*
int AcquireImage()
{
}*/

int main(int argc, char **argv) 
{
    // Variables
    XCHANDLE handle = 0; // Handle to the camera
    ErrCode errorCode = 0; // Used to store returned errorCodes from the SDK functions.
    word *frameBuffer = 0; // 16-bit buffer to store the capture frame.
    dword frameSize = 0; // The size in bytes of the raw image.
    ros::init(argc, argv, "imageconvert"); 
    ros::NodeHandle nh;   
    pub_image = nh.advertise<sensor_msgs::Image>("/convert_image8",100);  
    pub_image16 = nh.advertise<sensor_msgs::Image>("/convert_image16",100);  

    // Open a connection to the first detected camera by using connection string cam://0
    printf("Opening connection to cam://0\n");
    handle = XC_OpenCamera("cam://0");

    // When the connection is initialised, ...

    if(XC_IsInitialised(handle))
    {      
        // ... start capturing
        printf("Start capturing.\n");
        if ((errorCode = XC_StartCapture(handle)) != I_OK)
        {
            printf("Could not start capturing, errorCode: %lu\n", errorCode);
        }
        else if (XC_IsCapturing(handle)) // When the camera is capturing ...
        {
            while(ros::ok())
            {
                // Determine native framesize.
                frameSize = XC_GetFrameSize(handle);

                // Initialize the 16-bit buffer.
                frameBuffer = new word[frameSize / 2];
                    // ... grab a frame from the camera.
                printf("Grabbing a frame.\n");
                if ((errorCode = XC_GetFrame(handle, FT_NATIVE, XGF_Blocking, frameBuffer, frameSize)) != I_OK)
                {
                    printf("Problem while fetching frame, errorCode %lu", errorCode);
                }else
                {
                    int h = XC_GetHeight(handle);
                    int w = XC_GetWidth(handle);
                    thermal_img =
                        cv::Mat(h, w, CV_16UC1, frameBuffer); /*convert to OpenCV*/
                    Mat img8;
                    thermal_img.convertTo(img8, CV_8UC1,1 / 256.0); // convert image to 8bit

                    //Mat img8;
                    //normalize(thermal_img, img8, 0, 255, NORM_MINMAX);
                    //convertScaleAbs(img8, img8);
                    //medianBlur(img8,img8,3);
                    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
                    clahe->apply(img8, img8);
                    //cv::equalizeHist(img8,img8);
                        //   imshow("1",thermal_img);
                        // waitKey(1);
                    ros::Time t = ros::Time::now();
                    sensor_msgs::ImagePtr output_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", img8).toImageMsg();
                    output_msg->header.stamp = t;
                    pub_image.publish(output_msg);                    
                    sensor_msgs::ImagePtr msg_thermal = cv_bridge::CvImage(std_msgs::Header(), "mono16", thermal_img).toImageMsg();
                    msg_thermal->header.stamp = t;
                    pub_image16.publish(msg_thermal);
                    if (frameBuffer != 0)
                    {
                        delete [] frameBuffer;
                        frameBuffer = 0;
                    }
                }
            }
            if(XC_IsCapturing(handle))
            {
                // ... stop capturing.
                printf("Stop capturing.\n");
                if ((errorCode = XC_StopCapture(handle)) != I_OK)
                {
                    printf("Could not stop capturing, errorCode: %lu\n", errorCode);
                }
            }

            // When the handle to the camera is still initialised ...
            if (XC_IsInitialised(handle))
            {
                printf("Closing connection to camera.\n");
                XC_CloseCamera(handle);
            }

            printf("Clearing buffers.\n");
            if (frameBuffer != 0)
            {
                delete [] frameBuffer;
                frameBuffer = 0;
            }
        }
    else
    {
        printf("Initialization failed\n");
    }
    }
    return 0;
}
