/*//NBL: ROS Compliance
#include "ros/ros.h"
#include "std_msgs/String.h"*/

#include "stdio.h"   // C Standard Input/Output library.
#include "XCamera.h" // Xeneth SDK main header.
#include <iostream>  // std::cout
#include <string>    // std::string, std::to_string
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

#include "sensor_msgs/Image.h"
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/image_encodings.h"

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
// image_transport::Publisher image_pub;
ros::Publisher pub_image, pub_image_clahe, pub_image16;

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
string type2str(int type)
{
    string r;

    uchar depth = type & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (type >> CV_CN_SHIFT);

    switch (depth)
    {
    case CV_8U:
        r = "8U";
        break;
    case CV_8S:
        r = "8S";
        break;
    case CV_16U:
        r = "16U";
        break;
    case CV_16S:
        r = "16S";
        break;
    case CV_32S:
        r = "32S";
        break;
    case CV_32F:
        r = "32F";
        break;
    case CV_64F:
        r = "64F";
        break;
    default:
        r = "User";
        break;
    }

    r += "C";
    r += (chans + '0');

    return r;
}

int main(int argc, char **argv)
{
    // Variables
    XCHANDLE handle = 0;   // Handle to the camera
    ErrCode errorCode = 0; // Used to store returned errorCodes from the SDK functions.
    word *frameBuffer = 0; // 16-bit buffer to store the capture frame.
    dword frameSize = 0;   // The size in bytes of the raw image.
    ros::init(argc, argv, "thermalimagepublisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    // image_pub = it.advertise("/convert_image", 100);
    pub_image = nh.advertise<sensor_msgs::Image>("/thermal/cam1/image/8bit", 100);
    pub_image_clahe = nh.advertise<sensor_msgs::Image>("/thermal/cam1/image/8bit_clahe", 100);
    pub_image16 = nh.advertise<sensor_msgs::Image>("/thermal/cam1/image/16bit", 100);

    // Open a connection to the first detected camera by using connection string cam://0
    printf("Opening connection to cam 1 (IP:192.168.1.167)\n");
    handle = XC_OpenCamera("gev://192.168.1.167");


    if (!XC_IsInitialised(handle)) { 
        printf("Could not open a connection to camera.\n");
        return -1;
    }

    std::string config_file;
    nh.getParam("config_file", config_file);
    const char *settings = config_file.c_str();


    // When the connection is initialised, ...

    if (XC_IsInitialised(handle))
    {
        // ... start capturing

        // // Save settings
        // printf("Saving settings.\n");
        // XC_SaveSettings(handle, settings);

        XC_LoadSettings(handle, settings);
        printf("Load Settings\n");

        printf("Start capturing.\n");
        if ((errorCode = XC_StartCapture(handle)) != I_OK)
        {
            printf("Could not start capturing, errorCode: %lu\n", errorCode);
        }
        else if (XC_IsCapturing(handle)) // When the camera is capturing ...
        {
            while (ros::ok())
            {
                // Determine native framesize.
                frameSize = XC_GetFrameSize(handle);

                // Initialize the 16-bit buffer.
                frameBuffer = new word[frameSize / 2];
                // ... grab a frame from the camera.
                // printf("Grabbing a frame.\n");
                ROS_INFO("Grabbing a frame.\r");
                if ((errorCode = XC_GetFrame(handle, FT_NATIVE, XGF_Blocking, frameBuffer, frameSize)) != I_OK)
                {
                    printf("Problem while fetching frame, errorCode %lu", errorCode);
                }
                else
                {
                    int h = XC_GetHeight(handle);
                    int w = XC_GetWidth(handle);
                    thermal_img =
                        cv::Mat(h, w, CV_16UC1, frameBuffer); /*convert to OpenCV*/
                    Mat img8;
                    string ty = type2str(thermal_img.type());
                    // printf("Matrix: %s %dx%d \n", ty.c_str(), thermal_img.cols, thermal_img.rows);
                    thermal_img.convertTo(img8, CV_8UC1, 1 / 256.0); // convert image to 8bit
                    // Mat img8;
                    // normalize(thermal_img, img8, 0, 255, NORM_MINMAX);
                    // convertScaleAbs(img8, img8);
                    // medianBlur(img8,img8,3);
                    cv::Mat img8_clahe;
                    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(1.0, cv::Size(32, 32));
                    clahe->apply(img8, img8_clahe);
                    // cv::equalizeHist(img8,img8);
                    //    imshow("1",thermal_img);
                    //  waitKey(1);

                    ros::Time t = ros::Time::now();
                    
                    sensor_msgs::ImagePtr output_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", img8).toImageMsg();
                    output_msg->header.stamp = t;

                    sensor_msgs::ImagePtr output_msg_clahe = cv_bridge::CvImage(std_msgs::Header(), "mono8", img8_clahe).toImageMsg();
                    output_msg_clahe->header.stamp = t;

                    sensor_msgs::ImagePtr msg_thermal = cv_bridge::CvImage(std_msgs::Header(), "mono16", thermal_img).toImageMsg();
                    msg_thermal->header.stamp = t;

                    static long long iter_num;
                    if (iter_num % 4 == 0)
                    {
                        pub_image_clahe.publish(output_msg_clahe);
                        pub_image.publish(output_msg);
                        pub_image16.publish(msg_thermal);
                    }
                    iter_num++;

                    if (frameBuffer != 0)
                    {
                        delete[] frameBuffer;
                        frameBuffer = 0;
                    }
                }
            }
            if (XC_IsCapturing(handle))
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
                delete[] frameBuffer;
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
