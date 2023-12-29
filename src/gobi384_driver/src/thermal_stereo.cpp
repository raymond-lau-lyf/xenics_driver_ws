/*//NBL: ROS Compliance
#include "ros/ros.h"
#include "std_msgs/String.h"*/

#include "stdio.h"   // C Standard Input/Output library.
#include "XCamera.h" // Xeneth SDK main header.
#include <iostream>  // std::cout
#include <string>    // std::string, std::to_string
#include <chrono>
#include <ctime>
#include <thread>
#include <mutex>
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


// image_transport::Publisher image_pub;
ros::Publisher pub_image1, pub_image_clahe1, pub_image16_1;
ros::Publisher pub_image2, pub_image_clahe2, pub_image16_2;



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

void cameraThreadFirst(ros::NodeHandle nh)
{
    try{
        // Variables
        XCHANDLE handle = 0;   // Handle to the camera
        ErrCode errorCode = 0; // Used to store returned errorCodes from the SDK functions.
        word *frameBuffer = 0; // 16-bit buffer to store the capture frame.
        dword frameSize = 0;   // The size in bytes of the raw image.
        cv::Mat thermal_img, thermal_img_inv, color_img;

        // Open a connection to the first detected camera by using connection string cam://0
        printf("[cam 1] Opening connection to cam 1 \n");
        // handle = XC_OpenCamera("cam://0");
        handle = XC_OpenCamera("gev://169.254.1.1");


        if (!XC_IsInitialised(handle)) { 
            printf("[cam 1]Could not open a connection to camera.\n");
            throw std::runtime_error("[cam 1]Could not open a connection to camera.");
        }

        std::string config_file;
        nh.getParam("config_file", config_file);
        const char *settings = config_file.c_str();
        
        if (XC_IsInitialised(handle))
        {
            // ... start capturing

            // // Save settings
            // printf("Saving settings.\n");
            // XC_SaveSettings(handle, settings);

            // XC_LoadSettings(handle, settings);
            // printf("[cam 1] Load Settings\n");

            printf("[cam 1] Start capturing.\n");
            if ((errorCode = XC_StartCapture(handle)) != I_OK)
            {
                printf("[cam 1] Could not start capturing, errorCode: %lu\n", errorCode);
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
                    ROS_INFO("[cam 1] Grabbing a frame.\r");
                    if ((errorCode = XC_GetFrame(handle, FT_NATIVE, XGF_Blocking, frameBuffer, frameSize)) != I_OK)
                    {
                        printf("[cam 1] Problem while fetching frame, errorCode %lu", errorCode);
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



                        // // clahe
                        // cv::Mat img8_clahe;
                        // cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(1.0, cv::Size(32, 32));
                        // clahe->apply(img8, img8_clahe);


                        // cv::equalizeHist(img8,img8);
                        //    imshow("1",thermal_img);
                        //  waitKey(1);

                        ros::Time t = ros::Time::now();
                        
                        sensor_msgs::ImagePtr output_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", img8).toImageMsg();
                        output_msg->header.stamp = t;

                        // sensor_msgs::ImagePtr output_msg_clahe = cv_bridge::CvImage(std_msgs::Header(), "mono8", img8_clahe).toImageMsg();
                        // output_msg_clahe->header.stamp = t;

                        sensor_msgs::ImagePtr msg_thermal = cv_bridge::CvImage(std_msgs::Header(), "mono16", thermal_img).toImageMsg();
                        msg_thermal->header.stamp = t;

                        static long long iter_num;
                        if (iter_num % 4 == 0)
                        {
                            // pub_image_clahe1.publish(output_msg_clahe);
                            pub_image1.publish(output_msg);
                            pub_image16_1.publish(msg_thermal);
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
                    printf("[cam 1] Stop capturing.\n");
                    if ((errorCode = XC_StopCapture(handle)) != I_OK)
                    {
                        printf("[cam 1] Could not stop capturing, errorCode: %lu\n", errorCode);
                    }
                }

                // When the handle to the camera is still initialised ...
                if (XC_IsInitialised(handle))
                {
                    printf("[cam 1] Closing connection to camera.\n");
                    XC_CloseCamera(handle);
                }

                printf("[cam 1] Clearing buffers.\n");
                if (frameBuffer != 0)
                {
                    delete[] frameBuffer;
                    frameBuffer = 0;
                }
            }
            else
            {
                printf("[cam 1] Initialization failed\n");
            }
        }


    } catch (const std::exception& e) {
        std::cerr << "[cam1] Error: " << e.what() << std::endl;
    }
}
void cameraThreadSecond(ros::NodeHandle nh)
{
    std::this_thread::sleep_for(std::chrono::seconds(5));
    try{
        // Variables
        XCHANDLE handle = 0;   // Handle to the camera
        ErrCode errorCode = 0; // Used to store returned errorCodes from the SDK functions.
        word *frameBuffer = 0; // 16-bit buffer to store the capture frame.
        dword frameSize = 0;   // The size in bytes of the raw image.
        cv::Mat thermal_img, thermal_img_inv, color_img;

        // Open a connection to the first detected camera by using connection string cam://0
        printf("[cam 2] Opening connection to cam 2 \n");
        // handle = XC_OpenCamera("cam://1");
        handle = XC_OpenCamera("gev://169.254.2.2");


        if (!XC_IsInitialised(handle)) { 
            printf("[cam 2]Could not open a connection to camera.\n");
            throw std::runtime_error("[cam 2]Could not open a connection to camera.");
        }

        std::string config_file;
        nh.getParam("config_file", config_file);
        const char *settings = config_file.c_str();
        
        if (XC_IsInitialised(handle))
        {
            // ... start capturing

            // // Save settings
            // printf("Saving settings.\n");
            // XC_SaveSettings(handle, settings);

            // XC_LoadSettings(handle, settings);
            // printf("[cam 2] Load Settings\n");

            printf("[cam 2] Start capturing.\n");
            if ((errorCode = XC_StartCapture(handle)) != I_OK)
            {
                printf("[cam 2] Could not start capturing, errorCode: %lu\n", errorCode);
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
                    ROS_INFO("[cam 2] Grabbing a frame.\r");
                    if ((errorCode = XC_GetFrame(handle, FT_NATIVE, XGF_Blocking, frameBuffer, frameSize)) != I_OK)
                    {
                        printf("[cam 2] Problem while fetching frame, errorCode %lu", errorCode);
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



                        // // clahe
                        // cv::Mat img8_clahe;
                        // cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(1.0, cv::Size(32, 32));
                        // clahe->apply(img8, img8_clahe);


                        // cv::equalizeHist(img8,img8);
                        //    imshow("1",thermal_img);
                        //  waitKey(1);

                        ros::Time t = ros::Time::now();
                        
                        sensor_msgs::ImagePtr output_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", img8).toImageMsg();
                        output_msg->header.stamp = t;

                        // sensor_msgs::ImagePtr output_msg_clahe = cv_bridge::CvImage(std_msgs::Header(), "mono8", img8_clahe).toImageMsg();
                        // output_msg_clahe->header.stamp = t;

                        sensor_msgs::ImagePtr msg_thermal = cv_bridge::CvImage(std_msgs::Header(), "mono16", thermal_img).toImageMsg();
                        msg_thermal->header.stamp = t;

                        static long long iter_num;
                        if (iter_num % 4 == 0)
                        {
                            // pub_image_clahe1.publish(output_msg_clahe);
                            pub_image2.publish(output_msg);
                            pub_image16_2.publish(msg_thermal);
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
                    printf("[cam 2] Stop capturing.\n");
                    if ((errorCode = XC_StopCapture(handle)) != I_OK)
                    {
                        printf("[cam 2] Could not stop capturing, errorCode: %lu\n", errorCode);
                    }
                }

                // When the handle to the camera is still initialised ...
                if (XC_IsInitialised(handle))
                {
                    printf("[cam 2] Closing connection to camera.\n");
                    XC_CloseCamera(handle);
                }

                printf("[cam 2] Clearing buffers.\n");
                if (frameBuffer != 0)
                {
                    delete[] frameBuffer;
                    frameBuffer = 0;
                }
            }
            else
            {
                printf("[cam 2] Initialization failed\n");
            }
        }


    } catch (const std::exception& e) {
        std::cerr << "[cam 2] Error: " << e.what() << std::endl;
    } 
}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "thermalimagepublisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    // image_pub = it.advertise("/convert_image", 100);
    pub_image1 = nh.advertise<sensor_msgs::Image>("/thermal/cam1/image/8bit", 100);
    pub_image_clahe1 = nh.advertise<sensor_msgs::Image>("/thermal/cam1/image/8bit_clahe", 100);
    pub_image16_1 = nh.advertise<sensor_msgs::Image>("/thermal/cam1/image/16bit", 100);
    

    pub_image2 = nh.advertise<sensor_msgs::Image>("/thermal/cam2/image/8bit", 100);
    pub_image_clahe2 = nh.advertise<sensor_msgs::Image>("/thermal/cam2/image/8bit_clahe", 100);
    pub_image16_2 = nh.advertise<sensor_msgs::Image>("/thermal/cam2/image/16bit", 100);

   

    std::thread firstCameraThread(cameraThreadFirst,nh);
    std::thread secondCameraThread(cameraThreadSecond,nh);

    firstCameraThread.join();
    secondCameraThread.join();


    // When the connection is initialised, ...

   
    return 0;
}
