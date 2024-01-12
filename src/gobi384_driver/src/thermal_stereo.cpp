/*//NBL: ROS Compliance
#include "ros/ros.h"
#include "std_msgs/String.h"*/

#include "stdio.h"   // C Standard Input/Output library.
#include "XCamera.h" // Xeneth SDK main header.
#include <iostream>  // std::cout
#include <string>    // std::string, std::to_string
#include <chrono>
#include <deque>
#include <ctime>
#include <thread>
#include <condition_variable>
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
#include "xenethSDK_trigger.h"

#include "sensor_msgs/Image.h"
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/image_encodings.h"
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace cv;
using namespace std::chrono;


// image_transport::Publisher image_pub;
ros::Publisher pub_image1, pub_image_clahe1, pub_image16_1;
ros::Publisher pub_image2, pub_image_clahe2, pub_image16_2;
ros::Publisher pub_joint_image;


struct stereo_image_set{
    std::pair<sensor_msgs::ImagePtr,sensor_msgs::ImagePtr> imgs_0;                                                                                                                                                                                                                                                                                                                                   ;                                                                                                                                                                                                                
    std::pair<sensor_msgs::ImagePtr,sensor_msgs::ImagePtr> imgs_1;
    sensor_msgs::ImagePtr joint_image;                                                                                                                                                                                                                
};

sensor_msgs::ImagePtr makeJointImage(const sensor_msgs::ImagePtr &input_left,const sensor_msgs::ImagePtr &input_right)
{
    sensor_msgs::ImagePtr ret = input_right;
    cv::Mat left_cvimg,right_cvimg;
    cv_bridge::CvImagePtr cv_ptr_left,cv_ptr_right,cv_ptr_joint;
    try
    {
        cv_ptr_left = cv_bridge::toCvCopy(input_left, sensor_msgs::image_encodings::MONO8);
        cv_ptr_right = cv_bridge::toCvCopy(input_right, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return ret;
    }

    left_cvimg = cv_ptr_left->image;
    right_cvimg = cv_ptr_right->image;

    // Create a new image with the combined width
    cv::Mat joint_cvimg(left_cvimg.rows, left_cvimg.cols + right_cvimg.cols, left_cvimg.type());

    // Copy the left and right images into the combined image
    left_cvimg.copyTo(joint_cvimg(cv::Rect(0, 0, left_cvimg.cols, left_cvimg.rows)));
    right_cvimg.copyTo(joint_cvimg(cv::Rect(left_cvimg.cols, 0, left_cvimg.cols, right_cvimg.rows)));

    cv_bridge::CvImage cvImage;
    cvImage.encoding = "mono8";
    cvImage.image = joint_cvimg;
    ret = cvImage.toImageMsg();

    return ret;
}


std::deque<struct stereo_image_set> stereo_images;
std::deque<std::pair<sensor_msgs::ImagePtr,sensor_msgs::ImagePtr>> cam1_imgs_buffer;
std::deque<std::pair<sensor_msgs::ImagePtr,sensor_msgs::ImagePtr>> cam2_imgs_buffer;
std::mutex mtx_stereo_images;
std::condition_variable sig_buffer;


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


            /* configure camera in external triggered mode */
            if (!SetupExternalTriggeredMode_F027(handle)) AbortSession();

            /* configure camera to disable the automatic shutter calibration process */
            // if (!SetupShutterControl_F027(handle)) AbortSession();

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
                        // ros::Time t = ros::Time::now();
                        const auto p0 = std::chrono::time_point<std::chrono::high_resolution_clock>{};
                        const auto p3 = std::chrono::high_resolution_clock::now();

                        auto tstamp = p3 - p0;
                        int32_t sec = std::chrono::duration_cast<std::chrono::seconds>(tstamp).count();
                        int32_t nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(tstamp).count() % 1000000000UL;
                        ros::Time t(sec, nsec);

                        int h = XC_GetHeight(handle);
                        int w = XC_GetWidth(handle);
                        thermal_img =
                            cv::Mat(h, w, CV_16UC1, frameBuffer); /*convert to OpenCV*/
                        Mat img8;
                        string ty = type2str(thermal_img.type());
                        // printf("Matrix: %s %dx%d \n", ty.c_str(), thermal_img.cols, thermal_img.rows);
                        thermal_img.convertTo(img8, CV_8UC1, 1 / 256.0); // convert image to 8bit

                        sensor_msgs::ImagePtr image1_8bit_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", img8).toImageMsg();
                        image1_8bit_msg->header.stamp = t;

                        sensor_msgs::ImagePtr image1_16bit_msg = cv_bridge::CvImage(std_msgs::Header(), "mono16", thermal_img).toImageMsg();
                        image1_16bit_msg->header.stamp = t;

                        ROS_INFO("debug: 1 %lf",image1_16bit_msg->header.stamp.toSec());

                        cam1_imgs_buffer.push_back(std::make_pair(image1_8bit_msg,image1_16bit_msg));

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



            /* configure camera in external triggered mode */
            if (!SetupExternalTriggeredMode_F027(handle)) AbortSession();

            /* configure camera to disable the automatic shutter calibration process */
            // if (!SetupShutterControl_F027(handle)) AbortSession();

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
                        // ros::Time t = ros::Time::now();
                        const auto p0 = std::chrono::time_point<std::chrono::high_resolution_clock>{};
                        const auto p3 = std::chrono::high_resolution_clock::now();

                        auto tstamp = p3 - p0;
                        int32_t sec = std::chrono::duration_cast<std::chrono::seconds>(tstamp).count();
                        int32_t nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(tstamp).count() % 1000000000UL;
                        ros::Time t(sec, nsec);


                        int h = XC_GetHeight(handle);
                        int w = XC_GetWidth(handle);
                        thermal_img =
                            cv::Mat(h, w, CV_16UC1, frameBuffer); /*convert to OpenCV*/
                        Mat img8;
                        string ty = type2str(thermal_img.type());
                        // printf("Matrix: %s %dx%d \n", ty.c_str(), thermal_img.cols, thermal_img.rows);
                        thermal_img.convertTo(img8, CV_8UC1, 1 / 256.0); // convert image to 8bit
                        
                        sensor_msgs::ImagePtr image2_8bit_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", img8).toImageMsg();
                        image2_8bit_msg->header.stamp = t;

                        sensor_msgs::ImagePtr image2_16bit_msg = cv_bridge::CvImage(std_msgs::Header(), "mono16", thermal_img).toImageMsg();
                        image2_16bit_msg->header.stamp = t;

                        ROS_INFO("debug: 2 %lf",image2_16bit_msg->header.stamp.toSec());

                        
                        cam2_imgs_buffer.push_back(std::make_pair(image2_8bit_msg,image2_16bit_msg));

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

void pubThread(ros::NodeHandle nh)
{
    while(ros::ok())
    {
        if(stereo_images.empty())
        {
            continue;
        }
        else
        {
            mtx_stereo_images.lock();

            pub_image1.publish(stereo_images.front().imgs_0.first);
            pub_image16_1.publish(stereo_images.front().imgs_0.second);
            pub_image2.publish(stereo_images.front().imgs_1.first);
            pub_image16_2.publish(stereo_images.front().imgs_1.second);
            pub_joint_image.publish(stereo_images.front().joint_image);

            stereo_images.pop_front();
            mtx_stereo_images.unlock();
        }
    }
}

void syncThread(ros::NodeHandle nh)
{
    while(ros::ok())
    {
        if(cam1_imgs_buffer.empty() || cam2_imgs_buffer.empty())
        {
            continue;
        }
        else
        {   
            if(cam1_imgs_buffer.front().first->header.stamp.toSec()<=cam2_imgs_buffer.front().first->header.stamp.toSec())
            {
                if(cam2_imgs_buffer.front().first->header.stamp.toSec() - cam1_imgs_buffer.front().first->header.stamp.toSec() < 0.02)
                {
                    // std::cout<<"1"<<std::endl<<std::endl;
                    cam1_imgs_buffer.front().first->header.stamp =  cam2_imgs_buffer.front().first->header.stamp;
                    cam1_imgs_buffer.front().second->header.stamp =  cam2_imgs_buffer.front().second->header.stamp;
                    struct stereo_image_set stereo_image;
                    stereo_image.imgs_0=cam1_imgs_buffer.front();
                    stereo_image.imgs_1=cam2_imgs_buffer.front();
                    stereo_image.joint_image=makeJointImage(stereo_image.imgs_0.first,stereo_image.imgs_1.first);
                    stereo_image.joint_image->header.stamp = cam2_imgs_buffer.front().first->header.stamp;
                    mtx_stereo_images.lock();
                    stereo_images.push_back(stereo_image);
                    mtx_stereo_images.unlock();
                    cam1_imgs_buffer.pop_front();
                    cam2_imgs_buffer.pop_front();
                }
                else
                {
                    cam1_imgs_buffer.pop_front();
                    continue;
                }
            }
            else
            {
                if(cam1_imgs_buffer.front().first->header.stamp.toSec() - cam2_imgs_buffer.front().first->header.stamp.toSec() < 0.02)
                {
                    // std::cout<<"2"<<std::endl<<std::endl;
                    cam2_imgs_buffer.front().first->header.stamp =  cam1_imgs_buffer.front().first->header.stamp;
                    cam2_imgs_buffer.front().second->header.stamp =  cam1_imgs_buffer.front().second->header.stamp;
                    struct stereo_image_set stereo_image;
                    stereo_image.imgs_0=cam1_imgs_buffer.front();
                    stereo_image.imgs_1=cam2_imgs_buffer.front();
                    stereo_image.joint_image=makeJointImage(stereo_image.imgs_0.first,stereo_image.imgs_1.first);
                    stereo_image.joint_image->header.stamp = cam1_imgs_buffer.front().first->header.stamp;
                    mtx_stereo_images.lock();
                    stereo_images.push_back(stereo_image);
                    mtx_stereo_images.unlock();
                    cam1_imgs_buffer.pop_front();
                    cam2_imgs_buffer.pop_front();
                }
                else
                {
                    cam2_imgs_buffer.pop_front();
                    continue;
                }
            }
        }
    }

}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "thermalimagepublisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    // image_pub = it.advertise("/convert_image", 100);
    pub_image1 = nh.advertise<sensor_msgs::Image>("/thermal/cam1/image8bit", 100);
    pub_image_clahe1 = nh.advertise<sensor_msgs::Image>("/thermal/cam1/image8bit_clahe", 100);
    pub_image16_1 = nh.advertise<sensor_msgs::Image>("/thermal/cam1/image16bit", 100);
    

    pub_image2 = nh.advertise<sensor_msgs::Image>("/thermal/cam2/image8bit", 100);
    pub_image_clahe2 = nh.advertise<sensor_msgs::Image>("/thermal/cam2/image8bit_clahe", 100);
    pub_image16_2 = nh.advertise<sensor_msgs::Image>("/thermal/cam2/image16bit", 100);

    pub_joint_image = nh.advertise<sensor_msgs::Image>("/thermal/joint", 100);

    std::thread pubTopicThread(pubThread,nh);
    std::thread syncimgThread(syncThread,nh);
    std::thread firstCameraThread(cameraThreadFirst,nh);
    std::thread secondCameraThread(cameraThreadSecond,nh);

    pubTopicThread.join();
    syncimgThread.join();
    firstCameraThread.join();
    secondCameraThread.join();


    // When the connection is initialised, ...

   
    return 0;
}
