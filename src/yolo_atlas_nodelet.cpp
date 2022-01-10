/**
* Copyright 2020 Huawei Technologies Co., Ltd
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at

* http://www.apache.org/licenses/LICENSE-2.0

* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.

* File main.cpp
* Description: dvpp sample main func
*/

#include <thread>
#include <mutex>
#include <condition_variable>

#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/CompressedImage.h>

#include <cv_bridge/cv_bridge.h>

//接收检测器返回信息
#include <yolo_ros/DetectionMessage.h>
#include <yolo_ros/DetectionMessages.h>

#include <iostream>
#include <dirent.h>

#include "object_detect.h"
#include "utils.h"

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h> // 基类Nodelet所在的头文件

using namespace std;

namespace yolo_atlas_nodelet_ns
{
    class YoloAtlasNodelet : public nodelet::Nodelet
    {
    public:
        YoloAtlasNodelet() {}

    private:
        virtual void onInit()
        {
            ros::NodeHandle &nh = getMTNodeHandle();

            sub_img = nh.subscribe("/decompressed_img", 10, &YoloAtlasNodelet::img_callback, this);
            pub_results = nh.advertise<yolo_ros::DetectionMessages>("/untracked_info", 100);

            infer_thread = std::thread(&YoloAtlasNodelet::infer_process, this);
        }

        // Instantiate the target detection class with the parameters of the classification model path and the required width and height of the model input
        ObjectDetect *detect;

        std::thread infer_thread;
        std::mutex m_buf;
        std::condition_variable cond_buf;

        ros::Subscriber sub_img;
        ros::Publisher pub_results;

        std::string pkg_path;
        std::string model_path;

        bool is_init = false;

        queue<sensor_msgs::ImageConstPtr> img_buf;

        void infer_process()
        {
            uint32_t kModelWidth = 416;
            uint32_t kModelHeight = 416;

            pkg_path = ros::package::getPath("yolo_ros");
            model_path = pkg_path + "/model/yolov3.om";

            detect = new ObjectDetect(pkg_path.c_str(), model_path.c_str(), kModelWidth, kModelHeight);

            // Initializes the ACL resource for categorical reasoning, loads the model and requests the memory used for reasoning input
            Result ret = detect->Init();
            if (ret != SUCCESS)
            {
                ROS_ERROR("Classification Init resource failed");
            }

            while (1)
            {
                std::unique_lock<std::mutex> locker(m_buf);
                cond_buf.wait(locker, [this]()
                              { return !img_buf.empty(); });
                sensor_msgs::ImageConstPtr img_msg = img_buf.front();
                img_buf.pop();
                locker.unlock();

                // double start_t = ros::Time::now().toSec();

                cv_bridge::CvImagePtr cv_ptr;
                cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
                cv::Mat input_img = cv_ptr->image;

                //图像预处理&&图像格式转换为davinci框架下模型输入数据
                Result ret = detect->Preprocess(input_img);
                if (ret != SUCCESS)
                {
                    ERROR_LOG("Read file failed, continue to read next");
                }
                //模型推理，并通过指针获取推理结果
                aclmdlDataset *inferenceOutput = nullptr;
                ret = detect->Inference(inferenceOutput);
                if ((ret != SUCCESS) || (inferenceOutput == nullptr))
                {
                    ERROR_LOG("Inference model inference output data failed");
                }

                // Parses the inference output and sends the inference class, location, confidence, and image to the Presenter Server for display
                vector<BoundingBox> detectResults;
                ret = detect->Postprocess(input_img, inferenceOutput, detectResults);
                yolo_ros::DetectionMessages detection_msgs;
                detection_msgs.header = img_msg->header;
                if (ret != SUCCESS)
                {
                    ERROR_LOG("Process model inference output data failed");
                }
                else
                {
                    for (auto &detectResult : detectResults)
                    {
                        //收集检测信息
                        yolo_ros::DetectionMessage temp;
                        temp.x1 = detectResult.lt.x;
                        temp.y1 = detectResult.lt.y;
                        temp.x2 = detectResult.rb.x;
                        temp.y2 = detectResult.rb.y;
                        temp.class_pred = detectResult.attribute;
                        temp.label = detectResult.result_text;
                        temp.score = detectResult.score;
                        detection_msgs.data.push_back(temp);
                        detection_msgs.detection_num++;
                    }
                }
                pub_results.publish(detection_msgs);
                // double end_t = ros::Time::now().toSec();
                // cout << "YOLO cost: " << to_string(end_t - start_t) << " s" << endl;

                std::chrono::milliseconds dura(10);
                std::this_thread::sleep_for(dura);
            }
        }

        void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
        {
            m_buf.lock();
            img_buf.push(img_msg);
            m_buf.unlock();
            cond_buf.notify_one();
        }
    };
    PLUGINLIB_EXPORT_CLASS(yolo_atlas_nodelet_ns::YoloAtlasNodelet, nodelet::Nodelet)
}
