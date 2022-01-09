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

* File sample_process.h
* Description: handle acl resource
*/
#pragma once

#include <opencv2/opencv.hpp>

#include "utils.h"
#include "acl/acl.h"
#include "model_process.h"

#include <memory>

using namespace std;

//2020.10.13,acl版demo无后处理，而是将conf的阈值设置的非常高，并不算好方法
//参考其他实现移植nms算法进行处理
//移植自https://gitee.com/Atlas200DK/sample-objectdetectionbyyolov3/tree/1.3x.0.0/src/general_post
struct BoundingBox {
    cv::Point lt;
    cv::Point rb;
    std::string result_text;
    uint16_t attribute;
    float score;
};

/**
* ObjectDetect
*/
class ObjectDetect {
public:
    ObjectDetect(const char *pkgPath, const char *modelPath, uint32_t modelWidth,
                 uint32_t modelHeight);

    ~ObjectDetect();

    //Inference initialization
    Result Init();

    //nference frame image preprocessing
    Result Preprocess(cv::Mat &frame);

    //Inference frame picture
    Result Inference(aclmdlDataset *&inferenceOutput);

    //Inference output post-processing
    Result Postprocess(cv::Mat &frame, aclmdlDataset *modelOutput, std::vector<BoundingBox> &detectionResults);

    Result SetCreateContext();

    //Loading reasoning model
    Result InitModel();

private:
    //Initializes the ACL resource
    Result InitResource();


    Result CreateModelInputdDataset();

    //Establish a connection to the Presenter Server
//    Result OpenPresenterChannel();

    //Get data from model inference output aclmdlDataset to local
    void *GetInferenceOutputItem(uint32_t &itemDataSize,
                                 aclmdlDataset *inferenceOutput,
                                 uint32_t idx);

    //Serializes a frame image into a data stream
    void EncodeImage(vector<uint8_t> &encodeImg, cv::Mat &origImg);

//    Result SendImage(std::vector<BoundingBox> &detectionResults,
//                     cv::Mat &frame);

    //Release the requested resources
    void DestroyResource();

    //移植自1.32版demo，图像数据后处理
    std::vector<BoundingBox> decodeTensor(aclmdlDataset *modelOutput, uint ImgW, uint Imgh);

    std::vector<BoundingBox> nonMaximumSuppression(const float nmsThresh, std::vector<BoundingBox> binfo);

    std::vector<BoundingBox> nmsAllClasses(const float nmsThresh, std::vector<BoundingBox> &binfo,
                                           const uint numClasses);

private:
    int32_t deviceId_;  //Device ID, default is 0
    ModelProcess model_; //Inference model instance

    aclrtContext ctx1;

    const char *pkgPath_; //Offline pkg path
    const char *modelPath_; //Offline model file path
    uint32_t modelWidth_;   //The input width required by the model
    uint32_t modelHeight_;  //The model requires high input
    uint32_t imageDataSize_; //Model input data size
    void *imageDataBuf_;      //Model input data cache
    uint32_t imageInfoSize_;
    void *imageInfoBuf_;
    aclrtRunMode runMode_;   //Run mode, which is whether the current application is running on atlas200DK or AI1

//    Channel *channel_;  //A channel to connect to presenter Server
    bool isInited_;     //Initializes the tag to prevent inference instances from being initialized multiple times
};

