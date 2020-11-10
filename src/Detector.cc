/**
* This file is part of Crowd-SLAM.
* Copyright (C) 2020 João Carlos Virgolino Soares - Pontifical Catholic University of Rio de Janeiro - PUC-Rio
* Co-authors: Marcelo Gattass and Marco Antonio Meggiolaro
* For more information see <https://github.com/virgolinosoares/Crowd-SLAM>.
* Please report suggestions and comments to virgolinosoares@gmail.com
* 
* YOLO detection and processing functions are based on the OpenCV project. They are subject to the license terms at http://opencv.org/license.html
*/

#include <thread>
#include <iomanip>
#include "Detector.h"
#include <fstream>
#include <sstream>
#include <iostream>


namespace ORB_SLAM2
{


Detector::Detector()
{
	mbNewImgFlag = false;
	mbFinishRequested = false;
	
	std::cout << "\n--------------------------------------------"  << std::endl;
    
	//Check settings file
    cv::FileStorage YOLOsettings("cfg/YOLOpar.yaml", cv::FileStorage::READ);
    if(!YOLOsettings.isOpened())
    {
       cerr << "Failed to open settings file at: " << "cfg/YOLOpar.yaml" << endl;
       exit(-1);
    }

	std::cout << "loading YOLO network" << std::endl;
    	
	cout << "loading classes...";    	
	string classesFile = "cfg/CYTI.names";
    ifstream ifs(classesFile.c_str());
    string line;
    while (getline(ifs, line)) classes.push_back(line);
	std::cout << "done\n";

	// Load YOLO parameters
	confThreshold = YOLOsettings["yolo.confThreshold"];
	nmsThreshold = YOLOsettings["yolo.nmsThreshold"];
	inpWidth = YOLOsettings["yolo.inpWidth"];
	inpHeight = YOLOsettings["yolo.inpHeight"];
    
    std::cout << "loading weight files...";      
	string model_cfg = "cfg/CYTI.cfg";
    string model_weights = "weights/CYTI.weights";

    std::cout << "done\n";

    // Load the network
	cout << "loading network...";
    net = cv::dnn::readNetFromDarknet(model_cfg, model_weights);
    net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

	std::cout << "done\n";  
	std::cout << "--------------------------------------------"  << std::endl;


}

void Detector::Run()
{

	while(1)
    {		
	
        usleep(1);
        if(!isNewImgArrived()){
			continue;
		}
	
		// empty people_boxes vector
		people_boxes.clear();
		
		// perform YOLO detection
		Detect();

		if(isFinished())
        {
            break;
        }

	}

}


bool Detector::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}
  
void Detector::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested=true;
}


void Detector::SetTracker(Tracking *pTracker)
{
    mpTracker=pTracker;
}

void Detector::Detect(){

		fig_area = mImg.rows * mImg.cols;
		aprox_area = 0;

		fig_factor = fig_area/fig_param;

		cv::Mat blob;
    	cv::dnn::blobFromImage(mImg, blob, 1/255.0, cvSize(inpWidth, inpHeight), cv::Scalar(0,0,0), true, false);

		//blob = cv::dnn::blobFromImage(mImg, 1.0, cv::Size(32,32));
        
    	net.setInput(blob);
	
    	std::vector<cv::Mat> outs;
    	net.forward(outs, getOutsNames(net));

		std::vector<int> classId;
		std::vector<int> centersX;
		std::vector<int> centersY;
		std::vector<cv::Rect> boxes;	      

    	postprocess(mImg, outs, classId, centersX, centersY, boxes);

		//cv::imshow("ORB-SLAM2: Current Frame",mImg);
        //cv::waitKey(1);

		int people_counter {0};
		//
		for (size_t i = 0; i < classId.size(); ++i){
			if(classes[classId[i]] == "person"){
				people_counter++;		
				people_boxes.push_back(boxes[i]);

				aprox_area = aprox_area + (boxes[i].width * boxes[i].height)/fig_area;				
			}
		}
		//
		//std::cout << "people: " << people_counter << std::endl;
		SetDetectionFlag();
}



bool Detector::isNewImgArrived()
{
    unique_lock<mutex> lock(mMutexGetNewImg);
    if(mbNewImgFlag)
    {
        mbNewImgFlag=false;
        return true;
    }
    else
    	return false;
}


void Detector::SetDetectionFlag()
{
    std::unique_lock <std::mutex> lock(mMutexNewImgDetection);
   
    mpTracker->mbNewDetImgFlag=true;
}


std::vector<cv::String> Detector::getOutsNames(const cv::dnn::Net& net)
{
    static std::vector<cv::String> out_names;
    if (out_names.empty())
    {
        std::vector<int> outLayers = net.getUnconnectedOutLayers();
        std::vector<cv::String> layersNames = net.getLayerNames();
        out_names.resize(outLayers.size());
        
		for (size_t i = 0; i < outLayers.size(); ++i){
			out_names[i] = layersNames[outLayers[i] - 1];
		}
    }
    return out_names;
}

void Detector::postprocess(cv::Mat& frame, const std::vector<cv::Mat>& outs, std::vector<int>& classIds, std::vector<int> &centersX, std::vector<int> &centersY, std::vector<cv::Rect>& boxes)
{
    std::vector<float> confidences;
        
    for (size_t i = 0; i < outs.size(); ++i)
    {
        
        float* data = (float*)outs[i].data;
        for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols)
        {
            cv::Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
            cv::Point classIdPoint;
            double confidence;

            minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
            if (confidence > confThreshold)
            {
                int centerX = (int)(data[0] * frame.cols);
                int centerY = (int)(data[1] * frame.rows);
                int width = (int)(data[2] * frame.cols);
                int height = (int)(data[3] * frame.rows);
                int left = centerX - width / 2;
                int top = centerY - height / 2;
                
                classIds.push_back(classIdPoint.x);
                confidences.push_back((float)confidence);
                boxes.push_back(cv::Rect(left, top, width, height));
				centersX.push_back(centerX);
				centersY.push_back(centerY);
            }
        }
    }
    
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);
    for (size_t i = 0; i < indices.size(); ++i)
    {
        int idx = indices[i];
        cv::Rect box = boxes[idx];
        drawPred(classIds[idx], confidences[idx], box.x, box.y,
               box.x + box.width, box.y + box.height, frame);
    }
}

void Detector::drawPred(int classId, float conf, int left, int top, int right, int bottom, cv::Mat& frame)
{
    rectangle(frame, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(255, 178, 50), 3);
    
    string label = cv::format("%.2f", conf);
    if (!classes.empty())
    {
        CV_Assert(classId < (int)classes.size());
        label = classes[classId] + ":" + label;
    }

    int baseLine;
    cv::Size labelSize = getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
    top = max(top, labelSize.height);
    rectangle(frame, cv::Point(left, top - round(1.5*labelSize.height)), cv::Point(left + round(1.5*labelSize.width), top + baseLine), cv::Scalar(255, 255, 255), cv::FILLED);
    putText(frame, label, cv::Point(left, top), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0,0,0),1);
}


} //namespace ORB_SLAM2
