#pragma once
//�˵���ͼ��ƴ��

#include<string>
#include<vector>

#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp> 
#include<opencv2/xfeatures2d.hpp>
#include<opencv2/core/core.hpp>
#include<iostream>
#include<opencv2/stitching/warpers.hpp>
using namespace std;
using namespace cv::xfeatures2d;

enum Feature{SIFT, SURF, ORB, FERNS};


/*
���������img1_path��img2_path:��������ͼƬ·��
�������������Ϊ5�����飬�����ÿһ����һ��ͼƬ��·����string���󣬷ֱ�Ϊ��
0��ԭʼ2��ͼƬ���Ұڷź��ͼƬ·��
1����ע���������ͼƬ·��
2��ȥ��outliner���ͼƬ·��
3��������ƥ����ͼƬ·��
4������ƴ�Ӻ��ͼƬ·��
*/

class ImageStitcher {
public:
	void runPipeline();
	void loadImages();
	void loadImages(const std::string& img1_path, const std::string& img2_path);
	vector<string> saveImages(const char* path= "");
	void setLeftImagePath(const string path);
	void setRightImagePath(const string path);
	void setFeatureExtractor(Feature feature);

private:
	string left_img_path;
	string right_img_path;

	cv::Mat img1;
	cv::Mat img2;
	Feature feat;

	vector<cv::KeyPoint> raw_kpts1;
	vector<cv::KeyPoint> raw_kpts2;

	vector<cv::KeyPoint> ransac_kpts1;
	vector<cv::KeyPoint> ransac_kpts2;

	vector<cv::DMatch> matches;

	//result images
	cv::Mat img_origin;
	cv::Mat img_detect_kpts;
	cv::Mat img_rm_outlier;
	cv::Mat img_after_ransac;
	cv::Mat img_stitched;
	cv::Mat img_postprocess;

	void extractFeatures(cv::Mat& descriptors1, cv::Mat& descriptors2);
	void matchKeyPoints(cv::Mat& descriptors1, cv::Mat& descriptors2);
	void imageTransform();
	void optimizeSeam(cv::Mat& img1, cv::Mat& trans, cv::Mat& dst, int left_cover, int right_cover);
	void colorBalance();
	void AcE(int C = 3, int n = 2, float MaxCG = 7.5);


	string getSavePath(const char* raw_path);
};



