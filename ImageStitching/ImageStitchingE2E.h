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
using namespace cv;
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
	void runPipeline(Feature feat);
	void loadImages(const std::string& img1_path, const std::string& img2_path);
	void saveImages();

private:
	Mat img1;
	Mat img2;
	Feature feat;

	vector<KeyPoint> raw_kpts1;
	vector<KeyPoint> raw_kpts2;

	vector<KeyPoint> ransac_kpts1;
	vector<KeyPoint> ransac_kpts2;

	vector<DMatch> matches;

	//result images
	Mat img_detect_kpts;
	Mat img_after_ransac;
	Mat img_stitched;

	void extractFeatures(Mat& descriptors1, Mat& descriptors2);
	void matchKeyPoints(Mat& descriptors1, Mat& descriptors2);
	void imageTransform();
	void optimizeSeam();
};



