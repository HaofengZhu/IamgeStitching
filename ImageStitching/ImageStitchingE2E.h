#pragma once
//端到端图像拼接

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
输入参数：img1_path、img2_path:两张输入图片路径
输出参数：长度为5的数组，数组的每一项是一张图片的路径的string对象，分别为：
0：原始2张图片左右摆放后的图片路径
1：标注了特征点的图片路径
2：去除outliner后的图片路径
3：特征点匹配后的图片路径
4：最终拼接后的图片路径
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



