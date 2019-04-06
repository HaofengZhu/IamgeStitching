#include "ImageStitchingE2E.h"
#include <Windows.h>
#include <string.h>

using namespace cv;

void ImageStitcher::runPipeline(Feature feat)
{
	this->feat = feat;
	//vector<KeyPoint> keypoints1, keypoints2;
	Mat descriptors1, descriptors2;
	extractFeatures(descriptors1, descriptors2);
	
	vector<Point2f> p01, p02;
	matchKeyPoints(descriptors1, descriptors2);
	imageTransform();
	colorBalance();
 	AcE();
}


void ImageStitcher::loadImages(const std::string& img1_path, const std::string& img2_path)
{
	img1 = imread(img1_path);
	if (img1.data == NULL) {
		char msg[200] = "Cannot find image ";
		strcat(msg, img1_path.c_str());
		throw runtime_error(msg);
	}


	img2 = imread(img2_path);
	if (img2.data == NULL) {
		char msg[200] = "Cannot find image ";
		strcat(msg, img2_path.c_str());
		throw runtime_error(msg);
	}

	//img1 = imread(img1_path);
	//if (img1.data == NULL) return false;

	//img2 = imread(img2_path);
	//if (img2.data == NULL) return false;

	//return true;
}


string ImageStitcher::getSavePath(const char * raw_path)
{
	cout << "raw_path: " << raw_path << endl;

	string save_path;
	if (strcmp(raw_path, "") == 0) {
		char curr_path[MAX_PATH];
		GetCurrentDirectoryA(MAX_PATH, curr_path);
		cout << "curr_path: " << curr_path << endl;
		save_path = string(curr_path);
	}
	else {
		save_path = string(raw_path);
	}
	
	switch (feat) {
	case Feature::SIFT:
		save_path += "/SIFT";
		break;
	case Feature::SURF:
		save_path += "/SURF";
		break;
	case Feature::ORB:
		save_path += "/ORB";
		break;
	default:
		break;
	}
	return save_path;
}


void ImageStitcher::saveImages(const char* save_path)
{
	string save_dir = getSavePath(save_path);
	cout << "save_path: " << save_dir << endl;
	if (CreateDirectoryA(save_dir.c_str(), NULL) ||
		GetLastError() == ERROR_ALREADY_EXISTS) {
		try {
			imwrite(save_dir + string("/detect_keypoints.jpg"), img_detect_kpts);
			imwrite(save_dir + string("/ransac_result.jpg"), img_after_ransac);
			imwrite(save_dir + string("/stitching_result.jpg"), img_stitched);
			imwrite(save_dir + string("/postprocess.jpg"), img_postprocess);
		}
		catch (runtime_error& ex) {
			cerr << "cannot save image:" << ex.what() << endl;
			throw exception("Cannot save image.");
		}
	}
	else {
		throw exception("Save path error.");
	}
	
	
}


void ImageStitcher::extractFeatures(Mat& descriptors1, Mat& descriptors2)
{
	Ptr<Feature2D> f2d;

	switch (feat)
	{
	case Feature::SIFT:
		f2d = SIFT::create();
		break;
	case Feature::SURF:
		f2d = SURF::create();
		break;
	case Feature::ORB:
		f2d = ORB::create();
		break;
	case Feature::FERNS:
		//f2d = FERNS::create();
		break;
	default:
		break;
	}

	//Detect the keypoints
	double t0 = getTickCount();//当前滴答数
	//vector<KeyPoint> keypoints_1, keypoints_2;
	f2d->detect(img1, raw_kpts1);
	f2d->detect(img2, raw_kpts2);
	//cout << "The keypoints number of img1 is:" << keypoints1.size() << endl;
	//cout << "The keypoints number of img2 is:" << keypoints2.size() << endl;

	//Calculate descriptors (feature vectors)
	f2d->compute(img1, raw_kpts1, descriptors1);
	f2d->compute(img2, raw_kpts2, descriptors2);
	double freq = getTickFrequency();
	double tt = ((double)getTickCount() - t0) / freq;
	cout << "Extract Features Time:" << tt << "ms" << endl;

	drawMatches(img1, raw_kpts1, img2, raw_kpts2, vector<DMatch>(), img_detect_kpts);
	//imshow("detect keypoints", img_detect_kpts);
}


void ImageStitcher::matchKeyPoints(Mat& descriptors1, Mat& descriptors2)
{
	double t0 = getTickCount();
	BFMatcher matcher;
	vector<DMatch> bf_matches;
	matcher.match(descriptors1, descriptors2, bf_matches);
	cout << "The number of match:" << bf_matches.size() << endl;

	//绘制匹配出的关键点
	//Mat img_matches;
	//drawMatches(img1, raw_kpts1, img2, raw_kpts2, matches, img_matches);

	//imshow("Match image",img_matches);
	//计算匹配结果中距离最大和距离最小值
	double min_dist = bf_matches[0].distance, max_dist = bf_matches[0].distance;
	for (int m = 0; m < bf_matches.size(); m++)
	{
		if (bf_matches[m].distance < min_dist)
		{
			min_dist = bf_matches[m].distance;
		}
		if (bf_matches[m].distance > max_dist)
		{
			max_dist = bf_matches[m].distance;
		}
	}
	cout << "min dist=" << min_dist << endl;
	cout << "max dist=" << max_dist << endl;
	//筛选出较好的匹配点
	vector<DMatch> goodMatches;
	double ratio = (feat == Feature::ORB) ? 0.8 : 0.2;
	for (int m = 0; m < bf_matches.size(); m++)
	{
		if (bf_matches[m].distance < ratio * max_dist)
		{
			goodMatches.push_back(bf_matches[m]);
		}
	}
	cout << "The number of good matches:" << goodMatches.size() << endl;

	
	//红色连接的是匹配的特征点数，绿色连接的是未匹配的特征点数
	//matchColor – Color of matches (lines and connected keypoints). If matchColor==Scalar::all(-1) , the color is generated randomly.
	//singlePointColor – Color of single keypoints(circles), which means that keypoints do not have the matches.If singlePointColor == Scalar::all(-1), the color is generated randomly.
	//CV_RGB(0, 255, 0)存储顺序为R-G-B,表示绿色

	//Mat imgOut;
	//drawMatches(img1, keypoints1, img2, keypoints2, goodMatches, imgOut, Scalar::all(-1), CV_RGB(0, 0, 255), Mat(), 2);
	//imshow("good Matches", img_out);

	//RANSAC匹配过程

	//vector<DMatch> m_Matches;
	matches = goodMatches;
	int ptCount = goodMatches.size();
	if (ptCount < 100)
	{
		throw runtime_error("Lack of matched points after RANSAC.");
	}
	//坐标转换为float类型
		//size_t是标准C库中定义的，应为unsigned int，在64位系统中为long unsigned int,在C++中为了适应不同的平台，增加可移植性。
	for (::size_t i = 0; i < matches.size(); i++)
	{
		ransac_kpts1.push_back(raw_kpts1[goodMatches[i].queryIdx]);
		ransac_kpts2.push_back(raw_kpts2[goodMatches[i].trainIdx]);
	}

	double freq = getTickFrequency();
	double tt = ((double)getTickCount() - t0) / freq;
	cout << "Match KeyPoints Time:" << tt << "ms" << endl;
}


void ImageStitcher::imageTransform()
{
	double t0 = getTickCount();

	vector <Point2f> p01, p02;
	for (::size_t i = 0; i < matches.size(); i++)
	{
		p01.push_back(ransac_kpts1[i].pt);
		p02.push_back(ransac_kpts2[i].pt);
	}

	Mat m_homography;
	vector<uchar> m;
	//m_homography = findHomography(p01, p02, CV_RANSAC);
	m_homography = findHomography(p02, p01, CV_RANSAC);
	cout << "m_homography:" << m_homography << endl;
	//求基础矩阵 Fundamental,3*3的基础矩阵
	vector<uchar> RansacStatus;
	Mat Fundamental = findFundamentalMat(p02, p01, RansacStatus, FM_RANSAC);
	cout << "Fundamental:" << Fundamental << endl;

	//重新定义关键点RR_KP和RR_matches来存储新的关键点和基础矩阵，通过RansacStatus来删除误匹配点
	vector <KeyPoint> RR_KP1, RR_KP2;
	vector <DMatch> RR_matches;
	int index = 0;
	for (::size_t i = 0; i < matches.size(); i++)
	{
		if (RansacStatus[i] != 0)
		{
			RR_KP1.push_back(ransac_kpts1[i]);
			RR_KP2.push_back(ransac_kpts2[i]);
			matches[i].queryIdx = index;
			matches[i].trainIdx = index;
			RR_matches.push_back(matches[i]);
			index++;
		}
	}
	cout << "RANSAC后匹配点数" << RR_matches.size() << endl;
	//Mat img_RR_matches;
	drawMatches(img1, RR_KP1, img2, RR_KP2, RR_matches, img_after_ransac);
	//imshow("After RANSAC", img_after_ransac);

	vector<Point2f> imagePoints1, imagePoints2;

	for (int i = 0; i < RR_matches.size(); i++)
	{
		imagePoints2.push_back(RR_KP1[RR_matches[i].trainIdx].pt);
		imagePoints1.push_back(RR_KP2[RR_matches[i].queryIdx].pt);
	}

	Mat home;
	//invert(m_homography, home);
	home = m_homography;
	//cout << "invert:" << home << endl;

	std::vector<Point2f> obj_corners(4);//定义右图的四个角
	obj_corners[0] = cvPoint(0, 0);
	obj_corners[1] = cvPoint(img2.cols, 0);
	obj_corners[2] = cvPoint(img2.cols, img2.rows);
	obj_corners[3] = cvPoint(0, img2.rows);

	std::vector<Point2f> scene_corners(4);//定义左图的四个角
	perspectiveTransform(obj_corners, scene_corners, home);//将右图四角投影至左图

	cout << "left_top:" << scene_corners[0].x << " " << scene_corners[0].y << endl;
	cout << "right_top:" << scene_corners[1].x << " " << scene_corners[1].y << endl;
	cout << "right_bottom:" << scene_corners[2].x << " " << scene_corners[2].y << endl;
	cout << "left_bottom:" << scene_corners[3].x << " " << scene_corners[3].y << endl;


	Mat imageTransform1, imageTransform1_move;
	warpPerspective(img2, imageTransform1, home, Size(MAX(scene_corners[1].x, scene_corners[2].x), img1.rows));
	cv::Mat t_mat = cv::Mat::zeros(2, 3, CV_32FC1);

	//创建拼接后的图,需提前计算图的大小
	int dst_width = imageTransform1.cols;  //取最右点的长度为拼接图的长度
	int dst_height = img1.rows;
	
	img_stitched = Mat(dst_height, dst_width, CV_8UC3);
	img_stitched.setTo(0);
	imageTransform1.copyTo(img_stitched(Rect(0, 0, imageTransform1.cols, imageTransform1.rows)));
	img1.copyTo(img_stitched(Rect(0, 0, img1.cols, img1.rows)));
	//imshow("stitched image", img_stitched);
	img_postprocess = Mat(img_stitched);
	ImageStitcher::optimizeSeam(img1, imageTransform1, img_postprocess, MIN(scene_corners[0].x, scene_corners[3].x), img1.cols);
	//imshow("stitched image after optimize.", img_stitched);
	
}

void ImageStitcher::optimizeSeam(Mat& img1,Mat& trans, Mat& dst, int left_cover, int right_cover)
{
	int processWidth = right_cover-left_cover; 
	int start = left_cover+ processWidth/3*2;  

	int img1_row = img1.rows;
	int trans_row = trans.rows;
	int dst_row = dst.rows;
	int rows = MIN(MIN(img1_row, trans_row), dst_row);
	int cols = img1.cols; 
	double alpha = 1;
	for (int i = 0; i < rows; i++)
	{
		uchar* p = img1.ptr<uchar>(i);  
		uchar* t = trans.ptr<uchar>(i);
		uchar* d = dst.ptr<uchar>(i);
		for (int j = start; j < cols; j++)
		{
			if (t[j * 3] == 0 && t[j * 3 + 1] == 0 && t[j * 3 + 2] == 0)
			{
				alpha = 1;
			}
			else
			{
				alpha =(processWidth - (j - start)) / processWidth;
			}

			d[j * 3] =p[j * 3] * alpha + t[j * 3] * (1 - alpha);
			d[j * 3 + 1] =p[j * 3 + 1] * alpha + t[j * 3 + 1] * (1 - alpha);
			d[j * 3 + 2] =p[j * 3 + 2] * alpha + t[j * 3 + 2] * (1 - alpha);

		}
	}
}

void ImageStitcher::colorBalance()
{
	vector<Mat> imageRGB;
	split(img_stitched, imageRGB);
	double R, G, B;
	B = mean(imageRGB[0])[0];
	G = mean(imageRGB[1])[0];
	R = mean(imageRGB[2])[0];

	double KR, KG, KB;
	KB = (R + G + B) / (3 * B);
	KG = (R + G + B) / (3 * G);
	KR = (R + G + B) / (3 * R);

	imageRGB[0] = imageRGB[0] * KB;
	imageRGB[1] = imageRGB[1] * KG;
	imageRGB[2] = imageRGB[2] * KR;

	merge(imageRGB, img_stitched);
	//imshow("color balance", img_stitched);
}

Mat matrixWiseMulti(Mat &m1, Mat &m2) {
	Mat dst = m1.mul(m2);
	return dst;
}

void ImageStitcher::AcE(int C, int n, float MaxCG)
{
	
	int rows = img_stitched.rows;
	int cols = img_stitched.cols;

	Mat meanLocal; //图像局部均值  
	Mat varLocal;  //图像局部方差  
	Mat meanGlobal;//全局均值
	Mat varGlobal; //全局标准差  

	blur(img_stitched.clone(), meanLocal, Size(n, n));
	Mat highFreq = img_stitched - meanLocal;//高频成分 

	varLocal = matrixWiseMulti(highFreq, highFreq);
	blur(varLocal, varLocal, Size(n, n));
	//换算成局部标准差  
	varLocal.convertTo(varLocal, CV_32F);
	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < cols; j++) {
			varLocal.at<float>(i, j) = (float)sqrt(varLocal.at<float>(i, j));
		}
	}
	meanStdDev(img_stitched, meanGlobal, varGlobal);
	cout << meanGlobal << endl;

	Mat gainArr(varLocal.rows,varLocal.cols, CV_8UC3);//增益系数矩阵
	uchar Global_0 = meanGlobal.ptr<uchar>(0)[0];
	uchar Global_1 = meanGlobal.ptr<uchar>(1)[0];
	uchar Global_2 = meanGlobal.ptr<uchar>(2)[0];

	for (int i = 0; i < varLocal.rows; i++)
	{
		uchar* v= varLocal.ptr<uchar>(i);
		uchar* g = gainArr.ptr<uchar>(i);
		int alpha = 0.5;
		for (int j = 0; j < varLocal.cols; j++)
		{
			if(v[j * 3]!=0)
				g[j * 3] = alpha * Global_0 / v[j * 3];
			else
				g[j * 3] = 0;

			if (v[j * 3+1] != 0)
				g[j * 3+1] = alpha * Global_1 / v[j * 3+1];
			else
				g[j * 3+1] = 0;
			if (v[j * 3+2] != 0)
				g[j * 3+2] = alpha * Global_2 / v[j * 3+2];
			else
				g[j * 3+2] = 0;

			if (g[j * 3] > MaxCG)
				g[j * 3] = MaxCG;
			if (g[j * 3+1] > MaxCG)
				g[j * 3+1] = MaxCG;
			if (g[j * 3+2] > MaxCG)
				g[j * 3+2] = MaxCG;
		}
	}

	gainArr.convertTo(gainArr, CV_8U);
	gainArr = matrixWiseMulti(gainArr, highFreq);
	Mat dst1 = meanLocal + gainArr;
	//imshow("变增益方法", dst1);
	Mat dst2 = meanLocal + C * highFreq;
	//imshow("恒增益方法", dst2);

}