//#include<opencv2/highgui/highgui.hpp>
//#include<opencv2/imgproc/imgproc.hpp>
//#include <opencv2/opencv.hpp> 
//#include<opencv2/xfeatures2d.hpp>
//#include<opencv2/core/core.hpp>
//#include<iostream>
////#include <stddef.h>
//#include<opencv2/stitching/warpers.hpp>
//using namespace cv;
//using namespace std;
//using namespace cv::xfeatures2d;//ֻ�м�����������ռ䣬SiftFeatureDetector and SiftFeatureExtractor�ſ���ʹ��
//
//
//void OptimizeSeam(Mat& img1, Mat& img2, Mat& trans, Mat& dst, int left_cover, int right_cover);
//
//typedef struct
//{
//	Point2f left_top;
//	Point2f left_bottom;
//	Point2f right_top;
//	Point2f right_bottom;
//}four_corners_t;
//
//four_corners_t corners;
//int left_cover,right_cover;
//void CalcCorners(const Mat& H, const Mat& src)
//{
//	double v2[] = { 0, 0, 1 };//���Ͻ�
//	double v1[3];//�任�������ֵ
//	Mat V2 = Mat(3, 1, CV_64FC1, v2);  //������
//	Mat V1 = Mat(3, 1, CV_64FC1, v1);  //������
//
//	V1 = H * V2;
//	//���Ͻ�(0,0,1)
//	cout << "V2: " << V2 << endl;
//	cout << "V1: " << V1 << endl;
//	corners.left_top.x = v1[0] / v1[2];
//	corners.left_top.y = v1[1] / v1[2];
//
//	//���½�(0,src.rows,1)
//	v2[0] = 0;
//	v2[1] = src.rows;
//	v2[2] = 1;
//	V2 = Mat(3, 1, CV_64FC1, v2);  //������
//	V1 = Mat(3, 1, CV_64FC1, v1);  //������
//	V1 = H * V2;
//	corners.left_bottom.x = v1[0] / v1[2];
//	corners.left_bottom.y = v1[1] / v1[2];
//
//	//���Ͻ�(src.cols,0,1)
//	v2[0] = src.cols;
//	v2[1] = 0;
//	v2[2] = 1;
//	V2 = Mat(3, 1, CV_64FC1, v2);  //������
//	V1 = Mat(3, 1, CV_64FC1, v1);  //������
//	V1 = H * V2;
//	corners.right_top.x = v1[0] / v1[2];
//	corners.right_top.y = v1[1] / v1[2];
//
//	//���½�(src.cols,src.rows,1)
//	v2[0] = src.cols;
//	v2[1] = src.rows;
//	v2[2] = 1;
//	V2 = Mat(3, 1, CV_64FC1, v2);  //������
//	V1 = Mat(3, 1, CV_64FC1, v1);  //������
//	V1 = H * V2;
//	corners.right_bottom.x = v1[0] / v1[2];
//	corners.right_bottom.y = v1[1] / v1[2];
//
//}
//
//
//
///*
//Pipeline:
//		
//		loadImages -> extractFeatures -> matchKeyPoints -> computeHomographyMat -> imageTransform -> optimizeSeam
//*/
//
//int main(int argc, char *argv[])
//{
//	//Create SIFT class pointer
//	Ptr<Feature2D> f2d = xfeatures2d::SIFT::create();
//	//SiftFeatureDetector siftDetector;
//	//Loading images
//	Mat img_1 = imread("1.png");
//	Mat img_2 = imread("2.png");
//	if (!img_1.data || !img_2.data)
//	{
//		cout << "Reading picture error��" << endl;
//		return false;
//	}
//	//Detect the keypoints
//	double t0 = getTickCount();//��ǰ�δ���
//	vector<KeyPoint> keypoints_1, keypoints_2;
//	f2d->detect(img_1, keypoints_1);
//	f2d->detect(img_2, keypoints_2);
//	cout << "The keypoints number of img1 is:" << keypoints_1.size() << endl;
//	cout << "The keypoints number of img2 is:" << keypoints_2.size() << endl;
//	//Calculate descriptors (feature vectors)
//	Mat descriptors_1, descriptors_2;
//	f2d->compute(img_1, keypoints_1, descriptors_1);
//	f2d->compute(img_2, keypoints_2, descriptors_2);
//	double freq = getTickFrequency();
//	double tt = ((double)getTickCount() - t0) / freq;
//	cout << "Extract SIFT Time:" << tt << "ms" << endl;
//
//	////���ؼ���
//	//Mat img_keypoints_1, img_keypoints_2;
//	//drawKeypoints(img_1, keypoints_1, img_keypoints_1, Scalar::all(-1), 0);
//	//drawKeypoints(img_2, keypoints_2, img_keypoints_2, Scalar::all(-1), 0);
//	//imshow("img_keypoints_1",img_keypoints_1);
//	//imshow("img_keypoints_2",img_keypoints_2);
//
//	//Matching descriptor vector using BFMatcher
//	BFMatcher matcher;
//	vector<DMatch> matches;
//	matcher.match(descriptors_1, descriptors_2, matches);
//	cout << "The number of match:" << matches.size() << endl;
//	//����ƥ����Ĺؼ���
//	Mat img_matches;
//	drawMatches(img_1, keypoints_1, img_2, keypoints_2, matches, img_matches);
//	//imshow("Match image",img_matches);
//	//����ƥ�����о������;�����Сֵ
//	double min_dist = matches[0].distance, max_dist = matches[0].distance;
//	for (int m = 0; m < matches.size(); m++)
//	{
//		if (matches[m].distance < min_dist)
//		{
//			min_dist = matches[m].distance;
//		}
//		if (matches[m].distance > max_dist)
//		{
//			max_dist = matches[m].distance;
//		}
//	}
//	cout << "min dist=" << min_dist << endl;
//	cout << "max dist=" << max_dist << endl;
//	//ɸѡ���Ϻõ�ƥ���
//	vector<DMatch> goodMatches;
//	for (int m = 0; m < matches.size(); m++)
//	{
//		if (matches[m].distance < 0.5*max_dist)
//		{
//			goodMatches.push_back(matches[m]);
//		}
//	}
//	cout << "The number of good matches:" << goodMatches.size() << endl;
//
//	Mat img_out;
//	//��ɫ���ӵ���ƥ���������������ɫ���ӵ���δƥ�����������
//	//matchColor �C Color of matches (lines and connected keypoints). If matchColor==Scalar::all(-1) , the color is generated randomly.
//	//singlePointColor �C Color of single keypoints(circles), which means that keypoints do not have the matches.If singlePointColor == Scalar::all(-1), the color is generated randomly.
//	//CV_RGB(0, 255, 0)�洢˳��ΪR-G-B,��ʾ��ɫ
//	drawMatches(img_1, keypoints_1, img_2, keypoints_2, goodMatches, img_out, Scalar::all(-1), CV_RGB(0, 0, 255), Mat(), 2);
//	//imshow("good Matches", img_out);
//
//	//RANSACƥ�����
//	vector<DMatch> m_Matches;
//	m_Matches = goodMatches;
//	int ptCount = goodMatches.size();
//	if (ptCount < 100)
//	{
//		cout << "Don't find enough match points" << endl;
//		return 0;
//	}
//
//	//����ת��Ϊfloat����
//	vector <KeyPoint> RAN_KP1, RAN_KP2;
//	//size_t�Ǳ�׼C���ж���ģ�ӦΪunsigned int����64λϵͳ��Ϊlong unsigned int,��C++��Ϊ����Ӧ��ͬ��ƽ̨�����ӿ���ֲ�ԡ�
//	for (::size_t i = 0; i < m_Matches.size(); i++)
//	{
//		RAN_KP1.push_back(keypoints_1[goodMatches[i].queryIdx]);
//		RAN_KP2.push_back(keypoints_2[goodMatches[i].trainIdx]);
//		//RAN_KP1��Ҫ�洢img01������img02ƥ��ĵ�
//		//goodMatches�洢����Щƥ���Ե�img01��img02������ֵ
//	}
//	//����任
//	vector <Point2f> p01, p02;
//	for (::size_t i = 0; i < m_Matches.size(); i++)
//	{
//		p01.push_back(RAN_KP1[i].pt);
//		p02.push_back(RAN_KP2[i].pt);
//	}
//	/*vector <Point2f> img1_corners(4);
//	img1_corners[0] = Point(0,0);
//	img1_corners[1] = Point(img_1.cols,0);
//	img1_corners[2] = Point(img_1.cols, img_1.rows);
//	img1_corners[3] = Point(0, img_1.rows);
//	vector <Point2f> img2_corners(4);*/
//	//��ת������
//	Mat m_homography;
//	vector<uchar> m;
//	m_homography = findHomography(p01, p02, CV_RANSAC);//Ѱ��ƥ��ͼ��
//	cout << "m_homography:" << m_homography << endl;
//	//��������� Fundamental,3*3�Ļ�������
//	vector<uchar> RansacStatus;
//	Mat Fundamental = findFundamentalMat(p01, p02, RansacStatus, FM_RANSAC);
//	cout << "Fundamental:" << Fundamental << endl;
//
//	//���¶���ؼ���RR_KP��RR_matches���洢�µĹؼ���ͻ�������ͨ��RansacStatus��ɾ����ƥ���
//	vector <KeyPoint> RR_KP1, RR_KP2;
//	vector <DMatch> RR_matches;
//	int index = 0;
//	for (::size_t i = 0; i < m_Matches.size(); i++)
//	{
//		if (RansacStatus[i] != 0)
//		{
//			RR_KP1.push_back(RAN_KP1[i]);
//			RR_KP2.push_back(RAN_KP2[i]);
//			m_Matches[i].queryIdx = index;
//			m_Matches[i].trainIdx = index;
//			RR_matches.push_back(m_Matches[i]);
//			index++;
//		}
//	}
//	cout << "RANSAC��ƥ�����" << RR_matches.size() << endl;
//	Mat img_RR_matches;
//	drawMatches(img_1, RR_KP1, img_2, RR_KP2, RR_matches, img_RR_matches);
//	//imshow("After RANSAC", img_RR_matches);
//
//	vector<Point2f> imagePoints1, imagePoints2;
//
//	for (int i = 0; i < RR_matches.size(); i++)
//	{
//		imagePoints2.push_back(RR_KP2[RR_matches[i].trainIdx].pt);
//		imagePoints1.push_back(RR_KP1[RR_matches[i].queryIdx].pt);
//	}
//
//	Mat home;
//	invert(m_homography, home);
//	cout << "invert:" << home << endl;
//
//	std::vector<Point2f> obj_corners(4);//������ͼ���ĸ���
//	obj_corners[0] = cvPoint(0, 0);
//	obj_corners[1] = cvPoint(img_2.cols, 0);
//	obj_corners[2] = cvPoint(img_2.cols, img_2.rows);
//	obj_corners[3] = cvPoint(0, img_2.rows);
//
//	std::vector<Point2f> scene_corners(4);//������ͼ���ĸ���
//	perspectiveTransform(obj_corners, scene_corners, home);//����ͼ�Ľ�ͶӰ����ͼ
//
//	cout << "left_top:" << scene_corners[0].x << " " << scene_corners[0].y << endl;
//	cout << "right_top:" << scene_corners[1].x << " " << scene_corners[1].y << endl;
//	cout << "right_bottom:" << scene_corners[2].x << " " << scene_corners[2].y << endl;
//	cout << "left_bottom:" << scene_corners[3].x << " " << scene_corners[3].y << endl;
//
//
//	Mat imageTransform1, imageTransform1_move;
//	warpPerspective(img_2, imageTransform1, home, Size(MAX(scene_corners[1].x, scene_corners[2].x), img_1.rows));
//	cv::Mat t_mat = cv::Mat::zeros(2, 3, CV_32FC1);
//
//	//����ƴ�Ӻ��ͼ,����ǰ����ͼ�Ĵ�С
//	int dst_width = imageTransform1.cols;  //ȡ���ҵ�ĳ���Ϊƴ��ͼ�ĳ���
//	int dst_height = img_1.rows;
//	Mat dst(dst_height, dst_width, CV_8UC3);
//	dst.setTo(0);
//
//	imageTransform1.copyTo(dst(Rect(0, 0, imageTransform1.cols, imageTransform1.rows)));
//	img_1.copyTo(dst(Rect(0, 0, img_1.cols, img_1.rows)));
//
//	imshow("b_dst", dst);
//
//	/*imshow("merge img 1", img_1);
//	imshow("merge img 2", imageTransform1);
//	imshow("merge img 3", dst);*/
//
//	OptimizeSeam(img_1,img_2, imageTransform1, dst,MIN(scene_corners[0].x, scene_corners[3].x),img_1.cols);
//
//	imshow("dst", dst);
//
//	bool flag = true;
//	Stitcher stitcher = Stitcher::createDefault(flag);
//	vector<Mat>imgs;
//	imgs.push_back(img_1);
//	imgs.push_back(img_2);
//	Mat resultMat;
//	stitcher.stitch(imgs, resultMat);
//	imshow("UsingOpencvStitch", resultMat);
//
//
//	//imwrite("dst.jpg", dst);
//
//	waitKey();
//
//	return 0;
//}
//
//
////�Ż���ͼ�����Ӵ���ʹ��ƴ����Ȼ
//void OptimizeSeam(Mat& img1, Mat& img2, Mat& trans, Mat& dst,int left_cover,int right_cover)
//{
//	//dst = Mat(dst.rows, dst.cols, CV_8UC3);
//	int processWidth = right_cover-left_cover;//�ص�����Ŀ��  
//	int start = left_cover+ processWidth/3*2;//��ʼλ��  
//
//	int img1_row = img1.rows;
//	int trans_row = trans.rows;
//	int dst_row = dst.rows;
//	int rows = MIN(MIN(img1_row, trans_row), dst_row);
//	int cols = img1.cols; //ע�⣬������*ͨ����
//	double alpha = 1;//img1�����ص�Ȩ��  
//	for (int i = 0; i < rows; i++)
//	{
//		uchar* p = img1.ptr<uchar>(i);  //��ȡ��i�е��׵�ַ
//		uchar* t = trans.ptr<uchar>(i);
//		uchar* d = dst.ptr<uchar>(i);
//		for (int j = start; j < cols; j++)
//		{
//			//�������ͼ��trans�������صĺڵ㣬����ȫ����img1�е�����
//			if (t[j * 3] == 0 && t[j * 3 + 1] == 0 && t[j * 3 + 2] == 0)
//			{
//				alpha = 1;
//			}
//			else
//			{
//				//img1�����ص�Ȩ�أ��뵱ǰ�������ص�������߽�ľ�������ȣ�ʵ��֤�������ַ���ȷʵ��  
//				alpha =(processWidth - (j - start)) / processWidth;
//			}
//
//			d[j * 3] =p[j * 3] * alpha + t[j * 3] * (1 - alpha);
//			d[j * 3 + 1] =p[j * 3 + 1] * alpha + t[j * 3 + 1] * (1 - alpha);
//			d[j * 3 + 2] =p[j * 3 + 2] * alpha + t[j * 3 + 2] * (1 - alpha);
//
//		}
//	}
//
//}



#include "ImageStitchingE2E.h"

void lightProcessing(cv::Mat& image) {
	//Mat image = imread(path, 1);
	if (!image.data)
	{
		cout << "image loading error" << endl;
	}
	cv::Mat imageRGB[3];
	split(image, imageRGB);
	for (int i = 0; i < 3; i++)
	{
		equalizeHist(imageRGB[i], imageRGB[i]);
	}
	merge(imageRGB, 3, image);
	imshow("equalizeHist", image);
	cv::waitKey();

}

int main() {
	ImageStitcher stitcher;
	try {
		stitcher.loadImages("input/c10_1.jpg", "input/c10_2.jpg");
		stitcher.setFeatureExtractor(Feature::ORB);
		stitcher.runPipeline();
		stitcher.saveImages("C:/Projects/ImageStitching/ImageStitching/output");
		//stitcher.saveImages();
		cv::waitKey(0);
	}
	catch (exception ex) {
		cout << "Exception: " << ex.what() << endl;
	}
	

	return 0;
}