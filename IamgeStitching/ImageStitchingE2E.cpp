#include "ImageStitchingE2E.h"

void ImageStitcher::runPipeline(Feature feat)
{
	this->feat = feat;
	//vector<KeyPoint> keypoints1, keypoints2;
	Mat descriptors1, descriptors2;
	extractFeatures(descriptors1, descriptors2);
	
	vector<Point2f> p01, p02;
	matchKeyPoints(descriptors1, descriptors2);

	imageTransform();
}


void ImageStitcher::loadImages(const std::string& img1_path, const std::string& img2_path)
{
	img1 = imread(img1_path);
	if (img1.data == NULL) {
		char err_msg[200];
		sprintf_s(err_msg, "%s%s", "Cannot find image ", img1_path.c_str());
		throw exception(err_msg);
	}

	img2 = imread(img2_path);
	if (img2.data == NULL) {
		char err_msg[200];
		sprintf_s(err_msg, "%s%s", "Cannot find image ", img2_path.c_str());
		throw exception(err_msg);
	}
}

void ImageStitcher::saveImages()
{
	try {
		imwrite("output/keypoints_detected.png", img_detect_kpts);
		imwrite("output/ransac_result.png", img_after_ransac);
		imwrite("output/stitching_result.png", img_stitched);
	}
	catch (runtime_error& ex) {
		fprintf(stderr, "Cannot save image: ", ex.what());
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
	double t0 = getTickCount();//��ǰ�δ���
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
	//BFMatcher matcher;
	FlannBasedMatcher matcher;
	vector<DMatch> bf_matches;
	matcher.match(descriptors1, descriptors2, bf_matches);
	cout << "The number of match:" << bf_matches.size() << endl;

	//����ƥ����Ĺؼ���
	//Mat img_matches;
	//drawMatches(img1, raw_kpts1, img2, raw_kpts2, matches, img_matches);

	//imshow("Match image",img_matches);
	//����ƥ�����о������;�����Сֵ
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
	//ɸѡ���Ϻõ�ƥ���
	vector<DMatch> goodMatches;
	for (int m = 0; m < bf_matches.size(); m++)
	{
		if (bf_matches[m].distance < 0.2 * max_dist)
		{
			goodMatches.push_back(bf_matches[m]);
		}
	}
	cout << "The number of good matches:" << goodMatches.size() << endl;

	
	//��ɫ���ӵ���ƥ���������������ɫ���ӵ���δƥ�����������
	//matchColor �C Color of matches (lines and connected keypoints). If matchColor==Scalar::all(-1) , the color is generated randomly.
	//singlePointColor �C Color of single keypoints(circles), which means that keypoints do not have the matches.If singlePointColor == Scalar::all(-1), the color is generated randomly.
	//CV_RGB(0, 255, 0)�洢˳��ΪR-G-B,��ʾ��ɫ

	//Mat imgOut;
	//drawMatches(img1, keypoints1, img2, keypoints2, goodMatches, imgOut, Scalar::all(-1), CV_RGB(0, 0, 255), Mat(), 2);
	//imshow("good Matches", img_out);

	//RANSACƥ�����

	//vector<DMatch> m_Matches;
	matches = goodMatches;
	int ptCount = goodMatches.size();
	if (ptCount < 100)
	{
		throw exception("Lack of matched points after RANSAC.");
	}

	//����ת��Ϊfloat����
	//size_t�Ǳ�׼C���ж���ģ�ӦΪunsigned int����64λϵͳ��Ϊlong unsigned int,��C++��Ϊ����Ӧ��ͬ��ƽ̨�����ӿ���ֲ�ԡ�
	for (::size_t i = 0; i < matches.size(); i++)
	{
		ransac_kpts1.push_back(raw_kpts1[goodMatches[i].queryIdx]);
		ransac_kpts2.push_back(raw_kpts2[goodMatches[i].trainIdx]);
	}
}


void ImageStitcher::imageTransform()
{
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
	//��������� Fundamental,3*3�Ļ�������
	vector<uchar> RansacStatus;
	Mat Fundamental = findFundamentalMat(p02, p01, RansacStatus, FM_RANSAC);
	cout << "Fundamental:" << Fundamental << endl;

	//���¶���ؼ���RR_KP��RR_matches���洢�µĹؼ���ͻ�������ͨ��RansacStatus��ɾ����ƥ���
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
	cout << "RANSAC��ƥ�����" << RR_matches.size() << endl;
	//Mat img_RR_matches;
	drawMatches(img1, RR_KP1, img2, RR_KP2, RR_matches, img_after_ransac);
	imshow("After RANSAC", img_after_ransac);

	vector<Point2f> imagePoints1, imagePoints2;

	for (int i = 0; i < RR_matches.size(); i++)
	{
		imagePoints2.push_back(RR_KP1[RR_matches[i].trainIdx].pt);
		imagePoints1.push_back(RR_KP2[RR_matches[i].queryIdx].pt);
	}

	Mat home;
	//invert(m_homography, home);
	home = m_homography;
	cout << "invert:" << home << endl;

	std::vector<Point2f> obj_corners(4);//������ͼ���ĸ���
	obj_corners[0] = cvPoint(0, 0);
	obj_corners[1] = cvPoint(img2.cols, 0);
	obj_corners[2] = cvPoint(img2.cols, img2.rows);
	obj_corners[3] = cvPoint(0, img2.rows);

	std::vector<Point2f> scene_corners(4);//������ͼ���ĸ���
	perspectiveTransform(obj_corners, scene_corners, home);//����ͼ�Ľ�ͶӰ����ͼ

	cout << "left_top:" << scene_corners[0].x << " " << scene_corners[0].y << endl;
	cout << "right_top:" << scene_corners[1].x << " " << scene_corners[1].y << endl;
	cout << "right_bottom:" << scene_corners[2].x << " " << scene_corners[2].y << endl;
	cout << "left_bottom:" << scene_corners[3].x << " " << scene_corners[3].y << endl;


	Mat imageTransform1, imageTransform1_move;
	warpPerspective(img2, imageTransform1, home, Size(MAX(scene_corners[1].x, scene_corners[2].x), img1.rows));
	cv::Mat t_mat = cv::Mat::zeros(2, 3, CV_32FC1);

	//����ƴ�Ӻ��ͼ,����ǰ����ͼ�Ĵ�С
	int dst_width = imageTransform1.cols;  //ȡ���ҵ�ĳ���Ϊƴ��ͼ�ĳ���
	int dst_height = img1.rows;
	//Mat dst(dst_height, dst_width, CV_8UC3);
	//dst.setTo(0);
	//imageTransform1.copyTo(dst(Rect(0, 0, imageTransform1.cols, imageTransform1.rows)));
	//img1.copyTo(dst(Rect(0, 0, img1.cols, img1.rows)));
	//imshow("b_dst", dst);
	
	img_stitched = Mat(dst_height, dst_width, CV_8UC3);
	img_stitched.setTo(0);
	imageTransform1.copyTo(img_stitched(Rect(0, 0, imageTransform1.cols, imageTransform1.rows)));
	img1.copyTo(img_stitched(Rect(0, 0, img1.cols, img1.rows)));
	imshow("stitched image", img_stitched);
	
}
