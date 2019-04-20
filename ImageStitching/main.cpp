
#include "ImageStitching.h"

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

// int main() {
// 	ImageStitcher stitcher;
// 	try {
// 		stitcher.loadImages("input/c10_1.jpg", "input/c10_2.jpg");
// 		stitcher.setFeatureExtractor(Feature::ORB);
// 		stitcher.runPipeline();
// 		stitcher.saveImages("C:/Projects/ImageStitching/ImageStitching/output");
// 		//stitcher.saveImages();
// 		cv::waitKey(0);
// 	}
// 	catch (exception ex) {
// 		cout << "Exception: " << ex.what() << endl;
// 	}
	

// 	return 0;
// }