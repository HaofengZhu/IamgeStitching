#pragma once
//端到端图像拼接

#include<string>
#include<vector>
enum extract_methods{SIFT,SUFT,ORB,FERNS};

/*
输入参数：img1_path、img2_path:两张输入图片路径
输出参数：长度为5的数组，数组的每一项是一张图片的路径的string对象，分别为：
0：原始2张图片左右摆放后的图片路径
1：标注了特征点的图片路径
2：去除outliner后的图片路径
3：特征点匹配后的图片路径
4：最终拼接后的图片路径
*/
std::vector<std::string> ImageStitchingE2E(std::string img1_path, std::string img2_path,int method= extract_methods::SIFT);