#pragma once
//�˵���ͼ��ƴ��

#include<string>
#include<vector>
enum extract_methods{SIFT,SUFT,ORB,FERNS};

/*
���������img1_path��img2_path:��������ͼƬ·��
�������������Ϊ5�����飬�����ÿһ����һ��ͼƬ��·����string���󣬷ֱ�Ϊ��
0��ԭʼ2��ͼƬ���Ұڷź��ͼƬ·��
1����ע���������ͼƬ·��
2��ȥ��outliner���ͼƬ·��
3��������ƥ����ͼƬ·��
4������ƴ�Ӻ��ͼƬ·��
*/
std::vector<std::string> ImageStitchingE2E(std::string img1_path, std::string img2_path,int method= extract_methods::SIFT);