#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <iostream>
#include<fstream>
#include<vector>
using namespace cv;
using namespace std;
int main()
{
	//step1 load image
	Mat img1=imread("alcatraz1.jpg");
	Mat img2=imread("alcatraz2.jpg");
	Mat gimg1=imread("alcatraz1.jpg",CV_LOAD_IMAGE_GRAYSCALE);
	Mat gimg2=imread("alcatraz2.jpg",CV_LOAD_IMAGE_GRAYSCALE);
	//cvtColor(img1,gimg1,CV_BGR2GRAY);
	cout<<"compute keypoint"<<endl;
	//step2 compute keypoint
	SiftFeatureDetector detector;
	vector<KeyPoint> kp1, kp2; 
	detector.detect(gimg1,kp1);
	detector.detect(gimg2,kp2);
	//step3 compute descriptor
	SiftDescriptorExtractor extractor;
	Mat descriptor1,descriptor2;
	extractor.compute(gimg1,kp1,descriptor1);
	extractor.compute(gimg2,kp2,descriptor2);
	cout<<"compute match"<<endl;
	//step4 gimg1 <-->gimg2 match
	BFMatcher matcher(NORM_L2);
	vector<DMatch> matches1,matches2,twoside_matches;
	matcher.match(descriptor1,descriptor2,matches1);
	matcher.match(descriptor2,descriptor1,matches2);
	cout<<"match end"<<endl;
	vector<DMatch>::iterator it1;
	vector<DMatch>::iterator it2;
	for(it1 = matches1.begin();it1 != matches1.end();it1++)
	{	for(it2 =matches2.begin();it2 != matches2.end();it2++)
		{
			if((*it1).queryIdx == (*it2).trainIdx && (*it2).queryIdx == (*it1).trainIdx)
			{
				twoside_matches.push_back(DMatch((*it1).queryIdx,(*it1).trainIdx,(*it1).distance));
				//break;
			}	
		}
	}
	//step5 draw twoside_matches
	Mat imgmathces;
	drawMatches(gimg1,kp1,gimg2,kp2,twoside_matches,imgmathces);
	//load matches keypoint
	int n=twoside_matches.size();
	Mat_<float> matches_kp1(3,n),matches_kp2(3,n);
//	vector<DMatch>::iterator it3;
//	for(it3 = twoside_matches.begin();it3 != twoside_matches.end();it3++)
//	{
//		matches_kp1.puch_back(kp);
//	}
	for(int i=0;i < twoside_matches.size();i++)
	{
		Point2f x1=kp1[twoside_matches[i].queryIdx].pt;
		Point2f x2=kp2[twoside_matches[i].queryIdx].pt;
		matches_kp1(0,i)=x1.x;
		matches_kp1(1,i)=x1.y;
		matches_kp1(2,i)=1;
		matches_kp2(0,i)=x2.x;
		matches_kp2(1,i)=x2.y;
		matches_kp2(2,i)=1;
	}
	cout<<"save keypoints"<<endl;
	FileStorage fs("points.yml",FileStorage::WRITE);
	fs<<"x1"<<matches_kp1;
	fs<<"x2"<<matches_kp2;
	fs.release();
        cout<<"show match"<<endl;
	namedWindow("img1",WINDOW_NORMAL);
	namedWindow("img2",WINDOW_NORMAL);
	namedWindow("imgmatch",WINDOW_NORMAL);
	imshow("img1",gimg1);
	imshow("img2",gimg2);
	imshow("imgmatch",imgmathces);
	waitKey(0);

	return 0;
}
