#ifndef RANSAC_H
#define RANSAC_H
#include<iostream>
#include<vector>
#include<algorithm>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace std;
using namespace cv;
template<typename T1,typename T2,typename T3>
class RansacModel
{
	public:
		virtual T1 fit(T2&data)=0;
		virtual T3 get_error(T2&data,T1&model)=0;
};
template<typename T1,typename T2,typename T3>
void ransac(T2&data,RansacModel<T1,T2,T3>&model,int n,int k,float t,float d,T1&bestFit,T2&inliers)
{
	int iterations=0;
	Mat_<float> bestfit;
	float besterr=10000.0;
	Mat_<float> best_inlier_idxs;
	vector<int > all_idxs;
	Mat_<float> maybeinliers(6,n);
	Mat_<float> test_points(6,data.cols-n);
	Mat_<float> maybemodel(3,3);
	Mat_<float> test_err;
	while(iterations < k)
	{
		//step 1: random points
		all_idxs=random_partition(n,data.cols);//random index
		/*for(int i=0;i<20;i++)
		{
			cout<< all_idxs[i]<<" ";
		}
		*/
		for(int i=0;i<6;i++)//maybeinliers points
		{
			for(int j=0;j<n;j++)
			{
				maybeinliers(i,j)=data(i,all_idxs[j]);
			}
		}
		cout<<"iter is"<<iterations<<endl;
		//cout<< maybeinliers.t()<<endl;
		for(int i=0;i<6;i++)//test points i
		{
			for(int j=n;j<data.cols;j++)
			{
				test_points(i,j-n)=data(i,all_idxs[j]);
			}
		}

		// step 2:compute alsoinliers 
		maybemodel=model.fit(maybeinliers);
		cout<<"maybemodel:"<<maybemodel<<endl;
		test_err=model.get_error(test_points,maybemodel);
		//cout<<"test ok"<<endl;
		vector<int> also_idxs;
		for(int j=0;j<test_err.cols;j++)
		{
			if(test_err(0,j)<t)
			{
				also_idxs.push_back(j);
			}
		}
		int alsopoints_len=also_idxs.size();
		Mat_<float> alsoinliers(6,alsopoints_len);
		for(int i=0;i<6;i++)
		{
			for(int j=0;j<alsopoints_len;j++)
			{
				alsoinliers(i,j)=test_points(i,also_idxs[j]);
			}
		}
		// step 3:compute bestfit
		cout<<"number is"<<also_idxs.size()<<endl;
		if(also_idxs.size() > d)
		{
			int betterdata_len=8+alsopoints_len;
			Mat_<float> betterdata(6,betterdata_len);
			Mat_<float> bettermodel(3,3);
			Mat_<float> better_errs(1,betterdata_len);
			float thiserr=0;
			//compute betterdata 
			for(int i=0;i<6;i++)
			{
				for(int j=0;j<8;j++)
				{
					betterdata(i,j)=maybeinliers(i,j);
				}
			}
			for(int i=0;i<6;i++)
			{
				for(int j=0;j<alsopoints_len;j++)
				{
					betterdata(i,8+j)=alsoinliers(i,j);
				}
			}
			cout<<"betterdata point is "<<betterdata.size()<<endl;
			// bettermodel
			bettermodel=model.fit(betterdata);
			better_errs=model.get_error(betterdata,bettermodel);
			//thiserr=mean(better_errs);
			for(int i=0;i<betterdata_len;i++)
			{
				thiserr+=better_errs(0,i);
			}
			thiserr=thiserr/betterdata_len;
			if(thiserr<besterr)
			{
				bestFit=bettermodel;
				besterr=thiserr;
				inliers=betterdata;
			
			}			
		}
	
		iterations++;
	}

}

vector<int> random_partition(int n,int n_data)
{
	vector<int> all_idxs;
	for(int i=0;i<n_data;++i)
	{
		all_idxs.push_back(i);
	}
	random_shuffle(all_idxs.begin(),all_idxs.end());
	return all_idxs;
}

#endif

