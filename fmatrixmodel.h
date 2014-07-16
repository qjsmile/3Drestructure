#ifndef FMATRIX_MODEL_H
#define FMATRIX_MODEL_H
#include "ransac.h"
#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;
class FMatrixModel:public RansacModel< Mat_<float>, Mat_<float>, Mat_<float> >
{
	public:
		Mat_<float> fit(Mat_<float>&data)
		{
			Mat_<float> x1=data(Range(0,3),Range(0,8));
			Mat_<float> x2=data(Range(3,6),Range(0,8));
			Mat_<float> f=compute_fundamental(x1,x2);	
			return f;
		}
		Mat_<float> get_error(Mat_<float>&data,Mat_<float>&f)
		{
			Mat_<float> x1=data.rowRange(0,3);
			Mat_<float> x2=data.rowRange(3,6);
			Mat_<float> Fx1=f*x1;
			Mat_<float> Fx2=f*x2;
			Mat_<float> Fx10,Fx11,Fx20,Fx21;
			pow(Fx1.row(0),2,Fx10);
			pow(Fx1.row(1),2,Fx11);
			pow(Fx2.row(0),2,Fx20);
			pow(Fx2.row(1),2,Fx21);
			Mat_<float> denom=Fx10+Fx11+Fx20+Fx21;
			Mat_<float> d;
			pow((x1.t()*(f*x2)).diag(0),2,d);
			Mat_<float> err=d.t()/denom;
			//cout<<err.rows<<"\t"<<err.cols<<endl;
			return err;

		}


	private:
      		Mat_<float> compute_fundamental(Mat_<float> &x1,Mat_<float> &x2)
        	{
            		Mat_<float> f(3,3);
            		int n=x1.cols;
            		if(x2.cols!=n)
                	cout<<"Number of point don't match.";
            		//init A
            		Mat_<float> A=Mat::zeros(n,9,CV_32F);
            		for(int i=0;i<n;i++)
            		{
                		for(int j=0;j<9;j++)
                		{
                    			A(i,j)=x1(j/3,i)*x2(j%3,i);
                		}
            		}
            		//compute linear least squra solution and n=8 here and f is 3*3
			//cout<<A<<endl;
            		SVD svd(A,SVD::FULL_UV);
            		Mat_<float> S=svd.w;
            		Mat_<float> VT=svd.vt;
			Mat_<float> sx(8,9);
			Mat::diag(S)(Range::all(),Range::all()).copyTo(sx(Range(0,8),Range(0,8)));
			//cout<<svd.w<<endl;
			//cout<<svd.u*sx*svd.vt<<endl;
			//cout<<svd.vt.row(8)<<endl;
			//cout<<svd.vt.row(8)/svd.vt.at<float>(8,8)<<endl;
			f=svd.vt.row(8).reshape(0,3);
			//cout<<A*f.reshape(0,9)<<endl;
            		//constrain f
            		//make rank 2 by zeroing out last sigular value
            		SVD svd1(f,SVD::FULL_UV);
			//cout<<svd1.w<<endl;
			svd1.w.at<float>(2,0)=0;
			//cout<<svd1.w<<endl;
			f=svd1.u*Mat::diag(svd1.w)*svd1.vt;

			//cout<<A*f.reshape(0,9)<<endl;
            		return f;

        }














};
#endif
