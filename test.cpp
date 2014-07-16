#include<iostream>
#include<vector>
#include<algorithm>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include"ransac.h"
#include"fmatrixmodel.h"
using namespace cv;
using namespace std;
double mean(Mat&m,int index,bool row=true)
{
	if(row)
		return mean(m.row(index))(0);
	else
		return mean(m.col(index))(0);
}
Mat_<float> compute_P_from_essential(Mat_<float> &E);
int main()
{

//ff 
/*
	Mat_<double> data;
	FileStorage fs("data.yml",FileStorage::READ);
	fs["x1"]>>data;
	cout<<data<<endl;
	FMatrixModel model;
	Mat_<double> f=model.fit(data);
	cout<<f<<endl;
	Mat_<double> err=model.get_error(data,f);
	cout<<err<<endl;
	*/
	FMatrixModel model;
	Mat_<float> data(6,9620);
	Mat_<float> data_t(9620,6);
	Mat_<float> x1;
	Mat_<float> x2;
	Mat_<float> bestFit;
	Mat_<float> inliners;
        float array[9]={2394,0,932,0,2398,628,0,0,1};
	Mat_<float> K(3,3,array);
	cout<<K<<endl;
	
	
	//step 1: load x1,x2:3 * n matrix
	FileStorage fl("points.yml",FileStorage::READ);
	fl["x1"]>>x1;
	fl["x2"]>>x2;
//	cout<<x2.t()<<endl;
	fl.release();

	//step 2: compute K 的逆 与x1,x2 的点乘--> x1n ,x2n :3 * n matrix
	Mat_<float> x1n;
	Mat_<float> x2n;
	x1n=(K.inv()) * x1;
	x2n=(K.inv()) * x2;
//	cout<<x2n.t()<<endl;

	//step 3:data: 6 * n matrix
	for(int i=0;i<x1n.rows;i++)
	{
		for(int j=0;j<x1n.cols;j++)
		{
			data(i,j)=x1n(i,j);
			data(3+i,j)=x2n(i,j);
		}
	}
	ransac(data,model,8,2,10,10,bestFit,inliners);
	cout<<"2  "<<bestFit.size()<<endl;
	cout<< "inliners:: "<< inliners.size()<<endl;
	//P1 = array([[1,0,0,0],[0,1,0,0],[0,0,1,0]]) 
       	float pdata[12]={1,0,0,0,0,1,0,0,0,0,1,0};
	Mat_<float> P1(3,4,pdata);
	Mat_<float> P2=compute_P_from_essential(bestFit);
/*	Mat_<float> P2i;
	Mat_<float> d1,d2;
	Mat_<float> X;
	int ind=0;
	int maxres=0;
	for(int i=0;i<13;i+3)
	{
		P2i=P2.rowRange(i,i+3);
		X=triangulate(x1n,x2n,P1,P2i);
		d1=(P1 * X).rowRange(2,3);
		d2=(P2i * X).rowRange(2,3);
		int sum=0;
		for(int j=0;j<X.cols;j++)\
		{
			if(d1(0,j)>0)
			{
				sum++;
			}
			if(d2(0,j)>0)
			{
				sum++;
			}
		}
		if(sum > maxres)
		{
			maxres=sum;
			ind=i;
		}
		
	}
	X=triangulate(x1n,x2n,P1,P2.rowRange(i,i+3));
	//print "pick the solution with points in front of cameras"
	//
	//
*/	
	return 0;
}

	Mat_<float> compute_P_from_essential(Mat_<float> &E)
	{
		//make sure E is rank 2
		SVD svd(E,SVD::FULL_UV);
		Mat_<float> U=svd.u;
		Mat_<float> VT=svd.vt;
		if(determinant(U*VT)<0)
		{
			VT=-1*VT;
		}
		Mat_<float>a;
		a(0,0)=1;
		a(0,1)=1;
		a(0,2)=0;
		E=U*(a.diag(0)*VT);
		//create matrices (Hartley p 258)
	/*	float z[3][3]={{0,-a(0,2),a(0,1)},
		   	       {a(0,2),0,-a(0,0)},
			       {-a(0,1),a(0,0),0}};
	*/
		float z[9]={0,-a(0,2),a(0,1),a(0,2),0,-a(0,0),-a(0,1),a(0,0),0};
		Mat_<float>Z(3,3,z);
		float w[9]={0,-1,0,1,0,0,0,0,1};
		Mat_<float>W(3,3,w);
		//return all four solution
		
		Mat_<float> P201=(U*(W*VT)).t();
		Mat_<float> P202=U.colRange(2,3);
		Mat_<float> P203(3,4);
		P201(Range::all(),Range::all()).copyTo(P203(Range(0,3),Range(0,3));
		P202(Range::all(),Range::all()).copyTo(P203(Range(3,4),Range(0,3)));
		Mat_<float>P20=P203.t();

		Mat_<float> P211=(U*(W*VT)).t();
		Mat_<float> P212=-1*U.colRange(2,3);
		Mat_<float> P213(3,4);
		P211(Range::all(),Range::all()).copyTo(P213(Range(0,3),Range(0,3));
		P212(Range::all(),Range::all()).copyTo(P213(Range(3,4),Range(0,3)));
		Mat_<float> P21=P213.t();


		Mat_<float> P221=(U*(W.t()*VT)).t();
		Mat_<float> P222=U.colRange(2,3);
		Mat_<float> P223(3,4);
		P221(Range::all(),Range::all()).copyTo(P223(Range(0,3),Range(0,3));
		P222(Range::all(),Range::all()).copyTo(P223(Range(3,4),Range(0,3)));
		Mat_<float> P23=P223.t();


		Mat_<float> P231=(U*(W.t()*VT)).t();
		Mat_<float> P232=-1*U.colRange(2,3);
		Mat_<float> P233(3,4);
		P231(Range::all(),Range::all()).copyTo(P233(Range(0,3),Range(0,3));
		P232(Range::all(),Range::all()).copyTo(P233(Range(3,4),Range(0,3)));
		Mat_<float> P23=P233.t();

		Mat_<float> P2(12,4);
		P20(Range::all(),Range:all()).copyTo(P2(Range(0,3),Range(0,4)));
		P21(Range::all(),Range:all()).copyTo(P2(Range(3,6),Range(0,4)));
		P21(Range::all(),Range:all()).copyTo(P2(Range(6,9),Range(0,4)));
		P23(Range::all(),Range:all()).copyTo(P2(Range(9,12),Range(0,4)));

		return P2;
		
	}	




