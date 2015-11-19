

#include "stdafx.h"	 //if u get error remove this header file, i used precompiled header project
#include <stdio.h>      // printf, scanf, NULL 
#include <stdlib.h>     // malloc, free, rand 
#include <iostream>
#include <math.h>	 // exp, pow
#include <time.h>
#include <list>
#include <cv.h>
#include <highgui.h>
#include "opencv2/highgui/highgui.hpp"
#include <limits>

using namespace cv;
using namespace std;

short int arrey[20][2] =   {{200,0},{190,62},{162,118},{118,162},{62,190},{0,200},{-62,190},{-118,162},{-162,118},{-190,62},{-200,0},{-190,-62},{-162,-118},{-117,-162},{-62,-190},{0,-200},{62,-190},{118,-162},{162,-117},{190,-62}}; 
int node[50][2];
int nodecounter = 0;
short int cost = 1;
int count1= 0;
char d;


short int trnd = 1;
Mat imageMat = imread("obstacle.jpg", CV_LOAD_IMAGE_GRAYSCALE);
Mat output(imageMat.rows,imageMat.cols, CV_8UC(1), Scalar(0));
Mat binaryMat(imageMat.size(), imageMat.type());

	int xr = 40;
	int yr = 40; 
	int xg = 570;
	int yg = 160;

float dist( int x1,int y1,int x2,int y2)
{   
	float cost = sqrt((float)(abs(x1-x2)*abs(x1-x2)) + (float)(abs(y1-y2)*abs(y1-y2)));
	return cost;
}   

short int isgoal_h(int xn, int yn)
{
	short int is;
    if ((abs(xn-xg) < 20) && (abs(yn-yg) < 20))
	{
		is = 1; 
		cout<<"-----GOT GOAL POINT-----"<<endl;
	}
    else
        is = 0;
	return is;
}

float h_h(int xh,int yh)
{
	float hue = sqrt((float)(abs(xh-xg)*abs(xh-xg)) + (float)(abs(yh-yg)*abs(yh-yg)));
	return hue;
}

short int quantize_h(int x, int y, float m)
{
	float slope;

	if(isgoal_h(x,y) == 1)
	{
		node[nodecounter][0] = x;
		node[nodecounter][1] = y;
        //cout<<"exit"<<endl;
        return 0;
	}

	for (short int i=0;i<20;i++)
	{

		//cout<<y+arrey[i][1]<<" "<<x+arrey[i][0]<<endl;

		if  ((x + arrey[i][0] > 0) && (x + arrey[i][0] < 640) && (y + arrey[i][1] > 0) && (y + arrey[i][1] < 480))
		{
			if (output.at<uchar>(y+arrey[i][1],x+arrey[i][0])==255)
			{
				if(arrey[i][0] != 0)
					slope = arrey[i][1]/arrey[i][0];
				else
					slope = 100000;
				//cout<<"Slope: "<<slope<<endl;
				output.at<uchar>(y,x) = 0;

				if (m!=slope)
				{
				
					//cout<<"node no: "<<nodecounter<<endl;
					node[nodecounter][0] = x;
					node[nodecounter][1] = y;
					nodecounter++;
				}
    
			    short int quant =  quantize_h(x+arrey[i][0],y+arrey[i][1],slope);
				return	quant;
			}
		}
	}
}

float search_h(int xnode, int ynode, float g,float bound)
{                             
    float f = g + h_h(xnode, ynode);
	float find;
	//cout<<"g:  "<<g<<endl;
	//printf("b: %.15g\n", bound);
	//printf("f: %.15g\n", f);
count1++;
//imageMat.at<uchar>(ynode,xnode) = 255;
//imshow("Output", output);
//cvWaitKey(1);
//if(count1%100 == 0)
		//cout<<"count: "<<count1<<endl;
//cout<<"x: "<<xnode<<"y: "<<ynode<<endl;

   
    if (f>bound)
        return f;
		//printf("%.15g\n", bound);

	
	//cin>>d;
    
    output.at<uchar>(ynode,xnode) = 255;

    if (isgoal_h(xnode,ynode) == 1)
	{ 

		output = Scalar(0);
        output.at<uchar>(ynode,xnode) = 255;
        find = 10000;
        return find;
	}
   
    
    float min = std::numeric_limits<float>::max();
	float t;
    
    for (short int k=0; k<20 ; k++)
	{   
		//cout<<arrey[k][0]<<","<<arrey[k][1]<<endl;
		if  ((xnode + arrey[k][0] > 0) && (xnode + arrey[k][0] < 640) && (ynode + arrey[k][1] > 0) && (ynode + arrey[k][1] < 480)) 
		{
			//cout<< (int)binaryMat.at<uchar>(ynode+arrey[k][1],xnode+arrey[k][0]) <<endl;
			if ((binaryMat.at<uchar>(ynode+arrey[k][1],xnode+arrey[k][0]) == 0) && (output.at<uchar>(ynode+arrey[k][1],xnode+arrey[k][0]) != 255))
			{
				//cout<<arrey[k][0]<<","<<arrey[k][1]<<endl;
				t = search_h(xnode+arrey[k][0],ynode+arrey[k][1],g + dist(xnode,ynode,xnode+arrey[k][0],ynode+arrey[k][1]),bound);
				
                       
					if (t == 10000)
					{
						//cout<<"------------"<<endl;
						find = 10000;
                        output.at<uchar>(ynode,xnode) = 255;
                        //disp('[')
                        //disp(test((ynode),(xnode)))
                        //disp(']')
                        //disp('---------------')
                        //cout<<"["<<xnode<<","<<ynode<<"]";
                        //disp('---------------')
                        //imshow(output);
                        return find;
					}
            
                    if (t < min)
						min = t;
                    
			}
		}
	}

    find = min;
	return find;

}

int ida_h(int xs,int ys)
{
	float bound = h_h(xs,ys);
	float t = 0;
	float star;
    while (1) 
	{
		output = Scalar(0);
        t = search_h(xr,yr,0,bound);
 
	//cout<<"count ida: "<<count1<<endl;
    
        if (t == 10000)
		{
			star = 10000;
			return star;

		}
        else if(t ==  std::numeric_limits<float>::max())
		{
			star =   std::numeric_limits<float>::max();
			return star;
		}
        else
		{
			//cout<<"b chng:  "<<t<<endl;
			bound = t;
		}
	}
	return star;
}


int main()
{
	clock_t time;
    time = clock();
    if (imageMat.empty())
    {
        cout << "ERROR: Could not read image " <<endl;
		return 1;
    }

	cout << "1 | Got image" <<endl;

	//output = Scalar(0);
	cout << "2 | Output image" <<endl;

    //Binary image
	

    //Apply thresholding
	threshold(imageMat, binaryMat, 100, 255, THRESH_BINARY);
	cout << "3 | Thresholding" <<endl;
	namedWindow("Output");
    imshow("Output", binaryMat);
	cvWaitKey(3000);

	cout << "4 | Calling ida*()" <<endl;

	int fans =  ida_h(xr,yr);

	if (fans == std::numeric_limits<float>::max())
		cout<<"5 | Failed to find solution :("<<endl;
	else if (fans == 10000)
		cout<<"5 | Got solution :)"<<endl;
	
	namedWindow("Output1");
	imshow("Output1", output);
    cvWaitKey(3000);

	cout << "6 | Calling quantize()" <<endl;
	cout<<endl;
	quantize_h(xr,yr,20);


	cout << "7 | Quantize nodes" <<endl;

	for (short int x = 0;x<50;x++)
	{
		if((node[x][0]!=0) && (node[x][1]!= 0))
		{
			cout<<"     "<<x+1<<" - ["<<node[x][0]<<","<<node[x][1]<<"]"<<endl;
			circle(binaryMat, Point(node[x][0],node[x][1]), 5, Scalar(128),2);
			//cin>>d;
			if(x>0)
				line(binaryMat,Point(node[x][0],node[x][1]),Point(node[x-1][0],node[x-1][1]),Scalar(255),1);
		}
	}



    //Show the results
   	cout << "8 | Showing output" <<endl;
	time = clock() - time;
	double time_taken = ((double)time)/CLOCKS_PER_SEC; // in seconds
    printf("9 | Time taken %f secs\n", time_taken-6);
	
	namedWindow("Output");
    imshow("Output", binaryMat);

    waitKey(0);

    return 0;
}
