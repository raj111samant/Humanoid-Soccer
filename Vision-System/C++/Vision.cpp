
#include "stdafx.h"
#include <cv.h>
#include <highgui.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <math.h>
#include <iostream>
#include <windows.h>
#include <conio.h>
#include "zigbee.h"

using namespace cv;
using namespace std;

//-----------------------------------Macros-----------------------------------------------
#define Width 960 //Width of camera frame
#define Height 720 //Height of camera frame
#define FPS 30 //FPS of camera frame
#define conv 180/CV_PI
#define offset 10    //to set offset position
#define Balloffset 80  //min compliance for considering ball to be stationary 
#define step 200  //1 step distance of bot
#define BLUE_PORT	8 // COM3
#define RED_PORT	9 // COM3
//-----------------------------------Global Varialbles-------------------------------------
double sendpos=0;  // id to be sent
int statusofbots[3]={0,0,0}; 
int dilation_elem = 0;
int dilation_size = 0;

int erosion_elem = 0;
int erosion_size = 0;
int const max_elem = 2;
int const max_kernel_size = 21;
int ID = 0;      // ball,blue,red
double xp[8] = {0,0,0,0,0,20,940,0}, yp[8] = {0,0,0,0,0,360,360,0} ;   //co-ordinates of ball,   [ball,robot1(red),robot2(blue),robot1-green,robot2-green,goal1(red),goal2(blue),striker offset]
float goalangles[3]={0,0,0};
Mat erosionsrc,dilatedst,erosiondst; 
int ballpos=0;
float angles[3]={0.0,0.0,0.0};
int def=2,str=1;
float dist[3]={0,0,0};
float houghred=0,houghblue=0;
//-----------------------------------Function Prototype-------------------------------------
void Printintro(); //Printing info
void Dilation( int, void* );

void send(int data)
{
	if(zgb_tx_data(data) == 0)
	cout<<"Failed to transmit\n"<<endl;
}

void initialize(int id)
{
	if( zgb_initialize(id) != 0 )
	cout<<"ZIG DONE"<<endl;
	else
	cout<<"error zig intialization"<<endl;
}

//-------------------Function to decide who is Striker & who is Defender---------------------
void decide()    // decide who is striker and defender
{
	if(xp[0]<480)
	{	
	str=1;
	def=2;
	}
	else 
	{
	str=2;
	def=1;
	}

	//cout<< "STR = "<<str<<"\tDEF = "<<def<<endl;
}

void Erosion( int, void* )
{
 int erosion_type;
 if( erosion_elem == 0 ){ erosion_type = MORPH_RECT; }
 else if( erosion_elem == 1 ){ erosion_type = MORPH_CROSS; }
 else if( erosion_elem == 2) { erosion_type = MORPH_ELLIPSE; }

 Mat element = getStructuringElement( erosion_type,
                                      Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                      Point( erosion_size, erosion_size ) );

 /// Apply the erosion operation
 erode(  erosionsrc, erosiondst, element );
 imshow( "Erosion Demo", erosiondst );
}
//----------------------------NEW DEF------------------------------------------------
int defence_working()
{
	float a;
	initialize(def + 17); 
    cout<<"\tDEF status of bot def = "<<statusofbots[def]<<endl;
    //-----face goal----------------------------------
    if(statusofbots[def]==0)
    {
        sendpos = angles[def]-goalangles[1];
	cout<<"\tDEF sendpos   "<<sendpos<<endl;
	 if((sendpos < 189.0) && (sendpos > 171.0))
	{
	statusofbots[def] = 1;
	 }	
	else if((sendpos > 0.0) && (abs(sendpos) > 18.0))
        {
            if(def == 1)
	{
	a = (180.0 - abs(sendpos))/18.0;
	send(20 + (a>3?3:a));
	cout<<"\tDEF clock wise by angle = "<<(180.0 - abs(sendpos))<<endl; //red
	}
            else if(def == 2)
	{
	a = (180.0 - abs(sendpos))/18.0;
	send(30 + (a>3?5:3));
	cout<<"\tDEF anticlock wise by angle = "<<(180.0 - abs(sendpos))<<endl; //blue
	}             
        }
        else if((sendpos < 0.0) && (abs(sendpos)) > 18.0)
        {
            if(def == 1)
	{
	a = (180.0 - abs(sendpos))/18.0;
	send(30 + (a>3?3:a));
	cout<<"\tDEF anticlock wise by angle = "<<(180.0 - abs(sendpos))<<endl; //red
	}
	else if(def == 2)
	{	
	a = (180.0 - abs(sendpos))/18.0;
	send(20 + (a>3?3:a));
	cout<<"\tDEF clock wise by angle = "<<(180.0 - abs(sendpos))<<endl; //blue
	}
	}
       
    }
    //---------angle correcting done now walk------------------
    else if(statusofbots[def] == 1)
    {
	cout<<"\tDEF sendpos   "<<sendpos<<endl;
        if ((sendpos > 198.0) && (sendpos < 162.0) )
                statusofbots[def] = 0;
        else
        {
            if(dist[def] > step)
            {
	a = dist[def]/step;
	send(9 + (a>3?3:a));
	cout<<"\tDEF walk by "<<dist[def]<<endl;
	}
	else
                statusofbots[def] = 2;
        }
    }
    //------walking done now face ball---------------------------
    else if(statusofbots[def] == 2)
    {
	cout<<"\tDEF sendpos   "<<sendpos<<endl;
        sendpos = angles[def]-goalangles[0];
        if((sendpos > 0.0) && (abs(sendpos) > 18.0))
        {
            if(def == 1)
	{
	a = abs(sendpos)/18.0;
	        send(30 + (a>3?3:a)); 
	cout<<"\tDEF anticlock wise by angle = "<<abs(sendpos)<<endl; //red
	}
            else if(def == 2)
	{
	a = abs(sendpos)/18.0;
	        send(20 + (a>5?5:a));
	cout<<"\tDEF clock wise by angle = "<<abs(sendpos)<<endl; //blue
	}
        }
        else if((sendpos < 0.0) && (abs(sendpos)) > 18.0)
        {
            if(def == 1)
                {
	a = abs(sendpos)/18.0;
	        send(20 + (a>3?3:a));
	cout<<"\tDEF clock wise by angle = "<<abs(sendpos)<<endl; //red
	}
	else if(def == 2)
	{
	a = abs(sendpos)/18.0;
	        send(30 + (a>3?3:a));
	cout<<"\tDEF anticlock wise by angle = "<<abs(sendpos)<<endl; //blue
	}
	}
        else if(abs(sendpos) < 18.0)
            cout<<"\tDEF Ready to save"<<endl;
    }
	return 0;
}

//---------------------------NEW ALGO-------------------------------------------------
int strikernew()
{ 
	cout<<"sdhvn  "<<str<<"hieh "<<statusofbots[str]<<endl;
	//-----------for facing proper quadrant----------
	if(statusofbots[str] == 0)
	{
	if(yp[7] < yp[str])
	{
	if(angles[str] > 5.0)
	{
	if(str == 1)
	cout<<"move anitclock"<<endl;
	else if(str == 2)
	cout<<"move clock"<<endl;
	}
	else if((angles[str] <= 5.0)||(angles[str] >= 355.0))
	{
	cout<<"angle correction done!!!!"<<endl;
	statusofbots[str] = 1;
	}
	}
	else if(yp[7] > yp[str])
	{
	if(angles[str] < 175.0)
	{
	if(str == 1)
	cout<<"move clock"<<endl;
	else if(str ==2)
	cout<<"move anitclock"<<endl;
	}
	else if((angles[1] <= 185.0)&&(angles[1] >= 175.0))
	{
	cout<<"angle correction done!!!!"<<endl;
	statusofbots[str] = 1;

	}

	}

	}

	//-----------for walking till y of offset and ball are equal----------
	else if(statusofbots[str] == 1)
	{
	if(abs(yp[7]-yp[str]) > 10.0)
	cout<<"move forward"<<endl;
	else if(abs(yp[7]-yp[str]) <= 10.0)
	{
	cout<<"reached y co-ordi"<<endl;
	statusofbots[str] = 2;
	}	
	}

	//-----------for facing ball----------
	else if(statusofbots[str] == 2)
	{
	if(yp[7] < 360)
	{
	if(angles[str] <  85.0 )
	{
	if(str == 1)
	cout<<"move clock"<<endl;
	else if(str == 2)
	cout<<"move anticlock"<<endl;
	}
	else if((angles[str] <= 95.0)&&(angles[str] >= 85.0))
	{
	cout<<"angle correction done!!!!"<<endl;
	statusofbots[str] = 3;
	}
	}
	else if(yp[7] > 360)
	{
	if(angles[str] > 95.0)
	{
	if(str == 1)
	cout<<"move anticlock"<<endl;
	else if(str ==2)
	cout<<"move clock"<<endl;
	}
	else if((angles[1] <= 95.0)&&(angles[1] >= 85.0))
	{
	cout<<"angle correction done!!!!"<<endl;
	statusofbots[str] = 3;

	}

	}
	}

	//-----------for walking towads offset------------------
	else if(statusofbots[str] == 3)
	{
	if(abs(xp[7]-xp[str]) > 5.0)
	cout<<"move forward"<<endl;
	else if(abs(xp[7]-xp[str]) <= 5.0)
	{
	cout<<"reached x co-ordi"<<endl;
	cout<<"reached OFFSET"<<endl;
	statusofbots[str] = 4;
	}	
	}

	//--------for facing goal---------------
	else if(statusofbots[str] == 4)
	{
	if(yp[7] < 360)
	{
	if(abs(angles[str] -  goalangles[0]) > 10.0)
	{
	if(str == 1)
	cout<<"move clock"<<endl;
	else if(str == 2)
	cout<<"move anticlock"<<endl;
	}
	else if(abs(angles[str] -  goalangles[0]) <= 10.0)
	{
	cout<<"FACING BALL!!!!"<<endl;
	statusofbots[str] = 5;
	}
	}
	else if(yp[7] > 360)
	{
	if(abs(angles[str] -  goalangles[0]) > 10.0)
	{
	if(str == 1)
	cout<<"move anticlock"<<endl;
	else if(str ==2)
	cout<<"move clock"<<endl;
	}
	else if(abs(angles[str] -  goalangles[0]) < 10.0)
	{
	cout<<"FACING BALL!!!!"<<endl;
	statusofbots[str] = 5;

	}

	}
	}

	//----------------Kick------------------------------------------
	else if(statusofbots[str] == 5)
	cout<<"KICK!!!!"<<endl;

	return 0;


}

//---------------------------Function for Defender-------------------------------------
int defend()
{printf("status of bot def=%d",statusofbots[def]);
	if(statusofbots[def]==0)
	{
	
	sendpos = pow(-1,(double)def) * (angles[def]-goalangles[1])/15.0;
	//double sendpos2 =( pow(-1,(double)def) * (angles[def]-goalangles[1]));
	double sendpos2 = sendpos*15.0;
	printf("\n\n\n\ndef POS %f\n\n",sendpos2);
	if((sendpos2>0)  && abs(sendpos2)>25.0)
	{	
	sendpos = ( 180-abs(angles[def]-goalangles[1]))/15.0;
	printf("\npress 'anticlockwise + %f ' def=bot no %d  ",abs(sendpos)>5?5:abs(sendpos) ,def);   //first angle correction if goal angles are negative(more than 180)
	return 0;
	}
	else if(sendpos2<0 && abs(sendpos2)>25.0)   //compliance of 5
	{
	sendpos =  (180-abs(angles[def]-goalangles[1]))/15.0;
	printf("\npress 'clockwise + %f ' def=bot no %d  ",sendpos>5?5:sendpos ,def);  
	return 0;
	}
	else if(abs(sendpos2)<25.0)
	{
	printf("\nangle correction done");   // first angle correction if goal angles are right
	statusofbots[def]++;
	return 0;
	}
	}
	else if(statusofbots[def]==1)     //walk the distance to the goal 
	{
	
	sendpos=dist[def]/step;
	
	if(dist[def]>10)
	{
	printf("\npress forward + %f ' def=bot no %d  ",sendpos>5?5:sendpos ,def);    //send to walk straight
	}
	else if (dist[def]<10.0)
	statusofbots[def]++;
	
	    if(abs(angles[def]-goalangles[1])>20.0)      //check angle after walking
	statusofbots[def]=0;

	}

    else if(statusofbots[def]==2)
	{
	
	sendpos = pow(-1,(double)def) * (angles[def] - goalangles[0])/15.0;
	if((sendpos<0)  && abs(sendpos)>10.0)
	{	
	printf("\npress 'anticlockwise + %f ' def=bot no %d  ",sendpos>5?5:sendpos ,def);   //first angle correction if goal angles are negative(more than 180)
	return 0;
	}
	else if(sendpos>0 && abs(sendpos)>10.0)   //compliance of 5
	{
	printf("\npress 'clockwise + %f ' def=bot no %d  ",sendpos>5?5:sendpos ,def);  
	return 0;
	}
	else if(abs(sendpos)<20.0)
	{
	printf("\nangle correction done");   // first angle correction if goal angles are right
	statusofbots[def]++;
	return 0;
	}	
	}
	
     else if( statusofbots[def]==3)
	{
	printf("save buddy!");
        statusofbots[def]=0;
    }
}

//-------------------------STRIKER------------------------------------------------------
int striker()
{
	float a;
	initialize(str + 17);
	printf("STR status str %d ",statusofbots[str]);
	if(statusofbots[str]==0)
	{
	sendpos = angles[str] - goalangles[2];
	printf("STR SEND POS %f",sendpos);
	if((sendpos<0.0)  && abs(sendpos)>18.0)
	{
	if(str == 1)
	{
	a = abs(sendpos)/18.0;
	send(20 + (a>3?3:a));
	printf("\nSTR 'clockwise + %f ",abs(sendpos));   //first angle correction if goal angles are negative(more than 180)
	}
	else if (str == 2)
	{
	a = abs(sendpos)/18.0;
	send(30 + (a>3?3:a));
	printf("\nSTR 'anticlockwise + %f ",abs(sendpos));   //first angle correction if goal angles are negative(more than 180)
	}	
	return 0;
	}
	else if(sendpos>0 && abs(sendpos)>18.0)   //compliance of 5
	{
	if(str == 1)
	{
	a = abs(sendpos)/18.0;
	send(30 + (a>3?3:a));
	printf("\nSTR 'anticlockwise + %f ",abs(sendpos)); 
	}
	else if(str == 2)
	{
	a = abs(sendpos)/18.0;
	send(20 + (a>3?3:a));
	printf("\nSTR 'clockwise + %f ",abs(sendpos));
	}
	return 0;
	}
	else if(abs(sendpos)<18.0)
	{
	printf("\nSTR angle correction done");   // first angle correction if goal angles are right
	statusofbots[str] = 1;
	return 0;
	}
	}
	else if(statusofbots[str]==1)     //walk the distance to the goal 
	{
	sendpos=dist[str]/step;
	cout<<"STR status : "<<statusofbots[str]<<endl;
	if(abs(angles[str]-goalangles[2])>18.0)
	{
	//statusofbots[str] = 0;
	}
	else
	{	
	if(dist[str]>10.0)
	{
	a = dist[str];	
	send(9 + (a>3?3:a));
	printf("\nSTR forward + %f ",sendpos);    //send to walk straight
	}
	else if (dist[str] <= 10.0)
	statusofbots[str] = 2;
	
	}
	return 0;
	}

    else if(statusofbots[str] == 2)
	{
	
	sendpos = (angles[str] - 180 + goalangles[0]);
	if((sendpos<0.0)  && abs(sendpos)>18.0)
	{	
	if(str == 1)
	{
	a = abs(sendpos)/18.0;
	send(20 + (a>3?3:a));
	printf("\nSTR 'clockwise + %f ",abs(sendpos));   //first angle correction if goal angles are negative(more than 180)
	}
	else if(str == 2)
	{
	a = abs(sendpos)/18.0;
	send(30 + (a>3?3:a));
	printf("\nSTR 'anticlockwise + %f ",abs(sendpos));   //first angle correction if goal angles are negative(more than 180)
	}
	return 0;
	}
	else if(sendpos>0.0 && abs(sendpos)>18.0)   //compliance of 5
	{
	if(str == 1)
	{
	a = abs(sendpos)/18.0;
	send(30 + (a>3?3:a));
	printf("\nSTR 'anticlockwise + %f ",abs(sendpos));   //first angle correction if goal angles are negative(more than 180)
	}
	else if(str == 2)
	{
	a = abs(sendpos)/18.0;
	send(20 + (a>3?3:a));
	printf("\nSTR 'clockwise + %f ",abs(sendpos));   //first angle correction if goal angles are negative(more than 180)
	}
	return 0;
	}
	else if(abs(sendpos)<18.0)
	{
	printf("\nSTR angle correction done");   // first angle correction if goal angles are right
	statusofbots[str] = 3;
	return 0;
	}	
	}
	
    else if( statusofbots[str]==3)
	{
	//printf("\nSTR striker small forward==1");
	printf("STR [[kick buddy]]!");
	send(10);
	    //statusofbots[str]=0;
	}
}

//------------------------------- function to get all the distances and angls from the frame-------------------------------------------------//
void goals()
{
	if (def == 1)
	{	
	goalangles[0] = conv*atan2((xp[0]-xp[5]),(yp[5]-yp[0]));  // red goal and ball 
	xp[7] = xp[0] + Balloffset*sin((goalangles[0]/180)*CV_PI);
	yp[7] = yp[0] - Balloffset*cos((goalangles[0]/180)*CV_PI);
	goalangles[1] = conv*atan2((xp[1]-xp[5]),(yp[5]-yp[1]));  //red goal and red bot(defender)
	goalangles[2] = 180.0 - conv*atan2((xp[2]-xp[7]),(yp[7]-yp[2]));	// offset & blue striker	
	dist[def] = sqrt(pow(yp[5]-yp[1],2)+pow(xp[5]-xp[1],2));   // goal and defender(red)
	dist[str] = sqrt(pow(yp[7]-yp[2],2)+pow(xp[7]-xp[2],2));   //balloffset and striker(blue)

	
	}
	else if (def==2)
	{   
	goalangles[0] = conv*atan2((xp[6]-xp[0]),(yp[6]-yp[0])); //blue goal and ball
	xp[7] = xp[0] - Balloffset*sin((goalangles[0]/180)*CV_PI);
	yp[7] = yp[0] - Balloffset*cos((goalangles[0]/180)*CV_PI);
	goalangles[1] = (180/CV_PI)*atan2((xp[6]-xp[2]),(yp[6]-yp[2]));  //blue goal and blue bot
	goalangles[2] =180.0 - (conv*atan2((xp[7]-xp[1]),(yp[7]-yp[1]))); //offset and red striker
	dist[def] = sqrt(pow(yp[6]-yp[2],2)+pow(xp[6]-xp[2],2)); // goal and defender(blue)  // i have set new index as distance cannot be directly used as 0 and 1.It can be usedas str and def for simplicity later on
	dist[str] = sqrt(pow(yp[7]-yp[1],2)+pow(xp[7]-xp[1],2)); //balloffset and striker(red)

	
	}
	
	
	printf("\n------------------------\n");
	printf("\n DEF GOAL -> BALL /_ = %d",(int)goalangles[0]);
	printf("\n DEF GOAL -> DEF BOT /_ = %d",(int)goalangles[1]);
	printf("\n BALL OFFSET -> STR BOT /_ = %d",(int)goalangles[2]);
	printf("\n DEF GOAL - DEF BOT -- = %d",(int)dist[def]);
	printf("\n BALL OFFSET -> DTR BOT -- = %d",(int)dist[str]);
	printf("\n X = %f - Y = %f", sin((goalangles[0]/180)*CV_PI) ,cos((goalangles[0]/180)*CV_PI));
	printf("\n ANGLE RED BOT /_ = %d",(int)angles[1]);
	printf("\n ANGLE BLUE BOT /_ = %d ",(int)angles[2]);
	printf("\n------------------------\n");
	

}


//-------------------------------------Main Function----------------------------------------
int main()
{
//-----------------------------------Print Intro--------------------------------------------
   // Printintro(); //Call print intro function

//-------------------------------------Variables--------------------------------------------
    int c = 0; //For exiting while loop
    Point pt1, pt2;
    double a, b;
    double x0, y0,xr,yr;	// xr,yr used for centre detection(temp variable)
    double pixels = 0;
	int Hmax[5] = {93,15,124,31,31}; //Hue max for 
    int Hmin[5] = {60,0,104,20,20}; //Hue mmin for ball, Robot1 (blue), Robot2 (red)
    int Smax[5] = {255,255,188,255,255}; //Sat max for ball, Robot1 (red), Robot2 (blue)
    int Smin[5] = {174,190,78,71,71}; //Sat min for ball, Robot1 (red), Robot2 (blue)
    int Vmax[5] = {255,176,229,255,255}; //Val max for ball, Robot1 (red), Robot2 (blue)
    int Vmin[5] = {30,100,112,74,74}; //Val min for ball, Robot1 (red), Robot2 (blue)
	int ballp[2] = {0,0};
    int no = 0;
    double x = 0, y = 0, a1 = 0, b1 = 0;
    float theta1 = 0;   
	int startx,starty,xlimit,ylimit;
	//double answr = cos(3*CV_PI/4);
	//printf("%f",answr);
	
	
	//-------------------------------------Camera-----------------------------------------------  
	VideoCapture capture(1); //Open the default camera
    capture.set(CV_CAP_PROP_FRAME_WIDTH, Width); //Width
    capture.set(CV_CAP_PROP_FRAME_HEIGHT, Height); //Height
    capture.set(CV_CAP_PROP_FPS, FPS);  //FPS

	VideoWriter writer("test.avi",CV_FOURCC('D','I','V','X'),30,cvSize(960,720));

    if(!capture.isOpened())  //Check if we succeeded
    {
        printf( "ERROR: capture is NULL... PLZ Check Camera...\n" );
        getchar();
        return -1;
    }
    printf("  Camera Properties\n  Width = %d; Height = %d; FPS = %d\n",Width,Height,FPS);

//------------------------------------Frame--------------------------------------------------    
    Mat frame; //Take 1st frame
    capture >> frame;
	
    
	if ( !frame.data ) 
    {
        printf( "ERROR: frame is null... PLZ Check frame is not recieved from camera\n" );
        getchar();
        return -1;
    }
    Mat imgHSV = frame.clone(); //make HSV image with same parameters as frame
    vector<Vec2f> lines;
    Mat imgThresh;

//-------------------------------Create Windows for display----------------------------------
    namedWindow("Live Video"); //Create window by name Live Video for seeing output of camera
     //namedWindow("HSV Video"); //Create window by name Live Video for seeing output of camera
	namedWindow("Binary Video");
	namedWindow( "Dilation Demo", CV_WINDOW_AUTOSIZE );
	//namedWindow( "Erosion Demo", CV_WINDOW_AUTOSIZE );


//-------------------------------Code for adjustments----------------------------------------
	 while(1)
    {
	capture >> frame;
	//writer.write(frame);
	printf("\r PLZ adjust fast!!!");
	imshow("Live Video", frame);
	c = cvWaitKey(10); //Wait 10ms for user exit
	if((char)c==27 ) //If ESC then break while loop
	break; 
    }
	 //writer.release();
	system("cls");
//--------------------------------------------------------------------------------------------
	//Create Dilation Trackbar
	createTrackbar( "Element:\n 0: Rect \n 1: Cross \n 2: Ellipse", "Dilation Demo", &dilation_elem, max_elem, Dilation );
	createTrackbar( "Kernel size:\n 2n +1", "Dilation Demo", &dilation_size, max_kernel_size, Dilation );

//	 createTrackbar( "Element:\n 0: Rect \n 1: Cross \n 2: Ellipse", "Erosion Demo",&erosion_elem, max_elem,Erosion );

// createTrackbar( "Kernel size:\n 2n +1", "Erosion Demo",&erosion_size, max_kernel_size,Erosion );

//-----------------------------While starts------------------------------------------------
     while(1)
    {

	
        capture >> frame;	//get next frame from camera buffer
	line( frame,Point(0,360),Point(960,360), Scalar(255,255,255), 1, CV_AA);
	line( frame,Point(480,0),Point(480,720), Scalar(255,255,255), 1, CV_AA);
	if ( !frame.data ) 
	 {
	printf( "all frames done" );
	getchar();
	break;
	//return -1;
	 }
       // GaussianBlur( frame, frame, Size(9, 9), 2, 2 );	//smooth the original image using Gaussian Filter
        cvtColor(frame, imgHSV, CV_BGR2HSV);	//Change the color format from BGR to HSV
        
        while(ID < 5)
        {  
	                                                     //make the counters zero  
	startx=startx=1;	    // start pixels for counters(centroid algorithm)
	xlimit = 960;
	ylimit = 720;
//	printf("\nID = %d   ",ID);
            inRange(imgHSV, Scalar(Hmin[ID],Smin[ID],Vmin[ID]), Scalar(Hmax[ID],Smax[ID],Vmax[ID]), imgThresh);
              // erosiondst = imgThresh;
	
	//Erosion(0,0);
	//	Dilation( 0, 0 );
	//	imgThresh = dilatedst;
	//Erosion(0,0);
	//imgThresh = erosiondst;
	if(ID > 2)
	{
	startx=(480*(ID-3))+1;
	starty=1;
	xlimit =startx + 480;    // 100*100 pixels scanned
	ylimit = 720;
	}

            for(int i = (startx<1?1:startx) ;  i < (xlimit>960?960:xlimit);i++)          //960*720
            {
                for(int j =  (starty<1?1:starty); j< (ylimit>720?720:ylimit) ;j++)
                {
                    if((int)imgThresh.at<uchar>(j,i) == 255)	
                    {  //  printf("i= %d",i);
	 //printf("j= %d",j);
                        pixels++;
	xr += i;
                        yr += j;
                    }
                }
            }

	yp[ID] = yr/ pixels;    //send average into array
	xp[ID] = xr/ pixels;
	xr = yr = 0;
            pixels = 0;
	//cvWaitKey(10);

	circle(frame,Point((int)xp[ID],(int)yp[ID]),5,Scalar( 255, 0, 0 ),-1,8);
	
	//	imshow("Live Video", frame); //Display camera frame in Live Video window
	
	if (ID == 0) //For ball position
	{
	if(abs(ballp[0] - xp[0]) > Balloffset || abs(ballp[1] - yp[0]) > Balloffset) //If ball is moving
	{
	//	printf("ball offset x %f",abs(ballp[0] - xp[0]) );
	//	printf("ball offset y %f",abs(ballp[1] - yp[0]) );
	
	ballp[0] = xp[0];
	ballp[1] = yp[0];
	ID=0; 
	statusofbots[def]=0;
	statusofbots[str]=0;
	goto Ballmove;
	
	}	
	}
	//cvWaitKey(100);

            if(ID==1 || ID==2)
            {
	
	HoughLines(imgThresh, lines, 1, CV_PI/180, 45, 0, 0 );
                //printf("\rID- %d   N- %d   ",ID,(int)lines.size());
                no = (int)lines.size();
	
	if(no != 0)
	{

                for( size_t i = 0; ((i < lines.size())&&((int)no>0)); i++ )
                {
                    float rho = lines[i][0], theta = lines[i][1];
                    if ((theta*180/CV_PI) > 180)
                    {
                        theta = CV_PI*(360 - (theta*180/CV_PI))/180;
                    }    
        
                    theta1 += theta;
                    a = cos(theta), b = sin(theta);
                    x0 = a*rho, y0 = b*rho;
                    a1 += a;
                    b1 += b;
                    x += x0;
                    y += y0;
                }  
        
	if(no==0)   //if we get 0 lines
	no=1;

                b1 /= no;
                a1 /= no;
                x /= no;
                y /= no;
                theta1 /= no;
                
	pt1.x = cvRound(x + 1000*(-b1));
                pt1.y = cvRound(y + 1000*(a1));
                pt2.x = cvRound(x - 1000*(-b1));
                pt2.y = cvRound(y - 1000*(a1));
                angles[ID]= theta1*180/CV_PI;
	//printf("A = %f   x = %f   y = %f   ",theta1*180/CV_PI,x,y);
               // cvWaitKey(5);
	}
                x = y = a1 = b1 = 0;
                theta1 = 0;

	line( frame, pt1, pt2, Scalar(0,0,255), 1, CV_AA);    
	}

        imshow("Live Video", frame); //Display camera frame in Live Video window
	writer.write(frame);
	imshow("Binary Video", imgThresh); 
	imshow("Binary Video", imgThresh); 

	cvWaitKey(100);
        ID++;
        }
	//printf("\rR-%d/%d Y-%d/%d B-%d/%d Y-%d/%d",(int)xp[1],(int)yp[1],(int)xp[3],(int)yp[3],(int)xp[2],(int)yp[2],(int)xp[4],(int)xp[4]);
	//cvWaitKey(100);
	houghred=conv*atan2((xp[1]-xp[3]),(yp[3]-yp[1]));
	houghblue=conv*atan2((xp[4]-xp[2]),(yp[2]-yp[4]));
	//to change angle according to green patch(orientation)
	if(xp[3] > xp[1])
	{
	angles[1] += 180;
	houghred+=180;
	}
	
	if(xp[4] > xp[2])
	{
	angles[2] = 180 - angles[2];
	//angles[2] += 180;
	
	}
	if(xp[4] < xp[2])
	{
	angles[2] = 360 - angles[2];
	//angles[2] += 180;
	houghblue+=180;
	}
	
	if( (houghred<=10)&&(houghred>=0) || (houghred<=360)&&(houghred>=350) || (houghred>=170)&&(houghred<=190))   //error correction due to hough
	angles[1]=houghred;
	
	if( (houghblue<=10)&&(houghblue>=0) || (houghblue<=360)&&(houghblue>=350) || (houghblue>=170)&&(houghblue<=190))     //error correction due to hough
	angles[2]=houghblue;
	//getch();
	
	//printf("\nA1 - %f  A2 - %f",angles[1],360 - angles[2]);
	//printf("\nR-%d/%d Y-%d/%d",(int)xp[2],(int)yp[2],(int)xp[4],(int)yp[4]);
	decide();
	goals();
	defence_working();
	//strikernew();
	striker();
	//defend();
	
	circle(frame,Point((int)xp[7],(int)yp[7]),5,Scalar( 0, 0, 0 ),-1,8);	
	imshow("Live Video", frame); //Display camera frame in Live Video window
        
	//cvWaitKey(100);
	
	
Ballmove:       //come here is ball is moving 
	ID = 0;
//--------------------------------ESC KEY code as exit code-----------------------------------
    
	//	printf("xg= %d=",xp[4]);    //to check alignment spots
	//printf("xg= %d=",yp[4]);  //to check alignment spots
	c = cvWaitKey(10); //Wait 10ms for user exit
    if((char)c==27 ) //If ESC then break while loop
	{ writer.release();;break;} 
    }
//-----------------------------------------------Exit------------------------------------------
    return 0;
}


//-------------------------------------Print Intro Function-------------------------------------
void Printintro()
{
printf( "%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c\n",196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196 );
printf( " %c V.J.T.I \n",179 );
printf( " %c Bioloid Control Manager\n",179 );
printf( "%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c\n",196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196 );
Sleep(50);
printf( "  LOADING:\n" );
printf( "\r                                           0%c",37 );Sleep(1500);
printf( "\r  %c%c%c%c                                     10%c",219,219,219,219,37 );Sleep(3500);
printf( "\r  %c%c%c%c%c%c%c%c                                 20%c",219,219,219,219,219,219,219,219,37 );Sleep(50);
printf( "\r  %c%c%c%c%c%c%c%c%c%c%c%c                             30%c",219,219,219,219,219,219,219,219,219,219,219,219,37 );Sleep(1500);
printf( "\r  %c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c                         40%c",219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,37 );Sleep(5);
printf( "\r  %c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c                     50%c",219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,37 );Sleep(5);
printf( "\r  %c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c                 60%c",219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,37 );Sleep(5);
printf( "\r  %c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c             70%c",219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,37 );Sleep(50);
printf( "\r  %c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c         80%c",219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,37 );Sleep(1500);
printf( "\r  %c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c     90%c",219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,37 );Sleep(2500);
printf( "\r  %c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c 100%c\n",219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,37 );
}




void Dilation( int, void* )
{
  int dilation_type;
  if( dilation_elem == 0 ){ dilation_type = MORPH_RECT; }
  else if( dilation_elem == 1 ){ dilation_type = MORPH_CROSS; }
  else if( dilation_elem == 2) { dilation_type = MORPH_ELLIPSE; }

  Mat element = getStructuringElement( dilation_type,
                                       Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                       Point( dilation_size, dilation_size ) );
  /// Apply the dilation operation
  dilate( erosiondst, dilatedst, element );
  imshow( "Dilation Demo", dilatedst );
}


/*
#include "stdafx.h"          //the one i have just commented
#include <cv.h>
#include <highgui.h>
#include <iostream>
#include <conio.h>

IplImage* imgHSV =  0; 

// Global variables
int key = 0; // For mouse events
int Hmin = 60; // For HSV filter
int Smin = 211; // For HSV filter
int Vmin = 74; // For HSV filter
int Hmax = 92; // For HSV filter
int Smax = 255; // For HSV filter
int Vmax = 255; // For HSV filter

using namespace cv;
using namespace std;


/// Global variables
Mat src, erosion_dst, dilation_dst;

int erosion_elem = 0;
int erosion_size = 0;
int dilation_elem = 0;
int dilation_size = 0;
int const max_elem = 2;
int const max_kernel_size = 21;

IplImage* imgThresh = 0 ;


void Erosion( int, void* )
{
 int erosion_type;
 if( erosion_elem == 0 ){ erosion_type = MORPH_RECT; }
 else if( erosion_elem == 1 ){ erosion_type = MORPH_CROSS; }
 else if( erosion_elem == 2) { erosion_type = MORPH_ELLIPSE; }

 Mat element = getStructuringElement( erosion_type,
                                      Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                      Point( erosion_size, erosion_size ) );

 /// Apply the erosion operation
 erode(  dilation_dst, erosion_dst, element );
 imshow( "Erosion Demo", erosion_dst );
}
void Dilation( int, void* )
{
 int dilation_type;
 if( dilation_elem == 0 ){ dilation_type = MORPH_RECT; }
 else if( dilation_elem == 1 ){ dilation_type = MORPH_CROSS; }
 else if( dilation_elem == 2) { dilation_type = MORPH_ELLIPSE; }

 Mat element = getStructuringElement( dilation_type,
                                      Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                      Point( dilation_size, dilation_size ) );
 /// Apply the dilation operation
 dilate( src, dilation_dst, element );
 imshow( "Dilation Demo", dilation_dst );
}

IplImage* GetThresholdedImage()
{
IplImage* imgThreshed = cvCreateImage(cvGetSize(imgHSV), 8, 1); // Create a blank image
cvInRangeS(imgHSV, cvScalar(Hmin, Smin, Vmin), cvScalar(Hmax, Smax, Vmax), imgThreshed); // Filter image to allow only specific colour
return imgThreshed; // Return Black and White image
}

void on_trackbar( int, void* )
{

imgThresh = GetThresholdedImage(); // Call filter
cvNamedWindow("binary");
cvShowImage("binary", imgThresh);
}

int main()
{


//IplImage* img;
       cvNamedWindow("image");
cvNamedWindow("hsv");
IplImage* img = 0;
CvCapture* capture = cvCaptureFromCAM( 1 );
cvSetCaptureProperty(capture,CV_CAP_PROP_FRAME_WIDTH,320*3);
cvSetCaptureProperty(capture,CV_CAP_PROP_FRAME_HEIGHT,240*3);
cvSetCaptureProperty(capture, CV_CAP_PROP_FPS, 30);
if ( !capture ) 
{
fprintf( stderr, "ERROR: capture is NULL \n" );
getchar();
return -1;
}
img = cvQueryFrame( capture );
for (int countx = 0 ; countx < 20 ; countx++)
{img = cvQueryFrame( capture );imgHSV = cvCloneImage (img );}



 createTrackbar( "Hmax", "hsv", &Hmax, 255,on_trackbar);
 createTrackbar( "Hmin", "hsv", &Hmin, 255,on_trackbar);
     createTrackbar( "Smax", "hsv", &Smax, 255,on_trackbar);
 createTrackbar( "Smin", "hsv", &Smin, 255,on_trackbar);
 createTrackbar( "Vmax", "hsv", &Vmax, 255,on_trackbar);
 createTrackbar( "Vmin", "hsv", &Vmin, 255,on_trackbar);

 namedWindow( "Erosion Demo", CV_WINDOW_AUTOSIZE );
namedWindow( "Dilation Demo", CV_WINDOW_AUTOSIZE );
cvMoveWindow( "Dilation Demo", src.cols, 0 );

 

 /// Create Erosion Trackbar
 createTrackbar( "Element:\n 0: Rect \n 1: Cross \n 2: Ellipse", "Erosion Demo",
                 &erosion_elem, max_elem,
                 Erosion );

 createTrackbar( "Kernel size:\n 2n +1", "Erosion Demo",
                 &erosion_size, max_kernel_size,
                 Erosion );

 /// Create Dilation Trackbar
 createTrackbar( "Element:\n 0: Rect \n 1: Cross \n 2: Ellipse", "Dilation Demo",
                 &dilation_elem, max_elem,
                 Dilation );

 createTrackbar( "Kernel size:\n 2n +1", "Dilation Demo",
                 &dilation_size, max_kernel_size,
                 Dilation );

on_trackbar(0, 0 );
 Mat dst, cdst;
 vector<Vec2f> lines;

while(1)
{

img = cvQueryFrame( capture );
   cvCvtColor(img, imgHSV, CV_BGR2HSV);	
imgThresh = GetThresholdedImage();
src = imgThresh;
   //Canny(src, dst, 50, 200, 3);
   cvtColor(src, cdst, CV_GRAY2BGR);

   HoughLines(src, lines, 1, CV_PI/180, 65, 0, 0 );
printf("H done no. %d",(int)lines.size());
  int no=(int)lines.size();


       double x=0, y=0,a1=0,b1=0;
float theta1=0,rho1=0;
for( size_t i = 0; i < lines.size(); i++ )
   {
//printf("H done");
       float rho = lines[i][0], theta = lines[i][1];
rho1+=rho;

if ((theta*180/CV_PI) > 180)
{
theta = CV_PI*(360 - (theta*180/CV_PI))/180;
}
      theta1+=theta;
Point pt1, pt2;
       double a = cos(theta), b = sin(theta);
       double x0 = a*rho, y0 = b*rho;
      a1+=a;
  b1+=b;
x+=x0;
y+=y0;



}   

b1/=no;
a1/=no;
x/=no;
y/=no;
theta1/=no;


Point pt1, pt2;
       pt1.x=  cvRound(x + 1000*(-b1));
       pt1.y= cvRound(y + 1000*(a1));
       pt2.x= cvRound(x - 1000*(-b1));
       pt2.y= cvRound(y - 1000*(a1));

line( cdst, pt1, pt2, Scalar(0,0,255), 1, CV_AA);	
printf("\rangle = %f , x = %f , y = %f",theta1*180/CV_PI,x,y);
cvWaitKey(5);
imshow("detected lines", cdst);
cvShowImage( "image", img );
}
       //wait for key press
       cvWaitKey(0);


   cvDestroyAllWindows();

   cvReleaseCapture(&capture);

   return 0;
}
*/

/*
#include "stdafx.h"
#include <cv.h>
#include <highgui.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <math.h>
#include <iostream>
#include <windows.h>
#include <conio.h>

using namespace cv;
using namespace std;

//-----------------------------------Macros-----------------------------------------------
#define Width 960 //Width of camera frame
#define Height 720 //Height of camera frame
#define FPS 30 //FPS of camera frame
#define conv 180/CV_PI
#define offset 10    //to set offset position
#define Balloffset 25    //min compliance for considering ball to be stationary 
#define step   100  //1 step distance of bot
//-----------------------------------Global Varialbles-------------------------------------
int sendpos=0;  // id to be sent
int statusofbots[3]={0,0,0}; 
int dilation_elem = 0;
int dilation_size = 0;
int const max_elem = 2;
int const max_kernel_size = 21;
int ID = 0;      // ball,blue,red
double xp[8] = {0,0,0,0,0,360,360,0}, yp[8] = {0,0,0,0,0,0,960,0} ;   //co-ordinates of ball,   [ball,robot1(red),robot2(blue),robot1-green,robot2-green,goal1(red),goal2(blue),striker offset]
float goalangles[3]={0,0,0};
Mat dilatesrc,dilatedst; 
int ballpos=0;
float angles[3]={0.0,0.0,0.0};
int def=0,str=0;
float dist[3]={0,0,0};

//-----------------------------------Function Prototype-------------------------------------
void Printintro(); //Printing info
void Dilation( int, void* );

//-------------------Function to decide who is Striker & who is Defender---------------------
void decide()    // decide who is striker and defender
{
	if(yp[0]<480)
	{	
	str=1;
	def=2;
	}
	else 
	{
	str=2;
	def=1;
	}
}

//---------------------------Function for Defender-------------------------------------
int defend()
{
if(statusofbots[def]==0)
{
	if(goalangles[1] < 0)
	{	
	sendpos= int((360 + goalangles[1] - angles[def])/40.0);
	printf("press 'r + %d '", sendpos<5?sendpos:5);   //first angle correction if goal angles are negative(more than 180)
	return 0;
	}
	else if(int((360 + goalangles[1] - angles[def]))<10 || int((goalangles[1] - angles[def]))<10)
	{
	statusofbots[def]++;        //angle correction done
	
	return 0;
	}

	else

	{
	sendpos=	int((goalangles[1] - angles[def])/40.0);
	printf("press 'r + %d '", sendpos<5?sendpos:5);   // first angle correction if goal angles are right
	return 0;
	}

	
}
else if(statusofbots[0]==1)     //walk the distance to the goal 
{

	if(dist[1]>10)
	{
	sendpos=int(dist[def]/step);
	printf( "press 'd + %d'", sendpos<5?sendpos:5);   //send to walk straight
	return 0;
	}
	
	else statusofbots[def]++;

}

else if(statusofbots[0]==2)
{


	if(goalangles[0] < 0)
	{	
	 sendpos=int((360 + goalangles[0] - angles[def])/40.0);
	
	printf("press 'r + %d '", sendpos<5?sendpos:5);   //second angle correction if goal angles are negative(more than 180)
	return 0;
	}
	else if(int((360 + goalangles[0] - angles[def])/40.0)<10 || int((goalangles[0] - angles[def])/40.0)<10)
	{
	statusofbots[def]++;        //angle correction done
	return 0;
	}

	else

	{
	sendpos=int((goalangles[0] - angles[def])/40.0);
	printf("press 'r + %d '", sendpos<5?sendpos:5);   // second angle correction if goal angles are right
	return 0;
	}


}


else statusofbots[def]++;

}




int striker()
{

	if(statusofbots[str]==0)
{
	if(goalangles[2] < 0)
	{	
	printf("press 'r + %d '", int((360 + goalangles[2] - angles[str])/40.0));   //first angle correction if goal angles are negative(more than 180)
	return 0;
	}
	else if(int((360 + goalangles[2] - angles[str])/40.0)<5 || int((goalangles[2] - angles[str])/40.0)<5)   //compliance of 5
	{
	statusofbots[str]++;        //angle correction done
	return 0;
	}

	else

	{
	printf("press 'r + %d '", int((goalangles[2] - angles[str])/40.0));   // first angle correction if goal angles are right
	return 0;
	}

	
}
else if(statusofbots[str]==1)     //walk the distance to the goal 
{

	if(dist[str]>10)
	printf( "press 'd + %d'", int(dist[str]/step));   //send to walk straight

	else statusofbots[str]++;

}

else if(statusofbots[str]==2)
{


	if(goalangles[0] < 0)
	{	
	sendpos=int((360 + goalangles[0] - angles[str])/40.0);
	
	printf("press 'r + %d '", sendpos<5?sendpos:5);   //second angle correction if goal angles are negative(more than 180)
	return 0;
	}
	else if(int((360 + goalangles[0] - angles[str])/40.0)<10 || int((goalangles[0] - angles[str])/40.0)<10)
	{
	statusofbots[str]++;  //angle correction done
	return 0;
	}

	else

	{
	sendpos=int((goalangles[0] - angles[str])/40.0);
	printf("press 'r + %d '", sendpos<5?sendpos:5);   // second angle correction if goal angles are right
	return 0;
	}


}


else statusofbots[str]++;
	}




//------------------------------- function to get all the distances and angls from the frame-------------------------------------------------//
void goals()
{
	if (def == 1)
	{	
	goalangles[0] = conv*atan2((yp[5]-yp[0]),(xp[5]-xp[0]));  // red goal and ball 
	xp[7] = xp[0] + offset*cos(goalangles[0]/conv);
	yp[7] = yp[0] + offset*sin(goalangles[0]/conv);
	goalangles[1] = conv*atan2((yp[5]-yp[1]),(xp[5]-xp[1]));  //red goal and red bot(defender)
	goalangles[2] = conv*atan2((yp[7]-yp[2]),(xp[7]-xp[2]));	// offset & blue striker	
	dist[def] = sqrt(pow(yp[5]-yp[1],2)+pow(xp[5]-xp[1],2));   // goal and defender(red)
	dist[str] = sqrt(pow(yp[7]-yp[2],2)+pow(xp[7]-xp[2],2));   //balloffset and striker(blue)
	//printf("defence =2 and bluegoal to ball= %f", goalangles[0]);
	}
	else if (def==2)
	{
	goalangles[0] = conv*atan2((yp[6]-yp[0]),(xp[6]-xp[0])); //blue goal and ball
	xp[7] = xp[0] - offset*cos(goalangles[0]/conv);
	yp[7] = yp[0] - offset*sin(goalangles[0]/conv);
	goalangles[1] = conv*atan2((yp[6]-yp[2]),(xp[6]-xp[2])); //blue goal and blue bot
	goalangles[2] = conv*atan2((yp[7]-yp[1]),(xp[6]-xp[1])); //offset and red striker
	dist[def] = sqrt(pow(yp[6]-yp[2],2)+pow(xp[6]-xp[2],2)); // goal and defender(blue)  // i have set new index as distance cannot be directly used as 0 and 1.It can be usedas str and def for simplicity later on
	dist[str] = sqrt(pow(yp[7]-yp[1],2)+pow(xp[7]-xp[1],2)); //balloffset and striker(red)
	//printf("defence =2 and bluegoal to ball= %f", goalangles[0]);
	
	}
}


//-------------------------------------Main Function----------------------------------------
int main()
{
//-----------------------------------Print Intro--------------------------------------------
    //Printintro(); //Call print intro function

//-------------------------------------Variables--------------------------------------------
    int c = 0; //For exiting while loop
    Point pt1, pt2;
    double a, b;
    double x0, y0,xr,yr;	// xr,yr used for centre detection(temp variable)

	//double xp[3] = {0,0,0}, yp[3] = {0,0,0} ;
    double pixels = 0;

	int Hmax[5] = {92,102,187,56,56}; //Hue max for 
    int Hmin[5] = {60,83,129,23,23}; //Hue mmin for ball, Robot1 (blue), Robot2 (red)
    int Smax[5] = {255,255,252,255,255}; //Sat max for ball, Robot1 (blue), Robot2 (red)
    int Smin[5] = {211,202,74,94,94}; //Sat min for ball, Robot1 (blue), Robot2 (red)
    int Vmax[5] = {255,255,255,255,255}; //Val max for ball, Robot1 (blue), Robot2 (red)
    int Vmin[5] = {75,173,177,152,152}; //Val min for ball, Robot1 (blue), Robot2 (red)
	int ballp[2] = {0,0};
    int no = 0;
    double x = 0, y = 0, a1 = 0, b1 = 0;
    float theta1 = 0;    
	//double answr = (180/CV_PI)*atan2(-1.0,-1.0);
	//printf("%f",answr);
	int startx,starty,xlimit,ylimit;
	
	//-------------------------------------Camera-----------------------------------------------  

    VideoCapture capture(0); //Open the default camera
    capture.set(CV_CAP_PROP_FRAME_WIDTH, Width); //Width
    capture.set(CV_CAP_PROP_FRAME_HEIGHT, Height); //Height
    capture.set(CV_CAP_PROP_FPS, FPS);  //FPS
    if(!capture.isOpened())  //Check if we succeeded
    {
        printf( "ERROR: capture is NULL... PLZ Check Camera...\n" );
        getchar();
        return -1;
    }
    printf("  Camera Properties\n  Width = %d; Height = %d; FPS = %d\n",Width,Height,FPS);

//------------------------------------Frame--------------------------------------------------    
    Mat frame; //Take 1st frame
    capture >> frame;
    if ( !frame.data ) 
    {
        printf( "ERROR: frame is null... PLZ Check frame is not recieved from camera\n" );
        getchar();
        return -1;
    }
    Mat imgHSV = frame.clone(); //make HSV image with same parameters as frame
    vector<Vec2f> lines;
    Mat imgThresh;

//-------------------------------Create Windows for display----------------------------------
    namedWindow("Live Video"); //Create window by name Live Video for seeing output of camera
     //namedWindow("HSV Video"); //Create window by name Live Video for seeing output of camera
	//namedWindow("Binary Video");
	namedWindow( "Dilation Demo", CV_WINDOW_AUTOSIZE );


	 while(1)
    { capture >> frame;
	 printf("hi hi");imshow("Live Video", frame);c = cvWaitKey(100); //Wait 10ms for user exit
    if((char)c==27 ) //If ESC then break while loop
        break; 
    }

//--------------------------------------------------------------------------------------------
	//Create Dilation Trackbar
	createTrackbar( "Element:\n 0: Rect \n 1: Cross \n 2: Ellipse", "Dilation Demo", &dilation_elem, max_elem, Dilation );
	createTrackbar( "Kernel size:\n 2n +1", "Dilation Demo", &dilation_size, max_kernel_size, Dilation );

//--------------------------------While starts------------------------------------------------
     while(1)
    {
    
        printf("Inside main While ...   ");
        capture >> frame;	//get next frame from camera buffer
	 if ( !frame.data ) 
	 {
        printf( "all frames done" );
        getchar();
        break;
	//return -1;
	 }
        GaussianBlur( frame, frame, Size(9, 9), 2, 2 );	//smooth the original image using Gaussian Filter
        cvtColor(frame, imgHSV, CV_BGR2HSV);	//Change the color format from BGR to HSV
        
        while(ID < 5)
        {  
	xr=yr=0;	
	
	                                                     //make the counters zero  
	startx=startx=1;	    // start pixels for counters(centroid algorithm)
	xlimit = 960;
	ylimit = 720;
	printf("\nID = %d   ",ID);
            inRange(imgHSV, Scalar(Hmin[ID],Smin[ID],Vmin[ID]), Scalar(Hmax[ID],Smax[ID],Vmax[ID]), imgThresh);
                if(ID>2)
	{
	startx=xp[ID-2]-100;
	starty=yp[ID-2]-100;
	xlimit =startx+ 200;    // 100*100 pixels scanned
	ylimit =starty+ 200;
	}
            for(int i = (startx<1?1:startx) ;  i < ((xlimit>=960)?960:xlimit);i++)          //960*720
            {
                for(int j = (starty<1?1:starty); j <( (ylimit>=720)?720:ylimit);j++)
                {
                    if((int)imgThresh.at<uchar>(j,i) == 255)
                    {  //  printf("i= %d",i);
	 //printf("j= %d",j);
                        pixels++;
	yr += i;
                        xr += j;
                    }
                }
            }

	yp[ID] =yr/ pixels;    //send average into array
	xp[ID] =xr/ pixels;
	circle(frame,Point((int)xp[ID],(int)yp[ID]),10,Scalar( 255, 0, 0 ),-1,8);
	//xr = yr = 0;
            pixels = 0;

	imshow("Live Video", frame); //Display camera frame in Live Video window
	imshow("Binary Video", imgThresh); //Display binary image in Binary Video window

	if (ID == 0) //For ball position
	{
	if(abs(ballp[0] - xp[0]) > Balloffset || abs(ballp[1] - yp[0]) > Balloffset) //If ball is moving
	{
	printf("ball offset x %f",abs(ballp[0] - xp[0]) );
	printf("ball offset y %f",abs(ballp[1] - yp[0]) );
	
	ballp[0] = xp[0];
	ballp[1] = yp[0];
	ID=0; 
	goto Ballmove;
	
	}	
	}
	//cvWaitKey(100);

            if(ID==1 || ID==2)
            {
	dilatesrc = imgThresh;
	Dilation( 0, 0 );
	imgThresh = dilatedst;
	HoughLines(imgThresh, lines, 1, CV_PI/180, 70, 0, 0 );
                printf("H no. %d   ",(int)lines.size());
                no = (int)lines.size();
        
                for( size_t i = 0; ((i < lines.size())&&((int)no>0)); i++ )
                {
                    float rho = lines[i][0], theta = lines[i][1];
                    if ((theta*180/CV_PI) > 180)
                    {
                        theta = CV_PI*(360 - (theta*180/CV_PI))/180;
                    }    
        
                    theta1 += theta;
                    a = cos(theta), b = sin(theta);
                    x0 = a*rho, y0 = b*rho;
                    a1 += a;
                    b1 += b;
                    x += x0;
                    y += y0;
                }  
        if(no==0)   //if we get 0 lines
	no=1;
                b1 /= no;
                a1 /= no;
                x /= no;
                y /= no;
                theta1 /= no;
                pt1.x = cvRound(x + 1000*(-b1));
                pt1.y = cvRound(y + 1000*(a1));
                pt2.x = cvRound(x - 1000*(-b1));
                pt2.y = cvRound(y - 1000*(a1));
                line( frame, pt1, pt2, Scalar(0,0,255), 1, CV_AA);    
              angles[ID]= theta1*180/CV_PI;
	printf("angle = %f , x = %f , y = %f   ",theta1*180/CV_PI,x,y);
                cvWaitKey(5);
                x = y = a1 = b1 = 0;
                theta1 = 0;
            }
        //imshow("Live Video", frame); //Display camera frame in Live Video window
        //imshow("Binary Video", imgThresh);    
        ID++;
        }
	//to change angle according to green patch(orientation)
	if(xp[3]>xp[1] || yp[3]>yp[1])
	{
	angles[1] += 180;
	}
	if(xp[4]<xp[2] || yp[4]<yp[2])
	{
	angles[2] += 180;
	}
	printf("def %d", def);
	
	decide();
	goals();
	defend();
	striker();
Ballmove:       //come here is ball is moving 
	ID = 0;
//--------------------------------ESC KEY code as exit code-----------------------------------
    
	//	printf("xg= %d=",xp[4]);    //to check alignment spots
	//printf("xg= %d=",yp[4]);  //to check alignment spots
	c = cvWaitKey(1); //Wait 10ms for user exit
    if((char)c==27 ) //If ESC then break while loop
        break; 
    }
//-----------------------------------------------Exit------------------------------------------
    return 0;
}


//-------------------------------------Print Intro Function-------------------------------------
void Printintro()
{
printf( "%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c\n",196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196 );
printf( " %c V.J.T.I \n",179 );
printf( " %c Bioloid Control Manager\n",179 );
printf( "%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c\n",196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196,196 );
Sleep(50);
printf( "  LOADING:\n" );
printf( "\r                                           0%c",37 );Sleep(1500);
printf( "\r  %c%c%c%c                                     10%c",219,219,219,219,37 );Sleep(3500);
printf( "\r  %c%c%c%c%c%c%c%c                                 20%c",219,219,219,219,219,219,219,219,37 );Sleep(50);
printf( "\r  %c%c%c%c%c%c%c%c%c%c%c%c                             30%c",219,219,219,219,219,219,219,219,219,219,219,219,37 );Sleep(1500);
printf( "\r  %c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c                         40%c",219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,37 );Sleep(5);
printf( "\r  %c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c                     50%c",219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,37 );Sleep(5);
printf( "\r  %c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c                 60%c",219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,37 );Sleep(5);
printf( "\r  %c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c             70%c",219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,37 );Sleep(50);
printf( "\r  %c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c         80%c",219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,37 );Sleep(1500);
printf( "\r  %c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c     90%c",219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,37 );Sleep(2500);
printf( "\r  %c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c 100%c\n",219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,219,37 );
}




void Dilation( int, void* )
{
  int dilation_type;
  if( dilation_elem == 0 ){ dilation_type = MORPH_RECT; }
  else if( dilation_elem == 1 ){ dilation_type = MORPH_CROSS; }
  else if( dilation_elem == 2) { dilation_type = MORPH_ELLIPSE; }

  Mat element = getStructuringElement( dilation_type,
                                       Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                       Point( dilation_size, dilation_size ) );
  /// Apply the dilation operation
  dilate( dilatesrc, dilatedst, element );
  imshow( "Dilation Demo", dilatedst );
}

*/
/*
#include "Stdafx.h"
#include <windows.h>
#include <stdio.h>
#include <conio.h>
#include <time.h>
#include <iostream>
#include <fstream>
#include <Mmsystem.h>
#include "cv.h"
#include "highgui.h"
#include <stdlib.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"


#include <string.h>
//tess inits
using namespace cv;

void Threshold_Demo( int, void* );
void read_realtime();

int threshold_value = 160;
IplImage *image; 
IplImage *bitImage;
IplImage *diffImage;


//tab inits
	char input = '\0';
	char output = 's';
	char outputbuf = 'w';
	DWORD        bytes_read = 0;    // Number of bytes read from port
    DWORD        bytes_write = 0;    // Number of bytes written to the port
    HANDLE       comport = NULL;  // Handle COM port
	int   bStatus;
	int   WStatus;
    DCB          comSettings;          // Contains various port settings
    COMMTIMEOUTS CommTimeouts;

void msg();
void songs();
void songs1();
void songs2();
void play(int);
void contacts();
void contacts1();
void contacts2();
void datetime();
void readcam();
void writing();
void read();
void read1();
void read2();
void getinput();
void getinput_write();

	    

using namespace std;

int main()
{
	if ((comport = CreateFile(L"\\\\.\\COM3", GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL)) == INVALID_HANDLE_VALUE)
    
	CommTimeouts.ReadIntervalTimeout         = 0; 
    CommTimeouts.ReadTotalTimeoutMultiplier  = 10; 
    CommTimeouts.ReadTotalTimeoutConstant    = 1000;
    CommTimeouts.WriteTotalTimeoutMultiplier = 10;
    CommTimeouts.WriteTotalTimeoutConstant   = 100;
    bStatus = SetCommTimeouts(comport,&CommTimeouts);
	
	GetCommState(comport, &comSettings);
    comSettings.BaudRate = 9600;
    comSettings.StopBits = ONESTOPBIT;
    comSettings.ByteSize = 8;
    comSettings.Parity   = NOPARITY;
    comSettings.fParity  = FALSE;
    bStatus = SetCommState(comport, &comSettings);
	

	printf("\n \n---BLIND TAB--- \n \n");
	while(1)
	{	
	getinput();
	if(input == '1')
	msg();   
	}
	
	CloseHandle(comport);

	getch();
	return 0;
}

void getinput() //function for sync between USART and USB works flawless
{
	input='\0';
	while(input !='1' && input !='u' && input !='d' && input !='e' && input !='b' && input != 'w')
	{
	WStatus = WriteFile(comport, &output, 1, &bytes_write, NULL);
	//printf("wstatus %d \n",WStatus);
	//while(input !='1' && input !='u' && input !='d' && input !='e' && input !='b')
	//{	
	bStatus = ReadFile(comport, &input, 1, &bytes_read, NULL);
	//printf("%d\n", bStatus);
	//printf("input %c\n", input);
	}
	if(input == 'w')
	writing();
}

void getinput_write() //function for sync between USART and USB works flawless
{
	input='\0';
	while(input == '\0')
	{
	WStatus = WriteFile(comport, &outputbuf, 1, &bytes_write, NULL);
	//printf("wstatus %d \n",WStatus);
	//while(input !='1' && input !='u' && input !='d' && input !='e' && input !='b')
	//{	
	bStatus = ReadFile(comport, &input, 1, &bytes_read, NULL);

	if(input=='_')
	break;
	//printf("%d\n", bStatus);
	//printf("input %c\n", input);
	}
}

void msg()//actual name is notes bored to change it
{
	system ("cls");//clear screen
	printf("---BLIND TAB--- \n \n[ NOTES ]\n  SONGS\n  CONTACTS\n  DATE & TIME");//print GUI, [option] indicates cursor
	system ("espeak -a150 -p70 -s100 notes");//read notes
	getinput();//get i/p
	switch(input)
	{
	case 'u': {datetime();break;}//up
	case 'd': {songs();break;}//dwn
	case 'e': {readcam();break;}//enter
	case 'b': {msg();break;}//call self
	}
}

void readcam()
{
	
	system ("cls");
	printf("---BLIND TAB--- \n \n  NOTES  \n    [ READ CAMERA ]\n      READ\n  SONGS\n  CONTACTS\n  DATE & TIME");
	system ("espeak -a150 -p70 -s100 read_camera");
	getinput();
	switch(input)
	{
	case 'u': {read();break;}
	case 'd': {read();break;}
	case 'e': {read_realtime();break;}// need to call writing function here
	case 'b': {msg();break;}
	}
}

void writing()
{
	printf("writng");
	system("cls");
	printf("---BLIND TAB---\n|______________________________|\n TEXT TYPED IS DISPLAYED BELOW\n|______________________________| \n \n");
	ofstream myfile;
	myfile.open ("Write.txt");
	system ("espeak -a150 -p70 -s100 writing_started");	
	getinput_write();
	while(1)
	{
	getinput_write();
	if(input=='_')
	break;
	printf("%c", input);
	myfile << input;
	}
	myfile.close();
	system ("espeak -a150 -p70 -s100 -fWrite.txt");




	printf("\nPress Back To Exit");
	/*	dirs = opendir(path);
	if (dirs == NULL) 
	{
	printf("\n");	
	perror("opendir");
	}
	
	 while ((entry = readdir(dirs)))
	{	
	if(entry->d_name == "contacts.txt")
	{
	strcpy(src,path);
                strcat(src,"/");
                strcat(src, entry->d_name);
	fs = fopen(src, "w");


	strcpy(dest,path);
                strcat(dest,"/");
                strcat(dest, "Write.txt");
	fd = fopen(dest, "w");
	

	if(getc(fd) == 'E')
	{  while((ch=fgetc(fd)) != EOF)
                        {

                                        fputc(ch, fs);          
                        }
	
	}
	
	fclose(fs);
	fclose(fd);

	}
	 }	
	
	closedir(dirs);
	getinput();
	switch(input)
	{
	case 'u': {read();break;}
	case 'd': {read();break;}
	case 'e': {writing();break;}// need to call writing function here
	case 'b': {msg();break;}
	}

}
void read()
{
	
	system ("cls");
	printf("---BLIND TAB--- \n \n  NOTES  \n      READ CAMERA\n    [ READ ]\n  SONGS\n  CONTACTS\n  DATE & TIME");
	system ("espeak -a150 -p70 -s100 read ");
	getinput();
	switch(input)
	{
	case 'u': {readcam();break;}
	case 'd': {readcam();break;}
	case 'e': {read1();break;}
	case 'b': {msg();break;}
	}
}

void read1()
{
	
	system ("cls");
	printf("---BLIND TAB--- \n \n  NOTES  \n      READ CAMERA\n      READ  \n        [ BIRTHDAY ]\n          APPOINTMENT\n  SONGS\n  CONTACTS\n  DATE & TIME");
	system ("espeak -a150 -p70 -s100 birthday");
	getinput();
	switch(input)
	{
	case 'u': {read2();break;}
	case 'd': {read2();break;}
	case 'e': {system ("espeak -a150 -p70 -s100 -fbirthdaytext.txt");read();break;}//txt file in directory
	case 'b': {msg();break;}
	}
}

void read2()
{
	
	system ("cls");
	printf("---BLIND TAB--- \n \n  NOTES  \n      READ CAMERA\n      READ  \n          BIRTHDAY\n        [ APPOINTMENT ]\n  SONGS\n  CONTACTS\n  DATE & TIME");
	system ("espeak -a150 -p70 -s100 appointment");
	getinput();
	switch(input)
	{
	case 'u': {read1();break;}
	case 'd': {read1();break;}
	case 'e': {system ("espeak -a150 -p70 -s100 -fappointmenttext.txt");read();break;}
	case 'b': {printf("back pressed \n \n");read();break;}
	}
}

void songs()
{
	system ("cls");
	printf("---BLIND TAB--- \n \n  NOTES  \n[ SONGS ]\n  CONTACTS\n  DATE & TIME");
	system ("espeak -a150 -p70 -s100 songs");
	getinput();
	switch(input)
	{
	case 'u': {msg();break;}
	case 'd': {contacts();break;}
	case 'e': {songs1();break;}
	case 'b': {songs();break;}
	}
}

void songs1()
{
	
	system ("cls");
	printf("---BLIND TAB--- \n \n  NOTES\n  SONGS\n    [ CHALLA ]\n      NUMB\n  CONTACTS\n  DATE & TIME");
	system ("espeak -a150 -p70 -s100 challa");
	getinput();
	switch(input)
	{
	case 'u': {songs2();break;}
	case 'd': {songs2();break;}
	case 'e': {play(1);break;}
	case 'b': {songs();break;}
	}
}

void songs2()
{
	system ("cls");
	printf("---BLIND TAB--- \n \n  NOTES\n  SONGS\n      CHALLA  \n    [ NUMB ]\n  CONTACTS\n  DATE & TIME");
	system ("espeak -a150 -p70 -s100 numb");
	getinput();
	switch(input)
	{
	case 'u': {songs1();break;}
	case 'd': {songs1();break;}
	case 'e': {play(2);break;}
	case 'b': {songs();break;}
	}
}

void play(int a)
{
	//COMMAND TO PLAY SONG AS SONG NAME IS STORED IN ARREY A DECIDES WHICH SONG TO PLAY
	system ("cls");
	if(a==1)
	{printf("---BLIND TAB--- \n \n  NOTES\n  SONGS\n      CHALLA >>>\n      NUMB\n  CONTACTS\n  DATE & TIME");PlaySound(TEXT("jthj.wav"), NULL, SND_FILENAME | SND_ASYNC);}
	if(a==2)
	{printf("---BLIND TAB--- \n \n  NOTES\n  SONGS\n      CHALLA\n      NUMB >>>\n  CONTACTS\n  DATE & TIME");PlaySound(TEXT("numb.wav"), NULL, SND_FILENAME | SND_ASYNC);}	
	getinput();
	switch(input)
	{
	case 'u': {play(2);break;}
	case 'd': {play(2);break;}
	case 'e': {PlaySound(NULL, 0, 0);songs1();break;}
	case 'b': {PlaySound(NULL, 0, 0);songs1();break;}
	}
}

void contacts()
{
	system ("cls");
	printf("---BLIND TAB--- \n \n  NOTES  \n  SONGS  \n[ CONTACTS ]\n  DATE & TIME\n \n");
	system ("espeak -a150 -p70 -s100 contacts");
	getinput();
	switch(input)
	{
	case 'u': {songs();break;}
	case 'd': {datetime();break;}
	case 'e': {contacts1();break;}
	case 'b': {contacts();break;}
	}
}

void contacts1()
{
	system ("cls");
	printf("---BLIND TAB--- \n \n  NOTES\n  SONGS\n  CONTACTS\n    [ RAJ ]\n      DHARMENDRA\n  DATE & TIME");
	system ("espeak -a150 -p70 -s100 raj");
	getinput();
	switch(input)
	{
	case 'u': {contacts2();break;}
	case 'd': {contacts2();break;}
	case 'e': {system ("espeak -a150 -p70 -s100 -frajdetails.txt");contacts();break;}
	case 'b': {msg();break;}
	}
}

void contacts2()
{
	system ("cls");
	printf("---BLIND TAB--- \n \n  NOTES\n  SONGS\n  CONTACTS\n      RAJ\n    [ DHARMENDRAW ]\n  DATE & TIME");
	system ("espeak -a150 -p70 -s100 dharmendra");
	getinput();
	switch(input)
	{
	case 'u': {contacts1();break;}
	case 'd': {contacts1();break;}
	case 'e': {system ("espeak -a150 -p70 -s100 -fdharmendradetails.txt");contacts();break;}
	case 'b': {msg();break;}
	}
}

void datetime()
{
	system ("cls");
	printf("---BLIND TAB--- \n \n  NOTES  \n  SONGS  \n  CONTACTS  \n[ DATE & TIME ]\n \n");
	system ("espeak -a150 -p70 -s100 date_and_time");
	getinput();
	switch(input)
	{
	case 'u': {contacts();break;}
	case 'd': {msg();break;}
	case 'e': {time_t t;
	time(&t);
	printf("\n\n%.24s.\n", ctime(&t));	
	ofstream myfile;
	myfile.open ("datetime.txt");
	myfile << ctime(&t);
	myfile.close();
	system ("espeak -a150 -p70 -s100 -fdatetime.txt ");msg();break;}
	case 'b': {msg();break;}
	}
}


void read_realtime()
{
	IplImage* frame = NULL;IplImage* equalizedImg = NULL;
	CvCapture *capture = cvCreateCameraCapture( 0 );
	namedWindow( "Threshold Demo", CV_WINDOW_AUTOSIZE );
	frame = cvRetrieveFrame(capture);
	cvNamedWindow( "Undistort" );
	equalizedImg = cvCreateImage(cvGetSize(frame), 8, 1);
	//cvWaitKey(3000);
	
	input = '\0';
	while(input !='b')
	{
	frame = cvRetrieveFrame(capture);
	
	cvWaitKey(1);
	cvShowImage("Undistort", frame);	
	if(input == 'e')	
	{
	image=cvCreateImage(cvSize(frame->width,frame->height),8,1);
	bitImage=cvCreateImage(cvSize(frame->width,frame->height),8,1);
	
	cvCvtColor(frame, image,CV_BGR2GRAY);
	createTrackbar( "Thres_Val", "Threshold Demo", &threshold_value, 255, Threshold_Demo );
	
	cvShowImage("Threshold Demo", image);
	Threshold_Demo( 0, 0 );
	
	cvShowImage("Threshold Demo", bitImage);
	cvWaitKey(0);
	cvSaveImage ("tesseract.tif", bitImage);
	system ("tesseract tesseract.tif out");
	system ("espeak -a150 -p70 -s100 -fout.txt ");
	
	
	
	system ("espeak -a150 -p70 -s100 Do_you_want_to_read_more_if_yes_press_enter_else_press_back");

	}
	input = '\0';
	WStatus = WriteFile(comport, &output, 1, &bytes_write, NULL);
	
	bStatus = ReadFile(comport, &input, 1, &bytes_read, NULL);
	

	}
	cvReleaseImage(&image);
	cvReleaseImage(&bitImage);
	cvReleaseImage(&frame);
	cvDestroyWindow("Undistort");
	cvDestroyWindow("Threshold Demo");

	msg();
}


void Threshold_Demo( int, void* )
{
	cvThreshold(image,bitImage,threshold_value,255,CV_THRESH_BINARY);
	cvShowImage("Threshold Demo", bitImage);
}

*/  //
////STEREO TESTED WID 4mp iBALL WORKING!!!__________________________________________________________________________________
////STEREO TESTED WID 4mp iBALL WORKING!!!__________________________________________________________________________________
//#include "stdafx.h"
//#include "cv.h"
//#include "cxmisc.h"
//#include "highgui.h"
//#include "cvaux.h"
//#include <vector>
//#include <string>
//#include <algorithm>
//#include <stdio.h>
//#include <conio.h>
//#include <ctype.h>
//#include <tchar.h>
//#include <windows.h>
//
//#define WIDTH 640
//#define HEIGHT 480
//
//using namespace cv;
//using namespace std;
//
//int main()
//{
//	//---------Initial--------
//	int  nx=9, ny=6, frame = 0, n_boards =40, g = 35, h = 3, N; //variables 4 caliberation	
//	int count1 = 0,count2 = 0, result1=0, result2=0; //variables 4 results
//    int showUndistorted = 1, successes1 = 0,successes2 = 0 ;
//   	const int maxScale = 1;
//	const float squareSize = 1.f;	//Set this to your actual square size
//	CvSize imageSize = {WIDTH,HEIGHT};
//	CvSize board_sz = cvSize( nx,ny );
//
//	CvCapture *capture1= NULL, *capture2= NULL;
//	int i, j, n = nx*ny, N1 = 0, N2 = 0;
//
//	/* 	vector<CvPoint2D32f> points[2];
//	vector<int> npoints;
//	vector<CvPoint3D32f> objectPoints;
//	vector<CvPoint2D32f> temp1(n); 
//	vector<CvPoint2D32f> temp2(n);
//    
//    double M1[3][3], M2[3][3], D1[5], D2[5];
//    double R[3][3], T[3], E[3][3], F[3][3];
//	double Q[4][4];
//	CvMat _Q = cvMat(4,4, CV_64F, Q);
//    CvMat _M1 = cvMat(3, 3, CV_64F, M1 );
//    CvMat _M2 = cvMat(3, 3, CV_64F, M2 );
//    CvMat _D1 = cvMat(1, 5, CV_64F, D1 );
//    CvMat _D2 = cvMat(1, 5, CV_64F, D2 );
//    CvMat _R = cvMat(3, 3, CV_64F, R );
//    CvMat _T = cvMat(3, 1, CV_64F, T );
//    CvMat _E = cvMat(3, 3, CV_64F, E );
//    CvMat _F = cvMat(3, 3, CV_64F, F );
//	
//*/
//
//
////	//---------Starting WebCam----------	
//	capture1= cvCaptureFromCAM(1);
//	capture2= cvCaptureFromCAM(2);
//
//
///*
//	//assure capture size is correct...
//	int res=cvSetCaptureProperty(capture1,CV_CAP_PROP_FRAME_WIDTH,WIDTH);
//	printf("%d",res);
//	res=cvSetCaptureProperty(capture1,CV_CAP_PROP_FRAME_HEIGHT,HEIGHT);
//	printf("%d",res);
//	res=cvSetCaptureProperty(capture2,CV_CAP_PROP_FRAME_WIDTH,WIDTH);
//	printf("%d",res);
//	res=cvSetCaptureProperty(capture2,CV_CAP_PROP_FRAME_HEIGHT,HEIGHT);
//	printf("%d",res); fflush(stdout); 
//*/	
//
//	cvWaitKey(50);
//	IplImage *frame1 = cvQueryFrame( capture1 );
//	IplImage* gray_fr1 = cvCreateImage( cvGetSize(frame1), 8, 1 );
//	IplImage *frame2 = cvQueryFrame( capture2 );
//	IplImage* gray_fr2 = cvCreateImage( cvGetSize(frame1), 8, 1 );
//	cvWaitKey(50);
//	//IplImage* frame3 = cvCreateImage(cvSize(480,640), IPL_DEPTH_8U, 3);
//	//IplImage* frame4 = cvCreateImage(cvSize(480,640), IPL_DEPTH_8U, 3); 
//
//	
//
//	imageSize = cvGetSize(frame1);
//	
///*	//Show Window	
//	cvNamedWindow( "camera2", 0 );
//	cvNamedWindow( "camera1", 0 );
//	cvNamedWindow("corners camera1",0);
//	cvNamedWindow("corners camera2",0);	
//	while((successes1<n_boards)||(successes2<n_boards))	
//	{
//	//--------Find chessboard corner--------------------------------------------------	
//	for(;;) 
//	{ 
//	frame1 = cvRetrieveFrame(capture1, 0); 
//	frame2 = cvRetrieveFrame(capture2, 0); 
//	cvShowImage("camera1", frame1 );
//	cvShowImage("camera2", frame2 );
//
//	if( cvWaitKey(1) >= 0 ) 
//	break; 
//	} 
////	if((frame++ % 20) == 0)	
//	{
////----------------CAM1-------------------------------------------------------------------------------------------------------
//	result1 = cvFindChessboardCorners( frame1, board_sz,&temp1[0], &count1,CV_CALIB_CB_ADAPTIVE_THRESH|CV_CALIB_CB_FILTER_QUADS);
//	cvCvtColor( frame1, gray_fr1, CV_BGR2GRAY );
//	
//
////----------------CAM2--------------------------------------------------------------------------------------------------------
//	result2 = cvFindChessboardCorners( frame2, board_sz,&temp2[0], &count2,CV_CALIB_CB_ADAPTIVE_THRESH|CV_CALIB_CB_FILTER_QUADS);
//	cvCvtColor( frame2, gray_fr2, CV_BGR2GRAY );
//	
//	if(count1==n&&count2==n&&result1&&result2)
//	{
//	cvFindCornerSubPix( gray_fr1, &temp1[0], count1,cvSize(11, 11), cvSize(-1,-1),cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS,30, 0.01) );
//	cvDrawChessboardCorners( frame1, board_sz, &temp1[0], count1, result1 );
//	cvShowImage( "corners camera1", frame1 );
//	N1 = points[0].size();
//	points[0].resize(N1 + n, cvPoint2D32f(0,0));
//	copy( temp1.begin(), temp1.end(), points[0].begin() + N1 );
//	++successes1;
//	
//	cvFindCornerSubPix( gray_fr2, &temp2[0], count2,cvSize(11, 11), cvSize(-1,-1),cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS,30, 0.01) );
//	cvDrawChessboardCorners( frame2, board_sz, &temp2[0], count2, result2 );
//	cvShowImage( "corners camera2", frame2 );
//	N2 = points[1].size();
//	points[1].resize(N2 + n, cvPoint2D32f(0,0));
//	copy( temp2.begin(), temp2.end(), points[1].begin() + N2 );
//	++successes2;
//
//	putchar('$');
//	}
//	
//	else
//	{	cvShowImage( "corners camera2", gray_fr2 );	
//	cvShowImage( "corners camera1", gray_fr1 );	
//	}
//	
//	//frame1 = cvQueryFrame( capture1 );
//	//cvShowImage("camera1", frame1);
//	//frame2 = cvQueryFrame( capture2 );
//	//cvShowImage("camera2", frame2);
//	
//	if(cvWaitKey(15)==27) break;
//	}
//	}
//	cvReleaseCapture( &capture1 ); 
//	cvReleaseCapture( &capture2 );
//	cvDestroyWindow("camera1");
//	cvDestroyWindow("camera2");
//	cvDestroyWindow("corners camera1");
//	cvDestroyWindow("corners camera2");	
//	printf("\n");
//	
//	//--------------Calibaration-------------------
//	N = n_boards*n;
//	objectPoints.resize(N);
//	for( i = 0; i < ny; i++ )
//	for(j = 0; j < nx; j++ )   objectPoints[i*nx + j] = cvPoint3D32f(i*squareSize, j*squareSize, 0);
//	for( i = 1; i < n_boards; i++ ) copy( objectPoints.begin(), objectPoints.begin() + n, objectPoints.begin() + i*n );
//	npoints.resize(n_boards,n);
//	
//	CvMat _objectPoints = cvMat(1, N, CV_32FC3, &objectPoints[0] );
//	CvMat _imagePoints1 = cvMat(1, N, CV_32FC2, &points[0][0] );
//	CvMat _imagePoints2 = cvMat(1, N, CV_32FC2, &points[1][0] );
//	CvMat _npoints = cvMat(1, npoints.size(), CV_32S, &npoints[0] );
//	cvSetIdentity(&_M1);
//	cvSetIdentity(&_M2);
//	cvZero(&_D1);
//	cvZero(&_D2);
//	
//	printf("Running stereo calibration ... \n");
//	fflush(stdout);
//	cvStereoCalibrate( &_objectPoints, &_imagePoints1, &_imagePoints2, &_npoints,&_M1, &_D1, &_M2, &_D2,imageSize, &_R, &_T, &_E, &_F,
//	cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5),
//        CV_CALIB_FIX_ASPECT_RATIO+CV_CALIB_ZERO_TANGENT_DIST + CV_CALIB_SAME_FOCAL_LENGTH );
//	printf("done calibration\n");
//	//-------------Undistort------------------------------------------
//	cvUndistortPoints( &_imagePoints1, &_imagePoints1,&_M1, &_D1, 0, &_M1 );
//	cvUndistortPoints( &_imagePoints2, &_imagePoints2,&_M2, &_D2, 0, &_M2 );
////	//--------Using bouguet algorithm-------------------
//
//*/
//
//	//CvMat* mx1 = cvCreateMat( imageSize.height,imageSize.width, CV_32F );
//  //      CvMat* my1 = cvCreateMat( imageSize.height,imageSize.width, CV_32F );
//  //      CvMat* mx2 = cvCreateMat( imageSize.height,imageSize.width, CV_32F );
//  //      CvMat* my2 = cvCreateMat( imageSize.height,imageSize.width, CV_32F );
//  //      CvMat* frame1r = cvCreateMat( imageSize.height,imageSize.width, CV_8U );
//  //      CvMat* frame2r = cvCreateMat( imageSize.height,imageSize.width, CV_8U );
//        CvMat* disp = cvCreateMat( imageSize.height, imageSize.width, CV_16S );
//        CvMat* vdisp = cvCreateMat( imageSize.height,imageSize.width, CV_8U );
////	CvMat* Image3D = cvCreateMat(imageSize.height, imageSize.width, CV_32FC3);	
//        CvMat* pair;
////------------------init for pointer------------------------------------------------------
//	 //CvMat* imgc = cvLoadImageM("havethis.png");
//	int ac = 0, bc = 0, cc = 0, dc = 0, ec = 0, fc = 0; //pixel value varibl
//	int at = 200, bt = 200, ct = 200, dt = 200, et = 200, ft = 200;
//	int y1c = 0, x1c = 0, y2c = 0, x2c = 0, y3c = 0, x3c = 0; //pixel coordi varibl
//	int a_countc = 0, b_countc = 0, c_countc = 0, d_countc = 0, e_countc = 0, f_countc = 0; //counter varibl
////----------------------done---------------------------------
//
//	
//	
///*	double R1[3][3], R2[3][3], P1[3][4], P2[3][4];
//        CvMat _R1 = cvMat(3, 3, CV_64F, R1);
//        CvMat _R2 = cvMat(3, 3, CV_64F, R2);
//	//Calib with Bouguet algrithm
//            CvMat _P1 = cvMat(3, 4, CV_64F, P1);
//            CvMat _P2 = cvMat(3, 4, CV_64F, P2);
//            cvStereoRectify( &_M1, &_M2, &_D1, &_D2, imageSize,&_R, &_T,&_R1, &_R2, &_P1, &_P2, &_Q,0 );
//        //Find matrix for cvRemap()
//            cvInitUndistortRectifyMap(&_M1,&_D1,&_R1,&_P1,mx1,my1);
//            cvInitUndistortRectifyMap(&_M2,&_D2,&_R2,&_P2,mx2,my2);	
//
//*/
//
//       pair = cvCreateMat( imageSize.height, imageSize.width*2,CV_8UC3 );//Paramater for stereo corrrespondences
//	CvStereoBMState *BMState = cvCreateStereoBMState();
//        assert(BMState != 0);
//        BMState->preFilterSize=31;
//        BMState->preFilterCap=32;
//        BMState->SADWindowSize=35;
//        BMState->minDisparity= 10;
//        BMState->numberOfDisparities=16;	
//        BMState->textureThreshold=37;	//reduce noise 
//        BMState->uniquenessRatio=15;	// uniquenessRatio > (match_val–min_match)/min_match.
//	//	CvStereoBMState *state = cvCreateStereoBMState(CV_STEREO_BM_BASIC);
//	//	BMState->speckleRange = 50;
//	//	BMState->textureThreshold = 400;
//	
//	cvNamedWindow( "camera2", 1 );
//	cvNamedWindow( "camera1", 1 );
//	//cvNamedWindow( "rectified",1 ); //create windw 4 rectified img
//	cvNamedWindow( "disparity",1); //create windw 4 disparity o/p
//	//	cvNamedWindow("depthmap",1); //create windw 4 depthmap
//	cvNamedWindow( "para2", 0); //create window 4 trackbar
//	//cvNamedWindow( "para3", 1 ); //create window 4 trackbar
//
//	
//	//imp track bar
//	
//	cvCreateTrackbar("preFilterSize 5 to 255- ","para2",&BMState->preFilterSize,255,0);
//	cvCreateTrackbar("preFilterCap 1 to 63- ","para2",&BMState->preFilterCap,64,0);
//	cvCreateTrackbar("SADWindowSize odd & 5 to 255- ","para2",&g,100,0);
//	cvCreateTrackbar("minDisparity- ","para2",&BMState->minDisparity,60,0);
//	cvCreateTrackbar("numberOfDisparities divisble by 16- ","para2",&h,100,0);
//	cvCreateTrackbar("textureThreshold >0- ","para2",&BMState->textureThreshold,1000,0);
//	cvCreateTrackbar("uniquenessRatio >0- ","para2",&BMState->uniquenessRatio,255,0);
//	//printf("...create slider bars for cvStereoCorrespondenceBM done \n");
//
//
//	//cvCreateTrackbar("ac","para3",&at,255,0);
//	//cvCreateTrackbar("bc","para3",&bt,255,0);
//	//cvCreateTrackbar("cc","para3",&ct,255,0);
//	//cvCreateTrackbar("dc","para3",&dt,255,0);
//	//cvCreateTrackbar("ec","para3",&et,255,0);
//	//cvCreateTrackbar("fc","para3",&ft,255,0);
//
//	capture1= cvCaptureFromCAM(1);
//	capture2= cvCaptureFromCAM(2);
///*	res=cvSetCaptureProperty(capture1,CV_CAP_PROP_FRAME_WIDTH,WIDTH);
//	printf("%d",res);
//	res=cvSetCaptureProperty(capture1,CV_CAP_PROP_FRAME_HEIGHT,HEIGHT);
//	printf("%d",res);
//	res=cvSetCaptureProperty(capture2,CV_CAP_PROP_FRAME_WIDTH,WIDTH);
//	printf("%d",res);
//	res=cvSetCaptureProperty(capture2,CV_CAP_PROP_FRAME_HEIGHT,HEIGHT);
//	printf("%d",res); fflush(stdout);
//*/	
////-------------------------------------------------write for mx my loading!!!!----------------------------------------
//	 //mx1 = (CvMat*)cvLoad("mx1.xml");
//	 //mx2 = (CvMat*)cvLoad("mx2.xml");
//	 //my1 = (CvMat*)cvLoad("my1.xml");
//	 //my2 = (CvMat*)cvLoad("my2.xml");
////--------------------------------------------------UART INITIALIZATION---------------------------------------------------
//	//char INBUFFER[500];
//     char OUTBUFFER_YS[5];
//	 char OUTBUFFER_NT[5];
//    DWORD        bytes_read    = 0;    // Number of bytes read from port
//    DWORD        bytes_written = 0;    // Number of bytes written to the port
//    HANDLE       comport = NULL;  // Handle COM port
//	int   bStatus;
//    DCB          comSettings;          // Contains various port settings
//    COMMTIMEOUTS CommTimeouts;
//    strcpy(&OUTBUFFER_YS[0], "1");
//	strcpy(&OUTBUFFER_NT[0], "0");
//	//printf("%s", OUTBUFFER);
//// Open COM port
//    if ((comport = 
//         CreateFile(L"\\\\.\\COM31",           // open com5:
//                    GENERIC_READ | GENERIC_WRITE, // for reading and writing
//                    0,                            // exclusive access
//                    NULL,                         // no security attributes
//                    OPEN_EXISTING,              
//                    FILE_ATTRIBUTE_NORMAL,
//                    NULL)) == INVALID_HANDLE_VALUE)
//    {
//        // error processing code goes here
//    }
//    // Set timeouts in milliseconds
//    CommTimeouts.ReadIntervalTimeout         = 0; 
//    CommTimeouts.ReadTotalTimeoutMultiplier  = 0; 
//    CommTimeouts.ReadTotalTimeoutConstant    = 100;
//    CommTimeouts.WriteTotalTimeoutMultiplier = 0;
//    CommTimeouts.WriteTotalTimeoutConstant   = 10;
//    bStatus = SetCommTimeouts(comport,&CommTimeouts);
//    if (bStatus != 0)
//    {
//        //printf("Error!");
//    }
//    // Set Port parameters.
//    // Make a call to GetCommState() first in order to fill
//    // the comSettings structure with all the necessary values.
//    // Then change the ones you want and call SetCommState().
//    GetCommState(comport, &comSettings);
//    comSettings.BaudRate = 38400;
//    comSettings.StopBits = ONESTOPBIT;
//    comSettings.ByteSize = 8;
//    comSettings.Parity   = NOPARITY;
//    comSettings.fParity  = FALSE;
//    bStatus = SetCommState(comport, &comSettings);
//    if (bStatus != 0)
//    {
//        //printf("Error2!!");
//    }
////-------------------------------------------------------------------------------------
//	frame1 = cvRetrieveFrame(capture1, 0); 
//	frame2 = cvRetrieveFrame(capture2, 0); 
//	
//	CvMat part;
//	while(1)
//	{
//	//g of SADWindowSize value must be odd
//	if ( g % 2 == 0 )
//	*&BMState->SADWindowSize = g + 1; // if even mak it odd 
//	else  
//	*&BMState->SADWindowSize = g; // if odd keep same
//
//	*&BMState->numberOfDisparities = (16*h);  // assign values to BMState
//
//	cvCvtColor( frame1, gray_fr1, CV_BGR2GRAY );
//	cvCvtColor( frame2, gray_fr2, CV_BGR2GRAY );
//	//cvRemap( gray_fr1, frame1r, mx1, my1 );
//	//cvRemap( gray_fr2, frame2r, mx2, my2 );
//                    cvFindStereoCorrespondenceBM( gray_fr1, gray_fr2, disp, BMState);
//	
//	//cvShowImage("camera1", frame1);
//	//cvShowImage("camera2", frame2);	
//	//	cvConvertScale( disp, disp, 16, 0 );
//	cvNormalize( disp, vdisp, 0, 256, CV_MINMAX );                   
//                    cvShowImage( "disparity", vdisp );	
//	//cvReprojectImageTo3D(disp, Image3D, &_Q); //removed coz no use (coloured frame)	
//	//cvShowImage("depthmap",Image3D); //removed coz no use (coloured frame)
//	
//	//-----------------------pointer code-------------------------------
//	//for( y1c=0,y2c=HEIGHT/3+1,y3c=2*HEIGHT/3+1; (y1c < HEIGHT/3)&&(y2c < 2*HEIGHT/3)&&(y3c < HEIGHT); y1c++,y2c++,y3c++)  // y = rows
// //   {
//	//	for(x1c=0,x2c=WIDTH/2+1;(x1c < WIDTH/2)&&(x2c < WIDTH);x1c++,x2c++)  // x = cols
//	//	{
//	//	ac = CV_MAT_ELEM(*vdisp, unsigned char, y1c, x1c );  //get valu
//	//	bc = CV_MAT_ELEM(*vdisp, unsigned char, y2c, x1c );  //get valu
//	//	cc = CV_MAT_ELEM(*vdisp, unsigned char, y3c, x1c );  //get valu
//	//	dc = CV_MAT_ELEM(*vdisp, unsigned char, y1c, x2c );  //get valu
//	//	ec = CV_MAT_ELEM(*vdisp, unsigned char, y2c, x2c );  //get valu
//	//	fc = CV_MAT_ELEM(*vdisp, unsigned char, y3c, x2c );  //get valu
//
//	//	if(ac > 150)  //pixel valu > 200
//	//	a_countc++;  //count incrs
//	//	else if(bc > 150)  //pixel valu > 200
//	//	b_countc++;  //count incrs
//	//	else if(cc > 150)  //pixel valu > 200
//	//	c_countc++;  //count incrs
//	//	else if(dc > 150)  //pixel valu > 200
//	//	d_countc++;  //count incrs
//	//	else if(ec > 150)  //pixel valu > 200
//	//	e_countc++;  //count incrs
//	//	else if(fc > 150)  //pixel valu > 200
//	//	f_countc++;  //count incrs
//	//	}
// //   }
//
//	//if(a_countc>2000)
//	//	bStatus = WriteFile(comport, &OUTBUFFER_YS, 8, &bytes_written, NULL);
//	//else
//	//	bStatus = WriteFile(comport,&OUTBUFFER_NT,8,&bytes_written,NULL);
//
//	//	cvWaitKey(30);	
//
//	//if(b_countc>2000)
//	//	bStatus = WriteFile(comport,&OUTBUFFER_YS,8,&bytes_written,NULL);
//	//else
//	//	bStatus = WriteFile(comport,&OUTBUFFER_NT,8,&bytes_written,NULL);
//
//	//	cvWaitKey(30);
//
//	//if(c_countc>2000)
//	//	bStatus = WriteFile(comport,&OUTBUFFER_YS,8,&bytes_written,NULL);
//	//else
//	//	bStatus = WriteFile(comport,&OUTBUFFER_NT,8,&bytes_written,NULL);
//
//	//	cvWaitKey(30);
//
//	//if(d_countc>2000)
//	//	bStatus = WriteFile(comport,&OUTBUFFER_YS,8,&bytes_written,NULL);
//	//else
//	//	bStatus = WriteFile(comport,&OUTBUFFER_NT,8,&bytes_written,NULL);
//
//	//	cvWaitKey(30);
//
//	//if(e_countc>2000)
//	//	bStatus = WriteFile(comport,&OUTBUFFER_YS,8,&bytes_written,NULL);
//	//else
//	//	bStatus = WriteFile(comport,&OUTBUFFER_NT,8,&bytes_written,NULL);
//
//	//	cvWaitKey(30);
//
//	//if(f_countc>2000)
//	//	bStatus = WriteFile(comport,&OUTBUFFER_YS,8,&bytes_written,NULL);
//	//else
//	//	bStatus = WriteFile(comport,&OUTBUFFER_NT,8,&bytes_written,NULL);
//
//	//	cvWaitKey(30);
//
//
//	//	printf("...1st part count %d   \n\n", a_countc);  //print countr a
//	//printf("...2st part count %d   \n\n", b_countc);  //print countr b
//	//printf("...3st part count %d   \n\n", c_countc);  //print countr c
//	//printf("...4st part count %d   \n\n", d_countc);  //print countr d
//	//printf("...5st part count %d   \n\n", e_countc);  //print countr e
//	//printf("...6st part count %d   \n\n", f_countc);  //print countr f
//	
//	//a_countc=b_countc=c_countc=d_countc=e_countc=f_countc=0;
//
//
//
//	
//	//
//	//cvWaitKey(0);
////getch();
//  //-------------------------done----------------------------
//                
//	//Hien thi anh da rectify
//	//cvGetCols( pair, &part, 0, imageSize.width );
//     //               cvCvtColor( frame1r, &part, CV_GRAY2BGR );
//     //               cvGetCols( pair, &part, imageSize.width, imageSize.width*2 );
//     //               cvCvtColor( frame2r, &part, CV_GRAY2BGR ); //CV_GRAY2BGR
//     //    for( j = 0; j < imageSize.height; j += 16 )
//	//cvLine( pair, cvPoint(0,j), cvPoint(imageSize.width*2,j), CV_RGB(0,255,0));          	
//	//cvShowImage( "rectified", pair );	
//	frame1 = cvQueryFrame( capture1 );
//	frame2 = cvQueryFrame( capture2 );
//	//printf("%d\n", CV_MAT_ELEM(*vdisp, unsigned char, 240, 320 ));
//	
//	if( cvWaitKey(1) == 27 )  break;            	
//	}	
//
//	//cvWaitKey(0);
//	/*cvSave("mx1.xml",mx1);
//	cvSave("mx2.xml",mx2);
//	cvSave("my1.xml",my1);
//	cvSave("my2.xml",my2);*/
//
///*	printf("filter size==  %d  \n", &BMState->preFilterSize);
//	printf("filter cap==  %d  \n", &BMState->preFilterCap);
//	printf("sad window size==  %d  \n", &BMState->SADWindowSize);
//	printf("min disparity==  %d  \n", &BMState->minDisparity);
//	printf("num of disparity==  %d  \n", &BMState->numberOfDisparities);
//	printf("texture thresh==  %d  \n", &BMState->textureThreshold);
//	printf("uniqueness ratio==  %d  \n", &BMState->uniquenessRatio);
//*/
//        cvWaitKey(0);
//        cvReleaseStereoBMState(&BMState);
//        /*cvReleaseMat( &mx1 );
//        cvReleaseMat( &my1 );
//        cvReleaseMat( &mx2 );
//        cvReleaseMat( &my2 );	
//	*/cvReleaseCapture( &capture1 ); 
//	cvReleaseCapture( &capture2 );
//       /* cvReleaseMat( &frame1r );
//        cvReleaseMat( &frame2r );
//       */ cvReleaseMat( &disp );
//	cvDestroyWindow("para2");
//	//cvReleaseMat(&Image3D);
//
//}
//
////--------------------------------------------------------------------------------------------------------------------------------------
////virtual tagging wid sound output

//#include "stdafx.h"
//#include <cv.h>
//#include <cxcore.h>
//#include <highgui.h>
//#include "cxmisc.h"
//#include "cvaux.h"
//#include <vector>
//#include <string>
//#include <algorithm>
//#include <stdio.h>
//#include <ctype.h>
//#include <math.h>
//#include <stdio.h>
//#include <iostream>
//#include <stdlib.h>
//#include <conio.h>
//#include "opencv2\core\core.hpp"
//#include "opencv2\features2d\features2d.hpp"
//#include "opencv2\nonfree\features2d.hpp"
//#include "opencv2\highgui\highgui.hpp"
//#include "opencv2\imgproc\imgproc.hpp"
//#include "opencv2\calib3d\calib3d.hpp"
//
//using namespace cv;
//
//int main()
//{
//	
//    //Mat object = imread( "techno.jpg", CV_LOAD_IMAGE_GRAYSCALE );
//	Mat Objecta[2];
//	 Objecta[0] = imread( "techno.jpg", CV_LOAD_IMAGE_GRAYSCALE );
//	Objecta[1]=imread( "techno.jpg", CV_LOAD_IMAGE_GRAYSCALE );
//	 int m = 0;
//	int goodMatchesCounter = 0;
//
//    
//  //  Detect the keypoints using SURF Detector
//    int minHessian = 1000;
//	int n = 0,id = 0;
//
//    SurfFeatureDetector detector( minHessian );
//    std::vector<KeyPoint> kp_object;
//
//    detector.detect( Objecta[id], kp_object );
//
//    //Calculate descriptors (feature vectors)
//    SurfDescriptorExtractor extractor;
//    Mat des_object;
//
//    extractor.compute( Objecta[id], kp_object, des_object );
//
//    FlannBasedMatcher matcher;
//  
//  CvCapture* capture1 ; 
//  capture1 = cvCaptureFromCAM(1);
//  capture1.list();
//  printf("...camera property done \n");
//
//    namedWindow("Good Matches");
//
//    std::vector<Point2f> obj_corners(4);
//
//    //Get the corners from the object
//    obj_corners[0] = cvPoint(0,0);
//    obj_corners[1] = cvPoint( Objecta[id].cols, 0 );
//    obj_corners[2] = cvPoint( Objecta[id].cols, Objecta[id].rows );
//    obj_corners[3] = cvPoint( 0, Objecta[id].rows );
//
//	Mat des_image, img_matches, frame, H , image;
//    char key = 'a';
//	const char *WIN1 = "techno";
//	int count = 0, count1=0;
//    int framecount = 0;
//
//	   std::vector<KeyPoint> kp_image;
//        std::vector<vector<DMatch > > matches;
//        std::vector<DMatch > good_matches;
//        std::vector<Point2f> obj;
//        std::vector<Point2f> scene;
//        std::vector<Point2f> scene_corners(4);
//    
//	 obj_corners[0] = cvPoint(0,0);
//
//	while (key != 27)
//    {
//        m = 0;
//         frame = cvRetrieveFrame(capture1, 0);
//	
////--------------------------------GET IMAGES FROM DATABASE----------------------------------------
//	
//	if (framecount%5 == 0 )
//	{	//framecount = framecount + 1;
//	id = 1;
//	WIN1 = "techno";
//	detector.detect( Objecta[id], kp_object );
//	extractor.compute( Objecta[id], kp_object, des_object );
//	 }
//	if (framecount%9 == 0 )
//	{	//framecount = framecount + 1;
//	id = 0;
//	WIN1 = "techno";
//	detector.detect( Objecta[id], kp_object );
//	extractor.compute( Objecta[id], kp_object, des_object );
//	framecount=0;
//	 }
//	framecount = framecount + 1;
//
////--------------------------------------------------------------------------------------------------------
//	
//
//	  std::vector<KeyPoint> kp_image;
//        std::vector<vector<DMatch > > matches;
//        std::vector<DMatch > good_matches;
//        std::vector<Point2f> obj;
//        std::vector<Point2f> scene;
//        std::vector<Point2f> scene_corners(4);
//     
//        cvtColor(frame, image, CV_RGB2GRAY);//CONVERT TO GRAY SCALE
//	
//	
//
//        detector.detect( image, kp_image );
//        extractor.compute( image, kp_image, des_image );
//
//        matcher.knnMatch(des_object, des_image, matches, 2);
//
////_______________________________________________USE FOR DRAWING LINES AND SUB IMAGE : SLOWER______________________________________________________//
//
//        for(int i = 0; i < min(des_image.rows-1,(int) matches.size()); i++) //THIS LOOP IS SENSITIVE TO SEGFAULTS
//        {
//            if((matches[i][0].distance < 0.6*(matches[i][1].distance)) && ((int) matches[i].size()<=2 && (int) matches[i].size()>0))
//            {
//                good_matches.push_back(matches[i][0]);
//            }
//        }
//
//        //Draw only "good" matches
//        //drawMatches( Objecta[id], kp_object, image, kp_image, good_matches, img_matches, Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
//
//        if (good_matches.size() >= 5)
//        {
//	putText(frame, WIN1, cvPoint(450,50), FONT_HERSHEY_COMPLEX_SMALL, 1, cvScalar(0,255,0), 1, CV_AA);
//	if(id==0)
//	system ("espeak -a150 -p70 -s100 techno ");
//	if(id==1)
//	system ("espeak -a150 -p70 -s100 mobile ");
//	//cvWaitKey();
//            for( int i = 0; i < good_matches.size(); i++ )
//            {
//         //       Get the keypoints from the good matches
//                obj.push_back( kp_object[ good_matches[i].queryIdx ].pt );
//                scene.push_back( kp_image[ good_matches[i].trainIdx ].pt );
//            }
//
//            H = findHomography( obj, scene, CV_RANSAC );
//            perspectiveTransform( obj_corners, scene_corners, H);
//	//system ("espeak -a150 -p70 -s100 hi ");
//            //Draw lines between the corners (the mapped object in the scene image )
//	//Draw lines between the corners (the mapped object in the scene image )
//            /*line( img_matches, scene_corners[0] + Point2f( Objecta[id].cols, 0), scene_corners[1] + Point2f( Objecta[id].cols, 0), Scalar(0, 255, 0), 4 );
//            line( img_matches, scene_corners[1] + Point2f( Objecta[id].cols, 0), scene_corners[2] + Point2f( Objecta[id].cols, 0), Scalar( 0, 255, 0), 4 );
//            line( img_matches, scene_corners[2] + Point2f(Objecta[id] .cols, 0), scene_corners[3] + Point2f( Objecta[id].cols, 0), Scalar( 0, 255, 0), 4 );
//            line( img_matches, scene_corners[3] + Point2f( Objecta[id].cols, 0), scene_corners[0] + Point2f( Objecta[id].cols, 0), Scalar( 0, 255, 0), 4 );*/
//        }
//	
//       // Show detected matches
//        imshow( "Good Matches", frame );
//
////---------------------------------------------USE FOR NO LINES N NO SUB IMAGES : FASTER-------------------------------------------------
//	//for(int i = 0; i < min(des_image.rows-1,(int) matches.size()); i++) //THIS LOOP IS SENSITIVE TO SEGFAULTS
//	//{
//	//	if(((int) matches[i].size() <=2 && (int) matches[i].size()>0) && (matches[i][0].distance <0.6*(matches[i] [1].distance)))
//	//	{
//	//	good_matches.push_back(matches[i][0]);
//	//	obj.push_back( kp_object[ matches[i][0].queryIdx ].pt );//COUNT FOR MATCHES
//	//	scene.push_back( kp_image[ matches[i][0].trainIdx ].pt );
//	//	goodMatchesCounter++;//MATCH COUNTER
//	//	}
//	//}
//
//	//if(goodMatchesCounter >= 4)//MATCH FOUND CONDITION
//	//{
//	//	H = findHomography( obj, scene, CV_RANSAC );
//	//	perspectiveTransform( obj_corners, scene_corners, H);
//
//	//	PUT TEXT IN IMAGE OF NAME OF OBJECT DETECTED
//	//	putText(img_matches, WIN1 , cvPoint(450,20), FONT_HERSHEY_COMPLEX_SMALL, 1, cvScalar(0,255,0), 1, CV_AA);
//
//	//	------------------------------PLAY SOUND CASES-------------------------------------------
//	/*if(id==1)
//	system ("espeak -a150 -p70 -s100 S.R.A ");
//	if(id==2)
//	system ("espeak -a150 -p70 -s100 vista ");	
//
//	getch();*/
//
//	//Draw lines between the corners (the mapped object in the scene image )
//	//line( image, scene_corners[0], scene_corners[1], Scalar( 255, 255, 255), 4 );
//	//line( image, scene_corners[1], scene_corners[2], Scalar( 255, 255, 255), 4 );
//	//line( image, scene_corners[2], scene_corners[3], Scalar( 255, 255, 255), 4 );
//	//line( image, scene_corners[3], scene_corners[0], Scalar( 255, 255, 255), 4 );
//	
//	//imshow( "Good Matches", image );//DISPLAY IMAGE
//	key = waitKey(1);
//	
//	}
//    return 0;
//}
//
/*
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include "cxmisc.h"
#include "cvaux.h"
#include <vector>
#include <string>
#include <algorithm>
#include <stdio.h>
#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <iostream>
#include "opencv2\core\core.hpp"
#include "opencv2\features2d\features2d.hpp"
#include "opencv2\nonfree\features2d.hpp"
#include "opencv2\highgui\highgui.hpp"
#include "opencv2\imgproc\imgproc.hpp"
#include "opencv2\calib3d\calib3d.hpp"

using namespace cv;

int main()
{
    Mat object = imread( "sadi.jpg", CV_LOAD_IMAGE_GRAYSCALE );

    if( !object.data )
    {
        std::cout<< "Error reading object " << std::endl;
        return -1;
    }

    //Detect the keypoints using SURF Detector
    int minHessian = 1000;

    SurfFeatureDetector detector( minHessian );
    std::vector<KeyPoint> kp_object;

    detector.detect( object, kp_object );

    //Calculate descriptors (feature vectors)
    SurfDescriptorExtractor extractor;
    Mat des_object;

    extractor.compute( object, kp_object, des_object );

    FlannBasedMatcher matcher;

  CvCapture* capture1 ; 
  capture1 = cvCaptureFromCAM(0);

  printf("...camera property done \n");

    namedWindow("Good Matches");

    std::vector<Point2f> obj_corners(4);

    //Get the corners from the object
    obj_corners[0] = cvPoint(0,0);
    obj_corners[1] = cvPoint( object.cols, 0 );
    obj_corners[2] = cvPoint( object.cols, object.rows );
    obj_corners[3] = cvPoint( 0, object.rows );

	Mat des_image, img_matches, frame;
    char key = 'a';
    int framecount = 0;
    while (key != 27)
    {
        
         frame = cvQueryFrame(capture1);

        if (framecount < 5)
        {
            framecount++;
            continue;
        }

       
        std::vector<KeyPoint> kp_image;
        std::vector<vector<DMatch > > matches;
        std::vector<DMatch > good_matches;
        std::vector<Point2f> obj;
        std::vector<Point2f> scene;
        std::vector<Point2f> scene_corners(4);
        Mat H;
        Mat image;

        cvtColor(frame, image, CV_RGB2GRAY);

        detector.detect( image, kp_image );
        extractor.compute( image, kp_image, des_image );

        matcher.knnMatch(des_object, des_image, matches, 2);

        for(int i = 0; i < min(des_image.rows-1,(int) matches.size()); i++) //THIS LOOP IS SENSITIVE TO SEGFAULTS
        {
            if((matches[i][0].distance < 0.6*(matches[i][1].distance)) && ((int) matches[i].size()<=2 && (int) matches[i].size()>0))
            {
                good_matches.push_back(matches[i][0]);
            }
        }

        //Draw only "good" matches
        drawMatches( object, kp_object, image, kp_image, good_matches, img_matches, Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

        if (good_matches.size() >= 4)
        {
            for( int i = 0; i < good_matches.size(); i++ )
            {
                //Get the keypoints from the good matches
                obj.push_back( kp_object[ good_matches[i].queryIdx ].pt );
                scene.push_back( kp_image[ good_matches[i].trainIdx ].pt );
            }

            H = findHomography( obj, scene, CV_RANSAC );

            perspectiveTransform( obj_corners, scene_corners, H);

            //Draw lines between the corners (the mapped object in the scene image )
            line( img_matches, scene_corners[0] + Point2f( object.cols, 0), scene_corners[1] + Point2f( object.cols, 0), Scalar(0, 255, 0), 4 );
            line( img_matches, scene_corners[1] + Point2f( object.cols, 0), scene_corners[2] + Point2f( object.cols, 0), Scalar( 0, 255, 0), 4 );
            line( img_matches, scene_corners[2] + Point2f( object.cols, 0), scene_corners[3] + Point2f( object.cols, 0), Scalar( 0, 255, 0), 4 );
            line( img_matches, scene_corners[3] + Point2f( object.cols, 0), scene_corners[0] + Point2f( object.cols, 0), Scalar( 0, 255, 0), 4 );
        }
	putText(img_matches, "Differencing the two images.", cvPoint(600,100), FONT_HERSHEY_COMPLEX_SMALL, 1, cvScalar(200,200,250), 1, CV_AA);
        //Show detected matches
        imshow( "Good Matches", img_matches );

        key = waitKey(1);
    }
    return 0;
}
//*/
////
//////
//#include "stdafx.h"
//#include <cv.h>
//#include <cxcore.h>
//#include <highgui.h>
//#include "cxmisc.h"
//#include "cvaux.h"
//#include <vector>
//#include <string>
//#include <algorithm>
//#include <stdio.h>
//#include <ctype.h>
//#include <math.h>
//#include <stdio.h>
//#include <iostream>
//#include <stdlib.h>
//#include <conio.h>
//#include "opencv2\core\core.hpp"
//#include "opencv2\features2d\features2d.hpp"
//#include "opencv2\nonfree\features2d.hpp"
//#include "opencv2\highgui\highgui.hpp"
//#include "opencv2\imgproc\imgproc.hpp"
//#include "opencv2\calib3d\calib3d.hpp"
//
//using namespace cv;
//
//int main()
//{
//    Mat object = imread( "techno.jpg", CV_LOAD_IMAGE_GRAYSCALE );
//	int m = 0;
//	int goodMatchesCounter = 0;
//
//    if( !object.data )
//    {
//        std::cout<< "Error reading object " << std::endl;
//        return -1;
//     }
//
//    //Detect the keypoints using SURF Detector
//    int minHessian = 1000;
//	int n = 0,id = 0;
//
//    SurfFeatureDetector detector( minHessian );
//    std::vector<KeyPoint> kp_object;
//
//    detector.detect( object, kp_object );
//
//    //Calculate descriptors (feature vectors)
//    SurfDescriptorExtractor extractor;
//    Mat des_object;
//
//    extractor.compute( object, kp_object, des_object );
//
//    FlannBasedMatcher matcher;
//  
//  CvCapture* capture1 ; 
//  capture1 = cvCaptureFromCAM(0);
//
//  printf("...camera property done \n");
//
//    namedWindow("Good Matches");
//
//    std::vector<Point2f> obj_corners(4);
//	cvWaitKey(2000);
//    //Get the corners from the object
//    obj_corners[0] = cvPoint(0,0);
//    obj_corners[1] = cvPoint( object.cols, 0 );
//    obj_corners[2] = cvPoint( object.cols, object.rows );
//    obj_corners[3] = cvPoint( 0, object.rows );
//
//	Mat des_image, img_matches, frame, H , image;
//    char key = 'a';
//	const char *WIN1 = "boobs";
//	int count = 0, count1=0;
//    int framecount = 0;
//
//	 std::vector<KeyPoint> kp_image;
//        std::vector<vector<DMatch > > matches;
//        std::vector<DMatch > good_matches;
//        std::vector<Point2f> obj;
//        std::vector<Point2f> scene;
//        std::vector<Point2f> scene_corners(4);
//    frame = cvRetrieveFrame(capture1, 0);
//	if (count < 100)
//        {
//	printf("%d",count);
//            count++;
//        }
//	frame = cvRetrieveFrame(capture1, 0);
//	object = imread("techno.jpg", CV_LOAD_IMAGE_GRAYSCALE); 
//	obj_corners[1] = cvPoint( object.cols, 0 );
//	 obj_corners[2] = cvPoint( object.cols, object.rows );
//	 obj_corners[3] = cvPoint( 0, object.rows );
//
//	while (key != 27)
//    {
//        m = 0;
//         frame = cvRetrieveFrame(capture1, 0);
//	//	 cvWaitKey(2000);
//
////-------------------------------LOOP FOR DELAY FOR SETTLING CAMERA----------------------
//        
////--------------------------------------------------------------------------------------------
//	//	cvWaitKey(10);
//	
////--------------------------------GET IMAGES FROM DATABASE----------------------------------------
//	 
//	
////--------------------------------------------------------------------------------------------------------
//	detector.detect( object, kp_object );
//	 extractor.compute( object, kp_object, des_object );
//	 
//     
//        cvtColor(frame, image, CV_RGB2GRAY);//CONVERT TO GRAY SCALE
//
//	
//    extractor.compute( object, kp_object, des_object );
//        detector.detect( image, kp_image );
//        extractor.compute( image, kp_image, des_image );
//
//        matcher.knnMatch(des_object, des_image, matches, 2);
//
////_______________________________________________USE FOR DRAWING LINES AND SUB IMAGE : SLOWER______________________________________________________//
//
//  //      for(int i = 0; i < min(des_image.rows-1,(int) matches.size()); i++) //THIS LOOP IS SENSITIVE TO SEGFAULTS
//  //      {
//  //          if((matches[i][0].distance < 0.6*(matches[i][1].distance)) && ((int) matches[i].size()<=2 && (int) matches[i].size()>0))
//  //          {
//  //              good_matches.push_back(matches[i][0]);
//  //          }
//  //      }
//
//  //      Draw only "good" matches
//  //      drawMatches( object, kp_object, image, kp_image, good_matches, img_matches, Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
//
//  //      if (good_matches.size() >= 5)
//  //      {
//	//	m = 1;
//	//	putText(img_matches, ".jpg.", cvPoint(450,50), FONT_HERSHEY_COMPLEX_SMALL, 1, cvScalar(0,255,0), 1, CV_AA);
//	//	cvWaitKey();
//  //          for( int i = 0; i < good_matches.size(); i++ )
//  //          {
//  //              Get the keypoints from the good matches
//  //              obj.push_back( kp_object[ good_matches[i].queryIdx ].pt );
//  //              scene.push_back( kp_image[ good_matches[i].trainIdx ].pt );
//  //          }
//
//	//	putText(img_matches, WIN1 , cvPoint(450,20), FONT_HERSHEY_COMPLEX_SMALL, 1, cvScalar(0,255,0), 1, CV_AA);
//
//  //          H = findHomography( obj, scene, CV_RANSAC );
//
//  //          perspectiveTransform( obj_corners, scene_corners, H);
//	//	system ("espeak -a150 -p70 -s100 boobs ");
//  //  //        Draw lines between the corners (the mapped object in the scene image )
//  //          line( img_matches, scene_corners[0] + Point2f( object.cols, 0), scene_corners[1] + Point2f( object.cols, 0), Scalar(0, 255, 0), 4 );
//  //          line( img_matches, scene_corners[1] + Point2f( object.cols, 0), scene_corners[2] + Point2f( object.cols, 0), Scalar( 0, 255, 0), 4 );
//  //          line( img_matches, scene_corners[2] + Point2f( object.cols, 0), scene_corners[3] + Point2f( object.cols, 0), Scalar( 0, 255, 0), 4 );
//  //          line( img_matches, scene_corners[3] + Point2f( object.cols, 0), scene_corners[0] + Point2f( object.cols, 0), Scalar( 0, 255, 0), 4 );
//  //      }
//	//
//
//	//putText(img_matches, "objject recognized book.jpg.", cvPoint(450,50), FONT_HERSHEY_COMPLEX_SMALL, 1, cvScalar(0,255,0), 1, CV_AA);
//  ////      Show detected matches
//  //      imshow( "Good Matches", img_matches );
//
////---------------------------------------------USE FOR NO LINES N NO SUB IMAGES : FASTER-------------------------------------------------
//	for(int i = 0; i < min(des_image.rows-1,(int) matches.size()); i++) //THIS LOOP IS SENSITIVE TO SEGFAULTS
//	{
//	if(((int) matches[i].size() <=2 && (int) matches[i].size()>0) && (matches[i][0].distance <0.6*(matches[i] [1].distance)))
//	{
//	//good_matches.push_back(matches[i][0]);
//	obj.push_back( kp_object[ matches[i][0].queryIdx ].pt );//COUNT FOR MATCHES
//	scene.push_back( kp_image[ matches[i][0].trainIdx ].pt );
//	goodMatchesCounter++;//MATCH COUNTER
//	}
//	}
//
//	if(goodMatchesCounter >= 6)//MATCH FOUND CONDITION
//	{
//	H = findHomography( obj, scene, CV_RANSAC );
//	perspectiveTransform( obj_corners, scene_corners, H);
//
//	//PUT TEXT IN IMAGE OF NAME OF OBJECT DETECTED
//	
//
//	//------------------------------PLAY SOUND CASES-------------------------------------------
//	//if(id==1)
//	//system ("espeak -a150 -p70 -s100 S.R.A ");
//	//if(id==2)
//	//	system ("espeak -a150 -p70 -s100 vista ");	
//
//	//	getch();
//
//	//	Draw lines between the corners (the mapped object in the scene image )
//	line( image, scene_corners[0], scene_corners[1], Scalar( 255, 255, 255), 4 );
//	line( image, scene_corners[1], scene_corners[2], Scalar( 255, 255, 255), 4 );
//	line( image, scene_corners[2], scene_corners[3], Scalar( 255, 255, 255), 4 );
//	line( image, scene_corners[3], scene_corners[0], Scalar( 255, 255, 255), 4 );
//	}	
//	putText(image, WIN1 , cvPoint(45,20), FONT_HERSHEY_COMPLEX_SMALL, 1, cvScalar(0,255,0), 1, CV_AA);
//	imshow( "Good Matches", image );//DISPLAY IMAGE
////______________________________________________________________________________________________________________________________________________//
//
//	printf("...end loop \n");
//	key = waitKey(1);
//	
//	goodMatchesCounter= 0;
//    }
//    return 0;
//}
//
//
//




////
////
////#include <stdio.h>
////#if defined WIN32 || defined _WIN32
////	#include <conio.h>	// For _kbhit() on Windows
////	#include <direct.h>	// For mkdir(path) on Windows
////	#define snprintf sprintf_s	// Visual Studio on Windows comes with sprintf_s() instead of snprintf()
////#else
////	#include <stdio.h>	// For getchar() on Linux
////	#include <termios.h>	// For kbhit() on Linux
////	#include <unistd.h>
////	#include <sys/types.h>
////	#include <sys/stat.h>	// For mkdir(path, options) on Linux
////#endif
////#include <vector>
////#include <string.h>
////#include "cv.h"
////#include "cvaux.h"
////#include "highgui.h"
////#include <windows.h>
////#include "cxmisc.h"
////#include <algorithm>
////#include <stdio.h>
////#include <ctype.h>
////#include <tchar.h>
////
////
////#ifndef BOOL
////	#define BOOL bool
////#endif
////
////using namespace std;
////
////// Haar Cascade file, used for Face Detection.
////const char *faceCascadeFilename = "haarcascade_frontalface_alt.xml";
////
////int SAVE_EIGENFACE_IMAGES = 1;	// Set to 0 if you dont want images of the Eigenvectors saved to files (for debugging).
//////#define USE_MAHALANOBIS_DISTANCE	// You might get better recognition accuracy if you enable this.
////
////
////// Global variables
////IplImage ** faceImgArr        = 0; // array of face images
////CvMat    *  personNumTruthMat = 0; // array of person numbers
//////#define	MAX_NAME_LENGTH 256	// Give each name a fixed size for easier code.
//////char **personNames = 0;	// array of person names (indexed by the person number). Added by Shervin.
////vector<string> personNames;	// array of person names (indexed by the person number). Added by Shervin.
////int faceWidth = 120;	// Default dimensions for faces in the face recognition database. Added by Shervin.
////int faceHeight = 90;	//	"	"	"	"	"	"	"	"
////int nPersons                  = 0; // the number of people in the training set. Added by Shervin.
////int nTrainFaces               = 0; // the number of training images
////int nEigens                   = 0; // the number of eigenvalues
////IplImage * pAvgTrainImg       = 0; // the average image
////IplImage ** eigenVectArr      = 0; // eigenvectors
////CvMat * eigenValMat           = 0; // eigenvalues
////CvMat * projectedTrainFaceMat = 0; // projected training faces
////
////CvCapture* camera = 0;	// The camera device.
////
////
////// Function prototypes
////void printUsage();
////void learn(const char *szFileTrain);
////void doPCA();
////void storeTrainingData();
////int  loadTrainingData(CvMat ** pTrainPersonNumMat);
////int  findNearestNeighbor(float * projectedTestFace);
////int findNearestNeighbor(float * projectedTestFace, float *pConfidence);
////int  loadFaceImgArray(const char * filename);
////void recognizeFileList(const char *szFileTest);
////void recognizeFromCam(void);
////IplImage* getCameraFrame(void);
////IplImage* convertImageToGreyscale(const IplImage *imageSrc);
////IplImage* cropImage(const IplImage *img, const CvRect region);
////IplImage* resizeImage(const IplImage *origImg, int newWidth, int newHeight);
////IplImage* convertFloatImageToUcharImage(const IplImage *srcImg);
////void saveFloatImage(const char *filename, const IplImage *srcImg);
////CvRect detectFaceInImage(const IplImage *inputImg, const CvHaarClassifierCascade* cascade );
////CvMat* retrainOnline(void);
////
////// Show how to use this program from the command-line.
////void printUsage()
////{
////	printf("OnlineFaceRec, created by Shervin Emami (www.shervinemami.co.cc), 2nd Jun 2010.\n"
////	"Usage: OnlineFaceRec [<command>] \n"
////	"  Valid commands are: \n"
////	"    train <train_file> \n"
////	"    test <test_file> \n"
////	" (if no args are supplied, then online camera mode is enabled).\n"
////	);
////}
////
////
////// Startup routine.
////int main( int argc, char** argv )
////{
////	printUsage();
////
////
////	recognizeFromCam();
////	return 0;
////}
////
////#if defined WIN32 || defined _WIN32
////	// Wrappers of kbhit() and getch() for Windows:
////	#define changeKeyboardMode
////	#define kbhit _kbhit
////#else
////	// Create an equivalent to kbhit() and getch() for Linux,
////	// based on "http://cboard.cprogramming.com/c-programming/63166-kbhit-linux.html":
////	
////	#define VK_ESCAPE 0x1B	// Escape character
////
////	// If 'dir' is 1, get the Linux terminal to return the 1st keypress instead of waiting for an ENTER key.
////	// If 'dir' is 0, will reset the terminal back to the original settings.
////	void changeKeyboardMode(int dir)
////	{
////	static struct termios oldt, newt;
////
////	if ( dir == 1 ) {
////	tcgetattr( STDIN_FILENO, &oldt);
////	newt = oldt;
////	newt.c_lflag &= ~( ICANON | ECHO );
////	tcsetattr( STDIN_FILENO, TCSANOW, &newt);
////	}
////	else
////	tcsetattr( STDIN_FILENO, TCSANOW, &oldt);
////	}
////
////	// Get the next keypress.
////	int kbhit(void)
////	{
////	struct timeval tv;
////	fd_set rdfs;
////
////	tv.tv_sec = 0;
////	tv.tv_usec = 0;
////
////	FD_ZERO(&rdfs);
////	FD_SET (STDIN_FILENO, &rdfs);
////
////	select(STDIN_FILENO+1, &rdfs, NULL, NULL, &tv);
////	return FD_ISSET(STDIN_FILENO, &rdfs);
////	}
////
////	// Use getchar() on Linux instead of getch().
////	#define getch() getchar()
////#endif
////
////
////// Save all the eigenvectors as images, so that they can be checked.
////void storeEigenfaceImages()
////{
////	// Store the average image to a file
////	printf("Saving the image of the average face as 'out_averageImage.bmp'.\n");
////	cvSaveImage("out_averageImage.bmp", pAvgTrainImg);
////	// Create a large image made of many eigenface images.
////	// Must also convert each eigenface image to a normal 8-bit UCHAR image instead of a 32-bit float image.
////	printf("Saving the %d eigenvector images as 'out_eigenfaces.bmp'\n", nEigens);
////	if (nEigens > 0) {
////	// Put all the eigenfaces next to each other.
////	int COLUMNS = 8;	// Put upto 8 images on a row.
////	int nCols = min(nEigens, COLUMNS);
////	int nRows = 1 + (nEigens / COLUMNS);	// Put the rest on new rows.
////	int w = eigenVectArr[0]->width;
////	int h = eigenVectArr[0]->height;
////	CvSize size;
////	size = cvSize(nCols * w, nRows * h);
////	IplImage *bigImg = cvCreateImage(size, IPL_DEPTH_8U, 1);	// 8-bit Greyscale UCHAR image
////	for (int i=0; i<nEigens; i++) {
////	// Get the eigenface image.
////	IplImage *byteImg = convertFloatImageToUcharImage(eigenVectArr[i]);
////	// Paste it into the correct position.
////	int x = w * (i % COLUMNS);
////	int y = h * (i / COLUMNS);
////	CvRect ROI = cvRect(x, y, w, h);
////	cvSetImageROI(bigImg, ROI);
////	cvCopyImage(byteImg, bigImg);
////	cvResetImageROI(bigImg);
////	cvReleaseImage(&byteImg);
////	}
////	cvSaveImage("out_eigenfaces.bmp", bigImg);
////	cvReleaseImage(&bigImg);
////	}
////}
////
////// Train from the data in the given text file, and store the trained data into the file 'facedata.xml'.
////void learn(const char *szFileTrain)
////{
////	int i, offset;
////
////	// load training data
////	printf("Loading the training images in '%s'\n", szFileTrain);
////	nTrainFaces = loadFaceImgArray(szFileTrain);
////	printf("Got %d training images.\n", nTrainFaces);
////	if( nTrainFaces < 2 )
////	{
////	fprintf(stderr,
////	        "Need 2 or more training faces\n"
////	        "Input file contains only %d\n", nTrainFaces);
////	return;
////	}
////
////	// do PCA on the training faces
////	doPCA();
////
////	// project the training images onto the PCA subspace
////	projectedTrainFaceMat = cvCreateMat( nTrainFaces, nEigens, CV_32FC1 );
////	offset = projectedTrainFaceMat->step / sizeof(float);
////	for(i=0; i<nTrainFaces; i++)
////	{
////	//int offset = i * nEigens;
////	cvEigenDecomposite(
////	faceImgArr[i],
////	nEigens,
////	eigenVectArr,
////	0, 0,
////	pAvgTrainImg,
////	//projectedTrainFaceMat->data.fl + i*nEigens);
////	projectedTrainFaceMat->data.fl + i*offset);
////	}
////
////	// store the recognition data as an xml file
////	storeTrainingData();
////
////	// Save all the eigenvectors as images, so that they can be checked.
////	if (SAVE_EIGENFACE_IMAGES) {
////	storeEigenfaceImages();
////	}
////
////}
////
////
////// Open the training data from the file 'facedata.xml'.
////int loadTrainingData(CvMat ** pTrainPersonNumMat)
////{
////	CvFileStorage * fileStorage;
////	int i;
////
////	// create a file-storage interface
////	fileStorage = cvOpenFileStorage( "facedata.xml", 0, CV_STORAGE_READ );
////	if( !fileStorage ) {
////	printf("Can't open training database file 'facedata.xml'.\n");
////	return 0;
////	}
////
////	// Load the person names. Added by Shervin.
////	personNames.clear();	// Make sure it starts as empty.
////	nPersons = cvReadIntByName( fileStorage, 0, "nPersons", 0 );
////	if (nPersons == 0) {
////	printf("No people found in the training database 'facedata.xml'.\n");
////	return 0;
////	}
////	// Load each person's name.
////	for (i=0; i<nPersons; i++) {
////	string sPersonName;
////	char varname[200];
////	snprintf( varname, sizeof(varname)-1, "personName_%d", (i+1) );
////	sPersonName = cvReadStringByName(fileStorage, 0, varname );
////	personNames.push_back( sPersonName );
////	}
////
////	// Load the data
////	nEigens = cvReadIntByName(fileStorage, 0, "nEigens", 0);
////	nTrainFaces = cvReadIntByName(fileStorage, 0, "nTrainFaces", 0);
////	*pTrainPersonNumMat = (CvMat *)cvReadByName(fileStorage, 0, "trainPersonNumMat", 0);
////	eigenValMat  = (CvMat *)cvReadByName(fileStorage, 0, "eigenValMat", 0);
////	projectedTrainFaceMat = (CvMat *)cvReadByName(fileStorage, 0, "projectedTrainFaceMat", 0);
////	pAvgTrainImg = (IplImage *)cvReadByName(fileStorage, 0, "avgTrainImg", 0);
////	eigenVectArr = (IplImage **)cvAlloc(nTrainFaces*sizeof(IplImage *));
////	for(i=0; i<nEigens; i++)
////	{
////	char varname[200];
////	snprintf( varname, sizeof(varname)-1, "eigenVect_%d", i );
////	eigenVectArr[i] = (IplImage *)cvReadByName(fileStorage, 0, varname, 0);
////	}
////
////	// release the file-storage interface
////	cvReleaseFileStorage( &fileStorage );
////
////	printf("Training data loaded (%d training images of %d people):\n", nTrainFaces, nPersons);
////	printf("People: ");
////	if (nPersons > 0)
////	printf("<%s>", personNames[0].c_str());
////	for (i=1; i<nPersons; i++) {
////	printf(", <%s>", personNames[i].c_str());
////	}
////	printf(".\n");
////
////	return 1;
////}
////
////
////// Save the training data to the file 'facedata.xml'.
////void storeTrainingData()
////{
////	CvFileStorage * fileStorage;
////	int i;
////
////	// create a file-storage interface
////	fileStorage = cvOpenFileStorage( "facedata.xml", 0, CV_STORAGE_WRITE );
////
////	// Store the person names. Added by Shervin.
////	cvWriteInt( fileStorage, "nPersons", nPersons );
////	for (i=0; i<nPersons; i++) {
////	char varname[200];
////	snprintf( varname, sizeof(varname)-1, "personName_%d", (i+1) );
////	cvWriteString(fileStorage, varname, personNames[i].c_str(), 0);
////	}
////
////	// store all the data
////	cvWriteInt( fileStorage, "nEigens", nEigens );
////	cvWriteInt( fileStorage, "nTrainFaces", nTrainFaces );
////	cvWrite(fileStorage, "trainPersonNumMat", personNumTruthMat, cvAttrList(0,0));
////	cvWrite(fileStorage, "eigenValMat", eigenValMat, cvAttrList(0,0));
////	cvWrite(fileStorage, "projectedTrainFaceMat", projectedTrainFaceMat, cvAttrList(0,0));
////	cvWrite(fileStorage, "avgTrainImg", pAvgTrainImg, cvAttrList(0,0));
////	for(i=0; i<nEigens; i++)
////	{
////	char varname[200];
////	snprintf( varname, sizeof(varname)-1, "eigenVect_%d", i );
////	cvWrite(fileStorage, varname, eigenVectArr[i], cvAttrList(0,0));
////	}
////
////	// release the file-storage interface
////	cvReleaseFileStorage( &fileStorage );
////}
////
////// Find the most likely person based on a detection. Returns the index, and stores the confidence value into pConfidence.
////int findNearestNeighbor(float * projectedTestFace, float *pConfidence)
////{
////	//double leastDistSq = 1e12;
////	double leastDistSq = DBL_MAX;
////	int i, iTrain, iNearest = 0;
////
////	for(iTrain=0; iTrain<nTrainFaces; iTrain++)
////	{
////	double distSq=0;
////
////	for(i=0; i<nEigens; i++)
////	{
////	float d_i = projectedTestFace[i] - projectedTrainFaceMat->data.fl[iTrain*nEigens + i];
////#ifdef USE_MAHALANOBIS_DISTANCE
////	distSq += d_i*d_i / eigenValMat->data.fl[i];  // Mahalanobis distance (might give better results than Eucalidean distance)
////#else
////	distSq += d_i*d_i; // Euclidean distance.
////#endif
////	}
////
////	if(distSq < leastDistSq)
////	{
////	leastDistSq = distSq;
////	iNearest = iTrain;
////	}
////	}
////
////	// Return the confidence level based on the Euclidean distance,
////	// so that similar images should give a confidence between 0.5 to 1.0,
////	// and very different images should give a confidence between 0.0 to 0.5.
////	*pConfidence = 1.0f - sqrt( leastDistSq / (float)(nTrainFaces * nEigens) ) / 255.0f;
////
////	// Return the found index.
////	return iNearest;
////}
////
////// Do the Principal Component Analysis, finding the average image
////// and the eigenfaces that represent any image in the given dataset.
////void doPCA()
////{
////	int i;
////	CvTermCriteria calcLimit;
////	CvSize faceImgSize;
////
////	// set the number of eigenvalues to use
////	nEigens = nTrainFaces-1;
////
////	// allocate the eigenvector images
////	faceImgSize.width  = faceImgArr[0]->width;
////	faceImgSize.height = faceImgArr[0]->height;
////	eigenVectArr = (IplImage**)cvAlloc(sizeof(IplImage*) * nEigens);
////	for(i=0; i<nEigens; i++)
////	eigenVectArr[i] = cvCreateImage(faceImgSize, IPL_DEPTH_32F, 1);
////
////	// allocate the eigenvalue array
////	eigenValMat = cvCreateMat( 1, nEigens, CV_32FC1 );
////
////	// allocate the averaged image
////	pAvgTrainImg = cvCreateImage(faceImgSize, IPL_DEPTH_32F, 1);
////
////	// set the PCA termination criterion
////	calcLimit = cvTermCriteria( CV_TERMCRIT_ITER, nEigens, 1);
////
////	// compute average image, eigenvalues, and eigenvectors
////	cvCalcEigenObjects(
////	nTrainFaces,
////	(void*)faceImgArr,
////	(void*)eigenVectArr,
////	CV_EIGOBJ_NO_CALLBACK,
////	0,
////	0,
////	&calcLimit,
////	pAvgTrainImg,
////	eigenValMat->data.fl);
////
////	cvNormalize(eigenValMat, eigenValMat, 1, 0, CV_L1, 0);
////}
////
////// Read the names & image filenames of people from a text file, and load all those images listed.
////int loadFaceImgArray(const char * filename)
////{
////	FILE * imgListFile = 0;
////	char imgFilename[512];
////	int iFace, nFaces=0;
////	int i;
////
////	// open the input file
////	if( !(imgListFile = fopen(filename, "r")) )
////	{
////	fprintf(stderr, "Can\'t open file %s\n", filename);
////	return 0;
////	}
////
////	// count the number of faces
////	while( fgets(imgFilename, sizeof(imgFilename)-1, imgListFile) ) ++nFaces;
////	rewind(imgListFile);
////
////	// allocate the face-image array and person number matrix
////	faceImgArr        = (IplImage **)cvAlloc( nFaces*sizeof(IplImage *) );
////	personNumTruthMat = cvCreateMat( 1, nFaces, CV_32SC1 );
////
////	personNames.clear();	// Make sure it starts as empty.
////	nPersons = 0;
////
////	// store the face images in an array
////	for(iFace=0; iFace<nFaces; iFace++)
////	{
////	char personName[256];
////	string sPersonName;
////	int personNumber;
////
////	// read person number (beginning with 1), their name and the image filename.
////	fscanf(imgListFile, "%d %s %s", &personNumber, personName, imgFilename);
////	sPersonName = personName;
////	//printf("Got %d: %d, <%s>, <%s>.\n", iFace, personNumber, personName, imgFilename);
////
////	// Check if a new person is being loaded.
////	if (personNumber > nPersons) {
////	// Allocate memory for the extra person (or possibly multiple), using this new person's name.
////	for (i=nPersons; i < personNumber; i++) {
////	personNames.push_back( sPersonName );
////	}
////	nPersons = personNumber;
////	//printf("Got new person <%s> -> nPersons = %d [%d]\n", sPersonName.c_str(), nPersons, personNames.size());
////	}
////
////	// Keep the data
////	personNumTruthMat->data.i[iFace] = personNumber;
////
////	// load the face image
////	faceImgArr[iFace] = cvLoadImage(imgFilename, CV_LOAD_IMAGE_GRAYSCALE);
////
////	if( !faceImgArr[iFace] )
////	{
////	fprintf(stderr, "Can\'t load image from %s\n", imgFilename);
////	return 0;
////	}
////	}
////
////	fclose(imgListFile);
////
////	printf("Data loaded from '%s': (%d images of %d people).\n", filename, nFaces, nPersons);
////	printf("People: ");
////	if (nPersons > 0)
////	printf("<%s>", personNames[0].c_str());
////	for (i=1; i<nPersons; i++) {
////	printf(", <%s>", personNames[i].c_str());
////	}
////	printf(".\n");
////
////	return nFaces;
////}
////
////
////// Recognize the face in each of the test images given, and compare the results with the truth.
////void recognizeFileList(const char *szFileTest)
////{
////	int i, nTestFaces  = 0;         // the number of test images
////	CvMat * trainPersonNumMat = 0;  // the person numbers during training
////	float * projectedTestFace = 0;
////	const char *answer;
////	int nCorrect = 0;
////	int nWrong = 0;
////	double timeFaceRecognizeStart;
////	double tallyFaceRecognizeTime;
////	float confidence;
////
////	// load test images and ground truth for person number
////	nTestFaces = loadFaceImgArray(szFileTest);
////	printf("%d test faces loaded\n", nTestFaces);
////
////	// load the saved training data
////	if( !loadTrainingData( &trainPersonNumMat ) ) return;
////
////	// project the test images onto the PCA subspace
////	projectedTestFace = (float *)cvAlloc( nEigens*sizeof(float) );
////	timeFaceRecognizeStart = (double)cvGetTickCount();	// Record the timing.
////	for(i=0; i<nTestFaces; i++)
////	{
////	int iNearest, nearest, truth;
////
////	// project the test image onto the PCA subspace
////	cvEigenDecomposite(
////	faceImgArr[i],
////	nEigens,
////	eigenVectArr,
////	0, 0,
////	pAvgTrainImg,
////	projectedTestFace);
////
////	iNearest = findNearestNeighbor(projectedTestFace, &confidence);
////	truth    = personNumTruthMat->data.i[i];
////	nearest  = trainPersonNumMat->data.i[iNearest];
////
////	if (nearest == truth) {
////	answer = "Correct";
////	nCorrect++;
////	}
////	else {
////	answer = "WRONG!";
////	nWrong++;
////	}
////	printf("nearest = %d, Truth = %d (%s). Confidence = %f\n", nearest, truth, answer, confidence);
////	}
////	tallyFaceRecognizeTime = (double)cvGetTickCount() - timeFaceRecognizeStart;
////	if (nCorrect+nWrong > 0) {
////	printf("TOTAL ACCURACY: %d%% out of %d tests.\n", nCorrect * 100/(nCorrect+nWrong), (nCorrect+nWrong));
////	printf("TOTAL TIME: %.1fms average.\n", tallyFaceRecognizeTime/((double)cvGetTickFrequency() * 1000.0 * (nCorrect+nWrong) ) );
////	}
////
////}
////
////
////// Grab the next camera frame. Waits until the next frame is ready,
////// and provides direct access to it, so do NOT modify the returned image or free it!
////// Will automatically initialize the camera on the first frame.
////IplImage* getCameraFrame(void)
////{
////	IplImage *frame;
////
////	// If the camera hasn't been initialized, then open it.
////	if (!camera) {
////	printf("Acessing the camera ...\n");
////	camera = cvCaptureFromCAM( 0 );
////	if (!camera) {
////	printf("ERROR in getCameraFrame(): Couldn't access the camera.\n");
////	exit(1);
////	}
////	// Try to set the camera resolution
////	cvSetCaptureProperty( camera, CV_CAP_PROP_FRAME_WIDTH, 320 );
////	cvSetCaptureProperty( camera, CV_CAP_PROP_FRAME_HEIGHT, 240 );
////	// Wait a little, so that the camera can auto-adjust itself
////	#if defined WIN32 || defined _WIN32
////	Sleep(1000);	// (in milliseconds)
////	#endif
////	frame = cvQueryFrame( camera );	// get the first frame, to make sure the camera is initialized.
////	if (frame) {
////	printf("Got a camera using a resolution of %dx%d.\n", (int)cvGetCaptureProperty( camera, CV_CAP_PROP_FRAME_WIDTH), (int)cvGetCaptureProperty( camera, CV_CAP_PROP_FRAME_HEIGHT) );
////	}
////	}
////
////	frame = cvQueryFrame( camera );
////	if (!frame) {
////	fprintf(stderr, "ERROR in recognizeFromCam(): Could not access the camera or video file.\n");
////	exit(1);
////	//return NULL;
////	}
////	return frame;
////}
////
////// Return a new image that is always greyscale, whether the input image was RGB or Greyscale.
////// Remember to free the returned image using cvReleaseImage() when finished.
////IplImage* convertImageToGreyscale(const IplImage *imageSrc)
////{
////	IplImage *imageGrey;
////	// Either convert the image to greyscale, or make a copy of the existing greyscale image.
////	// This is to make sure that the user can always call cvReleaseImage() on the output, whether it was greyscale or not.
////	if (imageSrc->nChannels == 3) {
////	imageGrey = cvCreateImage( cvGetSize(imageSrc), IPL_DEPTH_8U, 1 );
////	cvCvtColor( imageSrc, imageGrey, CV_BGR2GRAY );
////	}
////	else {
////	imageGrey = cvCloneImage(imageSrc);
////	}
////	return imageGrey;
////}
////
////// Creates a new image copy that is of a desired size.
////// Remember to free the new image later.
////IplImage* resizeImage(const IplImage *origImg, int newWidth, int newHeight)
////{
////	IplImage *outImg = 0;
////	int origWidth;
////	int origHeight;
////	if (origImg) {
////	origWidth = origImg->width;
////	origHeight = origImg->height;
////	}
////	if (newWidth <= 0 || newHeight <= 0 || origImg == 0 || origWidth <= 0 || origHeight <= 0) {
////	printf("ERROR in resizeImage: Bad desired image size of %dx%d\n.", newWidth, newHeight);
////	exit(1);
////	}
////
////	// Scale the image to the new dimensions, even if the aspect ratio will be changed.
////	outImg = cvCreateImage(cvSize(newWidth, newHeight), origImg->depth, origImg->nChannels);
////	if (newWidth > origImg->width && newHeight > origImg->height) {
////	// Make the image larger
////	cvResetImageROI((IplImage*)origImg);
////	cvResize(origImg, outImg, CV_INTER_LINEAR);	// CV_INTER_CUBIC or CV_INTER_LINEAR is good for enlarging
////	}
////	else {
////	// Make the image smaller
////	cvResetImageROI((IplImage*)origImg);
////	cvResize(origImg, outImg, CV_INTER_AREA);	// CV_INTER_AREA is good for shrinking / decimation, but bad at enlarging.
////	}
////
////	return outImg;
////}
////
////// Returns a new image that is a cropped version of the original image. 
////IplImage* cropImage(const IplImage *img, const CvRect region)
////{
////	IplImage *imageTmp;
////	IplImage *imageRGB;
////	CvSize size;
////	size.height = img->height;
////	size.width = img->width;
////
////	if (img->depth != IPL_DEPTH_8U) {
////	printf("ERROR in cropImage: Unknown image depth of %d given in cropImage() instead of 8 bits per pixel.\n", img->depth);
////	exit(1);
////	}
////
////	// First create a new (color or greyscale) IPL Image and copy contents of img into it.
////	imageTmp = cvCreateImage(size, IPL_DEPTH_8U, img->nChannels);
////	cvCopy(img, imageTmp, NULL);
////
////	// Create a new image of the detected region
////	// Set region of interest to that surrounding the face
////	cvSetImageROI(imageTmp, region);
////	// Copy region of interest (i.e. face) into a new iplImage (imageRGB) and return it
////	size.width = region.width;
////	size.height = region.height;
////	imageRGB = cvCreateImage(size, IPL_DEPTH_8U, img->nChannels);
////	cvCopy(imageTmp, imageRGB, NULL);	// Copy just the region.
////
////    cvReleaseImage( &imageTmp );
////	return imageRGB;	
////}
////
////// Get an 8-bit equivalent of the 32-bit Float image.
////// Returns a new image, so remember to call 'cvReleaseImage()' on the result.
////IplImage* convertFloatImageToUcharImage(const IplImage *srcImg)
////{
////	IplImage *dstImg = 0;
////	if ((srcImg) && (srcImg->width > 0 && srcImg->height > 0)) {
////
////	// Spread the 32bit floating point pixels to fit within 8bit pixel range.
////	double minVal, maxVal;
////	cvMinMaxLoc(srcImg, &minVal, &maxVal);
////
////	//cout << "FloatImage:(minV=" << minVal << ", maxV=" << maxVal << ")." << endl;
////
////	// Deal with NaN and extreme values, since the DFT seems to give some NaN results.
////	if (cvIsNaN(minVal) || minVal < -1e30)
////	minVal = -1e30;
////	if (cvIsNaN(maxVal) || maxVal > 1e30)
////	maxVal = 1e30;
////	if (maxVal-minVal == 0.0f)
////	maxVal = minVal + 0.001;	// remove potential divide by zero errors.
////
////	// Convert the format
////	dstImg = cvCreateImage(cvSize(srcImg->width, srcImg->height), 8, 1);
////	cvConvertScale(srcImg, dstImg, 255.0 / (maxVal - minVal), - minVal * 255.0 / (maxVal-minVal));
////	}
////	return dstImg;
////}
////
////// Store a greyscale floating-point CvMat image into a BMP/JPG/GIF/PNG image,
////// since cvSaveImage() can only handle 8bit images (not 32bit float images).
////void saveFloatImage(const char *filename, const IplImage *srcImg)
////{
////	//cout << "Saving Float Image '" << filename << "' (" << srcImg->width << "," << srcImg->height << "). " << endl;
////	IplImage *byteImg = convertFloatImageToUcharImage(srcImg);
////	cvSaveImage(filename, byteImg);
////	cvReleaseImage(&byteImg);
////}
////
////// Perform face detection on the input image, using the given Haar cascade classifier.
////// Returns a rectangle for the detected region in the given image.
////CvRect detectFaceInImage(const IplImage *inputImg, const CvHaarClassifierCascade* cascade )
////{
////	const CvSize minFeatureSize = cvSize(20, 20);
////	const int flags = CV_HAAR_FIND_BIGGEST_OBJECT | CV_HAAR_DO_ROUGH_SEARCH;	// Only search for 1 face.
////	const float search_scale_factor = 1.1f;
////	IplImage *detectImg;
////	IplImage *greyImg = 0;
////	CvMemStorage* storage;
////	CvRect rc;
////	double t;
////	CvSeq* rects;
////	int i;
////
////	storage = cvCreateMemStorage(0);
////	cvClearMemStorage( storage );
////
////	// If the image is color, use a greyscale copy of the image.
////	detectImg = (IplImage*)inputImg;	// Assume the input image is to be used.
////	if (inputImg->nChannels > 1) 
////	{
////	greyImg = cvCreateImage(cvSize(inputImg->width, inputImg->height), IPL_DEPTH_8U, 1 );
////	cvCvtColor( inputImg, greyImg, CV_BGR2GRAY );
////	detectImg = greyImg;	// Use the greyscale version as the input.
////	}
////
////	// Detect all the faces.
////	t = (double)cvGetTickCount();
////	rects = cvHaarDetectObjects( detectImg, (CvHaarClassifierCascade*)cascade, storage,
////	search_scale_factor, 3, flags, minFeatureSize );
////	t = (double)cvGetTickCount() - t;
////	printf("[Face Detection took %d ms and found %d objects]\n", cvRound( t/((double)cvGetTickFrequency()*1000.0) ), rects->total );
////
////	// Get the first detected face (the biggest).
////	if (rects->total > 0) {
////        rc = *(CvRect*)cvGetSeqElem( rects, 0 );
////    }
////	else
////	rc = cvRect(-1,-1,-1,-1);	// Couldn't find the face.
////
////	//cvReleaseHaarClassifierCascade( &cascade );
////	//cvReleaseImage( &detectImg );
////	if (greyImg)
////	cvReleaseImage( &greyImg );
////	cvReleaseMemStorage( &storage );
////
////	return rc;	// Return the biggest face found, or (-1,-1,-1,-1).
////}
////
////// Re-train the new face rec database without shutting down.
////// Depending on the number of images in the training set and number of people, it might take 30 seconds or so.
////CvMat* retrainOnline(void)
////{
////	CvMat *trainPersonNumMat;
////	int i;
////
////	// Free & Re-initialize the global variables.
////	if (faceImgArr) {
////	for (i=0; i<nTrainFaces; i++) {
////	if (faceImgArr[i])
////	cvReleaseImage( &faceImgArr[i] );
////	}
////	}
////	cvFree( &faceImgArr ); // array of face images
////	cvFree( &personNumTruthMat ); // array of person numbers
////	personNames.clear();	// array of person names (indexed by the person number). Added by Shervin.
////	nPersons = 0; // the number of people in the training set. Added by Shervin.
////	nTrainFaces = 0; // the number of training images
////	nEigens = 0; // the number of eigenvalues
////	cvReleaseImage( &pAvgTrainImg ); // the average image
////	for (i=0; i<nTrainFaces; i++) {
////	if (eigenVectArr[i])
////	cvReleaseImage( &eigenVectArr[i] );
////	}
////	cvFree( &eigenVectArr ); // eigenvectors
////	cvFree( &eigenValMat ); // eigenvalues
////	cvFree( &projectedTrainFaceMat ); // projected training faces
////
////	// Retrain from the data in the files
////	printf("Retraining with the new person ...\n");
////	learn("train.txt");
////	printf("Done retraining.\n");
////
////	// Load the previously saved training data
////	if( !loadTrainingData( &trainPersonNumMat ) ) {
////	printf("ERROR in recognizeFromCam(): Couldn't load the training data!\n");
////	exit(1);
////	}
////
////	return trainPersonNumMat;
////}
////
////// Continuously recognize the person in the camera.
////void recognizeFromCam(void)
////{
////	int i;
////	CvMat * trainPersonNumMat;  // the person numbers during training
////	float * projectedTestFace;
////	double timeFaceRecognizeStart;
////	double tallyFaceRecognizeTime;
////	CvHaarClassifierCascade* faceCascade;
////	char cstr[256];
////	BOOL saveNextFaces = FALSE;
////	char newPersonName[256];
////	int newPersonFaces;
////
////	trainPersonNumMat = 0;  // the person numbers during training
////	projectedTestFace = 0;
////	saveNextFaces = FALSE;
////	newPersonFaces = 0;
////
////	printf("Recognizing person in the camera ...\n");
////
////	// Load the previously saved training data
////	if( loadTrainingData( &trainPersonNumMat ) ) {
////	faceWidth = pAvgTrainImg->width;
////	faceHeight = pAvgTrainImg->height;
////	}
////	else {
////	//printf("ERROR in recognizeFromCam(): Couldn't load the training data!\n");
////	//exit(1);
////	}
////
////	// Project the test images onto the PCA subspace
////	projectedTestFace = (float *)cvAlloc( nEigens*sizeof(float) );
////
////	// Create a GUI window for the user to see the camera image.
////	cvNamedWindow("Input", CV_WINDOW_AUTOSIZE);
////
////	// Make sure there is a "data" folder, for storing the new person.
////	#if defined WIN32 || defined _WIN32
////	mkdir("data");
////	#else
////	// For Linux, make the folder to be Read-Write-Executable for this user & group but only Readable for others.
////	mkdir("data", S_IRWXU | S_IRWXG | S_IROTH);
////	#endif
////
////	// Load the HaarCascade classifier for face detection.
////	faceCascade = (CvHaarClassifierCascade*)cvLoad(faceCascadeFilename, 0, 0, 0 );
////	if( !faceCascade ) {
////	printf("ERROR in recognizeFromCam(): Could not load Haar cascade Face detection classifier in '%s'.\n", faceCascadeFilename);
////	exit(1);
////	}
////
////	// Tell the Linux terminal to return the 1st keypress instead of waiting for an ENTER key.
////	changeKeyboardMode(1);
////
////	timeFaceRecognizeStart = (double)cvGetTickCount();	// Record the timing.
////
////	while (1)
////	{
////	int iNearest, nearest, truth;
////	IplImage *camImg;
////	IplImage *greyImg;
////	IplImage *faceImg;
////	IplImage *sizedImg;
////	IplImage *equalizedImg;
////	IplImage *processedFaceImg;
////	CvRect faceRect;
////	IplImage *shownImg;
////	int keyPressed = 0;
////	FILE *trainFile;
////	float confidence;
////
////	// Handle non-blocking keyboard input in the console.
////	if (kbhit())
////	keyPressed = getch();
////	
////	if (keyPressed == VK_ESCAPE) {	// Check if the user hit the 'Escape' key
////	break;	// Stop processing input.
////	}
////	switch (keyPressed) {
////	case 'n':	// Add a new person to the training set.
////	// Train from the following images.
////	printf("Enter your name: ");
////	strcpy(newPersonName, "newPerson");
////
////	// Read a string from the console. Waits until they hit ENTER.
////	changeKeyboardMode(0);
////	fgets(newPersonName, sizeof(newPersonName)-1, stdin);
////	changeKeyboardMode(1);
////	// Remove 1 or 2 newline characters if they were appended (eg: Linux).
////	i = strlen(newPersonName);
////	if (i > 0 && (newPersonName[i-1] == 10 || newPersonName[i-1] == 13)) {
////	newPersonName[i-1] = 0;
////	i--;
////	}
////	if (i > 0 && (newPersonName[i-1] == 10 || newPersonName[i-1] == 13)) {
////	newPersonName[i-1] = 0;
////	i--;
////	}
////	
////	if (i > 0) {
////	printf("Collecting all images until you hit 't', to start Training the images as '%s' ...\n", newPersonName);
////	newPersonFaces = 0;	// restart training a new person
////	saveNextFaces = TRUE;
////	}
////	else {
////	printf("Did not get a valid name from you, so will ignore it. Hit 'n' to retry.\n");
////	}
////	break;
////	case 't':	// Start training
////	saveNextFaces = FALSE;	// stop saving next faces.
////	// Store the saved data into the training file.
////	printf("Storing the training data for new person '%s'.\n", newPersonName);
////	// Append the new person to the end of the training data.
////	trainFile = fopen("train.txt", "a");
////	for (i=0; i<newPersonFaces; i++) {
////	snprintf(cstr, sizeof(cstr)-1, "data/%d_%s%d.pgm", nPersons+1, newPersonName, i+1);
////	fprintf(trainFile, "%d %s %s\n", nPersons+1, newPersonName, cstr);
////	}
////	fclose(trainFile);
////
////	// Now there is one more person in the database, ready for retraining.
////	//nPersons++;
////
////	//break;
////	//case 'r':
////
////	// Re-initialize the local data.
////	projectedTestFace = 0;
////	saveNextFaces = FALSE;
////	newPersonFaces = 0;
////
////	// Retrain from the new database without shutting down.
////	// Depending on the number of images in the training set and number of people, it might take 30 seconds or so.
////	cvFree( &trainPersonNumMat );	// Free the previous data before getting new data
////	trainPersonNumMat = retrainOnline();
////	// Project the test images onto the PCA subspace
////	cvFree(&projectedTestFace);	// Free the previous data before getting new data
////	projectedTestFace = (float *)cvAlloc( nEigens*sizeof(float) );
////
////	printf("Recognizing person in the camera ...\n");
////	continue;	// Begin with the next frame.
////	break;
////	}
////
////	// Get the camera frame
////	camImg = getCameraFrame();
////	if (!camImg) {
////	printf("ERROR in recognizeFromCam(): Bad input image!\n");
////	exit(1);
////	}
////	// Make sure the image is greyscale, since the Eigenfaces is only done on greyscale image.
////	greyImg = convertImageToGreyscale(camImg);
////
////	// Perform face detection on the input image, using the given Haar cascade classifier.
////	faceRect = detectFaceInImage(greyImg, faceCascade );
////	// Make sure a valid face was detected.
////	if (faceRect.width > 0) {
////	faceImg = cropImage(greyImg, faceRect);	// Get the detected face image.
////	// Make sure the image is the same dimensions as the training images.
////	sizedImg = resizeImage(faceImg, faceWidth, faceHeight);
////	// Give the image a standard brightness and contrast, in case it was too dark or low contrast.
////	equalizedImg = cvCreateImage(cvGetSize(sizedImg), 8, 1);	// Create an empty greyscale image
////	cvEqualizeHist(sizedImg, equalizedImg);
////	processedFaceImg = equalizedImg;
////	if (!processedFaceImg) {
////	printf("ERROR in recognizeFromCam(): Don't have input image!\n");
////	exit(1);
////	}
////
////	// If the face rec database has been loaded, then try to recognize the person currently detected.
////	if (nEigens > 0) {
////	// project the test image onto the PCA subspace
////	cvEigenDecomposite(
////	processedFaceImg,
////	nEigens,
////	eigenVectArr,
////	0, 0,
////	pAvgTrainImg,
////	projectedTestFace);
////
////	// Check which person it is most likely to be.
////	iNearest = findNearestNeighbor(projectedTestFace, &confidence);
////	nearest  = trainPersonNumMat->data.i[iNearest];
////
////	printf("Most likely person in camera: '%s' (confidence=%f).\n", personNames[nearest-1].c_str(), confidence);
////
////	}//endif nEigens
////
////	// Possibly save the processed face to the training set.
////	if (saveNextFaces) {
////// MAYBE GET IT TO ONLY TRAIN SOME IMAGES ?
////	// Use a different filename each time.
////	snprintf(cstr, sizeof(cstr)-1, "data/%d_%s%d.pgm", nPersons+1, newPersonName, newPersonFaces+1);
////	printf("Storing the current face of '%s' into image '%s'.\n", newPersonName, cstr);
////	cvSaveImage(cstr, processedFaceImg, NULL);
////	newPersonFaces++;
////	}
////
////	// Free the resources used for this frame.
////	cvReleaseImage( &greyImg );
////	cvReleaseImage( &faceImg );
////	cvReleaseImage( &sizedImg );
////	cvReleaseImage( &equalizedImg );
////	}
////
////	// Show the data on the screen.
////	shownImg = cvCloneImage(camImg);
////	if (faceRect.width > 0) {	// Check if a face was detected.
////	// Show the detected face region.
////	cvRectangle(shownImg, cvPoint(faceRect.x, faceRect.y), cvPoint(faceRect.x + faceRect.width-1, faceRect.y + faceRect.height-1), CV_RGB(0,255,0), 1, 8, 0);
////	if (nEigens > 0) {	// Check if the face recognition database is loaded and a person was recognized.
////	// Show the name of the recognized person, overlayed on the image below their face.
////	CvFont font;
////	cvInitFont(&font,CV_FONT_HERSHEY_PLAIN, 1.0, 1.0, 0,1,CV_AA);
////	CvScalar textColor = CV_RGB(0,255,255);	// light blue text
////	char text[256];
////	snprintf(text, sizeof(text)-1, "Name: '%s'", personNames[nearest-1].c_str());
////	cvPutText(shownImg, text, cvPoint(faceRect.x, faceRect.y + faceRect.height + 15), &font, textColor);
////	snprintf(text, sizeof(text)-1, "Confidence: %f", confidence);
////	cvPutText(shownImg, text, cvPoint(faceRect.x, faceRect.y + faceRect.height + 30), &font, textColor);
////	}
////	}
////
////	// Display the image.
////	cvShowImage("Input", shownImg);
////
////	// Give some time for OpenCV to draw the GUI and check if the user has pressed something in the GUI window.
////	keyPressed = cvWaitKey(10);
////	if (keyPressed == VK_ESCAPE) {	// Check if the user hit the 'Escape' key in the GUI window.
////	break;	// Stop processing input.
////	}
////
////	cvReleaseImage( &shownImg );
////	}
////	tallyFaceRecognizeTime = (double)cvGetTickCount() - timeFaceRecognizeStart;
////
////	// Reset the Linux terminal back to the original settings.
////	changeKeyboardMode(0);
////
////	// Free the camera and memory resources used.
////	cvReleaseCapture( &camera );
////	cvReleaseHaarClassifierCascade( &faceCascade );
////}

