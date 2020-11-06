#include<iostream>
#include<string>
#include<vector>
#include<opencv2/core/core.hpp>
#include<opencv2/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/video/tracking.hpp>
#include<opencv2/core/types.hpp>
#include<GLUT/glut.h>
#include<math.h>
#include<typeinfo>


using namespace std;
using namespace cv;


//Define global variables
#define pi 3.1415926
Mat frame,dstImg,tmpImg;
int accum=5;
int size=1;



const float D = 60;           // distance limit
const float A = 10;           // angle limit
const float alpha = 1;
const float beta = 1;
//const float gama = 0.1;


//rotation angle, one time
const float r = 10*pi/180.0f;


//Declare functions
static void on_hough(int,void*);
bool IsCompatible(Vec4i iConst, Vec4i rConst);
float distanceCalc(Point center, Vec4i rConst);
float distanceCalc2(Point icenter, Point rcenter);
float angleCalc(Vec4i iConst, Vec4i rConst);
float cost(Vec4i iConst, Vec4i rConst);
void CallBackFunc(int event, int x, int y, int flags, void* userdata);




struct mouse_control_t
{
        int x = 0;
        int y = 0;
        int t = 0;
}m1;



//struct (x,y),times=t
struct mouse_control

{
        int x = 0;
        int y = 0;
        int t = 0; //how many times to rotate
}m;





int main()
{
system("color 2F");

//open Camera
VideoCapture cap(0);

//wether success open
if(!cap.isOpened())
  {
    cout<<"fail to open！"<<endl;
    return false;
  }


//initialize frame
cap>>frame;
  
namedWindow("houghlines window",1);



 
    
    
   while(true)
  {

   //call-back(Note that the video stream should be placed in the loop, keep the callback refresh)
    on_hough(size,0);

      
setMouseCallback("houghlines window", CallBackFunc, &m);
//vertexes of the rectangle
    Point ipt1 =Point(-150*cos(m.t*r)+100*sin(m.t*r)+m.x,-150*sin(m.t*r)-100*cos(m.t*r)+m.y);
    Point ipt2 =Point(-150*cos(m.t*r)-100*sin(m.t*r)+m.x,-150*sin(m.t*r)+100*cos(m.t*r)+m.y);
    Point ipt3 =Point(150*cos(m.t*r)-100*sin(m.t*r)+m.x,150*sin(m.t*r)+100*cos(m.t*r)+m.y);
    Point ipt4 =Point(150*cos(m.t*r)+100*sin(m.t*r)+m.x,150*sin(m.t*r)-100*cos(m.t*r)+m.y);
    
    
    
    
//draw the 4 lines of rectangle as internal constraints
    line(frame, ipt1, ipt2, Scalar(255,0,0),4,CV_AVX);
    line(frame, ipt2, ipt3, Scalar(255,0,0),4,CV_AVX);
    line(frame, ipt3, ipt4, Scalar(255,0,0),4,CV_AVX);
    line(frame, ipt4, ipt1, Scalar(255,0,0),4,CV_AVX);
      //cout<<ipt4<<ipt1<<endl;


//Display the processed window
    imshow("houghlines window",frame);


//Update image frame
    cap>>frame;
//cout<<"type: "<<typeid(frame).name()<<endl;

//'q' to exit
   if(waitKey(100)==('q')){break;}
  
  }
 return 0;
}
  







static void on_hough(int,void*)
{

//Record number of frames
static int FrameNumber = 1;

Mat grayImg;
tmpImg=frame.clone();

//The start and end of the Hough line
static vector<Vec4i> lines;
  if(frame.channels()==3)
  {
  cvtColor(frame,grayImg,COLOR_BGR2GRAY);
  }
  else
  {
  grayImg=frame;
  }

 

//Gaussian blur, remove details, adjustable neighborhood size
GaussianBlur(grayImg,dstImg,Size(17,17),3,3);

//canny operator to get the edges
Canny(dstImg,dstImg,20,80,3);

if(FrameNumber==1)
{
//Hough line detection, adjustable intersection counter threshold
HoughLinesP(dstImg,lines,1,CV_PI/180,161,20,100);
}

    
    
//Draw the Hough line
  for(size_t i=0;i<lines.size();i++)
  {
  Point pt1=Point(lines[i][0],lines[i][1]);
  Point pt2=Point(lines[i][2],lines[i][3]);
  line(frame,pt1,pt2,Scalar(0,255,0),2,CV_AVX);
  }


//The output shows the number of Hough lines detected in the i-th frame
   //cout<<"No. "<<i<<"frame：\t";
   //cout<<"<check（ "<<lines.size()<<"）houghline>"<<endl;
    
   /* //test which is the No.1 houghline
    Point pt11 = Point(lines[0][0],lines[0][1]);
    Point pt22 = Point(lines[0][2],lines[0][3]);
    line(frame, pt11, pt22, Scalar(0,0,255));*/
    

// store internal constraints with "vector"
Point ipt1 =Point(-150*cos(m.t*r)+100*sin(m.t*r)+m.x,-150*sin(m.t*r)-100*cos(m.t*r)+m.y);
Point ipt2 =Point(-150*cos(m.t*r)-100*sin(m.t*r)+m.x,-150*sin(m.t*r)+100*cos(m.t*r)+m.y);
Point ipt3 =Point(150*cos(m.t*r)-100*sin(m.t*r)+m.x,150*sin(m.t*r)+100*cos(m.t*r)+m.y);
Point ipt4 =Point(150*cos(m.t*r)+100*sin(m.t*r)+m.x,150*sin(m.t*r)-100*cos(m.t*r)+m.y);
    //cout<<"ipt4 = "<<ipt1  <<ipt2  <<ipt3  <<ipt4<<endl;


//store the edges of rectangle
vector<Vec4i> inter_lines;
    inter_lines.push_back(Vec4i(ipt1.x,ipt1.y,ipt2.x,ipt2.y));
    inter_lines.push_back(Vec4i(ipt2.x,ipt2.y,ipt3.x,ipt3.y));
    inter_lines.push_back(Vec4i(ipt3.x,ipt3.y,ipt4.x,ipt4.y));
    inter_lines.push_back(Vec4i(ipt4.x,ipt4.y,ipt1.x,ipt1.y));
    //cout<<"XXXXX:  "<<inter_lines[2]<<endl;
    


    
float currentCost = 0;
static int bestRealConstraint = -1;
static int bestInterConstraint= -1;
float minCost = 10000;
if(m.t != m1.t || m.x != m1.x || m.y != m1.y ||FrameNumber==1)
{
m1.x = m.x;
m1.y = m.y;
m1.t = m.t;
//SnapToRealConstraint
    for (size_t j=0; j<inter_lines.size(); j++)
  {
       for (size_t i=0; i<lines.size(); i++)    // Real constraints are installed in lines
    {
        if (IsCompatible(lines[i],inter_lines[j]))
            {
             currentCost = cost(lines[i],inter_lines[j]);
             //cout<<"currentCost"<<currentCost<<endl;
              if (currentCost < minCost)
               {
                  bestRealConstraint  = i;
                  //cout<<"i=  "<<i<<endl;
                  bestInterConstraint = j;
                  //cout<<"Best.j=  "<<j<<endl;
                  minCost = currentCost;
               }
             }
    }
  }
    //cout<<bestRealConstraint<<endl;
    //cout<<bestInterConstraint<<endl;
}
    
    if (bestRealConstraint > -1 && bestInterConstraint >-1){
    //cout<<bestInterConstraint
    Point brpt1=Point(lines[bestRealConstraint][0],lines[bestRealConstraint][1]);
    Point brpt2=Point(lines[bestRealConstraint][2],lines[bestRealConstraint][3]);
    Point bipt1=Point(inter_lines[bestInterConstraint][0],inter_lines[bestInterConstraint][1]);
    Point bipt2=Point(inter_lines[bestInterConstraint][2],inter_lines[bestInterConstraint][3]);
    //cout<<inter_lines.size()<<endl;
    
    //test
    line(frame, brpt1, brpt2, Scalar(0,0,255),3,CV_AVX);
    //line(frame, bipt1, bipt2, Scalar(255,255,0),6,CV_AVX);
    //cout<<"bipt1 ="<<bipt1<<endl;
    //cout<<"bipt2 ="<<bipt2<<endl;
    
    
    float x_i = brpt2.x - brpt1.x;
    float y_i = brpt2.y - brpt1.y;
    float x_j = bipt2.x - bipt1.x;
    float y_j = bipt2.y - bipt1.y;
    
        
    // angle
    double cos_alpha = (x_i*x_j+ y_i*y_j)/(sqrt(x_i*x_i+y_i*y_i)*sqrt(x_j*x_j+y_j*y_j));
    float alpha = acos(cos_alpha);
    //cout<<alpha<<endl;
    
    //calculate increment of x,y
    Point biConst_center = Point((bipt1.x + bipt2.x)/2, (bipt1.y+bipt2.y)/2);
    Point brConst_center = Point((brpt1.x + brpt2.x)/2, (brpt1.y+brpt2.y)/2);
    float IRx = brConst_center.x - biConst_center.x;
    float IRy = brConst_center.y - biConst_center.y;
    
    
    
   
//transform
    Point rectpt1 =Point(-150*cos(m.t*r+alpha)+100*sin(m.t*r+alpha)+m.x+IRx,-150*sin(m.t*r+alpha)-100*cos(m.t*r+alpha)+m1.y+IRy);
    Point rectpt2 =Point(-150*cos(m.t*r+alpha)-100*sin(m.t*r+alpha)+m.x+IRx,-150*sin(m.t*r+alpha)+100*cos(m.t*r+alpha)+m1.y+IRy);
    Point rectpt3 =Point(150*cos(m.t*r+alpha)-100*sin(m.t*r+alpha)+m.x+IRx,150*sin(m.t*r+alpha)+100*cos(m.t*r+alpha)+m1.y+IRy);
    Point rectpt4 =Point(150*cos(m.t*r+alpha)+100*sin(m.t*r+alpha)+m.x+IRx,150*sin(m.t*r+alpha)-100*cos(m.t*r+alpha)+m1.y+IRy);
    
    line(frame, rectpt1, rectpt2, Scalar(0,0,255),3,CV_AVX);
    line(frame, rectpt2, rectpt3, Scalar(0,0,255),3,CV_AVX);
    line(frame, rectpt3, rectpt4, Scalar(0,0,255),3,CV_AVX);
    line(frame, rectpt4, rectpt1, Scalar(0,0,255),3,CV_AVX);
    }
FrameNumber++;//Increase the number of frames
//cout<<"No.Best："<<bestRealConstraint<<endl;
}








bool IsCompatible(Vec4i iConst, Vec4i rConst)
{
    Point iConst_center = Point((iConst[0] + iConst[2])/2, (iConst[1]+iConst[3])/2);
    
    // calculate the centriod of internal Constraint
    if ((distanceCalc(iConst_center, rConst)) > D)
    {
        return false;
    }
    if (angleCalc(iConst, rConst) > A)
    {
        return false;
    }
    //cout<<"distance smaller than 0    "<<distanceCalc(iConst_center, rConst)<<endl;
    return true;
}

float cost(Vec4i iConst, Vec4i rConst)
{
    Point iConst_center = Point((iConst[0] + iConst[2])/2, (iConst[1]+iConst[3])/2);
    //cout<<"iConst_center   "<<iConst_center<<endl;
    Point rConst_center = Point((rConst[0] + rConst[2])/2, (rConst[1]+rConst[3])/2);
    
    float c = 0.0;   // init cost as 0
    c = alpha * distanceCalc(iConst_center, rConst);
    c+= beta * distanceCalc2(iConst_center, rConst_center);
    //cout<<"c  ="<<c<<endl;
    return c;
}

//calc the distance between Point and line
float distanceCalc(Point center, Vec4i rConst)
{
    Point rcenter = Point((rConst[0] + rConst[2])/2, (rConst[1]+rConst[3])/2);
    //cout<<rConst[0]<<rConst[2]<<rConst[1]<<rConst[3]<<endl;
    float dis = (center.x - rcenter.x)*(center.x - rcenter.x) + (center.y - rcenter.y)*(center.y - rcenter.y);
    dis = sqrt(dis);
    //cout<<"Dis:    "<<dis<<endl;
    return dis;
}

//the distance between Point and Line
float distanceCalc2(Point icenter, Point rcenter)
{
    float dis = (icenter.x - rcenter.x)*(icenter.x - rcenter.x) + (icenter.y - rcenter.y)*(icenter.y - rcenter.y);
    dis = sqrt(dis);
    //cout<<"Dis2:   "<<dis<<endl;
    return dis;
}

//calc the angle between 2 lines
float angleCalc(Vec4i iConst, Vec4i rConst)
{
    float x1 = iConst[2] - iConst[0];
    float y1 = iConst[3] - iConst[1];
    float x2 = rConst[2] - rConst[0];
    float y2 = rConst[3] - rConst[1];
    
    double cos_theta = (x1*x2+ y1*y2)/(sqrt(x1*x1+y1*y1)*sqrt(x2*x2+y2*y2));
    float theta = acos(cos_theta)*180.0/pi;
    if (theta > 90.0f){
        theta = 180 -theta;
    }
    //cout<<"theta = "<<theta<<endl;
    //cout<<"acos = "<<theta<<endl;
    return theta;
}


//Mouse_control position detection
void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
   
     if  ( event == EVENT_LBUTTONDOWN )
     {
         
         struct mouse_control*p = (struct mouse_control*) userdata;
         p->x = x;
         p->y = y;
        //cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
          
     }
     else if  ( event == EVENT_RBUTTONDOWN)
     {
         struct mouse_control*p = (struct mouse_control*) userdata;
         //p->x = x;
         //p->y = y;
         p->t += 1;  //or m.t += 1
         //cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
     }
     else if  ( event == EVENT_MBUTTONDOWN )
     {
          //cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
     }
     else if ( event == EVENT_MOUSEMOVE )
     {
       
         cout << "Mouse move over the window - position (" << x << ", " << y << ")" << endl;
     }
}


