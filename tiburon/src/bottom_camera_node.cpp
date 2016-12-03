
// Front and Bottom Camera combined

// Uses a priority list for setting yaw
// |Bottom Yaw setting|
// |Front Yaw setting|
// |Reference Yaw setting|

// Bottom is top priority - Gives only directions of orange lines
// Bottom should update the reference yaw if it persists long enough

// Front will have a separate priority list - which  will take the various objects found and take the most prior one
// Front priority list should change depending on currently passes checkpoints

// A fixed value of speed is to be used
// The fixed value may be multiplied by a non zero heuristic - upto 30% minimum


#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <std_msgs/Float64.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/UInt16.h>
#include <geometry_msgs/Vector3.h>
#include <tiburon/ins_data.h>
#include <ctime>

using namespace std;
using namespace cv;

double cc_angle(Point vtx,Point p1,Point p12);
Point findCenter(vector<Point> contour);
double abs_distance(Point2i p1,Point2i p2);
int findLargestContour(vector<vector<Point> > contours);
double cc_angle(Point2i vtx,Point2i p2);
double globalYawDecision(double yaw1,double yaw2);
void frontImageCallback(const sensor_msgs::ImageConstPtr& msg);
void bottomImageCallback(const sensor_msgs::ImageConstPtr& msg);
//void origImageCallback(const sensor_msgs::ImageConstPtr& msg);
void insCallback(const tiburon::ins_data::ConstPtr& ypr);
void stopCallback(const std_msgs::UInt16::ConstPtr& stopf);

double getBottomYaw(Mat bottomCameraImage);
double getLineDirection(vector<Point> contour);
double getFrontYaw(Mat segImage);




double deg2rad(double);
double rad2deg(double);


#define USELESS -1
#define TOOCLOSE -10
#define MOVETOCENTER 10
#define BOTTOMCROP 0.6
#define FRONTCROP 1.0
#define OUTERCIRCLE 0.36
#define OUTER 100
#define INNERCIRCLE 0.2 // Radius = cols*innercircle
#define INNER 50

#define LINENOTFOUND -1
#define LINETOOFAR 1
#define DISTANCEPID 2
#define ANGLEPID 3

#define FRONTCAMFOV 78


double globalYaw = 133.0;
double currentYaw = 100.0; // #Todo Remove this
double yawUndeadband = 2.5;
double speedfactor = 4.0;
double frontareaLow = 0.0,frontareaHigh = 0.0;
double bottomareaLow = 0.0,bottomareaHigh = 0.0;

Mat orig,frontdisp,bottomdisp;
Mat frontIm,bottomIm;
vector<Point> largestContour;

bool frontImageReceived = false, bottomImageReceived = false;
int stopflag = 1;

// #Notes - Three outputs are possible - Angle PID - Will have 2 subparts - Small and Big angle
//                                      Distance PID - Line is between outer and inner circle
//                                      Move to Line Center - Line is out of outer circle


// #Notes Return Values - Set Yaw, Set Speed, Flag
// flag states - line not found , move to center(set yaw open loop, go little front open loop), forward distance pid,


int main(int argc, char **argv)
{
    ros::init(argc, argv, "BottomCameraNode");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber bottomImageSub = it.subscribe("/bottomCameraSegmented",1,bottomImageCallback);
    image_transport::Subscriber frontImageSub = it.subscribe("/frontCameraSegmented",1,frontImageCallback);
    //image_transport::Subscriber origImageSub = it.subscribe("/auv_cam1",1,origImageCallback);
    image_transport::Publisher frontPub = it.advertise("/frontImage", 1);
    image_transport::Publisher bottomPub = it.advertise("/bottomImage", 1);
    ros::Subscriber ins = nh.subscribe("/tiburon/ins_data",1,insCallback);
    ros::Publisher yawPub = nh.advertise<std_msgs::Float64>("/yawSP",1);
    ros::Publisher velPub = nh.advertise<std_msgs::Float64>("/velSP",1);
    ros::Subscriber stopflag = nh.subscribe("/stopflag",1,stopCallback);
    sensor_msgs::ImagePtr msg;
    std_msgs::Float64 f64msg;
    double setFinalYaw = currentYaw;
    double setFinalSpeed = 0;
    double yawTargets[3] ={globalYaw,globalYaw,globalYaw};
    //bottomIm = Mat::zeros(Size(640,480),CV_8UC1);
    double bottomAvg = USELESS,frontAvg = USELESS;
    double bottomMax = USELESS,frontMax = USELESS;
    double bottomMin = USELESS,frontMin = USELESS;
    double bottomframecount = 0,frontframecount = 0;
    double frontlatchtimer,frontlatchvalue;
    while(1)
    {
      ros::spinOnce();
      if(!bottomImageReceived || !frontImageReceived)
         continue;
    double bottomAvg = USELESS,frontAvg = USELESS;

      medianBlur(frontIm,frontIm,7);
      dilate(frontIm,frontIm,getStructuringElement(MORPH_RECT,Size(7,7)));
      Point frontcenter = Point(frontIm.cols/2,frontIm.rows/2);
      Size frontimsize = frontIm.size();
   // double imdiag = abs_distance(Point2i(0,0),Point2i(frontIm.cols,frontIm.rows));
      frontareaLow = 1.0*frontimsize.width*frontimsize.height*FRONTCROP/100.0;
      frontareaHigh = 40.0*frontimsize.width*frontimsize.height*FRONTCROP/100.0;
      frontdisp = Mat::zeros(frontimsize, CV_8UC3);

      medianBlur(bottomIm,bottomIm,7);
      Point bottomcenter = Point(bottomIm.cols/2,bottomIm.rows/2);
      Size bottomimsize = bottomIm.size();
//      double imdiag = abs_distance(Point2i(0,0),Point2i(bottomIm.cols,bottomIm.rows));
      bottomareaHigh = 33.0*bottomimsize.width*bottomimsize.height*BOTTOMCROP/100.0;
      bottomareaLow = 5.0*bottomimsize.width*bottomimsize.height*BOTTOMCROP/100.0;
      bottomdisp = Mat::zeros(bottomimsize, CV_8UC3);

      Mat frontsegLine = frontIm;
      Mat bottomseg = bottomIm;

      double frontcamLineYaw = getFrontYaw(frontsegLine);
      // double frontcamObjectYaw = getFrontYaw(frontsegObject);
      double bottomcamYaw = getBottomYaw(bottomseg);
      yawTargets[0] = bottomcamYaw;
      //Todo - select between object and line
      yawTargets[1] = frontcamLineYaw;
      cout << "Yawtargets array " << yawTargets[0] << "\t" << yawTargets[1] << "\t" << yawTargets[2] << endl;
            //Todo - if bottomcamerayaw value persists - the globalyaw should be updated
      if(yawTargets[0] != USELESS)
      {
            if(bottomframecount == 0)
            {
               bottomAvg = yawTargets[0];
               bottomMax = yawTargets[0];
               bottomMin = yawTargets[0];
            }
            else
            {
               bottomAvg = (bottomAvg*(bottomframecount)+yawTargets[0])/(bottomframecount+1); // running avg
               bottomMin = bottomMin<yawTargets[0]?bottomMin:yawTargets[0];
               bottomMax = bottomMax>yawTargets[0]?bottomMax:yawTargets[0];
            }
           bottomframecount++;
           frontframecount = 0;
           setFinalYaw = yawTargets[0];
      }
      else if(yawTargets[1] != USELESS)
      {

               if(frontframecount == 0)
               {
                  frontAvg = yawTargets[1];
                  frontMax = yawTargets[1];
                  frontMin = yawTargets[1];
               }
               else
               {
                  frontAvg = (frontAvg*(frontframecount)+yawTargets[1])/(frontframecount+1); // running avg
                  frontMin = frontMin<yawTargets[1]?frontMin:yawTargets[1];
                  frontMax = frontMax>yawTargets[1]?frontMax:yawTargets[1];
               }
            frontframecount++;
            bottomframecount=0;
           setFinalYaw = yawTargets[1];
      }
      else
      {
         if(frontframecount>10 && abs(rad2deg(atan2(sin(deg2rad(frontMin-frontMax)),cos(deg2rad(frontMin-frontMax)))))<10.0)
         {
            frontlatchtimer = clock();
            frontlatchvalue = frontAvg;
         }
         if(bottomframecount>10 && abs(rad2deg(atan2(sin(deg2rad(bottomMin - bottomMax)),cos(deg2rad(bottomMin - bottomMax)))))<10.0)
         {
            globalYaw = bottomAvg;
            cout << "global yaw set here " << endl;
            yawTargets[2] = globalYaw;
         }
         else if((clock()-frontlatchtimer)/CLOCKS_PER_SEC<5.0) // Change the time if needed
            yawTargets[2] = frontlatchvalue;
         else
            yawTargets[2] = globalYaw;
         setFinalYaw = yawTargets[2];
         frontframecount=0;
         bottomframecount=0;
      }
      //Todo - set velocity heuristic
      setFinalSpeed = 3.0;

         f64msg.data= setFinalSpeed;
         velPub.publish(f64msg);
         f64msg.data = setFinalYaw;
         yawPub.publish(f64msg);

        cout << "Yaw " << setFinalYaw << endl;
        cout << "Speed " << setFinalSpeed << endl;
//        cout << "Flag " << statusflag << endl;

        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frontdisp).toImageMsg();
        frontPub.publish(msg);
        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", bottomdisp).toImageMsg();
        bottomPub.publish(msg);
        waitKey(1);

    }
    return 0;
}
double getBottomYaw(Mat bottomCameraImage)
{
    vector<vector<Point> > contours;
    Mat bottomCrop = bottomCameraImage(Rect(0,0,bottomCameraImage.cols,bottomCameraImage.rows*BOTTOMCROP)); // Crop the top part , TOPCROP % from top corner
    findContours(bottomCrop.clone(),contours,CV_RETR_TREE,CV_CHAIN_APPROX_NONE);
    int largest = findLargestContour(contours);
    if(largest == -1)
    {
        cout << "Bottom nothing detected" << endl;
        return USELESS;
    }
    double currarea = contourArea(contours[largest]);
    if(currarea < bottomareaLow || currarea > bottomareaHigh) // arealow and areahigh are global
    {
          cout << "Bottom out of area limits" << endl;
          return USELESS;
    }
    drawContours(bottomdisp,contours,largest,Scalar(255,255,255),-1);

    circle(bottomdisp,Point(bottomCameraImage.cols/2,bottomCameraImage.rows/2),3,Scalar(255,255,0),-1);
    Point centerOfLine = findCenter(contours[largest]);
    largestContour = contours[largest];
    circle(bottomdisp,centerOfLine,3,Scalar(0,255,0),-1);
    double botToLineDistance = abs_distance(centerOfLine,Point(bottomCameraImage.cols/2,bottomCameraImage.rows/2));
    if(botToLineDistance > bottomCameraImage.cols*OUTERCIRCLE)
        return USELESS;
    else
    {
        double linedirection = getLineDirection(largestContour); // this line direction is according to the image
        double setFinalYaw = 90.0+currentYaw-linedirection;
        setFinalYaw = rad2deg(atan2(sin(deg2rad(setFinalYaw)),cos(deg2rad(setFinalYaw))));
        return setFinalYaw;
    }
}
double getLineDirection(vector<Point> contour)
{
    double targetyaw;
    vector<Point> approxContour;
    approxPolyDP(contour,approxContour,10,true); // works wonders but this epsilon how much to set? - 10 works wonders
    vector<double> angles;
    int iter = 0;
    while(approxContour.size()!=4 && iter <=10)
    {
        iter++;
        approxPolyDP(contour,approxContour,10+iter*2,true);
    }
    if(approxContour.size()!=4)
        return USELESS; // have to check in the function call
    double diffangle;

    vector<vector<Point> > polys;
    polys.push_back(approxContour);
    drawContours(bottomdisp,polys,0,Scalar(0,0,255),2);
    for(int i= 0;i<approxContour.size();i++)
    {
        if(i==approxContour.size()-1)
            diffangle = cc_angle(approxContour[i],approxContour[0]);
        else
            diffangle = cc_angle(approxContour[i],approxContour[i+1]);
        angles.push_back(rad2deg(diffangle));
        cout << "a " << angles[i] << endl;
    }
    double angleset1 = abs(angles[0]-angles[2]),angleset2 = abs(angles[1]-angles[3]);
    if(abs(angleset1-180.0)<15 && abs(angleset2-180.0)<15)
    {
        // use the closer angle to the globalyaw
        double targetyaw1 = globalYawDecision(angles[0],angles[2]);
        double targetyaw2 = globalYawDecision(angles[1],angles[3]);
        targetyaw = globalYawDecision(targetyaw1,targetyaw2);
    }
    else
    {
        //targetyaw=(abs(angleset1-180)<abs(angleset2-180))?((angles[0]+180.0-angles[2])/):((angles[1]+180.0-angles[2])/2);
        targetyaw=(abs(angleset1-180)<abs(angleset2-180))?angles[0]:angles[1]; // TODO: do the averaging properly
        if(targetyaw<=0.0)
            targetyaw=globalYawDecision(targetyaw,180+targetyaw);
        else
            targetyaw=globalYawDecision(targetyaw,targetyaw-180);

        // use the closer angle to the globalyaw
    }
    // approxpolydp will always give points in circular pattern so angles will be separated by 2 if sides = 4
    return targetyaw;
}
double getFrontYaw(Mat segImage)
{
    vector<vector<Point> > contours;
    // Todo - fix the correct crop size
    Mat frontCrop = segImage(Rect(0,segImage.rows*0.5,segImage.cols,segImage.rows*0.5)); // Crop the top part , TOPCROP % from top corner
    findContours(frontCrop.clone(),contours,CV_RETR_TREE,CV_CHAIN_APPROX_NONE);
    int largest = findLargestContour(contours);
    if(largest == -1)
    {
        cout << "Front nothing detected" << endl;
        return USELESS;
    }
    double currarea = contourArea(contours[largest]);
    if(currarea < frontareaLow || currarea > frontareaHigh) // arealow and areahigh are global
    {
          cout << "Front out of area limits" << endl;
          return USELESS;
    }
    drawContours(frontdisp,contours,largest,Scalar(255,255,255),-1);
    double tanthetamax = tan(deg2rad(FRONTCAMFOV/2));
    Point center = findCenter(contours[largest]);
    circle(frontdisp,center,3,Scalar(0,0,255),-1);
    double diffx = center.x - frontCrop.cols/2;
    cout << "diffx " << diffx  << " frontcropcolsby2 " << frontCrop.cols/2;
    double tantheta = (tanthetamax*diffx)/(frontCrop.cols/2);

    double theta = rad2deg(atan(tantheta));
    cout<< "thm " << tanthetamax << " th " << theta << " tantheta " << tantheta << endl;
    //Todo check the direction of the theta with yaw
    double frontYaw = currentYaw+theta;
    return frontYaw;
}
double globalYawDecision(double yaw1,double yaw2) // angles should be in degrees
{
    double globalangle = currentYaw-globalYaw+90;
    double dy1 = yaw1 - globalangle;
    double dy2 = yaw2 - globalangle;
    dy1 = atan2(sin(3.1415/180.0*dy1),cos(3.1415/180.0*dy1))*180.0/3.1415;
    dy2 = atan2(sin(3.1415/180.0*dy2),cos(3.1415/180.0*dy2))*180.0/3.1415;
    //cout<<"Select: "<<yaw1<<" "<<yaw2<< ((abs(dy1)<abs(dy2))?yaw1:yaw2)<<endl;
    return (abs(dy1)<abs(dy2))?yaw1:yaw2;
}
double rad2deg(double radval)
{
    return 180.0*radval/3.1415;
}
double deg2rad(double degval)
{
    return 3.1415*degval/180.0;
}
double cc_angle(Point vtx,Point p1,Point p2)
{
// 1st point is the vertex , then 1st point and 2nd point,angle is obtained in radians
    double modv1 = abs_distance(vtx,p1);		// Mod v1
    double modv2 = abs_distance(vtx,p2);		// Mod v2
    double dot = (p1.x-vtx.x)*(p2.x-vtx.x) + (p1.y-vtx.y)*(p2.y-vtx.y);
    double costheta = dot/(modv1*modv2);
    double sign =0;
    double term = (p1.x-vtx.x)*(p2.y-vtx.y)-(p1.y-vtx.y)*(p2.x-vtx.x);
    if(term <= 0)
        sign = 1.0;
    else
        sign = -1.0;
    double theta = 0;
    if(costheta >=1.0)
        theta = 0;
    else if(costheta <=-1.0)
        theta = acos(-1.0);
    else
        theta = acos(costheta);
    return theta*sign;
}
Point findCenter(vector<Point> contour)
{
    int count  = 0;
    Point2f center = Point2f(0,0);
    for(int i = 0;i < contour.size();i++)
    {
        center.x = (center.x*count+contour[i].x)/(count+1);
        center.y = (center.y*count+contour[i].y)/(count+1);
        count++;
    }
    return center;
}
double abs_distance(Point2i p1,Point2i p2)
{
    //Calculates euclidian distance
    double sqterm = pow(p1.x-p2.x,2.0)+pow(p1.y-p2.y,2.0);
    double dist = pow(sqterm,0.5);
    return dist;
}
int findLargestContour(vector<vector<Point> > contours)
{
    int areaMax = 0;//segm.rows*segm.cols;
    int maxindex = -1;
    for(int j = 0;j<contours.size();j++)
    {
        double currarea =  contourArea(contours[j]);
        if(areaMax < currarea)
        {
            maxindex = j;
            areaMax = currarea;
        }
    }
    return maxindex;
}
double cc_angle(Point2i vtx,Point2i p2)
{
    Point p1 = Point((vtx.x+30),vtx.y);
    return cc_angle(vtx,p1,p2);
}
void insCallback(const tiburon::ins_data::ConstPtr& ypr)
{
    currentYaw = ypr->YPR.x;
}
void stopCallback(const std_msgs::UInt16::ConstPtr& stopf)
{
   stopflag = 1 - stopflag;
}

void bottomImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
   try
   {
     bottomIm = (cv_bridge::toCvShare(msg, "mono8")->image).clone(); // Don't remove this clone call, else it will not finish copying the whole image and overwrite it prematurely

   }
   catch (cv_bridge::Exception& e)
   {
     ROS_ERROR("Could not convert from '%s' to 'mono8'.", msg->encoding.c_str());
   }
   bottomImageReceived = true;
}
void frontImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
   try
   {
     frontIm = (cv_bridge::toCvShare(msg, "mono8")->image).clone(); // Don't remove this clone call, else it will not finish copying the whole image and overwrite it prematurely

   }
   catch (cv_bridge::Exception& e)
   {
     ROS_ERROR("Could not convert from '%s' to 'mono8'.", msg->encoding.c_str());
   }
   frontImageReceived = true;
}
// void origImageCallback(const sensor_msgs::ImageConstPtr& msg)
// {
//     cout<<"orig"<<endl;
//    try
//    {
//      orig = cv_bridge::toCvShare(msg, "bgr8")->image;
//      waitKey(100);
//    }
//    catch (cv_bridge::Exception& e)
//    {
//      ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
//    }
//    origImageReceived = true;
//}
