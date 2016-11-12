#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <std_msgs/Float64.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Vector3.h>
#include <tiburon/ins_data.h>

using namespace std;
using namespace cv;

int findTargetPoint(Mat,Point&);
double cc_angle(Point vtx,Point p1,Point p12);
Point findCenter(vector<Point> contour);
double abs_distance(Point2i p1,Point2i p2);
int findLargestContour(vector<vector<Point> > contours);
double cc_angle(Point2i vtx,Point2i p2);
double getTargetYaw(vector<Point> contour);
double globalYawDecision(double yaw1,double yaw2);
void segmentedImageCallback(const sensor_msgs::ImageConstPtr& msg);
//void origImageCallback(const sensor_msgs::ImageConstPtr& msg);
void insCallback(const tiburon::ins_data::ConstPtr& ypr);


#define USELESS -1
#define TOOCLOSE -10
#define MOVETOCENTER 10
#define TOPCROP 0.6 // To go forward - we should only consider going forward
#define OUTERCIRCLE 0.36
#define OUTER 100
#define INNERCIRCLE 0.2 // Radius = cols*innercircle
#define INNER 50

#define LINENOTFOUND -1
#define LINETOOFAR 1
#define DISTANCEPID 2
#define ANGLEPID 3

double globalYaw = 133.0;
double currentYaw = 0.0; // #Todo Remove this

Mat orig,disp;
Mat origGr;
vector<Point> largestContour;

bool origImageReceived = false, segmentedImageReceived = false;

// #Notes - Three outputs are possible - Angle PID - Will have 2 subparts - Small and Big angle
//                                      Distance PID - Line is between outer and inner circle
//                                      Move to Line Center - Line is out of outer circle


// #Notes Return Values - Set Yaw, Set Speed, Flag
// flag states - line not found , move to center(set yaw open loop, go little front open loop), forward distance pid,

// TODO: insCallback, segmentedImageCallback, origImageCallback

int main(int argc, char **argv)
{
    ros::init(argc, argv, "BottomCameraNode");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber segmentedImageSub = it.subscribe("/bottomCameraSegmented",1,segmentedImageCallback);
    //image_transport::Subscriber origImageSub = it.subscribe("/auv_cam1",1,origImageCallback);
    image_transport::Publisher dispPub = it.advertise("/debugImage", 1);
    ros::Subscriber ins = nh.subscribe("/tiburon/ins_data",1,insCallback);
    ros::Publisher yawPub = nh.advertise<std_msgs::Float64>("/yawSP",1);
    ros::Publisher velPub = nh.advertise<std_msgs::Float64>("/velSP",1);
    sensor_msgs::ImagePtr msg;
    std_msgs::Float64 f64msg;
    while(1)
    {
        //cout<<"Enter"<<endl;
        ros::spinOnce();
        if(!segmentedImageReceived)
            continue;
        cout<<"NC"<<endl;
        imshow("gr",origGr);
        Point botcenter = Point(origGr.cols/2,origGr.rows/2);
        Size imsize = origGr.size();
        //waitKey(0);
        double imdiag = abs_distance(Point2i(0,0),Point2i(origGr.cols,origGr.rows));
        disp = Mat::zeros(imsize, CV_8UC3);
        char arr[50];
        //sprintf(arr,"/home/codestation/garbage/%d.bmp",rand()%20);
        //cout<<arr<<endl;
        //imwrite(arr,origGr);
        Point target = Point(1,0);
        double yawTarget=globalYaw,speedTarget=0.0,speednormalized = 0.0;
        bool statusflag = LINENOTFOUND;
        int targetStatus =  findTargetPoint(origGr,target);

        if(targetStatus == MOVETOCENTER)
        {
            // Set yaw, speed = 0, open loop =1, line not found flag = 0
            yawTarget = cc_angle(botcenter,target)*180.0/3.1415; // the angle between the camera center and the target
            speedTarget = abs_distance(botcenter,target);
            statusflag = LINETOOFAR;
            circle(disp,target,30,Scalar(255,255,0),-1);
        }
        else if(targetStatus == USELESS)
        {
            // set line not found =1
            speedTarget = 0.0;
            yawTarget = 90;
            statusflag = LINENOTFOUND;
        }
        else if(targetStatus == OUTER)
        {
            // TODO: why y component
            // Set yaw=currentyaw, speed = y component of distance, open loop =0, line not found flag = 0
            yawTarget = 90;
            speedTarget = abs(target.y-botcenter.y);
            statusflag = DISTANCEPID;
        }
        else if(targetStatus == INNER)
        {
            // Set yaw=currentyaw, speed = y component of distance, open loop =0, line not found flag = 0
            yawTarget = getTargetYaw(largestContour);
            cout << "yawtarget" <<  yawTarget << endl;
            // #Todo if yaw is out of limit ->  speed = 0 else speed = f(|yaw|)*basespeed
            statusflag = ANGLEPID;
        }
        speednormalized = speedTarget/imdiag;
        // TODO: are the next few lines okay?
        // All the angles are with respect to the x axis has to be converted to y axis
        double yawYaxis = 90 - yawTarget;
        // Set the final yaw angle - final yaw will be atan2 of currentyaw+yawTarget
        double setFinalYaw = currentYaw+yawYaxis;
        setFinalYaw = atan2(sin(setFinalYaw*3.1415/180.0),cos(setFinalYaw*3.1415/180.0))*180.0/3.1415;
        //# TODO - set the speed - which will be some factor of speednormalized
        // Publish values
        if(targetStatus == MOVETOCENTER)
        {
            if(abs(currentYaw-setFinalYaw)>2.5) // TODO what should be angle?
            {
                f64msg.data= 0;
                velPub.publish(f64msg);
                f64msg.data = setFinalYaw;
                yawPub.publish(f64msg);
                // ros::spinOnce();
            }
            // TODO: what to do about forward?
        }
        else if(targetStatus == USELESS)
        {
            f64msg.data = 0.0;
            velPub.publish(f64msg);
            ros::spinOnce();
        }
        else if(targetStatus == OUTER)
        {
            f64msg.data = speednormalized*4.0;
            velPub.publish(f64msg); // TODO: should be more I guess
            ros::spinOnce();
        }
        else if(targetStatus == INNER)
        {
            if(abs(currentYaw-setFinalYaw)>1.0) // TODO what should be angle?
            {
                f64msg.data= 0;
                velPub.publish(f64msg);
                f64msg.data = setFinalYaw;
                yawPub.publish(f64msg);
                //ros::spinOnce();
            }
            // TODO: Forward till the end of time or next POI, whichever is nearer
        }
        cout << "Yaw " << setFinalYaw << endl;
        cout << "Speed " << speednormalized << endl;
        cout << "Flag " << statusflag << endl;

        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", disp).toImageMsg();
        dispPub.publish(msg);
        //imshow("aa",disp);
        waitKey(10);

    }
    return 0;
}
int findTargetPoint(Mat bottomCameraImage,Point &targetPoint) //Pass segmented and noise filtered
{
    cout<<"Enter1"<<endl;
    vector<vector<Point> > contours;
    Mat topCrop = bottomCameraImage(Rect(0,0,bottomCameraImage.cols,bottomCameraImage.rows*TOPCROP)); // Crop the top part , TOPCROP % from top corner
    cout<<"size"<<topCrop.size()<<endl;
    findContours(topCrop.clone(),contours,CV_RETR_TREE,CV_CHAIN_APPROX_NONE);
    //waitKey(0);
    int largest = findLargestContour(contours);
    if(largest == -1)
    {
        cout << "No line found" << endl;

        return USELESS;
    }
    drawContours(disp,contours,largest,Scalar(0,0,255),3);
    circle(disp,Point(bottomCameraImage.cols/2,bottomCameraImage.rows/2),7,Scalar(255,255,0),-1);
    Point centerOfLine = findCenter(contours[largest]);
    largestContour = contours[largest];
    circle(disp,centerOfLine,3,Scalar(0,255,0),-1);
    double botToLineDistance = abs_distance(centerOfLine,Point(bottomCameraImage.cols/2,bottomCameraImage.rows/2));
    if(botToLineDistance > bottomCameraImage.cols*OUTERCIRCLE)
    {

       targetPoint = centerOfLine;
       cout << "Move to center";
       return MOVETOCENTER;
    }
    else if(botToLineDistance > bottomCameraImage.cols*INNERCIRCLE)
    {
        cout << "Apply distance PID " << endl;
        return OUTER;
   }
   else
   {
        cout << "Apply angle PID "  << endl;
        return INNER;
   }
   cout<<"Exit1"<<endl;
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
    cout<<"Enter2"<<endl;
    int areaMax = 0;//segm.rows*segm.cols;
    int maxindex = -1;
    for(int j = 0;j<contours.size();j++)
    {
        if(areaMax < contourArea(contours[j]))
        {
            maxindex = j;
            areaMax = contourArea(contours[j]);
        }
    }
    cout<<"Exit2"<<endl;
    return maxindex;
}
double cc_angle(Point2i vtx,Point2i p2)
{
    Point p1 = Point((vtx.x+30),vtx.y);
    return cc_angle(vtx,p1,p2);
}
double getTargetYaw(vector<Point> contour)
{
    cout<<"Enter3"<<endl;
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
    for(int i= 0;i<approxContour.size();i++)
    {
       if(i==approxContour.size()-1)
            diffangle = cc_angle(approxContour[i],approxContour[0]);
       else
            diffangle = cc_angle(approxContour[i],approxContour[i+1]);
       angles.push_back(diffangle*180/3.1415);
       cout << "a " << angles[i] << endl;
    }
    double angleset1 = abs(angles[0]-angles[2]),angleset2 = abs(angles[1]-angles[3]);
    if(abs(angleset1-180.0)<15 && abs(angleset2-180.0)<15)
    {
     // use the closer angle to the globalyaw
       double targetyaw1 = globalYawDecision(angles[0],angles[2]);
       double targetyaw2 = globalYawDecision(angles[1],angles[3]);
       targetyaw = globalYawDecision(targetyaw1,targetyaw2);
//       cout << targetyaw1 << " " << targetyaw2 << " decision made " << targetyaw <<endl;
    }
    else
    {
        targetyaw=(abs(angleset1-180)<abs(angleset2-180))?((angles[0]+180.0-angles[2])/2):((angles[1]+180.0-angles[2])/2);
        targetyaw=globalYawDecision(targetyaw,180-targetyaw);
     // use the closer angle to the globalyaw
    }
    // approxpolydp will always give points in circular pattern so angles will be separated by 2 if sides = 4
    cout<<"Exit3"<<endl;
   return targetyaw;
}
double globalYawDecision(double yaw1,double yaw2) // angles should be in degrees
{
    cout<<"Enter4"<<endl;
    double dy1 = yaw1 - globalYaw;
    double dy2 = yaw2 - globalYaw;
    dy1 = atan2(sin(3.1415/180.0*dy1),cos(3.1415/180.0*dy1))*180.0/3.1415;
    dy2 = atan2(sin(3.1415/180.0*dy2),cos(3.1415/180.0*dy2))*180.0/3.1415;
    cout<<"Exit4"<<endl;
    return (abs(dy1)<abs(dy2))?yaw1:yaw2;
}

void insCallback(const tiburon::ins_data::ConstPtr& ypr)
{
    currentYaw = ypr->YPR.x;
}

void segmentedImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    //cout<<"seg"<<endl;
   try
   {
     origGr = (cv_bridge::toCvShare(msg, "mono8")->image).clone(); // Don't remove this clone call, else it will not finish copying the whole image and overwrite it prematurely
     
     //waitKey(100);
   }
   catch (cv_bridge::Exception& e)
   {
     ROS_ERROR("Could not convert from '%s' to 'mono8'.", msg->encoding.c_str());
   }
   segmentedImageReceived = true;
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
