#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <tiburon/ins_data.h>

using namespace std;
using namespace cv;

double yaw = 0.0;

double abs_distance(Point2i p1,Point2i p2)
{
    //Calculates euclidian distance
    double sqterm = pow(p1.x-p2.x,2.0)+pow(p1.y-p2.y,2.0);
    double dist = pow(sqterm,0.5);
    return dist;
}

double cc_angle(Point2i vtx,Point2i p1,Point2i p2)
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

double cc_angle(Point2i vtx,Point2i p2)
{
    Point p1 = Point((vtx.x+30),vtx.y);
    return cc_angle(vtx,p1,p2);
}

void yaw_callback(const tiburon::ins_data::ConstPtr& ypr_val)
{
    yaw = ypr_val->YPR.x;
}

//CHECK!!
void camera_callback(const sensor_msgs::ImageConstPtr& msg)
{
    //http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    //http://wiki.ros.org/image_transport/Tutorials/SubscribingToImages
    try
    {
        cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
        cv::waitKey(30);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }

}

int main(int argc, char **argv)
{
    namedWindow("Orig",CV_WINDOW_NORMAL);
    resizeWindow("Orig",432,324);
    moveWindow("Orig",0,0);
    namedWindow("Output",CV_WINDOW_NORMAL);
    resizeWindow("Output",432,324);
    moveWindow("Output",0,400);

    ros::init(argc, argv, "bottom_camera_node");
    ros::NodeHandle nh;
    ros::Subscriber camera_sub = nh.subscribe("/auv_camera",1,camera_callback); //dooooo
    image_transport::Subscriber yaw_sub = nh.subscribe<tiburon::ins_data>("/tiburon/ins_data",1,yaw_callback);

    Mat orig,hsv,thresh;
    orig = imread("/home/r2d2/savedim1.jpg");
    if(orig.empty())
    {
        cout << "Image not read" << endl;
        return 0;
    }

    ros::Rate loop_rate(10); //doooooooooooo

    while(ros::ok())
    {
        Mat origdraw = orig.clone();
        cvtColor(orig,hsv,CV_BGR2HSV);
        Scalar colorlow = Scalar(14,122,93),colorhigh = Scalar(100,255,255);
        inRange(hsv,colorlow,colorhigh,thresh);
        bitwise_not(thresh,thresh);
        //invert(thresh,thresh);
        //Rect crop(100,100,540,380);
        //orig = orig(crop);
        vector<vector<Point> > contours;
        findContours(thresh.clone(), contours, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);
        Mat newim = Mat::zeros(orig.size(),CV_8UC3);
        //Mat edges; Canny(orig,edges,10,255); imshow("Orig",orig); imshow("Output",edges); waitKey(0); return 0;
        int maxindex = 0;
        if(contours.size()>0)
        {
            int maxarea = contourArea(contours[0]);

            for(int i=1;i<contours.size();i++)
            {
                int area = contourArea(contours[i]);
                if(maxarea < area)
                {
                    maxindex =i;
                    maxarea = area;
                }
            }
        }
        else
        {
            cout << "No contour found " << endl;
            return 0;
        }
        cout << "index " << maxindex << endl;
        cout << contours.size() << endl;
        //for(int i=0;i<contours.size();i++)
        vector<vector<Point> > polys;
        vector<Point> approxContour;
        approxPolyDP(contours[maxindex],approxContour,10,true); // works wonders but this epsilon how much to set? - 10 works wonders
        //Notes - Have to check whether returned polygon is of size 4
        //        Which 2 sides to check for angles? - Length of side is not ok to check
        //
        polys.push_back(approxContour);
        cout << "approx polygon\n" << approxContour << endl;
        drawContours(newim,polys,0,Scalar(0,255,255));
        drawContours(origdraw,polys,0,Scalar(0,255,255));

        vector<double> angles;
        double diffangle;
        cout << "angles ";
        for(int i= 0;i<approxContour.size();i++)
        {
            switch(i)
            {
                case 0: cv::putText(newim,"1",approxContour[i],cv::FONT_HERSHEY_COMPLEX_SMALL,1,cv::Scalar(255, 255, 255)); break;
                case 1: cv::putText(newim,"2",approxContour[i],cv::FONT_HERSHEY_COMPLEX_SMALL,1,cv::Scalar(255, 255, 255)); break;
                case 2: cv::putText(newim,"3",approxContour[i],cv::FONT_HERSHEY_COMPLEX_SMALL,1,cv::Scalar(255, 255, 255)); break;
                case 3: cv::putText(newim,"4",approxContour[i],cv::FONT_HERSHEY_COMPLEX_SMALL,1,cv::Scalar(255, 255, 255)); break;
            }
            if(i==approxContour.size()-1)
                diffangle = cc_angle(approxContour[i],approxContour[0]);
            else
                diffangle = cc_angle(approxContour[i],approxContour[i+1]);
            //diffangle = atan2(sin(diffangle),cos(diffangle));
            angles.push_back(diffangle*180/3.1415);
            cout << angles[i] << "\t";
        }
        cout << endl;
        cout << -angles[0]+angles[2] << endl;
    
        loop_rate.sleep();
    }
    

    //cout << "angles\t" << angles << endl;

// //This is not very nice ... le me no likes.
 //   drawContours(newim,contours,maxindex,Scalar::all(255));
 //   Rect bound = boundingRect(contours[maxindex]);
 //   cout << bound << endl;
//    int count = 0;
//    //double noiseDist = line.cols/80;
//    vector<double> linecent;
//    Point cent;

//    for(int i = bound.y+1;i<bound.y+bound.height-1;i++)
//    {
//        for(int j=bound.x;j<bound.x+bound.width;j++)
//        {
//            if(newim.at<Vec3b>(Point(j,i))[0])
//            {
//                if(count==0)
//                {
//                    cent = Point(j,i);
//                    count++;
//                    continue;
//                }
//            //	/*else if(dst(cent,Point(j,i)) < noiseDist)
//            //	{
//            //		cent.x = (cent.x+j)/(cnt+1);
//            //	}
//                else
//                {
//                    cent.x = (cent.x*count+j)/(count+1);
//                    count++;
//                }
//            }
//        }
//        circle(newim,cent,2,Scalar(0,0,255));
//        //cout << cent << endl;  imshow("Orig",orig);  imshow("Output",newim);   waitKey(0);
//        if(count>1)
//        linecent.push_back(cent.x);
//        cent = Point(0,0);
//        count = 0;
//    }
//    int grad = 0;
//    for (int i = 0; i < (linecent.size() - 2); i++)
//    {
//        grad += (linecent[i + 1] - linecent[i]);
//       // cout <<"grad = "<< grad << endl;
//        //cout << linecent[i] << endl;
//    }
//    cout <<"grad = "<< grad << endl;
    imshow("Orig",origdraw);
    imshow("Output",newim);
    waitKey(0);

    ros::spin();

    return 0;

}

