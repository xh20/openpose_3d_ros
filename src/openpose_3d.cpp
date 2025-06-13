//
// Created by hao on 03.12.19.
//
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <openpose_ros_msgs/OpenPoseHuman.h>
#include <openpose_ros_msgs/OpenPoseHumanList.h>
#include <iostream>
#include <bits/stdc++.h>
#include <bits/stl_algo.h>
#include <eigen3/Eigen/Dense>

using namespace openpose_ros_msgs;
using namespace std;
using namespace cv;

class OpenPoseDepth
{
public:
    OpenPoseDepth()
    {
        camera_matrix << 610.59228515625, 0.0, 314.3261413574219,  //   fx 0 cx
                        0.0, 610.5518798828125, 248.59820556640625, //  0 fy cy
                        0.0, 0.0, 1.0;                               // 0  0  1

        camera_matrix_inv = camera_matrix.inverse();
        normal_vec << 0.134222, 0.620703, 0.772472;
        camera_height = 1410;
        pub_ = nh_.advertise<openpose_ros_msgs::OpenPoseHumanList>("/openpose_ros/depth_human_list", 10);
        depth_image.subscribe(nh_, "/camera/depth_registered/image_raw", 1);
        human_list.subscribe(nh_, "/openpose_ros/human_list", 1);
        sync.reset(new Sync(sync_pol(10),depth_image,human_list));
        sync->registerCallback(boost::bind(&OpenPoseDepth::callback,this,_1,_2));
    }

    long int index = 0;
    void callback(const sensor_msgs::ImageConstPtr &msgD, const openpose_ros_msgs::OpenPoseHumanListConstPtr &human_list_msg)
    {
        cv_bridge::CvImageConstPtr cv_ptrD;
        cv_bridge::CvImageConstPtr cv_ptrD_save;
        try
        {
            cv_ptrD = cv_bridge::toCvShare(msgD, sensor_msgs::image_encodings::TYPE_16UC1);
            cv_ptrD_save = cv_bridge::toCvShare(msgD, sensor_msgs::image_encodings::TYPE_8UC1);
//	        string time = std::to_string(ros::Time::now().toSec());
            string time = std::to_string(index);
	        string name = "/home/hao/openpose_image/MDepth-"+time+".png";
	        index ++ ;
            imwrite(name, cv_ptrD_save->image);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        openpose_ros_msgs::OpenPoseHumanList human_list_3d_msg;
        human_list_3d_msg.header.stamp = ros::Time::now();
        double duration = ros::Time::now().toSec() - time_old;

        time_old = ros::Time::now().toSec();
        human_list_3d_msg.image_header = human_list_msg->header;
        human_list_3d_msg.num_humans = human_list_msg->human_list.size();
        std::vector<openpose_ros_msgs::OpenPoseHuman> human_list(human_list_msg->human_list.size());
        if(human_list_msg->human_list.size()>0){
            for(int person = 0; person<human_list_msg->human_list.size();person++){
                openpose_ros_msgs::OpenPoseHuman human = human_list_msg->human_list.at(person);
                int counter = 0;
                vector<double> x_head;
                vector<double> y_head;
                vector<double> Z_body;
                for(int bodyPart=0; bodyPart<human.body_key_points_with_prob.size(); bodyPart++){
                    openpose_ros_msgs::PointWithProb body_point_with_prob = human.body_key_points_with_prob.at(bodyPart);
                    double p = body_point_with_prob.prob;
                    if(p>0){
//                        if(bodyPart==0|| bodyPart==15||bodyPart==16||bodyPart==17||bodyPart==18){
//                            x_head.push_back(body_point_with_prob.x);
//                            y_head.push_back(body_point_with_prob.y);
//                        }
//                        else if(bodyPart!=19 && bodyPart!=20 && bodyPart!=21 && bodyPart!=22 && bodyPart!=23 && bodyPart!=24){
//
//                        }
//                        else{
//                            int Z_depth = cv_ptrD->image.at<u_int16_t>(body_point_with_prob.y,body_point_with_prob.x);;
//                            Eigen::Vector3d body_pixel(body_point_with_prob.x,body_point_with_prob.y,1);
//                            Eigen::Vector3d body_c = Z_depth*camera_matrix_inv*body_pixel;
//                            Z_body.push_back(camera_height - body_c.transpose()*normal_vec);
//                        }
//                    // calculate all keypoints' depth
//
		      Eigen::Vector3d pixel_vec(body_point_with_prob.x,body_point_with_prob.y,1);                                        
                      int x = int(body_point_with_prob.x);
                      int y = int(body_point_with_prob.y);
                      // double Z = cv_ptrD->image.at<u_int16_t>(y,x)/1000.0; // unit M: meter
                      // cv::Vec3d point_c = Z*OpenPoseDepth::camera_matrix*pixel_vec; // unit M
                      int Z_depth = cv_ptrD->image.at<u_int16_t>(y,x);;
                      Eigen::Vector3d body_pixel(body_point_with_prob.x,body_point_with_prob.y,1);
                      Eigen::Vector3d body_3d = Z_depth*camera_matrix_inv*body_pixel;
                      human.body_key_points_with_prob.at(bodyPart).x = body_3d(0);
                      human.body_key_points_with_prob.at(bodyPart).y = body_3d(1);
                      human.body_key_points_with_prob.at(bodyPart).z = Z; // unit M: meter
                    }
                }
//                double Z_body_min = *min_element(Z_body.begin(),Z_body.end());
//                double x_head_min = *min_element(x_head.begin(),x_head.end());
//                double x_head_max = *max_element(x_head.begin(),x_head.end());
//                double y_head_min = *min_element(y_head.begin(),y_head.end());
//                double y_head_max = *max_element(y_head.begin(),y_head.end());
//                int x_head_cen = (x_head_min + x_head_max) / 2;
//                int y_head_cen = (y_head_min + y_head_max) / 2;
//                int Z_head_cen = cv_ptrD->image.at<u_int16_t>(y_head_cen,x_head_cen);
                //                cout<<"x_pixel_center: "<< x_head_cen << endl;
//                cout<<"y_pixel_center: "<< y_head_cen << endl;
//                cout<<"Z_center: "<< Z_head_cen << endl;

//                for(int x_i = x_head_min+1; x_i<x_head_max; x_i++){
//                    for(int y_i=y_head_min+1; y_i<y_head_max; y_i++){
//                        z_vec.push_back(cv_ptrD->image.at<u_int16_t>(y_i,x_i));
//                    }
//                }
//                double Z_avg = accumulate(z_vec.begin(), z_vec.end(), 0.0)/z_vec.size();
//                double sq_sum = std::inner_product(z_vec.begin(), z_vec.end(), z_vec.begin(), 0.0);
//                double stdev = std::sqrt(sq_sum / z_vec.size() - Z_avg * Z_avg);
//                cout<< "Z_avg early: "<<Z_avg<<endl;

//                z_vec.erase(remove_if(z_vec.begin(), z_vec.end(), bind(greater<int>(), _1, Z_avg+2*stdev)), z_vec.end());
//                z_vec.erase(remove_if(z_vec.begin(), z_vec.end(), bind(less<int>(), _1, Z_avg-2*stdev)), z_vec.end());
//                Z_avg = accumulate(z_vec.begin(), z_vec.end(), 0.0)/z_vec.size();
                //                cout<< "Z_avg new: "<<Z_avg<<endl;

//                Eigen::Vector3d head_pixel(x_head_cen, y_head_cen ,1.0);

//                Head_p = Z_head_cen*camera_matrix_inv*head_pixel; // point vector in camera coordinate, unit mm

//                cout<<"head position: \n"<< Head_p<<endl;
//                human.body_key_points_with_prob.at(23).x = Head_p(0);
//                human.body_key_points_with_prob.at(23).y = Head_p(1);
//                human.body_key_points_with_prob.at(23).z = Head_p(2);
                human_list.at(person) = human;
//                head_height = camera_height - Head_p.transpose()*normal_vec;
//                if((Z_body_min<50 || head_height<50)){
//                    if(velocity_abs>1000)
//                        cout<<"Someone Fall Down"<<endl;
//                }
//                else{
//                    distance = Head_p - Head_p_old;
//                    velocity = distance/duration;
//                    velocity_abs = velocity.norm();
//                    cout<<"velocity_abs"<<velocity.norm()<<endl;
//                }
//                Head_p_old = Head_p;
            }
        }
        human_list_3d_msg.human_list = human_list;
        pub_.publish(human_list_3d_msg);
    }


private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    message_filters::Subscriber<sensor_msgs::Image> depth_image;
    message_filters::Subscriber<openpose_ros_msgs::OpenPoseHumanList> human_list;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, openpose_ros_msgs::OpenPoseHumanList> sync_pol;
    typedef message_filters::Synchronizer<sync_pol> Sync;
    boost::shared_ptr<Sync> sync;
    Eigen::Matrix3d camera_matrix, camera_matrix_inv;
    Eigen::Vector3d normal_vec, Head_p, velocity, distance, Head_p_old;
    double time_old, head_height, camera_height, velocity_abs;

};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "openpose_3d");
    ros::start();
    OpenPoseDepth openpose_3d_obj;
    ros::spin();
    ros::shutdown();

    return 0;
}
