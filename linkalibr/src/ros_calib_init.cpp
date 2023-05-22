//
// Created by usl on 3/5/21.
//

/*
1. 필요한 헤더 파일과 라이브러리를 포함시킵니다.

2. ROS 노드를 초기화합니다.

3. ROS 퍼블리셔를 설정합니다.

4. 필요한 ROS 토픽 (IMU 및 LIDAR)을 설정합니다.

5. ROS 매개 변수를 통해 ROS BAG 파일에서 로드할 데이터의 위치 및 기간을 설정합니다.

6. ROS BAG 파일을 로드하고, 설정된 기간 동안 데이터를 읽습니다.

7. LIDAR 오도메트리 객체를 만듭니다.

8. ROS BAG 파일에서 데이터를 읽으며, IMU 및 LIDAR 데이터를 처리하여 IMU 패킷 및 로봇의 위치 추정 결과를 발행합니다.

9. ROS 노드를 종료합니다.


 

*/



#define PCL_NO_PRECOMPILE // !! BEFORE ANY PCL INCLUDE!!

#include "ros/ros.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>

#include "utils/quat_ops.h"
#include "utils/pcl_utils.h"

#include  <iostream>
#include "track/lidarOdometry.h"

// #include <imuPacket/imuPacket.h>
#include <imu_packet/imu_packet.h>

int main(int argc, char** argv) {
    /// Launch ros node
    ros::init(argc, argv, "ros_pair_lodom_imu");
    ros::NodeHandle nh("~");
    

    ROS_INFO("aaa");

    ros::Publisher imu_packet_publisher = nh.advertise<imu_packet::imu_packet>("/imu_packet", 1);
    ros::Publisher lodom_publisher = nh.advertise<geometry_msgs::PoseStamped>("/lidar_odometry", 1);

    /// Our topics (IMU and LIDAR)
    std::string topic_imu;
    std::string topic_lidar;
    double ndt_resolution;
    std::string lo_trajectory_filename;
    std::string imu_data_filename;
    nh.param<std::string>("topic_imu", topic_imu, "/imu");
    nh.param<std::string>("topic_lidar", topic_lidar, "/lidar");
    nh.param<double>("ndt_resolution", ndt_resolution, 0.5);

    ROS_INFO_STREAM("topic_imu: " << topic_imu);
    ROS_INFO_STREAM("topic_lidar: " << topic_lidar);
    ROS_INFO_STREAM("ndt_resolution: " << ndt_resolution);

    /// Get our start location and how much of the bag we want to play
    /// Make the bag duration < 0 to just process to the end of the bag
    double bag_start, bag_durr;
    nh.param<double>("bag_start", bag_start, 0);
    nh.param<double>("bag_durr", bag_durr, -1);
    ROS_INFO_STREAM("bag start: " << bag_start);
    ROS_INFO_STREAM("bag duration: " << bag_durr);

    /// Location of the ROS bag we want to read in
    std::string path_to_bag;
    nh.param<std::string>("path_bag", path_to_bag, "/home/coui/catkin_ws/bags/2021-04-16-10-32-05_far.bag");
    ROS_INFO_STREAM("ROS BAG PATH is: " << path_to_bag.c_str());

    /// Load rosbag here, and find messages we can play
    rosbag::Bag bag;
    bag.open(path_to_bag, rosbag::bagmode::Read);

    /// We should load the bag as a view
    /// Here we go from beginning of the bag to the end of the bag
    rosbag::View view_full;
    rosbag::View view;

    /// Start a few seconds in from the full view time
    /// If we have a negative duration then use the full bag length
    view_full.addQuery(bag);
    ros::Time time_init = view_full.getBeginTime();
    time_init += ros::Duration(bag_start);
    ros::Time time_finish =
            (bag_durr < 0) ? view_full.getEndTime() : time_init + ros::Duration(bag_durr);
    ROS_INFO_STREAM("Time start = " << time_init.toSec());
    ROS_INFO_STREAM("Time end = " << time_finish.toSec());
    view.addQuery(bag, time_init, time_finish);
    /// Check to make sure we have data to play
    if (view.size() == 0) {
        ROS_ERROR_STREAM("No messages to play on specified topics.  Exiting.");
        ros::shutdown();
        return EXIT_FAILURE;
    }

    

    // ROS_INFO_STREAM("ndt resolution: ", ndt_resolution);
    /// Lidar Odometry object (Tracker)

    // Lidar Odometry 를 위한 객체 생성: lin_core::LidarOdometry --> NormalDistributionsTransform --> VoxelGridCovariance & ...
    lin_core::LidarOdometry::Ptr LOdom; 
    LOdom = std::make_shared<lin_core::LidarOdometry>(ndt_resolution, "", true);    // ndt_resolution 을 인수로 전달하여 초기화
    imu_packet::imu_packet imupacket;     // IMU 값 저장을 위한 객체 생성

    for (const rosbag::MessageInstance& m : view) {     // rosbag 의 view 값을 가져옴
        /// If ROS wants us to stop, break out  
        if(!ros::ok()){
            ROS_INFO("break");
            break;
        }
            
        /// Handle IMU measurement
        sensor_msgs::Imu::ConstPtr s_imu = m.instantiate<sensor_msgs::Imu>();       // rosbag 으로부터 현재 메시지의 타입이 sensor_msgs::Imu인지 확인하고, 그렇다면 s_imu 변수에 저장
        if (s_imu != nullptr && m.getTopic() == topic_imu) {        // rosbag에 imu 메세지가 있는지 확인
            imupacket.stamps.push_back(s_imu->header.stamp);        
            geometry_msgs::Vector3 accelReading;
            accelReading.x = s_imu->linear_acceleration.x;
            accelReading.y = s_imu->linear_acceleration.y;
            accelReading.z = s_imu->linear_acceleration.z;
            imupacket.accelreadings.push_back(accelReading);
            geometry_msgs::Vector3 gyroReading;
            gyroReading.x = s_imu->angular_velocity.x;
            gyroReading.y = s_imu->angular_velocity.y;
            gyroReading.z = s_imu->angular_velocity.z;
            imupacket.gyroreadings.push_back(gyroReading);          // IMU 의 gyro accel 값을 imupacket 에 저장
        }

        

        /// Handle Lidar measurement
        sensor_msgs::PointCloud2::ConstPtr s_lidar = m.instantiate<sensor_msgs::PointCloud2>();
    
        if (s_lidar != nullptr && m.getTopic() == topic_lidar) {
            // 라이다 데이터를 VPointCloud 형식으로 저장
            lin_core::VPointCloud::Ptr cloud_pcl(new lin_core::VPointCloud);
            pcl::fromROSMsg(*s_lidar, *cloud_pcl);
            

            // // odometry 기반의 지도 업데이트: key scan 만 업데이트 하고 NDT 알고리즘을 돌림
            LOdom->feedScan((*s_lidar).header.stamp.toSec(), cloud_pcl);
            LOdom->append_and_update(true);



            // 기존 odom 과 현재 odom 의 변환행렬(lastestRP)의 상단 3x3 부분을 추출하여 deltaR_L 에 저장
            Eigen::Matrix3d deltaR_L = LOdom->get_latest_relativePose().odometry_ij.block(0, 0, 3, 3);

            



            // 쿼터니언으로 변환
            Eigen::Quaterniond delta_qL(deltaR_L);

            // ros의 pose 메세지 형태로 쿼터니언과 lidar 스캔의 현재 시간정보를 저장하여 publish -> /lidar_odometry 토픽
            geometry_msgs::PoseStamped pose;
            pose.header.stamp = s_lidar->header.stamp;
            pose.pose.orientation.x = delta_qL.x();
            pose.pose.orientation.y = delta_qL.y();
            pose.pose.orientation.z = delta_qL.z();
            pose.pose.orientation.w = delta_qL.w();
            lodom_publisher.publish(pose);

            // IMU 정보를 publish -> /imu_packet 토픽
            // IMU와 Lidar의 정보를 동시에 Publish
            imupacket.header.stamp = s_lidar->header.stamp;
            imu_packet_publisher.publish(imupacket);

            // IMU 패킷 초기화
            imupacket.stamps.clear();
            imupacket.accelreadings.clear();
            imupacket.gyroreadings.clear();
        }
    }
    return EXIT_SUCCESS;        
    // EQUIRED process [topic_publisher-2] has died! process has finished cleanly 라는 문구가 뜸
}