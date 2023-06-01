//
// Created by usl on 12/10/20.
//

#include "lidarOdometry.h"

namespace lin_core {

    LidarOdometry::LidarOdometry(double ndt_resolution, std::string lo_trajectory_filename, bool downsample_for_mapping)
            : map_cloud_(new VPointCloud()), scan_in_target_global_(new VPointCloud()) {
        ndt_omp_ = ndtInit(ndt_resolution);     // 0.25
        latestRP.odometry_ij = Eigen::Matrix4d::Identity();
        trajfile_csv.open(lo_trajectory_filename);
        downsampleForMapping = downsample_for_mapping;  // true
    }

    pclomp::NormalDistributionsTransform<VPoint, VPoint>::Ptr  
    LidarOdometry::ndtInit(double ndt_resolution) {
        auto ndt_omp = pclomp::NormalDistributionsTransform<VPoint, VPoint>::Ptr
                (new pclomp::NormalDistributionsTransform<VPoint, VPoint>());
        ndt_omp->setResolution(ndt_resolution);
        ndt_omp->setNumThreads(16);
        ndt_omp->setNeighborhoodSearchMethod(pclomp::KDTREE);
        ndt_omp->setTransformationEpsilon(1e-3);
        ndt_omp->setStepSize(0.01);
        ndt_omp->setMaximumIterations(50);
        return ndt_omp;
    }

    void LidarOdometry::feedScan(double timestamp,
                                  VPointCloud::Ptr cur_scan,
                                 Eigen::Matrix4d pose_predict) {
        odom_curr.timestamp = timestamp;
        odom_curr.pose = Eigen::Matrix4d::Identity();   //초기값 0


        // 현재 스캔 데이터를 타겟으로 registration 과정을 거쳐 변환된 포인트 클라우드를 저장
        // 이미 저장된 맵 포인트 클라우드가 없을 경우에는, 현재 스캔 데이터를 타겟으로 지정하고, registration을 거치지 않고 그대로 사용
        // 그 외에는 이전 스캔 데이터의 포즈 예측값과 현재 스캔 데이터를 이용하여 registration 과정을 수행하여 이전스캔과 현재스캔 사이의 변환행렬을 구함
        // 스캔데이터의 차이(변환행렬)가 odom 데이터에 저장되고 odom 데이터의 차이가 최종적으로 구하는 변환행렬임
        pcl::PointCloud<pcl::PointXYZI>::Ptr scan_in_target(new pcl::PointCloud<pcl::PointXYZI>());
        current_scan = *cur_scan;
        if(map_cloud_->empty()) {
            scan_in_target = cur_scan;
        } 
        else {
            Eigen::Matrix4d T_LM_predict = odom_.back().pose*pose_predict;
            registration(cur_scan, T_LM_predict, odom_curr.pose, scan_in_target);
        }


        // 첫 스캔의 경우, 현재 위치(odom_curr)을 저장하고 update
        if(first_scan) {
            odom_.push_back(odom_curr);

            // updateKeyScan: Keyscan은 지도에 추가할 필요가 있는 스캔으로, 이전 스캔과 비교하여 판단. 
            updateKeyScan(current_scan, odom_curr);    
            first_scan = false;
        } 
        else {        
            // 첫 스캔이 아닌경우, 이전 스탭의 odom 과 비교하여 상대적인 위치의 변환을 행렬(latestRP)로 저장한다 
            size_t lastIdx = odom_.size() - 1;
            Odom odom_i = odom_[lastIdx];
            Odom odom_j = odom_curr;
            latestRP.timestamp_i = odom_i.timestamp;
            Eigen::Matrix4d w_T_i = odom_i.pose;
            latestRP.timestamp_j = odom_j.timestamp;
            Eigen::Matrix4d w_T_j = odom_j.pose;
            Eigen::Matrix4d i_T_j = w_T_i.inverse()*w_T_j;
            latestRP.odometry_ij = i_T_j;   
            // std::cout << "RP: " << std::endl;
        }
    }

    void LidarOdometry::append_and_update(bool update_map) {
        Eigen::Matrix3d R_curr = odom_curr.pose.block(0, 0, 3, 3);
        Eigen::Quaterniond quat_curr(R_curr);
        trajfile_csv << odom_curr.timestamp << "," << quat_curr.x() << "," << quat_curr.y() << "," << quat_curr.z() << ","<< quat_curr.w() << ","
                     << odom_curr.pose(0, 3) << "," << odom_curr.pose(1, 3) << ","<< odom_curr.pose(2, 3) << std::endl;

        odom_.push_back(odom_curr);

        if(update_map) {
            updateKeyScan(current_scan, odom_curr);
        }

    }

    // registration: 이전 스캔과 현재스캔 사이의 변환행렬(pose_out) 추정하기위해 NDT 알고리즘 사용 
    void LidarOdometry::registration(const VPointCloud::Ptr& cur_scan,
                                     const Eigen::Matrix4d& pose_predict,
                                     Eigen::Matrix4d& pose_out,
                                     VPointCloud::Ptr scan_in_target) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr p_filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);

        // 다운 샘플링
        downsampleCloud(cur_scan, p_filtered_cloud, 0.1);



        // NDT 알고리즘 수행
        ndt_omp_->setInputSource(p_filtered_cloud);
        ndt_omp_->align(*scan_in_target, pose_predict.cast<float>());
        pose_out = ndt_omp_->getFinalTransformation().cast<double>();
    }

    void LidarOdometry::updateKeyScan(const VPointCloud cur_scan,
                                      const Odom& odom) {
                                        
        //checkKeyScan: 현재 스캔이 keyscan 인지 확인                               
        if(checkKeyScan(odom)) {
            VPointCloud ::Ptr filtered_cloud(new VPointCloud());
            VPointCloud::Ptr input_scan(new VPointCloud);
            *input_scan = cur_scan;

            // 다운샘플링: 포인트 클라우드 수를 줄여 연산량을 줄임
            if(downsampleForMapping) {  
                downsampleCloud(input_scan, filtered_cloud, 0.1);
            } else {
                *filtered_cloud = *input_scan;
            }

            VPointCloud::Ptr scan_in_target(new VPointCloud ());

            // odom.pose 변환행렬을 사용하여 filtered_cloud 를 변환시켜 scan_in_target에 저장
            pcl::transformPointCloud(*filtered_cloud, *scan_in_target, odom.pose);

            
            
            scan_in_target_global_->clear();
            *scan_in_target_global_ = *scan_in_target;
            

            // // 지도에 추가
            *map_cloud_ += *scan_in_target;


            ndt_omp_->setInputTarget(map_cloud_); 

            // odom_.size: 객체수를 반환
            // key_frame_index_ 벡터에는 키프레임이 발생하는 시점의 odom 정보의 인덱스가 저장되게 됨
            // 이를 통해 추후에 해당 시점의 odom 정보와 매칭되는 지점군을 선택하여 지도에 추가할 수 있음
            key_frame_index_.push_back(odom_.size());
        }
    }


    bool LidarOdometry::checkKeyScan(const Odom &odom) {    
        /* 현재 odom 과 이전에 저장된 odom 을 비교하여 판단
        우선 이전 odom 정보와 현재 odom 정보 간의 거리(dist)를 계산하여 이 거리가 0.2m보다 크면 키 프레임으로 결정합니다. 
        이후 현재 odom 정보에서 yaw, pitch, roll 값을 추출하여 이전 yaw, pitch, roll 값과 비교합니다. 
        yaw, pitch, roll 값 중 하나라도 5.0도 이상 변했다면 이 역시 키 프레임으로 결정합니다.
        만약 이전 odom 정보와 거리가 0.2m보다 가깝고, yaw, pitch, roll 값도 5.0도 미만으로 변했다면
         이는 키 프레임이 아니므로 false를 반환합니다,        
        */

        static Eigen::Vector3d position_last(0,0,0);
        static Eigen::Vector3d ypr_last(0, 0, 0);

        Eigen::Vector3d position_now = odom.pose.block<3, 1>(0, 3);
        double dist = (position_now - position_last).norm();

        const Eigen::Matrix3d rotation(odom.pose.block<3, 3>(0, 0));
        Eigen::Vector3d ypr = mathutils::R2ypr(rotation);
        Eigen::Vector3d delta_angle = ypr - ypr_last;
        for (size_t i = 0; i < 3; i++)
            delta_angle(i) = normalize_angle(delta_angle(i));
        delta_angle = delta_angle.cwiseAbs();
        
        
        if (key_frame_index_.size() == 0 ||
            dist > 0.2 ||
            delta_angle(0) > 5.0 ||
            delta_angle(1) > 5.0 ||
            delta_angle(2) > 5.0) {
            // std::cout << "dist: " << dist << std::endl;
            // std::cout << "delta_angle Y: " << delta_angle(0) << std::endl;
            // std::cout << "delta_angle P: " << delta_angle(1) << std::endl;
            // std::cout << "delta_angle R: " << delta_angle(2) << std::endl;

            position_last = position_now;
            ypr_last = ypr;
            is_KF_ = true;
            kf_odom_.push_back(odom);
//            ROS_WARN_STREAM("[LidarOdometry::checkKeyScan] This is a Key Frame, returning true");
            return true;
        }
//        ROS_INFO_STREAM("[LidarOdometry::checkKeyScan] Not a Key Frame, returning false");
        is_KF_ = false;
        return false;
    }

    void LidarOdometry::setTargetMap(VPointCloud::Ptr map_cloud_in) {
        map_cloud_->clear();
        pcl::copyPointCloud(*map_cloud_in, *map_cloud_);
        ndt_omp_->setInputTarget(map_cloud_);
    }

    void LidarOdometry::clearOdometryData() {
        key_frame_index_.clear();
        odom_.clear();
        kf_odom_.clear();
    }

}