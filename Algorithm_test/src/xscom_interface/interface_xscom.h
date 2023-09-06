/**
 * Copyright(c)2022 ChangSha XingShen Technology Ltd .
 *
 * All rights reserved
 * Author:    wanghao
 * -------------------------------
 * Changes:
 * -------------------------------
 * v1.0 2022-1-13 :created by wanghao 新增xscom通信方式
 *
 **/
#pragma once
#include "xscom/xscom.h"
#include "channeldefine.h"
#include "nodedefine.h"
//proto stuffs
#include "interface_xsproto.h"

//class BasicPathPlan;

class InterfaceTimer
{
public:
    InterfaceTimer()
    {
        Reset();
    }

    void Reset()
    {
        _start_time.tv_sec = 0;
        _start_time.tv_usec = 0;
    }

    void UpdateToNow()
    {
        gettimeofday(&_start_time, NULL);
    }

    boost::int64_t ElapsedMin()
    {
        struct timeval _end_time;
        gettimeofday(&_end_time, NULL);
        return ((_end_time.tv_sec - _start_time.tv_sec ) + (_end_time.tv_usec - _start_time.tv_usec) / 1000000)/60;
    }

    boost::int64_t ElapsedSec()
    {
        struct timeval _end_time;
        gettimeofday(&_end_time, NULL);
        return _end_time.tv_sec - _start_time.tv_sec  + (_end_time.tv_usec - _start_time.tv_usec) / 1000000;
    }

    boost::int64_t ElapsedMill()
    {
        struct timeval _end_time;
        gettimeofday(&_end_time, NULL);
        return 1000 * (_end_time.tv_sec - _start_time.tv_sec)  + (_end_time.tv_usec - _start_time.tv_usec) / 1000;
    }

private:
    struct timeval _start_time;
};


class InterfaceXSCom : public InterfaceXsProto
{
public:
    InterfaceXSCom();
    virtual ~InterfaceXSCom();

    //
    bool Init();

    // 读数据
    bool Read(BasicPathPlan *plan);

    // 写数据
    void Write(BasicPathPlan *plan);


    void WriteLocalPlanLog();

    void WriteVisualizeImg(const BasicPathPlan *plan, int display);
    
private:
    //void WriteLocalPlanState(const BasicPathPlan *plan);
    //void WriteLocalPathInfo(std::shared_ptr<xsproto::planner::LocalPathInfo>& msg);
    
    //void WriteLiftCommand(const BasicPathPlan *plan);
    //void WriteLocalPlanVoice(const BasicPathPlan *plan);
    //void WriteSafetyWarning(const BasicPathPlan *plan);
    //void WriteNecControl(const BasicPathPlan *plan);

    

private:
    void OnLocalPoseCallback(const std::shared_ptr<xsproto::base::LocalPose>& msg);
    void OnVehicleStatusCallback(const std::shared_ptr<xsproto::base::VehicleStatus>& msg);
    void OnFusionMsgCallback(const std::shared_ptr<xsproto::perception::FusionMsg>& msg);
    void OnUltraSonicCallback(const std::shared_ptr<xsproto::perception::UltraSonicData>& msg);
    void OnPredictionInfoCallback(const std::shared_ptr<xsproto::perception::PredictionVehicleInfoMsg>& msg);
    void OnTrafficLightCallback(const std::shared_ptr<xsproto::perception::TrafficLightInfo>& msg);
    void OnRubbishInfoCallback(const std::shared_ptr<xsproto::perception::RubbishInfo>& msg);
    void OnCurbInfoCallback(const std::shared_ptr<xsproto::perception::CurbInfoMsg>& msg);
    void OnTrafficSignInfoCallback(const std::shared_ptr<xsproto::perception::TrafficSignInfo>& msg);
    void OnTaskListCallback(const std::shared_ptr<xsproto::globalpath::TaskList>& msg);
    void OnRemoteOperateCallback(const std::shared_ptr<xsproto::communication::RemoteOperate>& msg);
    void OnFaultInfoCallback(const std::shared_ptr<xsproto::communication::FaultInfo>& msg);
    void OnLocalHDMapCallback(const std::shared_ptr<xsproto::hdmap::LocalHDMap>& msg);
    void OnLocalHDMapExCallback(const std::shared_ptr<xsproto::hdmap::LocalHDMapEx>& msg);
    void OnMapPositionCallback(const std::shared_ptr<xsproto::perception::MapPosition>& msg);
    void OnAccelerationEstimationCallback(const std::shared_ptr<xsproto::acceleration_estimation::AccelerationEstimation>& msg);
    void OnAirportSpecialObjectCallback(const std::shared_ptr<xsproto::perception::AirportSpecialObject>& msg);
    void OnRemoteLightCallback(const std::shared_ptr<xsproto::perception::TrafficLightInfo>& msg);
    void OnFormationInfoCallback(const std::shared_ptr<xsproto::formation::FormationInfo>& msg);
    void OnFormationCommandCallback(const std::shared_ptr<xsproto::formation::FormationCommand>& msg);
private:

    //node
    std::unique_ptr<xscom::Node> planner_node_;

    //reder channels
    //base
    std::shared_ptr<xscom::Reader<xsproto::base::LocalPose>> localpose_reader_;
    std::shared_ptr<xscom::Reader<xsproto::base::VehicleStatus>> vehicle_status_reader_;
    std::shared_ptr<xscom::Reader<xsproto::acceleration_estimation::AccelerationEstimation>> acceleration_estimation_reader_;

    //perception
    std::shared_ptr<xscom::Reader<xsproto::perception::FusionMsg>> local_fusion_map_reader_;
    std::shared_ptr<xscom::Reader<xsproto::perception::UltraSonicData>> ultrasonic_reader_;
    std::shared_ptr<xscom::Reader<xsproto::perception::PredictionVehicleInfoMsg>> prediction_vehicle_reader_;
    std::shared_ptr<xscom::Reader<xsproto::perception::TrafficLightInfo>> trafficlight_reader_;
    std::shared_ptr<xscom::Reader<xsproto::perception::RubbishInfo>> rubbish_info_reader_;
    std::shared_ptr<xscom::Reader<xsproto::perception::CurbInfoMsg>> curb_info_reader_;
    std::shared_ptr<xscom::Reader<xsproto::perception::TrafficSignInfo>> traffic_sign_info_reader_;
    std::shared_ptr<xscom::Reader<xsproto::perception::AirportSpecialObject>> airport_special_objects_reader_;

    //global path task
    std::shared_ptr<xscom::Reader<xsproto::globalpath::TaskList>> tasklist_reader_;

    //communication
    std::shared_ptr<xscom::Reader<xsproto::communication::RemoteOperate>> remote_operate_reader_;
    //std::shared_ptr<xscom::Reader<xsproto::communication::FaultInfo>> fault_info_reader_;
    std::shared_ptr<xscom::Reader<xsproto::perception::TrafficLightInfo>> remote_light_reader_;

    //hdmap
    std::shared_ptr<xscom::Reader<xsproto::hdmap::LocalHDMap>> local_hdmap_reader_;
    std::shared_ptr<xscom::Reader<xsproto::hdmap::LocalHDMapEx>> local_hdmapex_reader_;

    //mapposition
    std::shared_ptr<xscom::Reader<xsproto::perception::MapPosition>> map_position_reader_;

    //formation
    std::shared_ptr<xscom::Reader<xsproto::formation::FormationCommand>> formation_command_reader_;
    std::shared_ptr<xscom::Reader<xsproto::formation::FormationInfo>> formation_info_reader_;

    //write channel
    std::shared_ptr<xscom::Writer<xsproto::planner::LocalPlanState>> local_plan_state_writer_;
    std::shared_ptr<xscom::Writer<xsproto::planner::LocalPathInfo>> local_path_info_writer_;
    std::shared_ptr<xscom::Writer<xsproto::planner::OcvMatInfo>> vehicle_result_viewer_writer_;
    std::shared_ptr<xscom::Writer<xsproto::planner::LocalPlanVoice>> local_plan_voice_writer_;
    std::shared_ptr<xscom::Writer<xsproto::planner::SafetyWarningMessage>> safety_warning_writer_;
    std::shared_ptr<xscom::Writer<xsproto::communication::NecControl>> nec_control_request_writer_;
    std::shared_ptr<xscom::Writer<xsproto::planner::LocalPlanLog>> local_plan_log_writer_;


    //主线程用数据
    //base
    std::shared_ptr<xsproto::base::LocalPose> thread_localpose_data_;
    std::shared_ptr<xsproto::base::VehicleStatus> thread_vehicle_status_data_;
    std::shared_ptr<xsproto::acceleration_estimation::AccelerationEstimation> thread_acceleration_estimation_data_;

    //perception
    std::shared_ptr<xsproto::perception::FusionMsg> thread_fusion_map_data_;
    std::shared_ptr<xsproto::perception::UltraSonicData> thread_ultrasonic_data_;
    std::shared_ptr<xsproto::perception::PredictionVehicleInfoMsg> thread_prediction_vehicle_data_;
    std::shared_ptr<xsproto::perception::TrafficLightInfo> thread_trafficlight_data_;
    std::shared_ptr<xsproto::perception::RubbishInfo> thread_rubbish_info_data_;
    std::shared_ptr<xsproto::perception::CurbInfoMsg> thread_curb_info_data_;
    std::shared_ptr<xsproto::perception::TrafficSignInfo> thread_traffic_sign_info_;
    std::shared_ptr<xsproto::perception::AirportSpecialObject> thread_airport_special_objects_;

    //global path task
    std::shared_ptr<xsproto::globalpath::TaskList> thread_tasklist_data_;

    //communication
    std::shared_ptr<xsproto::communication::RemoteOperate> thread_remote_operate_data_;
    //std::shared_ptr<xsproto::communication::FaultInfo> thread_fault_info_data_;
    std::shared_ptr<xsproto::perception::TrafficLightInfo> thread_remote_light_;

    //hdmap
    std::shared_ptr<xsproto::hdmap::LocalHDMap> thread_local_hdmap_data_;
    std::shared_ptr<xsproto::hdmap::LocalHDMapEx> thread_local_hdmapex_data_;

    //mapposition
    std::shared_ptr<xsproto::perception::MapPosition> thread_map_position_data_;

    //formation
    std::shared_ptr<xsproto::formation::FormationInfo> thread_formation_info_data_;
    std::shared_ptr<xsproto::formation::FormationCommand> thread_formation_command_data_;

    //write channel data
    std::shared_ptr<xsproto::planner::LocalPlanState> thread_local_plan_state_data_;
    std::shared_ptr<xsproto::planner::LocalPathInfo> thread_local_path_info_data_;
    std::shared_ptr<xsproto::planner::LocalPlanVoice> thread_local_plan_voice_data_;
    std::shared_ptr<xsproto::planner::SafetyWarningMessage> thread_safety_warning_data_;
    std::shared_ptr<xsproto::communication::NecControl> thread_nec_control_request_data_;

    //******    下面用于回调函数交换数据用   ******//

    // 线程锁，回调函数数据同步用
    std::mutex mutex_local_pose_;
    std::mutex mutex_vehicle_status_;
    std::mutex mutex_fusion_msg_;
    std::mutex mutex_ultrasonic_;
    std::mutex mutex_prediction_info_;
    std::mutex mutex_traffic_light_;
    std::mutex mutex_rubbish_info_;
    std::mutex mutex_curb_info_;
    std::mutex mutex_task_list_;
    std::mutex mutex_remote_operate_;
    std::mutex mutex_fault_info_;
    std::mutex mutex_local_hdmap_;
    std::mutex mutex_local_hdmapex_;
    std::mutex mutex_map_position_;
    std::mutex mutex_traffic_sign_info_;
    std::mutex mutex_acceleration_estimation_;
    std::mutex mutex_airport_special_objects_;
    std::mutex mutex_remote_light_;
    std::mutex mutex_formation_info_;
    std::mutex mutex_formation_command_;

    //cache data copy from callback
    //base
    std::shared_ptr<xsproto::base::LocalPose> cache_localpose_data_;
    std::shared_ptr<xsproto::base::VehicleStatus> cache_vehicle_status_data_;
    std::shared_ptr<xsproto::acceleration_estimation::AccelerationEstimation> cache_acceleration_estimation_data_;

    //perception
    std::shared_ptr<xsproto::perception::FusionMsg> cache_fusion_map_data_;
    std::shared_ptr<xsproto::perception::UltraSonicData> cache_ultrasonic_data_;
    std::shared_ptr<xsproto::perception::PredictionVehicleInfoMsg> cache_prediction_vehicle_data_;
    std::shared_ptr<xsproto::perception::TrafficLightInfo> cache_trafficlight_data_;
    std::shared_ptr<xsproto::perception::RubbishInfo> cache_rubbish_info_data_;
    std::shared_ptr<xsproto::perception::CurbInfoMsg> cache_curb_info_data_;
    std::shared_ptr<xsproto::perception::TrafficSignInfo> cache_traffic_sign_info_;
    std::shared_ptr<xsproto::perception::AirportSpecialObject> cache_airport_special_objects_;

    //global path task
    std::shared_ptr<xsproto::globalpath::TaskList> cache_tasklist_data_;

    //communication
    std::shared_ptr<xsproto::communication::RemoteOperate> cache_remote_operate_data_;
    std::shared_ptr<xsproto::communication::FaultInfo> cache_fault_info_data_;
    std::shared_ptr<xsproto::perception::TrafficLightInfo> cache_remote_light_;

    //hdmap
    std::shared_ptr<xsproto::hdmap::LocalHDMap> cache_local_hdmap_data_;
    std::shared_ptr<xsproto::hdmap::LocalHDMapEx> cache_local_hdmapex_data_;

    //mapposition
    std::shared_ptr<xsproto::perception::MapPosition> cache_map_position_data_;

    //formation
    std::shared_ptr<xsproto::formation::FormationInfo> cache_formation_info_data_;
    std::shared_ptr<xsproto::formation::FormationCommand> cache_formation_command_data_;

    //******    上面用于回调函数交换数据用   ******//

    std::shared_ptr<InterfaceTimer> timer_local_pose_;
    std::shared_ptr<InterfaceTimer> timer_vehicle_status_;
    std::shared_ptr<InterfaceTimer> timer_fusion_msg_;
    std::shared_ptr<InterfaceTimer> timer_ultrasonic_;
    std::shared_ptr<InterfaceTimer> timer_prediction_info_;
    std::shared_ptr<InterfaceTimer> timer_traffic_light_;
    std::shared_ptr<InterfaceTimer> timer_rubbish_info_;
    std::shared_ptr<InterfaceTimer> timer_curb_info_;
    std::shared_ptr<InterfaceTimer> timer_task_list_;
    std::shared_ptr<InterfaceTimer> timer_remote_operate_;
    std::shared_ptr<InterfaceTimer> timer_fault_info_;
    std::shared_ptr<InterfaceTimer> timer_local_hdmap_;
    std::shared_ptr<InterfaceTimer> timer_local_hdmapex_;
    std::shared_ptr<InterfaceTimer> timer_map_position_;
    std::shared_ptr<InterfaceTimer> timer_traffic_sign_info_;
    std::shared_ptr<InterfaceTimer> timer_acceleration_estimation_;
    std::shared_ptr<InterfaceTimer> timer_airport_special_objects_;
    std::shared_ptr<InterfaceTimer> timer_remote_light_;
    std::shared_ptr<InterfaceTimer> timer_formation_info_;
    std::shared_ptr<InterfaceTimer> timer_formation_command_;

    //std::string ugv_model;

};
