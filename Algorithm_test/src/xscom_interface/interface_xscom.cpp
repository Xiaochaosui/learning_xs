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

#include "interface_xscom.h"
#include "logistics/park_events.h"
#include <boost/algorithm/string.hpp>
#include "basic_path_plan.h"


// 标准帧间隔,毫秒
#define FRAME_STANDARD (100)


InterfaceXSCom::InterfaceXSCom()
{
    //宏命令,用于检查用的protobuf的库是否兼容
    GOOGLE_PROTOBUF_VERIFY_VERSION;

    PlannerInfo("@@@@@@ connect to xscom channel success! @@@@@@");
#if 0
    arrive_seqnum = -1;
    arrive_time = -99999;

#endif
}


InterfaceXSCom::~InterfaceXSCom()
{
    //delete any global objects that were allocated by the Protocol Buffer library
    google::protobuf::ShutdownProtobufLibrary();
}

//
bool InterfaceXSCom::Init()
{
    //初始创建内部指针
    //==============================================================================
    cache_localpose_data_ = std::make_shared<xsproto::base::LocalPose>();
    cache_vehicle_status_data_ = std::make_shared<xsproto::base::VehicleStatus>();
    cache_acceleration_estimation_data_ = std::make_shared<xsproto::acceleration_estimation::AccelerationEstimation>();

    //perception
    cache_fusion_map_data_ = std::make_shared<xsproto::perception::FusionMsg>();
    cache_ultrasonic_data_ = std::make_shared<xsproto::perception::UltraSonicData>();
    cache_prediction_vehicle_data_ = std::make_shared<xsproto::perception::PredictionVehicleInfoMsg>();
    cache_trafficlight_data_ = std::make_shared<xsproto::perception::TrafficLightInfo>();
    cache_rubbish_info_data_ = std::make_shared<xsproto::perception::RubbishInfo>();
    cache_curb_info_data_ = std::make_shared<xsproto::perception::CurbInfoMsg>();
    cache_traffic_sign_info_ = std::make_shared<xsproto::perception::TrafficSignInfo>();
    cache_airport_special_objects_ = std::make_shared<xsproto::perception::AirportSpecialObject>();

    //global path task
    cache_tasklist_data_ = std::make_shared<xsproto::globalpath::TaskList>();

    //communication
    cache_remote_operate_data_ = std::make_shared<xsproto::communication::RemoteOperate>();
    cache_fault_info_data_ = std::make_shared<xsproto::communication::FaultInfo>();
    cache_remote_light_ = std::make_shared<xsproto::perception::TrafficLightInfo>();

    //hdmap
    cache_local_hdmap_data_ = std::make_shared<xsproto::hdmap::LocalHDMap>();
    cache_local_hdmapex_data_ = std::make_shared<xsproto::hdmap::LocalHDMapEx>();

    //mapposition
    cache_map_position_data_ = std::make_shared<xsproto::perception::MapPosition>();

    //formation
    cache_formation_command_data_ = std::make_shared<xsproto::formation::FormationCommand>();
    cache_formation_info_data_ = std::make_shared<xsproto::formation::FormationInfo>();


    //==============================================================================


    //==============================================================================
    thread_localpose_data_ = std::make_shared<xsproto::base::LocalPose>();
    thread_vehicle_status_data_ = std::make_shared<xsproto::base::VehicleStatus>();
    thread_acceleration_estimation_data_ = std::make_shared<xsproto::acceleration_estimation::AccelerationEstimation>();

    //perception
    thread_fusion_map_data_ = std::make_shared<xsproto::perception::FusionMsg>();
    thread_ultrasonic_data_ = std::make_shared<xsproto::perception::UltraSonicData>();
    thread_prediction_vehicle_data_ = std::make_shared<xsproto::perception::PredictionVehicleInfoMsg>();
    thread_trafficlight_data_ = std::make_shared<xsproto::perception::TrafficLightInfo>();
    thread_rubbish_info_data_ = std::make_shared<xsproto::perception::RubbishInfo>();
    thread_curb_info_data_ = std::make_shared<xsproto::perception::CurbInfoMsg>();
    thread_traffic_sign_info_ = std::make_shared<xsproto::perception::TrafficSignInfo>();
    thread_airport_special_objects_ = std::make_shared<xsproto::perception::AirportSpecialObject>();

    //global path task
    thread_tasklist_data_ = std::make_shared<xsproto::globalpath::TaskList>();

    //communication
    thread_remote_operate_data_ = std::make_shared<xsproto::communication::RemoteOperate>();
    //thread_fault_info_data_ = std::make_shared<xsproto::communication::FaultInfo>();
    thread_remote_light_ = std::make_shared<xsproto::perception::TrafficLightInfo>();

    //hdmap
    thread_local_hdmap_data_ = std::make_shared<xsproto::hdmap::LocalHDMap>();
    thread_local_hdmapex_data_ = std::make_shared<xsproto::hdmap::LocalHDMapEx>();

    //mapposition
    thread_map_position_data_ = std::make_shared<xsproto::perception::MapPosition>();

    //formation
    thread_formation_command_data_ = std::make_shared<xsproto::formation::FormationCommand>();
    thread_formation_info_data_ = std::make_shared<xsproto::formation::FormationInfo>();

    thread_local_plan_state_data_ = std::make_shared<xsproto::planner::LocalPlanState>();
    thread_local_path_info_data_ = std::make_shared<xsproto::planner::LocalPathInfo>();
    thread_local_plan_voice_data_ = std::make_shared<xsproto::planner::LocalPlanVoice>();
    thread_safety_warning_data_ = std::make_shared<xsproto::planner::SafetyWarningMessage>();
    thread_nec_control_request_data_ = std::make_shared<xsproto::communication::NecControl>();
    //==============================================================================

    timer_local_pose_ = std::make_shared<InterfaceTimer>();
    timer_vehicle_status_ = std::make_shared<InterfaceTimer>();
    timer_fusion_msg_ = std::make_shared<InterfaceTimer>();
    timer_ultrasonic_ = std::make_shared<InterfaceTimer>();
    timer_prediction_info_ = std::make_shared<InterfaceTimer>();
    timer_traffic_light_ = std::make_shared<InterfaceTimer>();
    timer_rubbish_info_ = std::make_shared<InterfaceTimer>();
    timer_curb_info_ = std::make_shared<InterfaceTimer>();
    timer_task_list_ = std::make_shared<InterfaceTimer>();
    timer_remote_operate_ = std::make_shared<InterfaceTimer>();
    timer_fault_info_ = std::make_shared<InterfaceTimer>();
    timer_local_hdmap_ = std::make_shared<InterfaceTimer>();
    timer_local_hdmapex_ = std::make_shared<InterfaceTimer>();
    timer_map_position_ = std::make_shared<InterfaceTimer>();
    timer_traffic_sign_info_ = std::make_shared<InterfaceTimer>();
    timer_acceleration_estimation_ = std::make_shared<InterfaceTimer>();
    timer_airport_special_objects_ = std::make_shared<InterfaceTimer>();
    timer_remote_light_ = std::make_shared<InterfaceTimer>();
    timer_formation_info_ = std::make_shared<InterfaceTimer>();
    timer_formation_command_ = std::make_shared<InterfaceTimer>();

    //创建通信体
    planner_node_ = xscom::CreateNode(NODE_planner);

    auto local_pose_callback =  std::bind(&InterfaceXSCom::OnLocalPoseCallback, this, std::placeholders::_1);
    auto vehicle_status_callback = std::bind(&InterfaceXSCom::OnVehicleStatusCallback, this, std::placeholders::_1);
    auto acceleration_estimation_callback = std::bind(&InterfaceXSCom::OnAccelerationEstimationCallback, this, std::placeholders::_1);

    auto fusion_map_callback = std::bind(&InterfaceXSCom::OnFusionMsgCallback, this, std::placeholders::_1);
    auto ultrasonic_callback = std::bind(&InterfaceXSCom::OnUltraSonicCallback, this, std::placeholders::_1);
    auto prediction_vehicle_callback = std::bind(&InterfaceXSCom::OnPredictionInfoCallback, this, std::placeholders::_1);
    auto traffic_light_callback = std::bind(&InterfaceXSCom::OnTrafficLightCallback, this, std::placeholders::_1);
    auto rubbish_info_callback = std::bind(&InterfaceXSCom::OnRubbishInfoCallback, this, std::placeholders::_1);
    auto curb_info_callback = std::bind(&InterfaceXSCom::OnCurbInfoCallback, this, std::placeholders::_1);
    auto traffic_sign_info_callback = std::bind(&InterfaceXSCom::OnTrafficSignInfoCallback, this, std::placeholders::_1);
    auto airport_special_objects_callback = std::bind(&InterfaceXSCom::OnAirportSpecialObjectCallback, this, std::placeholders::_1);


    auto task_list_callback = std::bind(&InterfaceXSCom::OnTaskListCallback, this, std::placeholders::_1);
    auto remote_operate_callback = std::bind(&InterfaceXSCom::OnRemoteOperateCallback, this, std::placeholders::_1);
    //auto fault_info_callback = std::bind(&InterfaceXSCom::OnFaultInfoCallback, this, std::placeholders::_1);
    auto remote_light_callback = std::bind(&InterfaceXSCom::OnRemoteLightCallback, this, std::placeholders::_1);

    //
    auto local_hdmap_callback = std::bind(&InterfaceXSCom::OnLocalHDMapCallback, this, std::placeholders::_1);
    auto local_hdmapex_callback = std::bind(&InterfaceXSCom::OnLocalHDMapExCallback, this, std::placeholders::_1);

    auto map_position_callback = std::bind(&InterfaceXSCom::OnMapPositionCallback, this, std::placeholders::_1);

    auto formation_info_callback = std::bind(&InterfaceXSCom::OnFormationInfoCallback, this, std::placeholders::_1);
    auto formation_command_callback = std::bind(&InterfaceXSCom::OnFormationCommandCallback, this, std::placeholders::_1);

    //base
    localpose_reader_ = planner_node_->CreateReader<xsproto::base::LocalPose>(CHANNEL_LocalPose, local_pose_callback);
    vehicle_status_reader_ = planner_node_->CreateReader<xsproto::base::VehicleStatus>(CHANNEL_VehicleStatus, vehicle_status_callback);
    acceleration_estimation_reader_ = planner_node_->CreateReader<xsproto::acceleration_estimation::AccelerationEstimation>(CHANNEL_AccelerationEstimation, acceleration_estimation_callback);

    //perception
    local_fusion_map_reader_ = planner_node_->CreateReader<xsproto::perception::FusionMsg>(CHANNEL_Env_Fusion, fusion_map_callback);
    ultrasonic_reader_ = planner_node_->CreateReader<xsproto::perception::UltraSonicData>(CHANNEL_UltrasonicData, ultrasonic_callback);
    prediction_vehicle_reader_ = planner_node_->CreateReader<xsproto::perception::PredictionVehicleInfoMsg>(CHANNEL_Traj_Prediction, prediction_vehicle_callback);
    trafficlight_reader_ = planner_node_->CreateReader<xsproto::perception::TrafficLightInfo>(CHANNEL_TrafficLightInfo, traffic_light_callback);
    rubbish_info_reader_ = planner_node_->CreateReader<xsproto::perception::RubbishInfo>(CHANNEL_RubbishInfo, rubbish_info_callback);
    curb_info_reader_ = planner_node_->CreateReader<xsproto::perception::CurbInfoMsg>(CHANNEL_RoadCurbEdge, curb_info_callback);
    traffic_sign_info_reader_ = planner_node_->CreateReader<xsproto::perception::TrafficSignInfo>(CHANNEL_TrafficSignInfo, traffic_sign_info_callback);
    airport_special_objects_reader_ = planner_node_->CreateReader<xsproto::perception::AirportSpecialObject>(CHANNEL_AirportSpecialObject, airport_special_objects_callback);

    //global path task
    tasklist_reader_ = planner_node_->CreateReader<xsproto::globalpath::TaskList>(CHANNEL_TaskList, task_list_callback);

    //business
    remote_operate_reader_ = planner_node_->CreateReader<xsproto::communication::RemoteOperate>(CHANNEL_RemoteOperate, remote_operate_callback);
    //fault_info_reader_ = planner_node_->CreateReader<xsproto::communication::FaultInfo>(CHANNEL_FaultInfo, fault_info_callback);
    remote_light_reader_ = planner_node_->CreateReader<xsproto::perception::TrafficLightInfo>(CHANNEL_RemoteLight, remote_light_callback);

    //hdmap
    local_hdmap_reader_ = planner_node_->CreateReader<xsproto::hdmap::LocalHDMap>(CHANNEL_LocalHDMap, local_hdmap_callback);
    local_hdmapex_reader_ = planner_node_->CreateReader<xsproto::hdmap::LocalHDMapEx>(CHANNEL_LocalHDMap_XSLanenet, local_hdmapex_callback);

    //mapposition
    map_position_reader_ = planner_node_->CreateReader<xsproto::perception::MapPosition>(CHANNEL_MapPosition, map_position_callback);

    //formation
    formation_info_reader_ = planner_node_->CreateReader<xsproto::formation::FormationInfo>(CHANNEL_FormationInfo, formation_info_callback);
    formation_command_reader_ = planner_node_->CreateReader<xsproto::formation::FormationCommand>(CHANNEL_FormationCommand, formation_command_callback);

    //write channel
    local_plan_state_writer_ = planner_node_->CreateWriter<xsproto::planner::LocalPlanState>(CHANNEL_LocalPlanState);
    local_path_info_writer_ = planner_node_->CreateWriter<xsproto::planner::LocalPathInfo>(CHANNEL_LocalPathInfo);
    vehicle_result_viewer_writer_ = planner_node_->CreateWriter<xsproto::planner::OcvMatInfo>(CHANNEL_VehicleResultViwer);
    local_plan_voice_writer_ = planner_node_->CreateWriter<xsproto::planner::LocalPlanVoice>(CHANNEL_LocalPlanVoice);
    safety_warning_writer_ = planner_node_->CreateWriter<xsproto::planner::SafetyWarningMessage>(CHANNEL_SafetyWarning);
    nec_control_request_writer_ = planner_node_->CreateWriter<xsproto::communication::NecControl>(CHANNEL_NECControl);
    local_plan_log_writer_ = planner_node_->CreateWriter<xsproto::planner::LocalPlanLog>(CHANNEL_LocalPlanLog);

    return true;
}

// 读数据
bool InterfaceXSCom::Read(BasicPathPlan *plan)
{
    boost::int64_t elapse_local_pose = 0;
    boost::int64_t elapse_vehicle_status = 0;
    boost::int64_t elapse_fusion_msg = 0;
    boost::int64_t elapse_ultrasonic = 0;
    boost::int64_t elapse_prediction_info = 0;
    boost::int64_t elapse_traffic_light = 0;
    boost::int64_t elapse_rubbish_info = 0;
    boost::int64_t elapse_curb_info = 0;
    boost::int64_t elapse_traffic_sign_info = 0;
    boost::int64_t elapse_task_list = 0;
    boost::int64_t elapse_remote_operate = 0;
    boost::int64_t elapse_fault_info = 0;
    boost::int64_t elapse_local_hdmap = 0;
    boost::int64_t elapse_local_hdmapex = 0;
    boost::int64_t elapse_map_position = 0;
    boost::int64_t elapse_acceleration_estimation = 0;
    boost::int64_t elapse_lift_platform = 0;
    boost::int64_t elapse_remote_light = 0;
    boost::int64_t elapse_formation_command = 0;
    boost::int64_t elapse_formation_info = 0;
    {
        // 线程锁
        std::lock_guard<std::mutex> lock(mutex_local_pose_);

        // 更新主线程缓存数据
        thread_localpose_data_->CopyFrom(*cache_localpose_data_);
        elapse_local_pose = timer_local_pose_->ElapsedMill();
    }

    {
        // 线程锁
        std::lock_guard<std::mutex> lock(mutex_vehicle_status_);

        // 更新主线程缓存数据
        thread_vehicle_status_data_->CopyFrom(*cache_vehicle_status_data_);
        elapse_vehicle_status = timer_vehicle_status_->ElapsedMill();
    }


    {
        // 线程锁
        std::lock_guard<std::mutex> lock(mutex_fusion_msg_);

        // 更新主线程缓存数据
        thread_fusion_map_data_->CopyFrom(*cache_fusion_map_data_);
        elapse_fusion_msg = timer_fusion_msg_->ElapsedMill();
    }


    {
        // 线程锁
        std::lock_guard<std::mutex> lock(mutex_ultrasonic_);

        // 更新主线程缓存数据
        thread_ultrasonic_data_->CopyFrom(*cache_ultrasonic_data_);
        elapse_ultrasonic = timer_ultrasonic_->ElapsedMill();
    }

    {
        // 线程锁
        std::lock_guard<std::mutex> lock(mutex_prediction_info_);

        // 更新主线程缓存数据
        thread_prediction_vehicle_data_->CopyFrom(*cache_prediction_vehicle_data_);
        elapse_prediction_info = timer_prediction_info_->ElapsedMill();
    }

    {
        // 线程锁
        std::lock_guard<std::mutex> lock(mutex_traffic_light_);

        // 更新主线程缓存数据
        thread_trafficlight_data_->CopyFrom(*cache_trafficlight_data_);
        elapse_traffic_light = timer_traffic_light_->ElapsedMill();
    }

    {
        // 线程锁
        std::lock_guard<std::mutex> lock(mutex_rubbish_info_);

        // 更新主线程缓存数据
        thread_rubbish_info_data_->CopyFrom(*cache_rubbish_info_data_);
        elapse_rubbish_info = timer_rubbish_info_->ElapsedMill();
    }

    {
        // 线程锁
        std::lock_guard<std::mutex> lock(mutex_curb_info_);

        // 更新主线程缓存数据
        thread_curb_info_data_->CopyFrom(*cache_curb_info_data_);
        elapse_curb_info = timer_curb_info_->ElapsedMill();
    }

    {
        // 线程锁
        std::lock_guard<std::mutex> lock(mutex_traffic_sign_info_);

        // 更新主线程缓存数据
        thread_traffic_sign_info_->CopyFrom(*cache_traffic_sign_info_);
        elapse_traffic_sign_info = timer_traffic_sign_info_->ElapsedMill();
    }


    {
        // 线程锁
        std::lock_guard<std::mutex> lock(mutex_task_list_);

        // 更新主线程缓存数据
        thread_tasklist_data_->CopyFrom(*cache_tasklist_data_);
        elapse_task_list = timer_task_list_->ElapsedMill();
    }

    {
        // 线程锁
        std::lock_guard<std::mutex> lock(mutex_remote_operate_);

        // 更新主线程缓存数据
        thread_remote_operate_data_->CopyFrom(*cache_remote_operate_data_);
        elapse_remote_operate = timer_remote_operate_->ElapsedMill();
    }


    // {
    //     // 线程锁
    //     std::lock_guard<std::mutex> lock(mutex_fault_info_);

    //     // 更新主线程缓存数据
    //     thread_fault_info_data_->CopyFrom(*cache_fault_info_data_);
    //     elapse_fault_info = timer_fault_info_->ElapsedMill();
    // }

    {
        // 线程锁
        std::lock_guard<std::mutex> lock(mutex_local_hdmap_);

        // 更新主线程缓存数据
        thread_local_hdmap_data_->CopyFrom(*cache_local_hdmap_data_);
        elapse_local_hdmap = timer_local_hdmap_->ElapsedMill();
    }

    {
        // 线程锁
        std::lock_guard<std::mutex> lock(mutex_local_hdmapex_);

        // 更新主线程缓存数据
        thread_local_hdmapex_data_->CopyFrom(*cache_local_hdmapex_data_);
        elapse_local_hdmapex = timer_local_hdmapex_->ElapsedMill();
    }


    {
        // 线程锁
        std::lock_guard<std::mutex> lock(mutex_map_position_);

        // 更新主线程缓存数据
        thread_map_position_data_->CopyFrom(*cache_map_position_data_);
        elapse_map_position = timer_map_position_->ElapsedMill();
    }

    {
        // 线程锁
        std::lock_guard<std::mutex> lock(mutex_acceleration_estimation_);

        // 更新主线程缓存数据
        thread_acceleration_estimation_data_->CopyFrom(*cache_acceleration_estimation_data_);
        elapse_acceleration_estimation = timer_acceleration_estimation_->ElapsedMill();
    }

    {
        // 线程锁
        std::lock_guard<std::mutex> lock(mutex_airport_special_objects_);

        // 更新主线程缓存数据
        thread_airport_special_objects_->CopyFrom(*cache_airport_special_objects_);
        elapse_lift_platform = timer_airport_special_objects_->ElapsedMill();
    }

    {
        // 线程锁
        std::lock_guard<std::mutex> lock(mutex_remote_light_);

        // 更新主线程缓存数据
        thread_remote_light_->CopyFrom(*cache_remote_light_);
        elapse_remote_light = timer_remote_light_->ElapsedMill();
    }

    {
        // 线程锁
        std::lock_guard<std::mutex> lock(mutex_formation_command_);

        // 更新主线程缓存数据
        thread_formation_command_data_->CopyFrom(*cache_formation_command_data_);
        elapse_formation_command = timer_formation_command_->ElapsedMill();
    }

    {
        // 线程锁
        std::lock_guard<std::mutex> lock(mutex_formation_info_);

        // 更新主线程缓存数据
        thread_formation_info_data_->CopyFrom(*cache_formation_info_data_);
        elapse_formation_info = timer_formation_info_->ElapsedMill();
    }


    const boost::int64_t max_lev = 10000;
    // 时间除以每帧间隔，得到间隔帧数
    elapse_local_pose = std::min(elapse_local_pose/FRAME_STANDARD, max_lev);
    elapse_vehicle_status = std::min(elapse_vehicle_status/FRAME_STANDARD, max_lev);
    elapse_fusion_msg = std::min(elapse_fusion_msg/FRAME_STANDARD, max_lev);
    elapse_ultrasonic = std::min(elapse_ultrasonic/FRAME_STANDARD, max_lev);
    elapse_prediction_info = std::min(elapse_prediction_info/FRAME_STANDARD, max_lev);
    elapse_traffic_light = std::min(elapse_traffic_light/FRAME_STANDARD, max_lev);
    elapse_rubbish_info = std::min(elapse_rubbish_info/FRAME_STANDARD, max_lev);
    elapse_curb_info = std::min(elapse_curb_info/FRAME_STANDARD, max_lev);
    elapse_traffic_sign_info = std::min(elapse_traffic_sign_info/FRAME_STANDARD, max_lev);
    elapse_task_list = std::min(elapse_task_list/FRAME_STANDARD, max_lev);
    elapse_remote_operate = std::min(elapse_remote_operate/FRAME_STANDARD, max_lev);
    elapse_fault_info = std::min(elapse_fault_info/FRAME_STANDARD, max_lev);
    elapse_local_hdmap = std::min(elapse_local_hdmap/FRAME_STANDARD, max_lev);
    elapse_local_hdmapex = std::min(elapse_local_hdmapex/FRAME_STANDARD, max_lev);
    elapse_map_position = std::min(elapse_map_position/FRAME_STANDARD, max_lev);
    elapse_acceleration_estimation = std::min(elapse_acceleration_estimation/FRAME_STANDARD, max_lev);
    elapse_lift_platform = std::min(elapse_lift_platform/FRAME_STANDARD, max_lev);
    elapse_remote_light = std::min(elapse_remote_light/FRAME_STANDARD, max_lev);
    elapse_formation_command = std::min(elapse_formation_command/FRAME_STANDARD, max_lev);
    elapse_formation_info = std::min(elapse_formation_info/FRAME_STANDARD, max_lev);
    // PlannerInfo("elapse frame, local_pose: %ld, vehicle_status: %ld, lift_state: %ld, fusion_msg: %ld, ultrasonic: %ld, predict info : %ld",
    //         elapse_local_pose, elapse_vehicle_status, elapse_lift_state, elapse_fusion_msg, elapse_ultrasonic, elapse_prediction_info);
    // PlannerInfo("elapse frame, rubbish_info: %ld, task_list: %ld, remote_operate: %ld, fault_info: %ld, local_hdmap: %ld, map_position info : %ld",
    //        elapse_rubbish_info, elapse_task_list, elapse_remote_operate, elapse_fault_info, elapse_local_hdmap, elapse_map_position);

    //    plan->isNoData = true;

    // 清理数据
    plan->local_pose_->Reset();
    plan->global_pose_->Reset();
    plan->map_entity_->Reset();
    plan->map_attr_->Reset();
    plan->task_list_->Reset();
    plan->curb_info_->Reset();
    plan->remote_control_plan_->Reset();
    plan->rubbish_info_->Reset();
    plan->fault_info_->Reset();
    plan->plan_decision_->Clear();
    plan->formation_command_->Reset();
    plan->formation_info_->Reset();

    //停车时再清理超时数据
    bool is_stop = true;
    if(1 == plan->GetSlowStopMode() && nullptr != thread_vehicle_status_data_)
    {
        double speed = (thread_vehicle_status_data_->left_rear_wheel().speed() + thread_vehicle_status_data_->right_rear_wheel().speed()) / 2;
        if( speed > 0 )
        {
            is_stop = false;
        }
    }

    // 判断是否超时，清理数据
    if (elapse_local_pose > FRAME_MISSED && is_stop)
    {
        thread_localpose_data_->Clear();
    }
    else
    {
        ConvertLocalPose(thread_localpose_data_, plan);
    }

    if (elapse_vehicle_status > FRAME_MISSED && is_stop)
    {
        thread_vehicle_status_data_->Clear();
    }
    else
    {
        ConvertVehicleStatus(thread_vehicle_status_data_, plan);
    }

    if (elapse_acceleration_estimation > FRAME_MISSED)
    {
        thread_acceleration_estimation_data_->Clear();
    }
    else
    {
        ConvertAccelerationEstimation(thread_acceleration_estimation_data_, plan);
    }

    
    if (elapse_map_position > FRAME_MISSED && is_stop)
    {
        thread_map_position_data_->Clear();
    }
    else
    {
        if (!ConvertMapPosition(thread_map_position_data_, plan))
        {
            elapse_map_position = 10*FRAME_MISSED;
        }
    }

    if (elapse_lift_platform > FRAME_MISSED)
    {
        thread_airport_special_objects_->Clear();
    }
    else
    {
        ConvertAirportSpecialObject(thread_airport_special_objects_, plan);
    }


    if (elapse_formation_command > FRAME_MISSED)
    {
        thread_formation_command_data_->Clear();
    }
    else
    {
        ConvertFormationCommand(thread_formation_command_data_, plan);
    }

    //收到队列中的车辆信息后，会更新感知目标的信息，必须放在感知前面
    if (elapse_formation_info > FRAME_MISSED)
    {
        thread_formation_info_data_->Clear();
    }
    else
    {
        ConvertFormationInfo(thread_formation_info_data_, plan);
    }

    if (elapse_fusion_msg > FRAME_MISSED && is_stop)
    {
        thread_fusion_map_data_->Clear();
    }
    else
    {
        ConvertFusionMsg(thread_fusion_map_data_, thread_airport_special_objects_, plan);
    }

    if (elapse_ultrasonic > FRAME_MISSED )
    {
        thread_ultrasonic_data_->Clear();
    }
    else
    {
        ConvertUltraSonic(thread_ultrasonic_data_, plan);
    }
    if (elapse_prediction_info > FRAME_MISSED && is_stop)
    {
        thread_prediction_vehicle_data_->Clear();
    }
    else
    {
        ConvertPredictionInfo(thread_prediction_vehicle_data_, plan);
    }
    if (elapse_traffic_light > FRAME_MISSED)
    {
        thread_trafficlight_data_->Clear();
    }
    else
    {
        ConvertTrafficLight(thread_trafficlight_data_, plan);
    }
    if (elapse_rubbish_info > FRAME_MISSED )
    {
        thread_rubbish_info_data_->Clear();
    }
    else
    {
        ConvertRubbishInfo(thread_rubbish_info_data_, plan);
    }
    if (elapse_curb_info > FRAME_MISSED)
    {
        thread_curb_info_data_->Clear();
    }
    else
    {
        ConvertCurbInfo(thread_curb_info_data_, plan);
    }
    if(elapse_traffic_sign_info > FRAME_MISSED)
    {
        thread_traffic_sign_info_->Clear();
    }
    else
    {
        ConvertTrafficSignInfo(thread_traffic_sign_info_, plan);
    }

    if (elapse_remote_operate > FRAME_MISSED )
    {
        thread_remote_operate_data_->Clear();
    }
    else
    {
        ConvertRemoteOperate(thread_remote_operate_data_, plan);
    }
    // if (elapse_fault_info > FRAME_MISSED )
    // {
    //     thread_fault_info_data_->Clear();
    // }
    // else
    // {
    //     ConvertFaultInfo(thread_fault_info_data_, plan);
    // }

    if (elapse_remote_light > FRAME_MISSED )
    {
        thread_remote_light_->Clear();
    }
    else
    {
        ConvertRemoteLight(thread_remote_light_, plan);
    }

    if (elapse_local_hdmap > FRAME_MISSED && is_stop)
    {
        thread_local_hdmap_data_->Clear();
    }
    else
    {
        ConvertLocalHDMap(thread_local_hdmap_data_, plan);
    }
    if (elapse_local_hdmapex > FRAME_MISSED && is_stop)
    {
        thread_local_hdmapex_data_->Clear();
    }
    else
    {
        ConvertLocalHDMapEx(thread_local_hdmapex_data_, plan);
    }

    
    // 全局路里面有使用停车框计算属性的，必须放在最后面
    if (elapse_task_list > FRAME_MISSED && is_stop)
    {
        thread_tasklist_data_->Clear();
    }
    else
    {
        ConvertTaskList(thread_tasklist_data_, plan);
    }

    

    //plan->rubbish_info_->Reset();
    
    //
    ChoosePrecisionPoint(plan, thread_airport_special_objects_);

    FaultProcess(thread_vehicle_status_data_->driving_mode(),
                 elapse_local_pose,
                 elapse_vehicle_status,
                 elapse_fusion_msg,
                 elapse_ultrasonic,
                 elapse_prediction_info,
                 elapse_traffic_light,
                 elapse_rubbish_info,
                 elapse_curb_info,
                 elapse_task_list,
                 elapse_remote_operate,
                 elapse_fault_info,
                 elapse_local_hdmap,
                 elapse_local_hdmapex,
                 elapse_map_position);

    //    if(elapse_fusion_msg < 6 && elapse_local_pose < 6
    //            && elapse_task_list < 20 && elapse_map_position < 12)
    //    {
    //        plan->isNoData = false;
    //    }

    boost::int64_t data_flag = FLAG_ALL_DATA;
//    plan->isNoData = false;
    if(elapse_fusion_msg > FRAME_MISSED)
    {
        PlannerInfo("fusion_msg loss %d ", elapse_fusion_msg);
        data_flag |= FLAG_NO_FUSION_MSG;
    }
    if(elapse_local_pose > FRAME_MISSED || elapse_vehicle_status > FRAME_MISSED)
    {
        PlannerInfo("local_pose loss %d ", elapse_local_pose);
        data_flag |= FLAG_NO_LOCAL_POSE;
    }
    if(elapse_task_list >  FRAME_MISSED*2)
    {
        PlannerInfo("task_list loss %d ", elapse_task_list);
        data_flag |= FLAG_NO_TASK_LIST;
    }
    if(elapse_map_position > FRAME_MISSED)
    {
        PlannerInfo("map_position loss %d ", elapse_map_position);
        data_flag |= FLAG_NO_MAP_POSITION;
    }
    if(elapse_local_hdmap > FRAME_MISSED*2 && elapse_local_hdmapex > FRAME_MISSED*2)
    {
        PlannerInfo("local_hd_map loss %d adn local_hd_map_ex loss %d ", elapse_local_hdmap, elapse_local_hdmapex);
        data_flag |= FLAG_NO_LOCAL_HDMAP;
    }

    if(elapse_prediction_info > FRAME_MISSED*2 && 1 == plan->GetSlowStopMode())
    {
        PlannerInfo("prediction_data loss %d  %d ", elapse_prediction_info);
        data_flag |= FLAG_NO_PREDICTION_DATA;
    }

    if(elapse_formation_command > FRAME_MISSED)
    {
        PlannerInfo("formation_command loss %d  %d ", elapse_formation_command);
        data_flag |= FLAG_NO_FORMATION_COMMAND_DATA;
    }

    if(elapse_formation_info > FRAME_MISSED)
    {
        PlannerInfo("formation_info loss %d  %d ", elapse_formation_info);
        data_flag |= FLAG_NO_FORMATION_INFO_DATA;
    }

    //if(FLAG_ALL_DATA != data_flag && !(FLAG_NO_FORMATION_COMMAND_DATA & data_flag)
    //        && !(FLAG_NO_FORMATION_INFO_DATA & data_flag))
    if ((data_flag & FLAG_NO_FUSION_MSG) || (data_flag & FLAG_NO_LOCAL_POSE) || (data_flag & FLAG_NO_TASK_LIST) 
        || (data_flag & FLAG_NO_MAP_POSITION) || (data_flag & FLAG_NO_LOCAL_HDMAP) || (data_flag & FLAG_NO_PREDICTION_DATA))
    {
        plan->isNoData = 1;
    }
    else
    {
        plan->isNoData = 0;
    }

    plan->data_flag_ = data_flag;


    plan->localPosMissTime = HMath::max( elapse_local_pose, elapse_vehicle_status);
    plan->attrMapMissTime = elapse_fusion_msg;
    plan->entityMapMisstime = elapse_fusion_msg;
    plan->TaskListMissTime = elapse_task_list;
    plan->UltraSonicCellMissTime = elapse_ultrasonic;

    if(planPara.tk_use_hdmapex==1)
    {
        plan->LocalHDMapMissTime = elapse_local_hdmapex;
    }
    else
    {
        plan->LocalHDMapMissTime = elapse_local_hdmap;
    }

    plan->MapPositionMissTime = elapse_map_position;
    plan->RemotePlanMissTime = elapse_remote_operate;
    plan->CurbInfoMissTime = elapse_curb_info;
    plan->PredictionMissTime = elapse_prediction_info;
    plan->FormationCommandMissTime = elapse_formation_command;
    plan->FormationInfoMissTime = elapse_formation_info;

    //    if(elapse_fusion_msg < 4)
    //    {
    //        return true;
    //    }
    //    else
    //    {
    //        return false;
    //    }

    return true;

}


// 写数据
void InterfaceXSCom::Write(BasicPathPlan *plan)
{
    // 清理历史结果
    thread_local_plan_state_data_->Clear();
    thread_local_path_info_data_->Clear();
    thread_local_plan_voice_data_->Clear();
    thread_safety_warning_data_->Clear();
    thread_nec_control_request_data_->Clear();

    double elapse_local_pose = 0;
    {
        // 线程锁
        std::lock_guard<std::mutex> lock(mutex_local_pose_);

        // 获取时间
        elapse_local_pose = cache_localpose_data_->timestamp();
        boost::int64_t time_diff = timer_local_pose_->ElapsedMill() / 1000.0;
        elapse_local_pose += time_diff;
    }

    // 生成规划和控制之间的接口数据
    ProduceLocalPathInfo(plan, elapse_local_pose, thread_fusion_map_data_, thread_local_path_info_data_);

    // 生成规划和业务之间的接口数据
    ProduceLocalPlanState(plan, thread_fusion_map_data_, 
                                thread_vehicle_status_data_->driving_mode(), 
                                thread_local_plan_state_data_);

    // 生成规划和语音提醒的接口数据
    ProduceLocalPlanVoice(plan, thread_fusion_map_data_, thread_local_plan_voice_data_);

    // 生成清扫车的接口数据
    ProduceNecControl(plan, thread_nec_control_request_data_);
    
    // 通信中间件发送数据
    bool path_ret =  local_path_info_writer_->Write(thread_local_path_info_data_);
    // if (!path_ret)
    // {
    //     PlannerError("local_path_info write false ");
    // }
    // else 
    // {
    //     PlannerInfo("local_path_info write true");
    // }
    local_plan_state_writer_->Write(thread_local_plan_state_data_);

    if(thread_local_plan_voice_data_->voice()!=xsproto::planner::Voice_Unknown
            || thread_local_plan_voice_data_->global_task_control() != xsproto::planner::TaskControl_Unknown)
    {
        //PlannerInfo("Local plan voice info is {}", LocalPlanVoiceInfo_data->voice);
        //PlannerInfo("Local plan task control is {}", LocalPlanVoiceInfo_data->global_task_control);
        //writeReturn = LOCALPLAN_VOICE_CHANNEL->write(LocalPlanVoiceInfo_data);
        local_plan_voice_writer_->Write(thread_local_plan_voice_data_);
    }

    nec_control_request_writer_->Write(thread_nec_control_request_data_);
    //WriteLocalPathInfo(thread_local_path_info_data_);
    //WriteLocalPlanState(plan);
    //WriteLiftCommand(plan);
    //WriteLocalPlanVoice(plan);
    //WriteSafetyWarning(plan);
    //WriteNecControl(plan);


    // 发到达后，短时间清除任务
    static int arrive_seq = -1;
    static double arrive_time = 0.0;
    if (xsproto::planner::Arrive_EndPoint == thread_local_plan_state_data_->arrive_type()
            || xsproto::planner::Arrive_ChargeSuccess == thread_local_plan_state_data_->arrive_type())
    {
        // debug模式下，不清理，影响调试
        PlannerInfo("!!! arrive,  ClearTaskBuffer(1)!!!");
        arrive_seq = plan->task_list_->taskSeqNum;
        arrive_time = HMath::getCurrentMilliseconds();
        plan->task_list_->pointNum = 0;
        plan->task_list_->effective_length = 0.0;
        thread_tasklist_data_->Clear();
        cache_tasklist_data_->Clear();
        
        {
            // 线程锁
            std::lock_guard<std::mutex> lock(mutex_task_list_);
            timer_task_list_->Reset();
        }
    }

    // 20210615判断上次到达的seqnum和时间，如果是很短时间内的相同sequm则清理内部全局任务
    double current_time = HMath::getCurrentMilliseconds();
    double time_diff = fabs(arrive_time - current_time);
    if (time_diff < 4000 && plan->task_list_->taskSeqNum == arrive_seq)
    {
        PlannerInfo("!!! arrive,  ClearTaskBuffer(2)!!!");
        plan->task_list_->pointNum = 0;
        plan->task_list_->effective_length = 0.0;
        thread_tasklist_data_->Clear();
        cache_tasklist_data_->Clear();
        {
            // 线程锁
            std::lock_guard<std::mutex> lock(mutex_task_list_);
            timer_task_list_->Reset();
        }
    }
}

void InterfaceXSCom::OnLocalPoseCallback(const std::shared_ptr<xsproto::base::LocalPose>& msg)
{
    // 线程锁
    std::lock_guard<std::mutex> lock(mutex_local_pose_);

    // 更新主线程缓存数据
    cache_localpose_data_->CopyFrom(*msg);
    timer_local_pose_->UpdateToNow();
}
void InterfaceXSCom::OnVehicleStatusCallback(const std::shared_ptr<xsproto::base::VehicleStatus>& msg)
{
    // 线程锁
    std::lock_guard<std::mutex> lock(mutex_vehicle_status_);

    // 更新主线程缓存数据
    cache_vehicle_status_data_->CopyFrom(*msg);
    timer_vehicle_status_->UpdateToNow();
}

void InterfaceXSCom::OnAccelerationEstimationCallback(const std::shared_ptr<xsproto::acceleration_estimation::AccelerationEstimation>& msg)
{
    // 线程锁
    std::lock_guard<std::mutex> lock(mutex_acceleration_estimation_);

    // 更新主线程缓存数据
    cache_acceleration_estimation_data_->CopyFrom(*msg);
    timer_acceleration_estimation_->UpdateToNow();
}

void InterfaceXSCom::OnFusionMsgCallback(const std::shared_ptr<xsproto::perception::FusionMsg>& msg)
{
    // 线程锁
    std::lock_guard<std::mutex> lock(mutex_fusion_msg_);

    // 更新主线程缓存数据
    cache_fusion_map_data_->CopyFrom(*msg);
    timer_fusion_msg_->UpdateToNow();
}
void InterfaceXSCom::OnUltraSonicCallback(const std::shared_ptr<xsproto::perception::UltraSonicData>& msg)
{
    // 线程锁
    std::lock_guard<std::mutex> lock(mutex_ultrasonic_);

    // 更新主线程缓存数据
    cache_ultrasonic_data_->CopyFrom(*msg);
    timer_ultrasonic_->UpdateToNow();
}
void InterfaceXSCom::OnPredictionInfoCallback(const std::shared_ptr<xsproto::perception::PredictionVehicleInfoMsg>& msg)
{
    // 线程锁
    std::lock_guard<std::mutex> lock(mutex_prediction_info_);

    // 更新主线程缓存数据
    cache_prediction_vehicle_data_->CopyFrom(*msg);
    timer_prediction_info_->UpdateToNow();
}
void InterfaceXSCom::OnTrafficLightCallback(const std::shared_ptr<xsproto::perception::TrafficLightInfo>& msg)
{
    // 线程锁
    std::lock_guard<std::mutex> lock(mutex_traffic_light_);

    // 更新主线程缓存数据
    cache_trafficlight_data_->CopyFrom(*msg);
    timer_traffic_light_->UpdateToNow();
}
void InterfaceXSCom::OnRubbishInfoCallback(const std::shared_ptr<xsproto::perception::RubbishInfo>& msg)
{
    // 线程锁
    std::lock_guard<std::mutex> lock(mutex_rubbish_info_);

    // 更新主线程缓存数据
    cache_rubbish_info_data_->CopyFrom(*msg);
    timer_rubbish_info_->UpdateToNow();
}
void InterfaceXSCom::OnCurbInfoCallback(const std::shared_ptr<xsproto::perception::CurbInfoMsg>& msg)
{
    // 线程锁
    std::lock_guard<std::mutex> lock(mutex_curb_info_);

    // 更新主线程缓存数据
    cache_curb_info_data_->CopyFrom(*msg);
    timer_curb_info_->UpdateToNow();
}

void InterfaceXSCom::OnTrafficSignInfoCallback(const std::shared_ptr<xsproto::perception::TrafficSignInfo> &msg)
{
    // 线程锁
    std::lock_guard<std::mutex> lock(mutex_traffic_sign_info_);

    // 更新主线程缓存数据
    cache_traffic_sign_info_->CopyFrom(*msg);
    timer_traffic_sign_info_->UpdateToNow();
}
void InterfaceXSCom::OnTaskListCallback(const std::shared_ptr<xsproto::globalpath::TaskList>& msg)
{
    // 线程锁
    std::lock_guard<std::mutex> lock(mutex_task_list_);

    // 更新主线程缓存数据
    cache_tasklist_data_->CopyFrom(*msg);
    timer_task_list_->UpdateToNow();
}
void InterfaceXSCom::OnRemoteOperateCallback(const std::shared_ptr<xsproto::communication::RemoteOperate>& msg)
{
    // 线程锁
    std::lock_guard<std::mutex> lock(mutex_remote_operate_);

    // 更新主线程缓存数据
    cache_remote_operate_data_->CopyFrom(*msg);
    timer_remote_operate_->UpdateToNow();
}
void InterfaceXSCom::OnFaultInfoCallback(const std::shared_ptr<xsproto::communication::FaultInfo>& msg)
{
    // 线程锁
    std::lock_guard<std::mutex> lock(mutex_fault_info_);

    // 更新主线程缓存数据
    cache_fault_info_data_->CopyFrom(*msg);
    timer_fault_info_->UpdateToNow();
}
void InterfaceXSCom::OnLocalHDMapCallback(const std::shared_ptr<xsproto::hdmap::LocalHDMap>& msg)
{
    // 线程锁
    std::lock_guard<std::mutex> lock(mutex_local_hdmap_);

    // 更新主线程缓存数据
    cache_local_hdmap_data_->CopyFrom(*msg);
    timer_local_hdmap_->UpdateToNow();

}
void InterfaceXSCom::OnLocalHDMapExCallback(const std::shared_ptr<xsproto::hdmap::LocalHDMapEx>& msg)
{
    // 线程锁
    std::lock_guard<std::mutex> lock(mutex_local_hdmapex_);

    // 更新主线程缓存数据
    cache_local_hdmapex_data_->CopyFrom(*msg);
    timer_local_hdmapex_->UpdateToNow();
}

void InterfaceXSCom::OnMapPositionCallback(const std::shared_ptr<xsproto::perception::MapPosition>& msg)
{
    // 线程锁
    std::lock_guard<std::mutex> lock(mutex_map_position_);

    // 更新主线程缓存数据
    cache_map_position_data_->CopyFrom(*msg);
    timer_map_position_->UpdateToNow();
}

void InterfaceXSCom::OnAirportSpecialObjectCallback(const std::shared_ptr<xsproto::perception::AirportSpecialObject>& msg)
{
    // 线程锁
    std::lock_guard<std::mutex> lock(mutex_airport_special_objects_);

    // 更新主线程缓存数据
    cache_airport_special_objects_->CopyFrom(*msg);
    timer_airport_special_objects_->UpdateToNow();
}

void InterfaceXSCom::OnRemoteLightCallback(const std::shared_ptr<xsproto::perception::TrafficLightInfo>& msg)
{
    // 线程锁
    std::lock_guard<std::mutex> lock(mutex_remote_light_);

    // 更新主线程缓存数据
    cache_remote_light_->CopyFrom(*msg);
    timer_remote_light_->UpdateToNow();
}

void InterfaceXSCom::OnFormationCommandCallback(const std::shared_ptr<xsproto::formation::FormationCommand>& msg)
{
    // 线程锁
    std::lock_guard<std::mutex> lock(mutex_formation_command_);

    // 更新主线程缓存数据
    cache_formation_command_data_->CopyFrom(*msg);
    timer_formation_command_->UpdateToNow();
}

void InterfaceXSCom::OnFormationInfoCallback(const std::shared_ptr<xsproto::formation::FormationInfo>& msg)
{
    // 线程锁
    std::lock_guard<std::mutex> lock(mutex_formation_info_);

    // 更新主线程缓存数据
    cache_formation_info_data_->CopyFrom(*msg);
    timer_formation_info_->UpdateToNow();
}

void InterfaceXSCom::WriteLocalPlanLog()
{
    xsproto::planner::LocalPlanLog plan_log;
    for (std::size_t i = 0; i < SINGLETON_SHMLOG.CacheSize(); ++i)
    {
        const std::string& log_str = SINGLETON_SHMLOG.CacheLog(i);
        plan_log.add_str(log_str);
    }
    SINGLETON_SHMLOG.Clear();
    local_plan_log_writer_->Write(plan_log);
}


void InterfaceXSCom::WriteVisualizeImg(const BasicPathPlan *plan, int display)
{
    // 如果规划程序本身开了显示，就不输出
    if (display > 0)
    {
        return ;
    }

    // 保存opencv原始图片
    xsproto::planner::OcvMatInfo ocv_mat_info;
    
    const int img_rows = plan->planResult->visImage.rows;
    const int img_cols = plan->planResult->visImage.cols;
    const int img_type = plan->planResult->visImage.type();
    const int img_ele  = plan->planResult->visImage.elemSize();
    ocv_mat_info.set_rows(img_rows);
    ocv_mat_info.set_cols(img_cols);
    ocv_mat_info.set_type(img_type);
    ocv_mat_info.set_size(img_ele);

    if (img_rows < 100 || img_cols < 100)
    {
        return ;
    }

    size_t img_size = img_rows * img_cols * img_ele;
    ocv_mat_info.set_data(plan->planResult->visImage.data, img_size);
    
    vehicle_result_viewer_writer_->Write(ocv_mat_info);
    // VehicleResultViewerProto_data->len = ocv_mat_info.ByteSize();
    
    // PlannerInfo("write img data, bytes: %d", VehicleResultViewerProto_data->len);
    // if (ocv_mat_info.ByteSize() < VEHICLERESULTVIEWER_PROTO_MSG_SIZE)
    // {
    //     ocv_mat_info.SerializeToArray(VehicleResultViewerProto_data->data, VehicleResultViewerProto_data->len);
    //     int ret = VEHICLERESULTVIEWER_PROTO_CHANNEL->write(VehicleResultViewerProto_data);
    // }
}
