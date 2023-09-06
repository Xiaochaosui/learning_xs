#include "CyberMsg.h"

CyberRTMsg::CyberRTMsg()
{
    main_node_ = xscom::CreateNode(NODE_lidar_object_detector);
}

CyberRTMsg *CyberRTMsg::instance()
{
    static CyberRTMsg obj;
    return &obj;
}

std::shared_ptr<xscom::Reader<Env_Obstacle_Msg> > CyberRTMsg::ObstacleReader()
{
    return main_node_->CreateReader<Env_Obstacle_Msg>(CHANNEL_FusionEnvObstacle);
}
//[xcsy]
std::shared_ptr<xscom::Writer<Env_Obstacle_Msg> > CyberRTMsg::ObstacleWriter()
{
    return main_node_->CreateWriter<Env_Obstacle_Msg>(CHANNEL_FusionEnvObstacle);
}


std::shared_ptr<xscom::Reader <Env_Object_Msg> > CyberRTMsg::ObjectInfoReader()
{
    return main_node_->CreateReader<Env_Object_Msg>(CHANNEL_FusionObjectInfo);
}

std::shared_ptr<xscom::Reader <Perception_Object_Msg> > CyberRTMsg::PerceptionObjectReader()
{
    return main_node_->CreateReader<Perception_Object_Msg>(CHANNEL_LateFusionObjectInfo);
}

std::shared_ptr<xscom::Reader <Env_Fusion_Msg> > CyberRTMsg::EnvFusionReader()
{
    return main_node_->CreateReader<Env_Fusion_Msg>(CHANNEL_Env_Fusion);
}

std::shared_ptr<xscom::Reader <LocalPose_Msg> > CyberRTMsg::LocalPoseReader()
{
    return main_node_->CreateReader<LocalPose_Msg>(CHANNEL_LocalPose);
}

std::shared_ptr<xscom::Reader <Traffic_Light_Msg> > CyberRTMsg::TrafficLightReader()
{
    return main_node_->CreateReader<Traffic_Light_Msg>(CHANNEL_TrafficLightInfo);
}

std::shared_ptr<xscom::Reader <AutoTarget_Msg> > CyberRTMsg::AutoTargetReader()
{
    return main_node_->CreateReader<AutoTarget_Msg>(CHANNEL_RemoteEntity);
}

std::shared_ptr<xscom::Reader <Traffic_Sign_Msg> > CyberRTMsg::TrafficSignReader()
{
    return main_node_->CreateReader<Traffic_Sign_Msg>(CHANNEL_TrafficSignInfo);
}

std::shared_ptr<xscom::Reader <TaskListProto_Msg> > CyberRTMsg::TaskListProtoReader()
{
    return main_node_->CreateReader<TaskListProto_Msg>(CHANNEL_TaskList);
}

std::shared_ptr<xscom::Reader <LocalHDMap_Msg> > CyberRTMsg::HDMapReader()
{
    return main_node_->CreateReader<LocalHDMap_Msg>(CHANNEL_LocalHDMap);
}

std::shared_ptr<xscom::Reader <InterSection_Msg> > CyberRTMsg::InterSectionReader()
{
    return main_node_->CreateReader<InterSection_Msg>(CHANNEL_TrafficMapIntersection);
}

std::shared_ptr<xscom::Reader <MapPosition_Msg> > CyberRTMsg::MapPositionReader()
{
    return main_node_->CreateReader<MapPosition_Msg>(CHANNEL_MapPosition);
}

std::shared_ptr<xscom::Reader <Radar_Target_Msg> > CyberRTMsg::RadarObjectReader()
{
    return main_node_->CreateReader<Radar_Target_Msg>(CHANNEL_RadarObjectInfo);
}

std::shared_ptr<xscom::Writer <Fusion_Msg> > CyberRTMsg::EnvFusionWriter()
{
    return main_node_->CreateWriter<Fusion_Msg>(CHANNEL_Env_Fusion);
}

//std::shared_ptr<xscom::Writer <Prediction_Msg> > CyberRTMsg::CreatePredictionWriter()
//{
//    return main_node_->CreateWriter<Predicmaketion_Msg>(CHANNEL_Traj_Prediction);
//}


std::shared_ptr<xscom::Writer <PredictionInfo_Msg> > CyberRTMsg::PredictionInfoWriter()
{
    return main_node_->CreateWriter<PredictionInfo_Msg>(CHANNEL_PredictionInfo);
}


//<------    Lidar and Radar     ------>
std::shared_ptr<xscom::Reader<Lidar_Data_Msg> > CyberRTMsg::LidarReader(const std::string& lidar_channel_name)
{
    return main_node_->CreateReader<Lidar_Data_Msg>(lidar_channel_name);
}



std::shared_ptr<xscom::Reader<Radar_Data_Msg> > CyberRTMsg::RaderReader(const std::string& radar_channel_name)
{
    return main_node_->CreateReader<Radar_Data_Msg>(radar_channel_name);
}


std::shared_ptr<xscom::Reader<Sync_Lidar_Msg> > CyberRTMsg::SyncLidarReader()
{
    return main_node_->CreateReader<Sync_Lidar_Msg>(CHANNEL_SyncLidarData);
}
std::shared_ptr<xscom::Reader<Plan_Msg> > CyberRTMsg::PlanReader()
{
     return main_node_->CreateReader<Plan_Msg>(CHANNEL_LocalPathInfo);
}
