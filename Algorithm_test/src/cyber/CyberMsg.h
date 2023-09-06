#ifndef CYBERMSGINSTANCE_H
#define CYBERMSGINSTANCE_H

#include "protobuffer.h"

class CyberRTMsg
{
    std::unique_ptr<xscom::Node> main_node_;
private:
    CyberRTMsg();

public:

    static CyberRTMsg* instance();

public:
    std::shared_ptr<xscom::Reader<Plan_Msg> > PlanReader();
    std::shared_ptr<xscom::Reader<Env_Obstacle_Msg> > ObstacleReader();
    std::shared_ptr<xscom::Writer<Env_Obstacle_Msg> > ObstacleWriter();

    std::shared_ptr<xscom::Reader <Env_Object_Msg> > ObjectInfoReader();
    std::shared_ptr<xscom::Reader <Perception_Object_Msg> > PerceptionObjectReader();
    std::shared_ptr<xscom::Reader <Env_Fusion_Msg> > EnvFusionReader();
    std::shared_ptr<xscom::Reader <LocalPose_Msg> > LocalPoseReader();
    std::shared_ptr<xscom::Reader <Traffic_Light_Msg> > TrafficLightReader();
    std::shared_ptr<xscom::Reader <AutoTarget_Msg> > AutoTargetReader();
    std::shared_ptr<xscom::Reader <Traffic_Sign_Msg> > TrafficSignReader();
    std::shared_ptr<xscom::Reader <TaskListProto_Msg> > TaskListProtoReader();
    std::shared_ptr<xscom::Reader <LocalHDMap_Msg> > HDMapReader();
    std::shared_ptr<xscom::Reader <InterSection_Msg> > InterSectionReader();
    std::shared_ptr<xscom::Reader <MapPosition_Msg> > MapPositionReader();
    std::shared_ptr<xscom::Reader <Radar_Target_Msg> > RadarObjectReader();
    std::shared_ptr<xscom::Writer <Fusion_Msg> > EnvFusionWriter();
    std::shared_ptr<xscom::Writer <Prediction_Msg> > CreatePredictionWriter();
    std::shared_ptr<xscom::Writer <PredictionInfo_Msg> > PredictionInfoWriter();

    std::shared_ptr<xscom::Reader<Lidar_Data_Msg>> LidarReader(const std::string& lidar_channel_name);
    std::shared_ptr<xscom::Reader<Radar_Data_Msg>> RaderReader(const std::string& radar_channel_name);

    void RemoveLidarReader(const std::string& lidar_channel_name);
    void RemoveRaderReader(const std::string& radar_channel_name);

    std::shared_ptr<xscom::Reader<Sync_Lidar_Msg> > SyncLidarReader();
};

#endif
