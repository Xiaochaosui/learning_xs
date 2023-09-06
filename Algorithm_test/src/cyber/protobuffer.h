#ifndef PROTONODECLS_H
#define PROTONODECLS_H

//<------ node/channels ------>
#include "xscom/xscom.h"
// ---- macros ----
#include "channeldefine.h"
#include "nodedefine.h"

//<------------- data structure of messages ------->
// obstacle map
#include "perception/environment_obstacle_info.pb.h"
// objects
#include "perception/obstacle_object_info.pb.h"
#include "perception/perception_object_info.pb.h"
// traffic light
#include "perception/traffic_light_info.pb.h"
// traffic sign
#include "perception/traffic_sign_info.pb.h"
// vector diagram
#include "hdmap/local_hdmap.pb.h"
// traffic map intersection
#include "hdmap/trafficmap_intesersection.pb.h"

//automatic driving targets
#include "business/remote_entity.pb.h"

//tasklist: indoor/outdoor
#include "globalpath/task_list.pb.h"

//output msg: fusion
#include "perception/fusion_map.pb.h"
//output msg: prediction
#include "perception/prediction.pb.h"
#include "perception/prediction_obstacle.pb.h"

#include "perception/radar_data.pb.h"
#include "perception/lidar_data.pb.h"
#include "perception/radar_object_detector.pb.h"

#include "perception/map_position.pb.h"
#include "perception/sync_lidar_data.pb.h"
//plan
#include "planner/local_path_info.pb.h"
//----------------- abbreviation of messages -----------------
/// required items (fusion)
// input #1: obstacle
//using Env_Obstacle_Msg = xsproto::perception::EnvironmentObstacleOldInfo;
using Env_Obstacle_Msg = xsproto::perception::EnvironmentObstacleInfo;
// input #2: objects
using Env_Fusion_Msg = xsproto::perception::FusionMsg;
using Env_Object_Msg = xsproto::perception::ObstacleObjectInfo;
using Perception_Object_Msg = xsproto::perception::PerceptionObjectInfo;
// input #3: LP
using LocalPose_Msg = xsproto::base::LocalPose;

/// optional items (prediction)
// input local hd map
using LocalHDMap_Msg = xsproto::hdmap::LocalHDMap;
// intersection
using InterSection_Msg = xsproto::hdmap::TrafficMapIntersectionMsg;

// traffic light
using Traffic_Light_Msg = xsproto::perception::TrafficLightInfo;
// traffic sign
using Traffic_Sign_Msg = xsproto::perception::TrafficSignInfo;

//radar
using Radar_Data_Msg = xsproto::perception::RadarData;
using Radar_Target_Msg = xsproto::perception::RadarObjectDetector;
//Lidar
using Lidar_Data_Msg = xsproto::perception::LidarData;
using Sync_Lidar_Msg = xsproto::perception::SyncLidarData;

//autonomous vehicles
using AutoTarget_Msg = xsproto::communication::RemoteEntity;

//task list proto
using TaskListProto_Msg = xsproto::globalpath::TaskList;

//map position
using MapPosition_Msg = xsproto::perception::MapPosition;

//outputs of fusion
using Fusion_Msg = xsproto::perception::FusionMsg;
using Prediction_Msg = xsproto::perception::PredictionVehicleInfoMsg;
using PredictionInfo_Msg = xsproto::perception::PredictionObstacles;
//<---- abbreviation of messages ---->

//plan
using Plan_Msg = xsproto::planner::LocalPathInfo;
#endif // PROTONODECLS_H
