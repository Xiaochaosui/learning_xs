#ifndef CHANNELDEFINE_H
#define CHANNELDEFINE_H

//CHANNE通信通道名定义
//==========globalpath==========
#define CHANNEL_TaskList                    	"TaskList"
#define CHANNEL_TaskPath                    	"TaskPath"
#define CHANNEL_TaskStateInfo                   "TaskStateInfo"
#define CHANNEL_TaskResponse                    "TaskResponse"

//==========hdmap==========
#define CHANNEL_LocalHDMap                  	"LocalHDMap"
#define CHANNEL_TrafficMap                  	"TrafficMap"
#define CHANNEL_LocalHDMap_XSLanenet        	"LocalHDMapXSLanenet"
#define CHANNEL_TrafficMapIntersection      	"TrafficMapIntersection"



//==========localization==========
#define CHANNEL_MapPositionInit             	"MapPositionInit"
#define CHANNEL_MapPosition                 	"MapPosition"
#define CHANNEL_LidarPosition                   "LidarPosition"




//==========monitor==========
#define CHANNEL_DataCollection			"DataCollection"




//==========perception==========
//perception.sensor
// 1-LIDAR
#define CHANNEL_LidarMiddleMiddleTop		"LidarMiddleMiddleTop"
#define CHANNEL_LidarMiddleFrontBottom		"LidarMiddleFrontBottom"
#define CHANNEL_LidarRightFrontBottom		"LidarRightFrontBottom"
#define CHANNEL_LidarRightMiddleBottom		"LidarRightMiddleBottom"
#define CHANNEL_LidarRightRearBottom		"LidarRightRearBottom"
#define CHANNEL_LidarMiddleRearBottom		"LidarMiddleRearBottom"
#define CHANNEL_LidarLeftRearBottom             "LidarLeftRearBottom"
#define CHANNEL_LidarLeftMiddleBottom		"LidarLeftMiddleBottom"
#define CHANNEL_LidarLeftFrontBottom		"LidarLeftFrontBottom"
#define CHANNEL_LidarMiddleFrontTop             "LidarMiddleFrontTop"
#define CHANNEL_LidarRightFrontTop              "LidarRightFrontTop"
#define CHANNEL_LidarRightMiddleTop             "LidarRightMiddleTop"
#define CHANNEL_LidarRightRearTop               "LidarRightRearTop"
#define CHANNEL_LidarMiddleRearTop              "LidarMiddleRearTop"
#define CHANNEL_LidarLeftRearTop                "LidarLeftRearTop"
#define CHANNEL_LidarLeftMiddleTop              "LidarLeftMiddleTop"
#define CHANNEL_LidarLeftFrontTop               "LidarLeftFrontTop"
//2-Camera
#define CHANNEL_CameraMiddleFrontShort		"CameraMiddleFrontShort"
#define CHANNEL_CameraMiddleFrontLong		"CameraMiddleFrontLong"
#define CHANNEL_CameraMiddleFrontSpecial	"CameraMiddleFrontSpecial"
#define CHANNEL_CameraMiddleFront               "CameraMiddleFront"
#define CHANNEL_CameraRightFront                "CameraRightFront"
#define CHANNEL_CameraRightMiddle               "CameraRightMiddle"
#define CHANNEL_CameraRightRear                 "CameraRightRear"
#define CHANNEL_CameraMiddleRear                "CameraMiddleRear"
#define CHANNEL_CameraLeftRear                  "CameraLeftRear"
#define CHANNEL_CameraLeftMiddle                "CameraLeftMiddle"
#define CHANNEL_CameraLeftFront                 "CameraLeftFront"
#define CHANNEL_CameraTrafficlightMain      	"CameraTrafficlightMain"
#define CHANNEL_CameraTrafficlightSlave	        "CameraTrafficlightSlave"
#define CHANNEL_CameraTrafficlight4K  	        "CameraTrafficlight4K"
//3-Radar
#define CHANNEL_RadarMiddleFront                "RadarMiddleFront"
#define CHANNEL_RadarRightFront                 "RadarRightFront"
#define CHANNEL_RadarRightMiddle                "RadarRightMiddle"
#define CHANNEL_RadarRightRear                  "RadarRightRear"
#define CHANNEL_RadarMiddleRear                 "RadarMiddleRear"
#define CHANNEL_RadarLeftRear                   "RadarLeftRear"
#define CHANNEL_RadarLeftMiddle                 "RadarLeftMiddle"
#define CHANNEL_RadarLeftFront                  "RadarLeftFront"
#define CHANNEL_RadarDetector  			"RadarDetector"

//4-Ultrasonic
#define CHANNEL_UltrasonicData                  "UltrasonicData"
//5-LIDAR Fusion
#define CHANNEL_SyncLidarData                   "SyncLidarData"
#define CHANNEL_SyncLidarData4Map               "SyncLidarData4Map"

//6-Camera-Lidar Fusion
#define CHANNEL_MiddleFrontCLFusionData		"MiddleFrontCLFusionData"
#define CHANNEL_LeftFrontCLFusionData		"LeftFrontCLFusionData"
#define CHANNEL_RightFrontCLFusionData		"RightFrontCLFusionData"
#define CHANNEL_MiddleRearCLFusionData		"MiddleRearCLFusionData"
//7-Camera-Radar Fusion
#define CHANNEL_MiddleFrontCRFusionData		"MiddleFrontCRFusionData"
#define CHANNEL_MiddleRearCRFusionData		"MiddleRearCRFusionData"

//8-Lidar-Segmentation
#define CHANNEL_LidarObstaclePointCloud	       	"LidarObstaclePointCloud"

//perception.autodrive
//1-目标
#define CHANNEL_LidarObjectInfo                 "LidarObjectInfo"
#define CHANNEL_FrontObjectInfo                 "FrontObjectInfo"
#define CHANNEL_RearObjectInfo                  "RearObjectInfo"
#define CHANNEL_CameraObjectInfo                "CameraObjectInfo"
#define CHANNEL_RadarObjectInfo                 "RadarObjectInfo"
#define CHANNEL_FusionObjectInfo                "FusionObjectInfo"
#define CHANNEL_AroundLidarObjectInfo           "AroundLidarObjectInfo"
#define CHANNEL_FrontLidarObjectInfo            "FrontLidarObjectInfo"
#define CHANNEL_RearLidarObjectInfo             "RearLidarObjectInfo"
#define CHANNEL_AroundCameraObjectInfo          "AroundCameraObjectInfo"
#define CHANNEL_FrontCameraObjectInfo           "FrontCameraObjectInfo"
#define CHANNEL_RearCameraObjectInfo            "RearCameraObjectInfo"
#define CHANNEL_AroundRadarObjectInfo           "AroundLidarObjectInfo"
#define CHANNEL_FrontRadarObjectInfo            "FrontLidarObjectInfo"
#define CHANNEL_RearRadarObjectInfo             "RearLidarObjectInfo"
#define CHANNEL_AroundUSSObjectInfo             "AroundUSSObjectInfo"
#define CHANNEL_LateFusionObjectInfo            "LateFusionObjectInfo"
//2-环境障碍
#define CHANNEL_AroundEnvObstacle               "AroundEnvObstacle"
#define CHANNEL_FrontEnvObstacle                "FrontEnvObstacle"
#define CHANNEL_RearEnvObstacle                 "RearEnvObstacle"
#define CHANNEL_UltraSonicEnvObstacle           "UltraSonicEnvObstacle"
#define CHANNEL_FusionEnvObstacle               "FusionEnvObstacle"
//3-交通信息
#define CHANNEL_TrafficLightInfo                "TrafficLightInfo"
#define CHANNEL_TrafficSignInfo                 "TrafficSignInfo"
#define CHANNEL_RoadCurbInfo                    "RoadCurbInfo"

//perception.service
//1-零售
#define CHANNEL_BeckonInfo                      "BeckonInfo"              //招手识别
//2-清扫
#define CHANNEL_RoadCurbEdge                    "RoadCurbEdge"          //路沿检测
#define CHANNEL_RubbishInfo                     "RubbishInfo"            //垃圾识别

//3-机场升降平台车
#define CHANNEL_LiftPlatformVehicles            "LiftPlatformVehicles"


//==========Environmental Fusion==========
//perception.fused_predictor
//1-fusion data
#define CHANNEL_Env_Fusion                      "EnvFusion"     
#define CHANNEL_MultiSensorFusion               "MultiSensorFusion"
//2-prediction data
#define CHANNEL_Traj_Prediction                 "TrajPrediction"
#define CHANNEL_PredictionInfo                  "PredictionInfo"


 
//==========planner==========
#define CHANNEL_LocalPathInfo      		"LocalPathInfo"
#define CHANNEL_LocalPlanState     		"LocalPlanState"
#define CHANNEL_SafetyWarning      		"SafetyWarning"
#define CHANNEL_LiftCommand        		"LiftCommand"
#define CHANNEL_LocalPlanVoice     		"LocalPlanVoice"
#define CHANNEL_VehicleResultViwer 		"VehicleResultViwer"
#define CHANNEL_NECControl         		"NECControl"
#define CHANNEL_LocalPlanLog                    "LocalPlanLog"



//==========CI==========
#define CHANNEL_MotionCmd                       "MotionCmd"
#define CHANNEL_MapperRequest                   "MapperRequest"
#define CHANNEL_MapperResponse                  "MapperResponse"
#define CHANNEL_MaySyncRequest                  "MapSyncRequest"
#define CHANNEL_MapSyncResponse                 "MapSyncResponse"
#define CHANNEL_RouteRecordRequest              "RouteRecordRequest"
#define CHANNEL_RouteRecordResponse             "RouteRecordResponse"
#define CHANNEL_LocalisationInitRequest         "LocalisationInitRequest"
#define CHANNEL_LocalisationInitResponse        "LocalisationInitResponse"
#define CHANNEL_LocalisationInitState           "LocalisationInitState"
#define CHANNEL_TaskRequest                     "TaskRequest"
#define CHANNEL_RemoteOperate                   "RemoteOperate"
#define CHANNEL_ReloadMapRequest                "ReloadMapRequest"
#define CHANNEL_ReloadMapResponse               "ReloadMapResponse"
#define CHANNEL_MotionControlRequest            "MotionControlRequest"
#define CHANNEL_MotionControlResponse           "MotionControlResponse"
#define CHANNEL_NecControlRequest               "NecControlRequest"
#define CHANNEL_NecControlResponse              "NecControlResponse"
#define CHANNEL_GlobalStatus                    "GlobalStatus"
#define CHANNEL_FLocalPose                      "FLocalPose"
#define CHANNEL_DataCollectRequest              "DataCollectRequest"
#define CHANNEL_DataCollectResponse             "DataCollectResponse"
#define CHANNEL_StationCmd                      "StationCmd"
#define CHANNEL_PathNetInfo                     "PathNetInfo"
#define CHANNEL_RemoteEntity                    "RemoteEntity"
#define CHANNEL_RemoteLight                     "RemoteLight"
#define CHANNEL_TaskBusInfo                     "TaskBusInfo"    //全局路公交报站信息
#define CHANNEL_RubbishDetectorControlRequest   "RubbishDetectorControlRequest"	   //垃圾识别开关控制
#define CHANNEL_RubbishDetectorControlResponse  "RubbishDetectorControlResponse"   //垃圾识别控制回复
#define CHANNEL_VehicleExtraInfo                "VehicleExtraInfo"
#define CHANNEL_ReloadMapTrafficMapDetector     "ReloadMapTrafficMapDetector"
#define CHANNEL_ReloadMapLocalizerInit          "ReloadMapLocalizerInit"
#define CHANNEL_ReloadMapLocalizer3D_xs         "ReloadMapLocalizer3D_xs"
#define CHANNEL_SelfCheck                       "SelfCheck"


//=========FaultReceive=======
#define CHANNEL_FaultInfo                       "FaultInfo"



//========== UGV Interface ==========
#define CHANNEL_VehicleStatus             	"VehicleStatus"
#define CHANNEL_GlobalPose                	"GlobalPose"
#define CHANNEL_LocalPose                 	"LocalPose"
#define CHANNEL_LocalPose_20                 	"LocalPose_20"

//========= Fusion Localizer Use ========
#define CHANNEL_FusionGlobalPose              "FusionGlobalPose"
#define CHANNEL_GlobalPose_ToFusion           "GlobalPose_ToFusion"
#define CHANNEL_LocalPose_ToFusion           "LocalPose_ToFusion"

#define CHANNEL_LiftState                       "LiftState"

//=========== Control ===================
#define CHANNEL_ControlCommand                  "ControlCommand"
#define CHANNEL_AccelerationEstimation          "AccelerationEstimation"

//=========== sys ===================
#define CHANNEL_SysData                         "SysData"


#endif // CHANNELDEFINE_H
