#ifndef NML_DEFINE_H
#define NML_DEFINE_H



//======== perception ========
//perception.sensor
// 1-LIDAR
#define RCS_BUF_LidarMiddleMiddleTop        "buffer_10mb.hh"
#define RCS_BUF_LidarMiddleFrontBottom      "buffer_10mb.hh"
#define RCS_BUF_LidarRightFrontBottom       "buffer_10mb.hh"
#define RCS_BUF_LidarRightMiddleBottom      "buffer_10mb.hh"
#define RCS_BUF_LidarRightRearBottom        "buffer_10mb.hh"
#define RCS_BUF_LidarMiddleRearBottom       "buffer_10mb.hh"
#define RCS_BUF_LidarLeftRearBottom         "buffer_10mb.hh"
#define RCS_BUF_LidarLeftMiddleBottom       "buffer_10mb.hh"
#define RCS_BUF_LidarLeftFrontBottom        "buffer_10mb.hh"
#define RCS_BUF_LidarMiddleFrontTop         "buffer_10mb.hh"
#define RCS_BUF_LidarRightFrontTop          "buffer_10mb.hh"
#define RCS_BUF_LidarRightMiddleTop         "buffer_10mb.hh"
#define RCS_BUF_LidarRightRearTop           "buffer_10mb.hh"
#define RCS_BUF_LidarMiddleRearTop          "buffer_10mb.hh"
#define RCS_BUF_LidarLeftRearTop            "buffer_10mb.hh"
#define RCS_BUF_LidarLeftMiddleTop          "buffer_10mb.hh"
#define RCS_BUF_LidarLeftFrontTop           "buffer_10mb.hh"
//2-Camera
#define RCS_BUF_CameraMiddleFrontShort      "buffer_10mb.hh"
#define RCS_BUF_CameraMiddleFrontLong       "buffer_10mb.hh"
#define RCS_BUF_CameraMiddleFrontSpecial    "buffer_10mb.hh"
#define RCS_BUF_CameraMiddleFront           "buffer_10mb.hh"
#define RCS_BUF_CameraRightFront            "buffer_10mb.hh"
#define RCS_BUF_CameraRightMiddle           "buffer_10mb.hh"
#define RCS_BUF_CameraRightRear             "buffer_10mb.hh"
#define RCS_BUF_CameraMiddleRear            "buffer_10mb.hh"
#define RCS_BUF_CameraLeftRear              "buffer_10mb.hh"
#define RCS_BUF_CameraLeftMiddle            "buffer_10mb.hh"
#define RCS_BUF_CameraLeftFront             "buffer_10mb.hh"
#define RCS_BUF_CameraTrafficlightMain      "buffer_10mb.hh"
#define RCS_BUF_CameraTrafficlightSlave     "buffer_10mb.hh"
#define RCS_BUF_CameraTrafficlight4K        "buffer_25mb.hh"
//3-Radar
#define RCS_BUF_RadarMiddleFront                "buffer_1mb.hh"
#define RCS_BUF_RadarRightFront                 "buffer_1mb.hh"
#define RCS_BUF_RadarRightMiddle                "buffer_1mb.hh"
#define RCS_BUF_RadarRightRear                  "buffer_1mb.hh"
#define RCS_BUF_RadarMiddleRear                 "buffer_1mb.hh"
#define RCS_BUF_RadarLeftRear                   "buffer_1mb.hh"
#define RCS_BUF_RadarLeftMiddle                 "buffer_1mb.hh"
#define RCS_BUF_RadarLeftFront                  "buffer_1mb.hh"
#define RCS_BUF_RadarDetector                   "buffer_1mb.hh"
//4-Ultrasonic
#define RCS_BUF_UltrasonicData                  "buffer_5kb.hh"
//5-LIDAR Fusion
#define RCS_BUF_SyncLidarData                   "buffer_20mb.hh"
#define RCS_BUF_SyncLidarData4Map               "buffer_20mb.hh"
//6-Camera-Lidar Fusion
#define RCS_BUF_MiddleFrontCLFusionData         "buffer_20mb.hh"
#define RCS_BUF_LeftFrontCLFusionData           "buffer_20mb.hh"
#define RCS_BUF_RightFrontCLFusionData          "buffer_20mb.hh"
#define RCS_BUF_MiddleRearCLFusionData          "buffer_20mb.hh"
//7-Camera-Radar Fusion
#define RCS_BUF_MiddleFrontCRFusionData         "buffer_10mb.hh"
#define RCS_BUF_MiddleRearCRFusionData          "buffer_10mb.hh"
//8-Lidar-Segmentation
#define RCS_BUF_LidarObstaclePointCloud	       	"buffer_10mb.hh"

//perception.autodrive
//1-目标
#define RCS_BUF_LidarObjectInfo                 "buffer_1mb.hh"
#define RCS_BUF_FrontObjectInfo                 "buffer_1mb.hh"
#define RCS_BUF_RearObjectInfo                  "buffer_1mb.hh"
#define RCS_BUF_CameraObjectInfo                "buffer_1mb.hh"
#define RCS_BUF_RadarObjectInfo                 "buffer_1mb.hh"
#define RCS_BUF_FusionObjectInfo                "buffer_1mb.hh"
#define RCS_BUF_AroundLidarObjectInfo           "buffer_1mb.hh"
#define RCS_BUF_FrontLidarObjectInfo            "buffer_1mb.hh"
#define RCS_BUF_RearLidarObjectInfo             "buffer_1mb.hh"
#define RCS_BUF_AroundCameraObjectInfo          "buffer_1mb.hh"
#define RCS_BUF_FrontCameraObjectInfo           "buffer_1mb.hh"
#define RCS_BUF_RearCameraObjectInfo            "buffer_1mb.hh"
#define RCS_BUF_AroundRadarObjectInfo           "buffer_1mb.hh"
#define RCS_BUF_FrontRadarObjectInfo            "buffer_1mb.hh"
#define RCS_BUF_RearRadarObjectInfo             "buffer_1mb.hh"
#define RCS_BUF_AroundUSSObjectInfo             "buffer_1mb.hh"
#define RCS_BUF_LateFusionObjectInfo            "buffer_1mb.hh"
//2-环境障碍
#define RCS_BUF_AroundEnvObstacle               "buffer_1mb.hh"
#define RCS_BUF_FrontEnvObstacle                "buffer_1mb.hh"
#define RCS_BUF_RearEnvObstacle                 "buffer_1mb.hh"
#define RCS_BUF_UltraSonicEnvObstacle           "buffer_1mb.hh"
#define RCS_BUF_FusionEnvObstacle               "buffer_1mb.hh"
//3-交通信息
#define RCS_BUF_TrafficLightInfo                "buffer_10kb.hh"
#define RCS_BUF_TrafficSignInfo                 "buffer_10kb.hh"
#define RCS_BUF_RoadCurbInfo                    "buffer_10kb.hh"

//perception.service
//1-零售: 招手识别
#define RCS_BUF_BeckonInfo                      "buffer_10kb.hh"
//2-清扫: 路沿检测
#define RCS_BUF_RoadCurbEdge                    "buffer_10kb.hh"
//       垃圾识别
#define RCS_BUF_RubbishInfo                     "buffer_10kb.hh"


#define RCS_BUF_LocalPose                       "buffer_10kb.hh"
#define RCS_BUF_VehicleStatus                   "buffer_10kb.hh"
#define RCS_BUF_GlobalPose                      "buffer_10kb.hh"


//======== localization ========
#define RCS_BUF_MapPosition                     "buffer_20kb.hh"
#define RCS_BUF_LidarPosition			"buffer_20kb.hh"

//======== hdmap ========
#define RCS_BUF_LocalHDMap                      "buffer_2mb.hh"

//======== global path ========
#define RCS_BUF_TaskList                        "buffer_500kb.hh"
#define RCS_BUF_TaskStateInfo                   "buffer_50kb.hh"
#define RCS_BUF_TaskResponse                    "buffer_1mb.hh"

//======== fusion ========
#define RCS_BUF_Fusion                          "buffer_2mb.hh"

//======== prediction =======
#define RCS_BUF_Prediction                      "buffer_1mb.hh"

//======== planner ========
#define RCS_BUF_LocalPathInfo                   "buffer_20kb.hh"
#define RCS_BUF_LocalPlanLog                    "buffer_500kb.hh"
#define RCS_BUF_LocalPlanState                  "buffer_20kb.hh"
#define RCS_BUF_LocalPlanVoice                  "buffer_20kb.hh"


//======== control ========


//======== monitor ========


//======== CI ========
#define RCS_BUF_FaultInfo                            "buffer_500kb.hh"
#define RCS_BUF_TaskRequest                          "buffer_20kb.hh"
#define RCS_BUF_ReloadMapRequest                     "buffer_2kb.hh"
#define RCS_BUF_ReloadMapResponse                    "buffer_2kb.hh"
#define RCS_BUF_DataCollectRequest                   "buffer_10kb.hh"
#define RCS_BUF_DataCollectResponse                  "buffer_10kb.hh"
#define RCS_BUF_StationCmd                           "buffer_5kb.hh"
#define RCS_BUF_MotionCmd                            "buffer_10kb.hh"
#define RCS_BUF_MotionControlResponse                "buffer_10kb.hh"
#define RCS_BUF_MotionControlRequest                 "buffer_10kb.hh"
#define RCS_BUF_RubbishDetectorControlRequest        "buffer_5kb.hh"
#define RCS_BUF_RubbishDetectorControlResponse       "buffer_5kb.hh"

#define RCS_BUF_MapperRequest                        "buffer_10kb.hh"
#define RCS_BUF_MapperResponse                       "buffer_20kb.hh"
#define RCS_BUF_PathNetInfo                          "buffer_5kb.hh"

#define RCS_BUF_LocalisationRequest                  "buffer_10kb.hh"
#define RCS_BUF_LocalisationResponse                 "buffer_20kb.hh"
#define RCS_BUF_LocalisationState                    "buffer_5kb.hh"


#define RCS_BUF_GlobalStatus                         "buffer_1kb.hh"
#define RCS_BUF_MapSyncRequest                       "buffer_10kb.hh"

#define RCS_BUF_RemoteOperate                        "buffer_5kb.hh"
#define RCS_BUF_RouteRecordRequest                   "buffer_20kb.hh"
#define RCS_BUF_RouteRecordResponse                  "buffer_10kb.hh"

#define RCS_BUF_BeckonInfo                           "buffer_10kb.hh"    
#define RCS_BUF_SafetyWarning                        "buffer_2kb.hh"
#define RCS_BUF_VehicleExtraInfo                     "buffer_1kb.hh"    

#define RCS_BUF_NecControlResponse                   "buffer_5kb.hh"
#define RCS_BUF_ReloadMapTrafficMapDetector          "buffer_2kb.hh"
#define RCS_BUF_ReloadMapLocalizerInit               "buffer_2kb.hh"
#define RCS_BUF_ReloadMapLocalizer3D_xs              "buffer_2kb.hh"
#define RCS_BUF_SelfCheck                            "buffer_2kb.hh"

//======== interface ========
#define RCS_BUF_LiftState                        "buffer_5kb.hh"
#define RCS_BUF_LiftCommand                      "buffer_5kb.hh"
#define RCS_BUF_NecControl                       "buffer_5kb.hh"
#define RCS_BUF_RemoteEntity                     "buffer_500kb.hh"
#define RCS_BUF_RemoteLight                      "buffer_10kb.hh"

//======== sys ========
//=========== sys ===================
#define RCS_BUF_SysData           "buffer_5kb.hh"
#endif  // NML_DEFINE_H
