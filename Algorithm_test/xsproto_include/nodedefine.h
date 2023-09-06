#ifndef NODEDEFINE_H
#define NODEDEFINE_H


//==========globalpath==========
#define NODE_global_path_task                   "global_path_task"



//=========planner===========
#define NODE_planner                            "local_planner"
#define NODE_planner_log                        "local_planner_log"
#define NODE_planner_show                       "local_planner_show"


//==========CI==========
#define NODE_communication_interface            "communication_interface"

//==========FaultReceive=======
#define NODE_fault_receive                      "fault_receive"



//==========hdmap==========
#define NODE_traffic_map_detector		"traffic_map_detector"



//==========environmental fusion==========
// environmental fusion (now)
#define NODE_fused_predictor			"fused_predictor"
// reserved nodes (future)
#define NODE_env_fusion			        "env_fusion"
#define NODE_traj_predictor			"traj_predictor"



//==========perception==========
//perception.sensor
// 1-LIDAR
#define NODE_lidar_receiver			"lidar_receiver"
// 2-CAMERA
#define NODE_camera_receiver			"camera_receiver"
// 3-Radar
#define NODE_radar_receiver			"radar_receiver"
#define NODE_radar_object_detector              "radar_object_detector"
// 4-Ultrasonic
#define NODE_ultrasonic_receiver		"ultrasonic_receiver"
// 5-SensorFusion
#define NODE_lidar_synchronizator		"lidar_synchronizator"
#define NODE_camera_lidar_synchronizator	"camera_lidar_synchronizator"
#define NODE_camera_radar_synchronizator	"camera_radar_synchronizator"

//perception.autodrive
#define NODE_lidar_object_detector	        "lidar_object_detector"
#define NODE_front_object_detector	        "front_object_detector"
#define NODE_low_obstacle_detector          "low_obstacle_detector"
#define NODE_perception_fuser	                "perception_fuser"
#define NODE_trafficlight_detector	        "trafficlight_detector"
#define NODE_enhance_trafficlight_detector	"enhance_trafficlight_detector"
#define NODE_trafficsign_detector           "trafficsign_detector"
#define NODE_dual_traffic_light_detector    "dual_traffic_light_detector"
#define NODE_lift_platform_vehicle_detector "lift_platform_vehicle_detector"

//perception.service
#define NODE_beckon_detector	                "beckon_detector"
#define NODE_rubbish_detector	                "rubbish_detector"

//road curb detection
#define NODE_road_curb_detector                 "road_curb_detector"
#define NODE_dl_road_curb_detector              "dl_road_curb_detector"

//==========localizer and mapper ==========
//Localizer
#define NODE_fusion_localizer	                "fusion_localizer"
#define NODE_map_localizer3d                    "map_localizer3d"
#define NODE_map_localizer2d                    "map_localizer2d"
#define NODE_localizer_init                     "localizer_init"
//mapper
#define NODE_mapper_android                     "mapper_android"



//==========monitor==========
#define NODE_data_record			"data_record"
#define NODE_data_player			"data_player"


//========== ugv interface ==========
// ugv interface
#define NODE_ugv_interface	                "ugv_interface"
// global pose
#define NODE_global_pose	                "global_pose"
// local pose
#define NODE_local_pose	                    	"local_pose"

//========== sys ==========
#define NODE_sys_info                       "sysinfo_receiver"

#endif // NODEDEFINE_H

