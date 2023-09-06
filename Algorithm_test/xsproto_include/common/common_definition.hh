/*
 *
*	New hh File starts here.
*/
#ifndef COMMONDEFINITION_HH
#define COMMONDEFINITION_HH

namespace xsproto {
namespace common {

#ifndef __INT64__
#define __INT64__
typedef  signed long long        INT64;
#endif

#ifndef __UINT64__
#define __UINT64__
typedef  unsigned long long     UINT64;
#endif

#ifndef __INT32__
#define __INT32__
typedef  signed     int         INT32;
#endif

#ifndef __UINT32__
#define __UINT32__
typedef  unsigned   int         UINT32;
#endif

#ifndef __INT16__
#define __INT16__
typedef  signed     short       INT16;
#endif

#ifndef __UINT16__
#define __UINT16__
typedef  unsigned   short       UINT16;
#endif

#ifndef __INT8__
#define __INT8__
typedef  signed     char        INT8;
#endif

#ifndef __UINT8__
#define __UINT8__
typedef  unsigned   char        UINT8;
#endif


#ifndef PI
#define PI 3.14159265358979
#endif

#define MAX_PC_NUM 524288

// 障碍检测输出: 局部坐标系，前方80米，后方20米，左右25米
// 10cm障碍图: 前方80米，后方20米，左右25米
#define OBSTACLEMAP_MINX_10CM   -25.0f
#define OBSTACLEMAP_MAXX_10CM    25.0f
#define OBSTACLEMAP_MINY_10CM   -20.0f
#define OBSTACLEMAP_MAXY_10CM    80.0f
#define OBSTACLEMAP_WIDTH_10CM   500      // 500  * 10  =  5000 cm
#define OBSTACLEMAP_HEIGHT_10CM  1000     // 1000 * 10  = 10000 cm
#define OBSTACLEMAP_VEHICLEX_10CM   250   // 250  * 10  =  2500 cm
#define OBSTACLEMAP_VEHICLEY_10CM   800   // 800  * 10  =  8000 cm
#define OBSTACLEMAP_SIZE_10CM OBSTACLEMAP_WIDTH_10CM * OBSTACLEMAP_HEIGHT_10CM
#define GRID_SIZE_10CM   10.0f
// 2cm障碍图: 前方6米，后方1米，左右3米
#define OBSTACLEMAP_MINX_2CM    -3.0f
#define OBSTACLEMAP_MAXX_2CM     3.0f
#define OBSTACLEMAP_MINY_2CM    -1.0f
#define OBSTACLEMAP_MAXY_2CM     6.0f
#define OBSTACLEMAP_WIDTH_2CM   300      // 300 * 2  =  600 cm
#define OBSTACLEMAP_HEIGHT_2CM  350      // 350 * 2  =  700 cm
#define OBSTACLEMAP_VEHICLEX_2CM   150   // 150 * 2  =  300 cm
#define OBSTACLEMAP_VEHICLEY_2CM   300   // 300 * 2  =  600 cm
#define OBSTACLEMAP_SIZE_2CM OBSTACLEMAP_WIDTH_2CM * OBSTACLEMAP_HEIGHT_2CM
#define GRID_SIZE_2CM   2.0f

// 点云结构体
#pragma pack(push, 1)
struct PointXYZI
{
    float x;      //车体坐标系, 单位m
    float y;
    float z;
    unsigned char intensity;
};
#pragma pack(pop)


struct LidarCoor
{
    UINT32 scan_num;
    UINT32 line_num;
    double start_time;
    double end_time;
    int angle[MAX_PC_NUM];
    float range[MAX_PC_NUM];
    PointXYZI data[MAX_PC_NUM];
};

}  // namespace common
}  // namespace xsproto

#endif 	// COMMONDEFINITION_HH

