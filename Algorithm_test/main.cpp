#include <iostream>
#include <vector>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include "src/Algorithm.h"
#include <fftw3.h>
// A C++ program to find convex hull of a set of points. Refer
// https://www.geeksforgeeks.org/orientation-3-ordered-points/
// for explanation of orientation()
#include <bits/stdc++.h>
using namespace std;
using chrono_clock = std::chrono::high_resolution_clock;
using chrono_time_point = std::chrono::time_point<std::chrono::high_resolution_clock>;
sig_atomic_t no_int_sig_ = 0;
struct Point
{
    int x, y;
};



bool isPointOnBoundary(const std::vector<Point>& polygon, const Point& point) {
    int n = polygon.size();
    for (int i = 0; i < n; ++i) {
        int j = (i + 1) % n;

        if (point.x >= std::min(polygon[i].x, polygon[j].x) &&
            point.x <= std::max(polygon[i].x, polygon[j].x) &&
            point.y >= std::min(polygon[i].y, polygon[j].y) &&
            point.y <= std::max(polygon[i].y, polygon[j].y)) {
            double dx = polygon[j].x - polygon[i].x;
            double dy = polygon[j].y - polygon[i].y;
            double cross = dx * (point.y - polygon[i].y) - dy * (point.x - polygon[i].x);

            if (std::abs(cross) < 1e-6) {
                return true; // Point lies on the boundary
            }
        }
    }
    return false;
}
bool isInsidePolygon(const std::vector<Point>& polygon, const Point& point) {
    int intersections = 0;
    int n = polygon.size();

    for (int i = 0; i < n; ++i) {
        int j = (i + 1) % n;

        if ((polygon[i].y >= point.y) != (polygon[j].y >= point.y) &&
            point.x <= (polygon[j].x - polygon[i].x) * (point.y - polygon[i].y) / (polygon[j].y - polygon[i].y) + polygon[i].x) {
            intersections++;
        }
    }

    return (intersections % 2) == 1 || isPointOnBoundary(polygon, point);
}

// To find orientation of ordered triplet (p, q, r).
// The function returns following values
// 0 --> p, q and r are collinear
// 1 --> Clockwise
// 2 --> Counterclockwise
int orientation(Point p, Point q, Point r)
{
    int val = (q.y - p.y) * (r.x - q.x) -
              (q.x - p.x) * (r.y - q.y);

    if (val == 0) return 0;  // collinear
    return (val > 0)? 1: 2; // clock or counterclock wise
}

// Prints convex hull of a set of n points.
void convexHull(Point points[], int n)
{
    // There must be at least 3 points
    if (n < 3) return;

    // Initialize Result
    vector<Point> hull;

    // Find the leftmost point
    int l = 0;
    for (int i = 1; i < n; i++)
        if (points[i].x < points[l].x)
            l = i;

    // Start from leftmost point, keep moving counterclockwise
    // until reach the start point again.  This loop runs O(h)
    // times where h is number of points in result or output.
    int p = l, q;
    do
    {
        // Add current point to result
        hull.push_back(points[p]);

        // Search for a point 'q' such that orientation(p, q,x) is counterclockwise for all points 'x'. The ideais to keep track of last visited most counterclockwise point in q. If any point 'i' is more counterclock-wise than q, then update q.
        q = (p+1)%n;
        for (int i = 0; i < n; i++)
        {
            // If i is more counterclockwise than current q, then update q
            if (orientation(points[p], points[i], points[q]) == 2)
                q = i;
        }

        // Now q is the most counterclockwise with respect to p
        // Set p as q for next iteration, so that q is added to
        // result 'hull'
        p = q;

    } while (p != l);  // While we don't come to first point

    // Print Result
    for (int i = 0; i < hull.size(); i++)
        cout << "(" << hull[i].x << ", "
             << hull[i].y << ")\n";
}




// Driver program to test above functions



void hull(){
    Point points[] = {{1, 1}, {1, 4}, {4, 4}, {4, 1}, {1, 2}, {3, 1}, {3, 2}};
    std:vector<Point> poly= {{1, 1}, {1, 4}, {4, 4}, {4, 1}};
    int n = sizeof(points)/sizeof(points[0]);
    convexHull(points, n);
    Point p = {1, 1};
    if (isInsidePolygon( poly,p)) {
        std::cout << "Point is inside the polygon." << std::endl;
    } else {
        std::cout << "Point is outside the polygon." << std::endl;
    }

}

#include <iostream>
#include <vector>
#include <cmath>

//struct Point {
//    double x, y;
//};

// 计算两点之间的距离
double distance(const Point& p1, const Point& p2) {
    return std::sqrt((p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y));
}

// 查找两条离散点曲线的交点
void findIntersections(const std::vector<Point>& curve1, const std::vector<Point>& curve2,cv::Mat &img) {
    double min_d = 999;
    cv::Point p,q;
    for (const Point& p1 : curve1) {
        for (const Point& p2 : curve2) {
            double dist = distance(p1, p2);
            if (min_d > dist){
                min_d = dist;
                p.x = p1.x;
                p.y = p1.y;
                q.x = p2.x;
                q.y = p2.y;
            }

        }
    }

    std::cout<<"dist:"<<min_d <<endl;
    std::cout << "p,q:" << p.x << ", " << p.y << std::endl;
    std::cout << "p,q:" << q.x << ", " << q.y << std::endl;
    if (min_d < 10) {
        std::cout << "Intersection found at (" << q.x << ", " << q.y << ")" << std::endl;
    }

//    cv::circle(img, p, 2, cv::Scalar(0,255,0), 1, cv::LINE_AA);
//    cv::circle(img, q, 2, cv::Scalar(0,255,0), 1, cv::LINE_AA);
}

cv::Point
interpolate(cv::Point p1, cv::Point p2, double t) {
    cv::Point interpolatedPoint;
    interpolatedPoint.x = p1.x + t * (p2.x - p1.x);
    interpolatedPoint.y = p1.y + t * (p2.y - p1.y);
    return interpolatedPoint;
}
std::vector<cv::Point> interpolatePoints(cv::Point p1, cv::Point p2, cv::Point p3, int numSamples) {
    std::vector<cv::Point> interpolatedPoints;

    for (int i = 0; i < numSamples; ++i) {
        double t = static_cast<double>(i) / (numSamples - 1);
        cv::Point interpolatedPoint = interpolate(p2, p3, t);
        if(i<=5)
        {   double k = static_cast<double>(i) / (5 - 1);
            cv::Point interpolatedPoint_ = interpolate(p1, p2, k);
            interpolatedPoints.push_back(interpolatedPoint_);
        }

        interpolatedPoints.push_back(interpolatedPoint);

    }

    return interpolatedPoints;
}

cv::Point quadraticBezier(cv::Point p0, cv::Point p1, cv::Point p2, float t) {
    float u = 1 - t;
    float tt = t * t;
    float uu = u * u;

    cv::Point p;
    p.x = uu * p0.x + 2 * u * t * p1.x + tt * p2.x;
    p.y = uu * p0.y + 2 * u * t * p1.y + tt * p2.y;

    return p;
}

void test() {
    cv::Mat img = cv::Mat::zeros(1200, 800, CV_8UC(3));
    // 轨迹
    std::vector<Point> curve1 = {{516,599}, {516,598}, {516,596}, {516,595}, {516,593}, {516,592}, {516,591}, {516,589}, {516,588}, {516,587}, {516,585}, {516,584}, {516,582}, {516,581}, {516,580}, {516,578}, {517,577}, {517,576}, {517,574}, {517,573}};

    // 车道poly
    std::vector<Point> curve2 = {{416,733}, {417,724}, {418,716}, {419,708}, {422,693}, {425,678}, {430,663}, {435,648}, {441,634}, {447,621}, {454,607}, {462,594}, {471,582}, {480,570}, {490,558}, {501,547}, {512,536}, {524,525}, {531,520}, {537,515}, {544,510}, {551,506}, {558,501}, {565,496}, {572,492}, {580,488}, {588,483}, {596,479}, {604,475}, {612,471}, {621,467}, {629,463}, {638,459}, {647,456}, {648,456}, {451,737}, {452,724}, {454,711}, {456,699}, {460,686}, {464,674}, {469,662}, {474,650}, {481,638}, {488,627}, {496,615}, {505,604}, {514,593}, {525,583}, {536,572}, {548,562}, {554,556}, {560,551}, {567,546}, {574,541}, {581,536}, {588,532}, {596,527}, {603,522}, {611,517}, {619,512}, {628,508}, {636,503}, {645,499}, {653,494}, {662,490}, {669,486}};
    std::vector<cv::Point> curve1_;
    std::vector<cv::Point> curve2_;



    cv::Point p1 {505,621};
    cv::Point p2 {505,604};
    cv::Point p3 {530,549};

//    cv::Point2d dir_v{0.0603517,0.0603517};
//    double mid_x = (p1.x + p3.x) / 2;
//    double mid_y = (p1.y + p3.y) / 2;
//
//    p2.x = mid_x + dir_v.x;
//    p2.y = mid_y + dir_v.y;
    for(auto &p :curve1)
    {
        cv::Point p_;
        p_.x = p.x;
        p_.y = p.y;
        curve1_.push_back(p_);
    }
    for(auto &p :curve2)
    {
        cv::Point p_;
        p_.x = p.x;
        p_.y = p.y;
        curve2_.push_back(p_);
        cv::circle(img, p_, 1, cv::Scalar(255,0,0), 1, cv::LINE_AA);
    }

    findIntersections(curve1, curve2,img);
    cv::circle(img, p1, 3, cv::Scalar(0,255,0), cv::FILLED);
    cv::circle(img, p2, 3, cv::Scalar(0,255,0), cv::FILLED);
    cv::circle(img, p3, 3, cv::Scalar(0,255,0), cv::FILLED);
    std::vector<cv::Point> curvePoints;

    for (float t = 0; t <= 1; t += 0.1) {
        cv::Point p = quadraticBezier(p1, p2, p3, t);
        curvePoints.push_back(p);
    }

    for (const cv::Point& p : curvePoints) {
        cv::circle(img, p, 1, cv::Scalar(0,0,255), cv::FILLED);
    }

    cv::namedWindow("testing", cv::WINDOW_AUTOSIZE);
    cv::imshow("testing", img);
    cv::waitKey();

}

// 突变状态检测和纠正函数
std::vector<int> correctMutation(const std::vector<int>& inputSequence) {
    std::vector<int> correctedSequence;

    int windowSize = 5; // 窗口大小

    for (int i = 0; i < inputSequence.size(); i++) {
        int countZeros = 0;
        int countOnes = 0;

        // 检查窗口内的状态
        for (int j = i - windowSize + 1; j <= i; j++) {
            if (j >= 0 && j < inputSequence.size()) {
                if (inputSequence[j] == 0) {
                    countZeros++;
                } else if (inputSequence[j] == 1) {
                    countOnes++;
                }
            }
        }

        // 如果窗口内只有一个0，则将当前位置的状态纠正为1
        if (countZeros < countOnes) {
            correctedSequence.push_back(1);
        } else {
            correctedSequence.push_back(inputSequence[i]);
        }
    }

    return correctedSequence;
}

void test_20230829()
{
    int **q;
    int *w;
    std::vector<int> p = {1,2,3,4,5,6,7,8,9,10};
    q = new int*[2];
    int idx = 0;
    for (int i = 0; i < 2; ++i) {
        q[i] = new int[5];
        for (int j = 0; j < 5; ++j) {
            q[i][j] = p[idx];
            idx++;
            std::cout<<q[i][j]<<std::endl;
        }
    }

}

void interrupt_handler(int sig)
{
//    std::cout << std::endl << "received signal: " << sig << std::endl;
    std::ostringstream exit_text;
    std::cout << "%%%%:"<< exit_text.str().c_str()  << std::endl;

    no_int_sig_ = sig;
}
void test_ObstacleObjectInfo(int argc,char *argv[])
{
    xscom::Init(argv[0]);
    double loop_freq = 10.0;
    xscom::Rate xs_rate(loop_freq);
    CyberRTMsg::instance();
    std::shared_ptr<xscom::Reader <Env_Object_Msg> > obstacle_obj_reader;
    std::shared_ptr<Env_Object_Msg> obstacle_obj_msg; // 装object 消息的

    std::shared_ptr<xscom::Reader <Sync_Lidar_Msg> > sync_lidar_reader;
    std::shared_ptr<Sync_Lidar_Msg> sync_lidar_msg; // 装消息的lidar
    sync_lidar_reader = CyberRTMsg::instance()->SyncLidarReader();
    obstacle_obj_reader = CyberRTMsg::instance()->ObjectInfoReader();
    double pre_sync_time=0,pre_obj_time=0;
    while(!xscom::IsShutdown() && !no_int_sig_)
    {
        chrono_time_point start_read = chrono_clock::now();

        sync_lidar_reader->Observe();
        sync_lidar_msg = sync_lidar_reader->GetLatestObserved();
        if(sync_lidar_msg)
        {
//            std::cout << "同步雷达时间戳:" << sync_lidar_msg->header().timestamp()  << std::endl;

            if(pre_sync_time!=0)
            {
                double deta_time = sync_lidar_msg->header().timestamp() - pre_sync_time;
                if(deta_time > 0.15)
                {
                    std::cout << "同步雷达-上1帧与当前帧的差>0.1:  " << deta_time  << std::endl;
                }

            }
            pre_sync_time = sync_lidar_msg->header().timestamp();
        }

        obstacle_obj_reader->Observe();
        obstacle_obj_msg = obstacle_obj_reader->GetLatestObserved();
        if(obstacle_obj_msg)
        {
//            std::cout << "obstacle_obj_时间戳:" << obstacle_obj_msg->header().timestamp()  << std::endl;
            if(pre_obj_time!=0)
            {
                double deta_time = obstacle_obj_msg->header().timestamp() - pre_obj_time;
                if(deta_time > 0.15)
                {
                    std::cout << "obstacle_obj-上1帧与当前帧的差>0.1:  " << deta_time  << std::endl;
                }

            }
            pre_obj_time = obstacle_obj_msg->header().timestamp();
        }

        xs_rate.Sleep();
//        std::cout << "程序用时:" << std::chrono::duration<double>(chrono_clock::now() - start_read).count()  << std::endl;
    }
}
int main(int argc,char *argv[])
{
//    test_20230829();
//    hull();
    // test-时延问题
    test_ObstacleObjectInfo(argc,argv);

    return 0;
}