#include <ros/ros.h>
#include <math.h>
#include <Control.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

#define freq 1440 // 激光雷达的频率
#define speed 1825 // 小车的速度

// 定义一个结构体，用于存储点的直角坐标
typedef struct {
    double x;
    double y;
} Point_xy;

// 定义一个结构体，用于存储点的极坐标
typedef struct {
    double range;
    double theta;
} Point_p;

// 定义一个类，用于发布和订阅 ROS 话题
class PubAndSub
{  
private:
    ros::NodeHandle nh_; // ROS 节点句柄
    ros::Publisher pub_; // ROS 发布者
    ros::Subscriber sub_; // ROS 订阅者
public:
    PubAndSub()
    {
        // 初始化发布者，发布到 /car/cmd_vel 话题
        pub_ = nh_.advertise<geometry_msgs::Twist>("/car/cmd_vel", 5);
        // 初始化订阅者，订阅 /scan 话题
        sub_ = nh_.subscribe("/scan", 5, &PubAndSub::Callback, this);
    }
    // 回调函数，用于处理激光雷达数据
    void Callback(const sensor::LaserScan::ConstPtr &laser);
};

// namespace sensor_msgs
// {
//   struct LaserScan
//   {
//     std_msgs::Header header; // 消息头，包括时间戳和坐标系ID
//     float angle_min; // 扫描的起始角度
//     float angle_max; // 扫描的结束角度
//     float angle_increment; // 每个测量之间的角度增量
//     float time_increment; // 每个测量之间的时间增量
//     float scan_time; // 扫描的时间
//     float range_min; // 有效测量的最小范围
//     float range_max; // 有效测量的最大范围
//     std::vector<float> ranges; // 测量的距离数据
//     std::vector<float> intensities; // 测量的强度数据（如果激光雷达支持）
//   };
// }
// 回调函数的实现
void PubAndSub::Callback(const sensor::LaserScan::ConstPtr &laser)
{
    int i, j = 0;
    char negetiveNum = 0, positiveNum = 0;// 负方向和正方向锥桶的数量
    double range = 0, error = 0, negetiveSum = 0, positiveSum = 0; // 锥桶的距离、偏差、负方向和正方向锥桶的x坐标和
    geometry_msgs::Twist twist;
    point_xy point[30] = {0.0}; // 存储点的直角坐标
    point_p point_p[30] = {0.0}; // 存储点的极坐标

    // 遍历激光雷达数据
    for(i = 1; i < freq; i++)
    {
        // 若变化大于2m识别为锥桶
        if(laser->ranges[i-1] - laser->ranges[i] >= 2.0)
        {
            // 获取2.5m以内各锥桶的距离
            if(laser->ranges[i] > 0.5 && laser->ranges[i] < 2.5)
            {
                point_p[j].range = laser->ranges[i];
                // 获取2.5m以内各锥桶的角度
                point_p[j].theta = i * laser->angle_increment + laser->angle_min;
                j++;
            }
        }
    }

    // 遍历存储的点
    for(i = 0; i < 30; i++)
    {
        if(point_p[i].range)
        {
            // 获取2.5m以内各锥桶的直角坐标
            point[i].x = point_p[i].range * sin(point_p[i].theta);
            point[i].y = point_p[i].range * cos(point_p[i].theta);

            // 排除后方距离小于0.25m的锥桶
            if(point[i].y >= -0.25)
                ROS_INFO("found point: x = %f, y = %f", point[i].x, point[i].y);
            else
            {
                point[i].x = 0;
                point[i].y = 0;
            }

            // 统计正方向和负方向锥桶的数量和坐标和
            if(point[i].x > 0)
            {
                positiveNum++;
                positiveSum += point[i].x; // 正方向锥桶的x坐标和
            }
            else if(point[i].x < 0)
            {
                negetiveNum++;
                negetiveSum += point[i].x; // 负方向锥桶的x坐标和
            }
        }
    }

    // 过滤掉异常点
    for(i = 0; i < 30; i++)
    {
        if(point[i].x > 0)
        {
            // 若正方向锥桶的x坐标大于平均值的1.75倍
            if(point[i].x >= 1.75 * positiveSum / positiveNum)
            {
                positiveSum -= point[i].x;
                positiveNum--;
            }
        }
        else if(point[i].x < 0)
        {
            // 若负方向锥桶的x坐标小于平均值的1.75倍
            if(point[i].x <= 1.75 * positiveSum / positiveNum)
            {
                negetiveNum--;
                negetiveSum -= point[i].x;
            }
        }
    }

    // 计算偏差
    error = positiveSum + negetiveSum;
    error = pid.PIDposition(error); // 位置式PID控制
    // error = pid.PIDincrement(error); // 增量式PID控制

    // 设置小车的角度和速度
    twist.angular.z = 90 + error; // 角度
    twist.linear.x = speed;

    // 限制角度范围
    if(twist.angular.z > 180)
    {
        twist.angular.z = 180;
    }
    else if(twist.angular.z < 0)
    {
        twist.angular.z = 0;
    }

    // 设置其他线速度和角速度为0
    twist.linear.y = 0;
    twist.linear.z = 0;
    twist.angular.x = 0;
    twist.angular.y = 0;

    // 发布控制指令
    pub_.publish(twist);
    ROS_INFO("error = %.2f", error);
}

// 主函数
int main(int argc, char **argv)
{
    pid.init(); // 初始化PID控制器
    ros::init(argc, argv, "laser_go"); // 初始化ROS节点
    PubAndSub pub_sub; // 创建发布和订阅对象
    ros::spin(); // 进入循环，等待回调函数被调用
    return 0;
}