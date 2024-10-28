#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/TeleportAbsolute.h>
#include <turtlesim/SetPen.h>

void drawLine(ros::Publisher &pub, double speed, double distance, bool isForward)
{
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = isForward ? speed : -speed;
    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;
    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;
    vel_msg.angular.z = 0;

    double t0 = ros::Time::now().toSec();
    double current_distance = 0.0;

    ros::Rate loop_rate(100);
    do {
        pub.publish(vel_msg);
        double t1 = ros::Time::now().toSec();
        current_distance = speed * (t1 - t0);
        ros::spinOnce();
        loop_rate.sleep();
    } while (current_distance < distance);

    vel_msg.linear.x = 0;
    pub.publish(vel_msg);
}

void turn(ros::Publisher &pub, double angular_speed, double angle, bool clockwise)
{
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = 0;
    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;
    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;
    vel_msg.angular.z = clockwise ? -angular_speed : angular_speed;

    double t0 = ros::Time::now().toSec();
    double current_angle = 0.0;

    ros::Rate loop_rate(100);
    do {
        pub.publish(vel_msg);
        double t1 = ros::Time::now().toSec();
        current_angle = angular_speed * (t1 - t0);
        ros::spinOnce();
        loop_rate.sleep();
    } while (current_angle < angle);

    vel_msg.angular.z = 0;
    pub.publish(vel_msg);
}

void setPen(ros::ServiceClient &pen_client, bool off)
{
    turtlesim::SetPen pen_srv;
    pen_srv.request.off = off;
    pen_client.call(pen_srv);
}

void teleport(ros::ServiceClient &teleport_client, double x, double y, double theta)
{
    turtlesim::TeleportAbsolute srv;
    srv.request.x = x;
    srv.request.y = y;
    srv.request.theta = theta;
    teleport_client.call(srv);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "draw_chessboard");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    ros::ServiceClient pen_client = nh.serviceClient<turtlesim::SetPen>("/turtle1/set_pen");
    ros::ServiceClient teleport_client = nh.serviceClient<turtlesim::TeleportAbsolute>("/turtle1/teleport_absolute");

    ros::Rate loop_rate(0.5);

    double square_size = 1.0;
    int num_squares = 8;

    for (int i = 0; i <= num_squares; ++i)
    {
        setPen(pen_client, false);
        drawLine(pub, 1.0, square_size * num_squares, true);
        turn(pub, 1.0, M_PI / 2, false);
        drawLine(pub, 1.0, square_size, true);
        turn(pub, 1.0, M_PI / 2, false);
        drawLine(pub, 1.0, square_size * num_squares, true);
        turn(pub, 1.0, M_PI / 2, true);
        drawLine(pub, 1.0, square_size, true);
        turn(pub, 1.0, M_PI / 2, true);
    }

    teleport(teleport_client, 0.5, 0.5, 0);
    turn(pub, 1.0, M_PI / 2, false);

    for (int i = 0; i <= num_squares; ++i)
    {
        setPen(pen_client, false);
        drawLine(pub, 1.0, square_size * num_squares, true);
        turn(pub, 1.0, M_PI / 2, true);
        drawLine(pub, 1.0, square_size, true);
        turn(pub, 1.0, M_PI / 2, true);
        drawLine(pub, 1.0, square_size * num_squares, true);
        turn(pub, 1.0, M_PI / 2, false);
        drawLine(pub, 1.0, square_size, true);
        turn(pub, 1.0, M_PI / 2, false);
    }

    ros::spin();
    return 0;
}