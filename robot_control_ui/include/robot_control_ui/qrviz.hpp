#ifndef QRVIZ_HPP
#define QRVIZ_HPP

#include <QObject>
#include <ros/ros.h>
#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display.h>
#include <rviz/tool_manager.h>
#include <rviz/tool.h>
#include <QVBoxLayout>
class qrviz
{
public:
    qrviz(QVBoxLayout* layout);
    void Set_fixedFrame(QString Frame_name);
    void Display_grid(int cell_count,QColor color,bool enable);
    void Display_TF(bool enable);
    void Display_LaserScan(QString laser_topic, bool enable);
    void Display_RobotModel(bool enable);
    void Display_Map(QString topic,QString color_scheme,bool enable);
    void Display_Path(QString topic,QColor color,bool enable);
    void Set_Start_Pose();
    void Set_Goal_Pose();
    void Display_Local_Map(QString costmap_topic,QString cost_map_color,QString path_topic, QColor path_color,bool enable);
    void Display_Global_Map(QString costmap_topic,QString cost_map_color,QString path_topic, QColor path_color,bool enable);
    void Display_Marker(QString marker_topic,bool enable);
    void Set_Start_pose();
    void Set_Goal_pose();
private:
    rviz::RenderPanel* render_panel;
    rviz::VisualizationManager* manager_;
    rviz::Display* Grid_=NULL;
    rviz::Display* TF_=NULL;
    rviz::Display* LaserScan_=NULL;
    rviz::Display* RobotModel_=NULL;
    rviz::Display* Map_=NULL;
    rviz::Display* Path_=NULL;
    rviz::ToolManager* tool_manager_=NULL;
    //nav
    rviz::Display* Global_Map=NULL;
    rviz::Display* Global_Path=NULL;
    rviz::Display* Local_Map=NULL;
    rviz::Display* Local_Path=NULL;
    rviz::Display* Marker_ = nullptr;
};

#endif // QRVIZ_HPP
