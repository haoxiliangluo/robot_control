#include "../include/robot_control_ui/qrviz.hpp"
#include <QDebug>
qrviz::qrviz(QVBoxLayout* layout)
{   //创建rviz panel
    render_panel=new rviz::RenderPanel();
    //向layout添加
    layout->addWidget(render_panel);
    //创建rviz控制对象,需要render_panel类型初始化。
    manager_ =  new rviz::VisualizationManager(render_panel);
    tool_manager_=manager_->getToolManager();
    //初始化render_panel实现放大缩小
    render_panel->initialize(manager_->getSceneManager(),manager_);
    manager_->setFixedFrame("map");
    //初始化rviz控制对象
    manager_->initialize();
    manager_->startUpdate();
    manager_->removeAllDisplays();


}
void qrviz::Display_Local_Map(QString costmap_topic,QString cost_map_color,QString path_topic, QColor path_color,bool enable)
{
    if(Local_Map!=NULL)
    {
        delete Local_Map;
        Local_Map = NULL;
    }
    if(Local_Path!=NULL)
    {
        delete Local_Path;
        Local_Path = NULL;
    }
    Local_Map = manager_->createDisplay("rviz/Map","localmap",enable);
    ROS_ASSERT(Local_Map!=NULL);//断言
    Local_Path = manager_->createDisplay("rviz/Path","localmap",enable);
    ROS_ASSERT(Local_Path!=NULL);//断言
    Local_Map->subProp("Topic")->setValue(costmap_topic);
    Local_Map->subProp("Color Scheme")->setValue(cost_map_color);
    Local_Path->subProp("Topic")->setValue(path_topic);
    Local_Path->subProp("Color")->setValue(path_color);
}
void qrviz::Display_Global_Map(QString costmap_topic,QString cost_map_color,QString path_topic, QColor path_color,bool enable)
{   if(Global_Map!=NULL)
    {
        delete Local_Map;
        Global_Map = NULL;
    }
    if(Global_Path!=NULL)
    {
        delete Global_Path;
        Global_Path = NULL;
    }
    Global_Map = manager_->createDisplay("rviz/Map","Globalmap",enable);
    ROS_ASSERT(Global_Map!=NULL);//断言
    Global_Path = manager_->createDisplay("rviz/Path","Globalmap",enable);
    ROS_ASSERT(Global_Path!=NULL);//断言
    Global_Map->subProp("Topic")->setValue(costmap_topic);
    Global_Map->subProp("Color Scheme")->setValue(cost_map_color);
    Global_Path->subProp("Topic")->setValue(path_topic);
    Global_Path->subProp("Color")->setValue(path_color);
}
void qrviz::Set_Start_Pose()
{
   rviz::Tool* current_tool = tool_manager_->addTool("rviz/SetInitialPose");
   //设置当前使用的工具
   tool_manager_->setCurrentTool(current_tool);
}
void qrviz::Set_Goal_Pose()
{
    rviz::Tool* current_tool = tool_manager_->addTool("rviz/SetGoal");
    //获取属性容器
    rviz::Property* pro=current_tool->getPropertyContainer();
    //设置发布导航点的topic
    pro->subProp("Topic")->setValue("/move_base_simple/goal");
    //设置当前使用的工具
    tool_manager_->setCurrentTool(current_tool);
}
void qrviz::Display_Path(QString topic,QColor color,bool enable)
{
    if(Path_!=NULL)
    {
        delete Path_;
        Path_ = NULL;
    }
    Path_ = manager_->createDisplay("rviz/Path","myPath",enable);
    ROS_ASSERT(Path_!=NULL);//断言
    Path_->subProp("Topic")->setValue(topic);
    Path_->subProp("Color")->setValue(color);


}
void qrviz::Display_Map(QString topic,QString color_scheme,bool enable)
{
    if(Map_!=NULL)
    {
        delete Map_;
        Map_ = NULL;
    }
    Map_ = manager_->createDisplay("rviz/Map","myMap_",enable);
    ROS_ASSERT(Map_!=NULL);//断言
    Map_->subProp("Topic")->setValue(topic);
    Map_->subProp("Color Scheme")->setValue(color_scheme);


}
void qrviz::Display_TF(bool enable)
{
    if(TF_!=NULL)
    {
        delete TF_;
        TF_ = NULL;//制空后重新分配对象，不然未分配内存访问会使程序崩溃
    }
     TF_ = manager_->createDisplay("rviz/TF","myTF",enable);
     //设置cell count
     //Grid_->subProp("Plan Cell Count")->setValue(cell_count);
     //设置颜色
    // Grid_->subProp("Color")->setValue(color);
     ROS_ASSERT(TF_!=NULL);//断言
}
void qrviz::Display_RobotModel(bool enable)
{
    if(RobotModel_!=NULL)
    {
        delete RobotModel_;
        RobotModel_ = NULL;//制空后重新分配对象，不然未分配内存访问会使程序崩溃
    }
    RobotModel_ = manager_->createDisplay("rviz/RobotModel","myRobotModel_",enable);
    ROS_ASSERT(RobotModel_!=NULL);//断言
}
void qrviz::Display_LaserScan(QString laser_topic,bool enable)
{
    if(LaserScan_!=NULL)
    {
        delete LaserScan_;
        LaserScan_ = NULL;//制空后重新分配对象，不然未分配内存访问会使程序崩溃
    }
    LaserScan_ = manager_->createDisplay("rviz/LaserScan","myLaserScan",enable);
    LaserScan_->subProp("Topic")->setValue(laser_topic);
    ROS_ASSERT(LaserScan_!=NULL);//断言
}
void qrviz::Set_fixedFrame(QString Frame_name)
{
    manager_->setFixedFrame(Frame_name);
    qDebug()<<manager_->getFixedFrame();
}
void qrviz::Display_grid(int cell_count,QColor color,bool enable)
 {
    if(Grid_!=NULL)
    {
        delete Grid_;
        Grid_ = NULL;//制空后重新分配对象，不然未分配内存访问会使程序崩溃
    }
     Grid_ = manager_->createDisplay("rviz/Grid","myGrid",enable);
     //设置cell count
     Grid_->subProp("Plan Cell Count")->setValue(cell_count);
     //设置颜色
     Grid_->subProp("Color")->setValue(color);
     ROS_ASSERT(Grid_!=NULL);//断言
 }
void qrviz::Set_Start_pose()
{
    rviz::Tool *current_tool_ = tool_manager_->addTool("rviz/SetInitialPose");
    //设置当前使用的工具
    tool_manager_->setCurrentTool(current_tool_);
}

void qrviz::Set_Goal_pose()
{
    rviz::Tool *current_tool_ = tool_manager_->addTool("rviz/SetGoal");
    //获取属性容器
    rviz::Property *pro = current_tool_->getPropertyContainer();
    //设置发布导航目标点的topic
    pro->subProp("Topic")->setValue("/move_base_simple/goal");
    //设置当前使用的工具
    tool_manager_->setCurrentTool(current_tool_);
}
void qrviz::Display_Marker(QString marker_topic,bool enable)
{
    if(Marker_!=NULL)
    {
        delete Marker_;
        Marker_=NULL;
    }
    Marker_=manager_->createDisplay("rviz/Marker","myMarker",enable);
    ROS_ASSERT(Marker_!=NULL);
    Marker_->subProp("Marker Topic")->setValue(marker_topic);
//    qDebug()<< "test Display_Marker";
}
