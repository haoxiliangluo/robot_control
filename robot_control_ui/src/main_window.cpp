/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/robot_control_ui/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace robot_control_ui {

using namespace Qt;
//单点导航储存 14个
std::vector<MyPose> single_nav_goals(14);

//多点导航存储
std::vector<MyPose> points_nav_goals;
/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    ReadSettings();
	setWindowIcon(QIcon(":/images/icon.png"));
	ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

	/*********************
	** Logging
	**********************/
	ui.view_logging->setModel(qnode.loggingModel());
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

    /*********************
    ** Auto Start
    **********************/
    if ( ui.checkbox_remember_settings->isChecked() ) {
        on_button_connect_clicked(true);
    }
    connect(ui.horizontalSlider_linera,SIGNAL(valueChanged(int)),this,SLOT(slot_linera_value_change(int)));
    connect(ui.horizontalSlider_raw,SIGNAL(valueChanged(int)),this,SLOT(slot_raw_value_change(int)));
    connect(ui.pushButton_i,SIGNAL(clicked()),this,SLOT(slot_pushbtn_click()));
    connect(ui.pushButton_j,SIGNAL(clicked()),this,SLOT(slot_pushbtn_click()));
    connect(ui.pushButton_l,SIGNAL(clicked()),this,SLOT(slot_pushbtn_click()));
    connect(ui.pushButton_n,SIGNAL(clicked()),this,SLOT(slot_pushbtn_click()));
    connect(ui.pushButton_m,SIGNAL(clicked()),this,SLOT(slot_pushbtn_click()));
    connect(ui.pushButton_br,SIGNAL(clicked()),this,SLOT(slot_pushbtn_click()));
    connect(ui.pushButton_u,SIGNAL(clicked()),this,SLOT(slot_pushbtn_click()));
    connect(ui.pushButton_o,SIGNAL(clicked()),this,SLOT(slot_pushbtn_click()));

    connect(ui.btn_stop,&QPushButton::clicked,this,&MainWindow::slot_cmd_stop);//一舰艇之；
    //查看话题界面
    connect(ui.btn_topic,&QPushButton::clicked,[=](){
        ui.stackedWidget->setCurrentIndex(1);
    });
    connect(ui.btn_goto,&QPushButton::clicked,this,&MainWindow::slot_refresh_topic);
    //刷新话题列表
    connect(ui.btn_msg,&QPushButton::clicked,this,&MainWindow::slot_refresh_topic);
    //跳转初始界面
    connect(ui.btn_goto,&QPushButton::clicked,[=](){
        ui.stackedWidget->setCurrentIndex(0);
    });
    //单点导航界面
    connect(ui.btn_onepoint,&QPushButton::clicked,[=](){
        ui.stackedWidget->setCurrentIndex(2);
    });
    //多点巡航界面
    connect(ui.btn_more,&QPushButton::clicked,[=](){
        ui.stackedWidget->setCurrentIndex(3);
    });
    //单点查看界面
    connect(ui.btn_print_ont,&QPushButton::clicked,[=](){
        ui.stackedWidget->setCurrentIndex(4);
    });
    //多点查看界面
    connect(ui.btn_print_more,&QPushButton::clicked,[=](){
        ui.stackedWidget->setCurrentIndex(5);
    });


    //init ui
    speed_x_dashBoard=new CCtrlDashBoard(ui.widget_speed_x);
    speed_y_dashBoard=new CCtrlDashBoard(ui.widget_speed_y);
    speed_x_dashBoard->setGeometry(ui.widget_speed_x->rect());
    speed_y_dashBoard->setGeometry(ui.widget_speed_y->rect());
    speed_x_dashBoard->setValue(0);
    speed_y_dashBoard->setValue(0);
    ui.horizontalSlider_linera->setValue(50);
    ui.horizontalSlider_raw->setValue(50);

//    ui.treeWidget->setWindowTitle("Display");
//    ui.treeWidget->setWindowIcon(QIcon(":/images/left_dock.svg"));

    //header
    ui.treeWidget->setHeaderLabels(QStringList()<<"key"<<"value");
    ui.treeWidget->setHeaderHidden(true);
    //Globel Options
    QTreeWidgetItem* Global = new QTreeWidgetItem(QStringList()<<"Global Options");
    Global->setIcon(0,QIcon(":/images/display.png"));

    ui.treeWidget->addTopLevelItem(Global);
    Global->setExpanded(true);
    //FixFrame
    QTreeWidgetItem* Fixed_frame = new QTreeWidgetItem(QStringList()<<"Fixed Frame");
    fixed_box = new QComboBox();
    fixed_box->addItem("map");
    fixed_box->setMaximumWidth(150);
    fixed_box->setEditable(true);
    connect(fixed_box,SIGNAL(currentTextChanged(QString)),this,SLOT(slot_treewidget_value_change(QString)));

    Global->addChild(Fixed_frame);


    ui.treeWidget->setItemWidget(Fixed_frame,1,fixed_box);



    //设置各种图标ui和外观

    //方向控制按键是通过qss设置的
    ui.btn_stop->setIcon(QIcon(":/images/png/images/kai_2.png"));
    ui.tabWidget->setTabText(0,"基础操作属性");
    ui.tabWidget->setTabText(1,"进阶操作属性");
    //设置摇杆
    rock_widget = new JoyStick(ui.widget_3);
    rock_widget->show();
    rock_widget->resize(180,180);
    rock_widget->move(10,10);
    //Grid
    QTreeWidgetItem* Grid=new QTreeWidgetItem(QStringList()<<"Grid");
    //设置图标
    Grid->setIcon(0,QIcon(":/images/left_dock.svg"));
    //checkbox
    QCheckBox* Grid_Check=new QCheckBox();
    connect(Grid_Check,SIGNAL(stateChanged(int)),this,SLOT(slot_display_grid(int)));
    //添加top节点
    ui.treeWidget->addTopLevelItem(Grid);
    //添加checkbox
    ui.treeWidget->setItemWidget(Grid,1,Grid_Check);
    //默认展开状态
    Grid->setExpanded(true);

    //添加cell count子结点
    QTreeWidgetItem* cell_Count=new QTreeWidgetItem(QStringList()<<"Plane cell Count");
    Grid->addChild(cell_Count);
    //Cellcount添加Spinbox
    cell_count_box = new QSpinBox();
    cell_count_box->setValue(13);
    //设置spinbox宽度
    cell_count_box->setMaximumWidth(150);
    ui.treeWidget->setItemWidget(cell_Count,1,cell_count_box);

    //添加color子结点
    QTreeWidgetItem* Grid_color=new QTreeWidgetItem(QStringList()<<"Color");
    Grid->addChild(Grid_color);
    //Color添加combobox
    Grid_color_box=new QComboBox();
    Grid_color_box->addItem("160;160;160");
    //设置combox可编辑
    Grid_color_box->setEditable(true);
    //设置combox宽度
    Grid_color_box->setMaximumWidth(150);
    ui.treeWidget->setItemWidget(Grid_color,1,Grid_color_box);

    //TF ui
    QTreeWidgetItem* TF=new QTreeWidgetItem(QStringList()<<"TF");
    //设置图标
    TF->setIcon(0,QIcon(":/images/left_dock.svg"));
    //checkbox
    QCheckBox* TF_Check=new QCheckBox();
    connect(TF_Check,SIGNAL(stateChanged(int)),this,SLOT(slot_display_tf(int)));
    //添加top节点
    ui.treeWidget->addTopLevelItem(TF);
    //添加checkbox
    ui.treeWidget->setItemWidget(TF,1,TF_Check);
    //默认展开状态
    //TF->setExpanded(true);

    //laserscan
    QTreeWidgetItem* LaserScan=new QTreeWidgetItem(QStringList()<<"LaserScan");
    //设置图标
    TF->setIcon(0,QIcon(":/images/left_dock.svg"));
    //checkbox
    QCheckBox* Laser_Check=new QCheckBox();
    connect(Laser_Check,SIGNAL(stateChanged(int)),this,SLOT(slot_display_laser(int)));
    //添加top节点
    ui.treeWidget->addTopLevelItem(LaserScan);
    //添加checkbox
    ui.treeWidget->setItemWidget(LaserScan,1,Laser_Check);
    //laser topic
    QTreeWidgetItem* LaserTopic=new QTreeWidgetItem(QStringList()<<"Topic");
    Laser_Topic_box = new QComboBox();
    Laser_Topic_box->addItem("/scan");
    Laser_Topic_box->setEditable(true);
    Laser_Topic_box->setMaximumWidth(150);
    LaserScan->addChild(LaserTopic);
    ui.treeWidget->setItemWidget(LaserTopic,1,Laser_Topic_box);

    //RobotModel ui
    QTreeWidgetItem* RobotModel=new QTreeWidgetItem(QStringList()<<"RobotModel");
    //设置图标
    RobotModel->setIcon(0,QIcon(":/images/left_dock.svg"));
    //checkbox
    QCheckBox* RobotModel_Check=new QCheckBox();
    connect(RobotModel_Check,SIGNAL(stateChanged(int)),this,SLOT(slot_display_RobotModel(int)));
    //添加top节点
    ui.treeWidget->addTopLevelItem(RobotModel);
    //添加checkbox
    ui.treeWidget->setItemWidget(RobotModel,1,RobotModel_Check);

    //map ui
    QTreeWidgetItem* Map=new QTreeWidgetItem(QStringList()<<"Map");
    //设置图标
    Map->setIcon(0,QIcon(":/images/left_dock.svg"));
    //checkbox
    QCheckBox* Map_Check=new QCheckBox();
    connect(Map_Check,SIGNAL(stateChanged(int)),this,SLOT(slot_display_Map(int)));
    //添加top节点
    ui.treeWidget->addTopLevelItem(Map);
    //添加checkbox
    ui.treeWidget->setItemWidget(Map,1,Map_Check);
    //Map topic
    QTreeWidgetItem* MapTopic=new QTreeWidgetItem(QStringList()<<"Topic");
    Map_Topic_box = new QComboBox();
    Map_Topic_box->addItem("/map");
    Map_Topic_box->setEditable(true);
    Map_Topic_box->setMaximumWidth(150);
    Map->addChild(MapTopic);
    ui.treeWidget->setItemWidget(MapTopic,1,Map_Topic_box);
    //Map colorscheme
    QTreeWidgetItem* MapcolorScheme=new QTreeWidgetItem(QStringList()<<"Color Scheme");
    Map_Color_Scheme_box=new QComboBox();
    Map_Color_Scheme_box->addItem("map");
    Map_Color_Scheme_box->addItem("costmap");
    Map_Color_Scheme_box->addItem("raw");
    Map_Color_Scheme_box->setMaximumWidth(150);
    Map->addChild(MapcolorScheme);
    ui.treeWidget->setItemWidget(MapcolorScheme,1,Map_Color_Scheme_box);

    //Path ui
    QTreeWidgetItem* Path=new QTreeWidgetItem(QStringList()<<"Path");
    //设置图标
    Path->setIcon(0,QIcon(":/images/left_dock.svg"));
    //checkbox
    QCheckBox* Path_Check=new QCheckBox();
    connect(Path_Check,SIGNAL(stateChanged(int)),this,SLOT(slot_display_Path(int)));
    //添加top节点
    ui.treeWidget->addTopLevelItem(Path);
    //添加checkbox
    ui.treeWidget->setItemWidget(Path,1,Path_Check);
    //Path topic
    QTreeWidgetItem* PathTopic=new QTreeWidgetItem(QStringList()<<"Topic");
    Path_Topic_box = new QComboBox();
    Path_Topic_box->addItem("/move_base/DWAPlannerROS/local_plan");
    Path_Topic_box->setEditable(true);
    Path_Topic_box->setMaximumWidth(150);
    Path->addChild(PathTopic);
    ui.treeWidget->setItemWidget(PathTopic,1,Path_Topic_box);
    //Path colorscheme
    QTreeWidgetItem* PathcolorScheme=new QTreeWidgetItem(QStringList()<<"Color");
    Path_Color_box=new QComboBox();
    Path_Color_box->addItem("0;12;255");
    Path_Color_box->setMaximumWidth(150);
    Path->addChild(PathcolorScheme);
    ui.treeWidget->setItemWidget(PathcolorScheme,1,Path_Color_box);
    //Marker
    QTreeWidgetItem* Marker=new QTreeWidgetItem(QStringList()<<"Marker");
    //设置图标
    Marker->setIcon(0,QIcon("://images/Marker.png"));
    //checkbox
    QCheckBox* Marker_Check=new QCheckBox();
    Marker_Check->setChecked(true);
//    connect(Marker_Check,SIGNAL(stateChanged(int)),this,SLOT(slot_display_marker(int)));
    connect(Marker_Check,&QCheckBox::stateChanged,this,&MainWindow::slot_display_marker);
    //向Treewidget添加Marker Top节点
    ui.treeWidget->addTopLevelItem(Marker);
    //向Marker添加checkbox
    ui.treeWidget->setItemWidget(Marker,1,Marker_Check);
    //Marker topic
    QTreeWidgetItem* MarkerTopic=new QTreeWidgetItem(QStringList()<<"Marker Topic");
    Marker_Topic_box=new QComboBox();
    Marker_Topic_box->addItem("/marker");
    Marker_Topic_box->setEditable(true);
    Marker_Topic_box->setMaximumWidth(150);
    Marker->addChild(MarkerTopic);
    ui.treeWidget->setItemWidget(MarkerTopic,1,Marker_Topic_box);
    //机器人导航相关ui
    //golabalmap
    QTreeWidgetItem* GolabalMap=new QTreeWidgetItem(QStringList()<<"Golabal Map");
    //设置图标
    GolabalMap->setIcon(0,QIcon(":/images/left_dock.svg"));
    //checkbox
    QCheckBox* GolabalMap_Check=new QCheckBox();
    connect(GolabalMap_Check,SIGNAL(stateChanged(int)),this,SLOT(slot_display_GolabalMap(int)));
    //添加top节点
    ui.treeWidget->addTopLevelItem(GolabalMap);
    //添加checkbox
    ui.treeWidget->setItemWidget(GolabalMap,1,GolabalMap_Check);

    //golabal costmap
    QTreeWidgetItem* Golabal_CostMap=new QTreeWidgetItem(QStringList()<<"CostMap");
    //设置图标
    Golabal_CostMap->setIcon(0,QIcon(":/images/left_dock.svg"));
    GolabalMap->addChild(Golabal_CostMap);
    //GolabalMap topic
    QTreeWidgetItem* Golabal_CostMap_Topic=new QTreeWidgetItem(QStringList()<<"Topic");
    Golabal_CostMap_Topic_box = new QComboBox();
    Golabal_CostMap_Topic_box->addItem("/move_base/global_costmap/costmap");
    Golabal_CostMap_Topic_box->setEditable(true);
    Golabal_CostMap_Topic_box->setMaximumWidth(150);
    Golabal_CostMap->addChild(Golabal_CostMap_Topic);
    ui.treeWidget->setItemWidget(Golabal_CostMap_Topic,1,Golabal_CostMap_Topic_box);
    //Map colorscheme
    QTreeWidgetItem* GolabalMapcolorScheme=new QTreeWidgetItem(QStringList()<<"Color Scheme");
    GolabalMapcolorScheme_box=new QComboBox();
    GolabalMapcolorScheme_box->addItem("costmap");
    GolabalMapcolorScheme_box->addItem("map");
    GolabalMapcolorScheme_box->addItem("raw");
    GolabalMapcolorScheme_box->setMaximumWidth(150);
    GolabalMap->addChild(GolabalMapcolorScheme);
    ui.treeWidget->setItemWidget(GolabalMapcolorScheme,1,GolabalMapcolorScheme_box);

    //golabal planner
    QTreeWidgetItem* Golabal_Planner=new QTreeWidgetItem(QStringList()<<"Topic");
    //设置图标
    Golabal_Planner->setIcon(0,QIcon(":/images/left_dock.svg"));
    GolabalMap->addChild(Golabal_Planner);

    //path topic
    QTreeWidgetItem* Golabal_Planner_Topic=new QTreeWidgetItem(QStringList()<<"Topic");
    Golabal_Planner_Topic_box = new QComboBox();
    Golabal_Planner_Topic_box->addItem("/move_base/DWAPlanner/global_plan");
    Golabal_Planner_Topic_box->setEditable(true);
    Golabal_Planner_Topic_box->setMaximumWidth(150);
    Golabal_Planner->addChild(Golabal_Planner_Topic);
    ui.treeWidget->setItemWidget(Golabal_Planner_Topic,1,Golabal_Planner_Topic_box);
    //path color scheme
    QTreeWidgetItem* GolabalPlannercolorScheme=new QTreeWidgetItem(QStringList()<<"Color Scheme");
    GolabalPlannercolorScheme_box=new QComboBox();
    GolabalPlannercolorScheme_box->addItem("255;0;0");
    GolabalPlannercolorScheme_box->setEditable(true);
    GolabalPlannercolorScheme_box->setMaximumWidth(150);
    GolabalPlannercolorScheme_box->setMaximumWidth(150);
    Golabal_Planner->addChild(GolabalPlannercolorScheme);
    ui.treeWidget->setItemWidget(GolabalPlannercolorScheme,1,GolabalPlannercolorScheme_box);

    //Localmap
    QTreeWidgetItem* LocalMap=new QTreeWidgetItem(QStringList()<<"Local Map");
    //设置图标
    LocalMap->setIcon(0,QIcon(":/images/left_dock.svg"));
    //checkbox
    QCheckBox* LocalMap_Check=new QCheckBox();
    connect(LocalMap_Check,SIGNAL(stateChanged(int)),this,SLOT(slot_display_LocalMap(int)));
    //添加top节点
    ui.treeWidget->addTopLevelItem(LocalMap);
    //添加checkbox
    ui.treeWidget->setItemWidget(LocalMap,1,LocalMap_Check);

    //Local Costmap
    QTreeWidgetItem* Local_CostMap=new QTreeWidgetItem(QStringList()<<"CostMap");
    //设置图标
    Local_CostMap->setIcon(0,QIcon(":/images/left_dock.svg"));
    LocalMap->addChild(Local_CostMap);
    //GolabalMap topic
    QTreeWidgetItem* Local_CostMap_Topic=new QTreeWidgetItem(QStringList()<<"Topic");
    Local_CostMap_Topic_box = new QComboBox();
    Local_CostMap_Topic_box->addItem("/move_base/local_costmap/costmap");
    Local_CostMap_Topic_box->setEditable(true);
    Local_CostMap_Topic_box->setMaximumWidth(150);
    Local_CostMap->addChild(Local_CostMap_Topic);
    ui.treeWidget->setItemWidget(Local_CostMap_Topic,1,Local_CostMap_Topic_box);
    //Map colorscheme
    QTreeWidgetItem* LocalMapcolorScheme=new QTreeWidgetItem(QStringList()<<"Color Scheme");
    LocalMapcolorScheme_box=new QComboBox();
    LocalMapcolorScheme_box->addItem("costmap");
    LocalMapcolorScheme_box->addItem("map");
    LocalMapcolorScheme_box->addItem("raw");
    LocalMapcolorScheme_box->setMaximumWidth(150);
    LocalMap->addChild(LocalMapcolorScheme);
    ui.treeWidget->setItemWidget(LocalMapcolorScheme,1,LocalMapcolorScheme_box);

    //Local planner
    QTreeWidgetItem* Local_Planner=new QTreeWidgetItem(QStringList()<<"Planner");
    //设置图标
    Local_Planner->setIcon(0,QIcon(":/images/left_dock.svg"));
    LocalMap->addChild(Local_Planner);

    //path topic
    QTreeWidgetItem* Local_Planner_Topic=new QTreeWidgetItem(QStringList()<<"Topic");
    Local_Planner_Topic_box = new QComboBox();
    Local_Planner_Topic_box->addItem("/move_base/DWAPlannerROS/local_plan");
    Local_Planner_Topic_box->setEditable(true);
    Local_Planner_Topic_box->setMaximumWidth(150);
    Local_Planner->addChild(Local_Planner_Topic);
    ui.treeWidget->setItemWidget(Local_Planner_Topic,1,Local_Planner_Topic_box);
    //path color scheme
    QTreeWidgetItem* LocalPlannercolorScheme=new QTreeWidgetItem(QStringList()<<"Color Scheme");
    LocalPlannercolorScheme_box=new QComboBox();
    LocalPlannercolorScheme_box->addItem("0;12;255");
    LocalPlannercolorScheme_box->setEditable(true);
    LocalPlannercolorScheme_box->setMaximumWidth(150);
    LocalPlannercolorScheme_box->setMaximumWidth(150);
    Local_Planner->addChild(LocalPlannercolorScheme);
    ui.treeWidget->setItemWidget(LocalPlannercolorScheme,1,LocalPlannercolorScheme_box);

    //虚拟摇杆控制

    connect(rock_widget,&JoyStick::keyNumchanged,this,&MainWindow::slot_rockKeyChange);
    //运动控制在右侧和rviz显示冲突，在左边单开一页用来显示rviz和订阅图像。


    //set_start_btn,设置起始位置
    connect(ui.Set_Start_btn,&QPushButton::clicked,this,&MainWindow::slot_set_start_pose);
    //set_goal_btn，设置导航点
    connect(ui.set_goal_btn,&QPushButton::clicked,this,&MainWindow::slot_set_goal_pose);
    //坐标
    connect(&qnode,&QNode::position,this,&MainWindow::slot_update_pose);
    //返航
    connect(ui.set_return_pos,&QPushButton::clicked,this,&MainWindow::slot_set_return_pose);
    connect(ui.return_btn,&QPushButton::clicked,this,&MainWindow::slot_return_pose);

    //connectsaf
    connect(&qnode,SIGNAL(speed_vel(float,float)),this,SLOT(slot_update_dashboard(float,float)));
    connect(&qnode,SIGNAL(power_vel(float)),this,SLOT(slot_update_power(float)));
    connect(&qnode,SIGNAL(image_val(QImage)),this,SLOT(slot_update_image(QImage)));
    connect(&qnode,SIGNAL(position(double,double,double)),this,SLOT(slot_update_pos(double,double,double)));
    connect(ui.pushButton_sub_image,SIGNAL(clicked()),this,SLOT(slot_sub_image()));
    connect(ui.laser_btn,SIGNAL(clicked()),this,SLOT(slot_quick_cmd_clicked()));
    //set start pose
    connect(ui.Set_Start_btn,SIGNAL(clicked()),this,SLOT(slot_set_start_btn()));
    connect(ui.set_goal_btn,SIGNAL(clicked()),this,SLOT(slot_set_goal_btn()));
    connect(ui.set_return_pos,SIGNAL(clicked()),this,SLOT(slot_set_return_pos()));
    connect(ui.return_btn,SIGNAL(clicked()),this,SLOT(slot_return()));
}
void MainWindow::slot_set_return_pose()
{
    ui.return_x->setText(ui.pos_X->text());
    ui.return_y->setText(ui.pos_y->text());
    ui.pos_z_2->setText(ui.pos_z->text());
}
void MainWindow::slot_return_pose()
{
    qnode.set_goal(ui.return_x->text().toDouble(),ui.return_y->text().toDouble(),ui.pos_z_2->text().toDouble());
}
void MainWindow::slot_update_pose(double x,double y,double z)
{
    ui.pos_X->setText(QString::number(x));
    ui.pos_y->setText(QString::number(y));
    ui.pos_z->setText(QString::number(z));
}
void MainWindow::slot_set_start_pose()
{
    myrviz->Set_Start_pose();

}

void MainWindow::slot_set_goal_pose()
{
    myrviz->Set_Goal_pose();

}
void MainWindow::slot_set_return_pos()
{
    ui.return_x->setText(ui.pos_X->text());
    ui.return_y->setText(ui.pos_y->text());
    ui.pos_z_2->setText(ui.pos_z->text());
}
void MainWindow::slot_display_marker(int state)
{
    bool enable=state>1?true:false;
    myrviz->Display_Marker(Marker_Topic_box->currentText(),enable);
//    myrviz_2->Display_Marker(Marker_Topic_box->currentText(),enable);
//    myrviz_3->Display_Marker(Marker_Topic_box->currentText(),enable);
}
void MainWindow::initTopicList()
{
    ui.listWidget_msg->clear();
    ui.listWidget_msg->addItem(QString("%1   (%2)").arg("Name","Type"));
    QMap<QString,QString> topic_list= qnode.get_topic_list();
    for(QMap<QString,QString>::iterator iter=topic_list.begin();iter!=topic_list.end();iter++)
    {
       ui.listWidget_msg->addItem(QString("%1   (%2)").arg(iter.key(),iter.value()));
//       ui.topic_listWidget->addItem("hello");
    }
}
void MainWindow::slot_refresh_topic()
{
    initTopicList();
//    ui.topic_listWidget->clear();
//    ui.topic_listWidget->addItem(QString("%1   (%2)").arg("Name","Type"));
//    QMap<QString,QString> topic_list= qnode.get_topic_list();
//    for(QMap<QString,QString>::iterator iter=topic_list.begin();iter!=topic_list.end();iter++)
//    {
//       ui.topic_listWidget->addItem(QString("%1   (%2)").arg(iter.key(),iter.value()));
//       ui.topic_listWidget->addItem("hello");
//    }
}
void MainWindow::slot_rockKeyChange(int key) {
  //速度
  float liner = ui.label_linera->text().toFloat()*0.01;
  float turn = ui.label_raw->text().toFloat()*0.01;

  bool is_all = ui.checkBox_is_all->isChecked();

  switch (key) {
    case upleft:
      qnode.set_cmd_vel(is_all ? 'U' : 'u', liner, turn);
      break;
    case up:
      qnode.set_cmd_vel(is_all ? 'I' : 'i', liner, turn);
      break;
    case upright:
      qnode.set_cmd_vel(is_all ? 'O' : 'o', liner, turn);
      break;
    case left:
      qnode.set_cmd_vel(is_all ? 'J' : 'j', liner, turn);
      break;
    case right:
      qnode.set_cmd_vel(is_all ? 'L' : 'l', liner, turn);
      break;
    case down:
      qnode.set_cmd_vel(is_all ? 'M' : 'm', liner, turn);
      break;
    case downleft:
      qnode.set_cmd_vel(is_all ? '<' : ',', liner, turn);
      break;
    case downright:
      qnode.set_cmd_vel(is_all ? '>' : '.', liner, turn);
      break;
  }
}
void MainWindow::slot_return()
{
    qnode.set_goal(ui.return_x->text().toDouble(),ui.return_y->text().toDouble(),ui.pos_z_2->text().toDouble());
}
void MainWindow::slot_update_pos(double x,double y,double z)
{
    ui.pos_X->setText(QString::number(x));
    ui.pos_y->setText(QString::number(y));
    ui.pos_z->setText(QString::number(z));
}
void MainWindow::slot_display_GolabalMap(int state)
{
    bool enable=state>1?true:false;
    QStringList qli=GolabalPlannercolorScheme_box->currentText().split(";");
    QColor color=QColor(qli[0].toInt(),qli[1].toInt(),qli[2].toInt());
    myrviz->Display_Global_Map(Golabal_CostMap_Topic_box->currentText(),GolabalMapcolorScheme_box->currentText(),Golabal_Planner_Topic_box->currentText(),color,enable);
}
void MainWindow::slot_display_LocalMap(int state)
{
     bool enable=state>1?true:false;
     QStringList qli=LocalPlannercolorScheme_box->currentText().split(";");
     QColor color=QColor(qli[0].toInt(),qli[1].toInt(),qli[2].toInt());
     myrviz->Display_Local_Map(Local_CostMap_Topic_box->currentText(),LocalMapcolorScheme_box->currentText(),Local_Planner_Topic_box->currentText(),color,enable);
}
void MainWindow::slot_set_start_btn()
{
    myrviz->Set_Start_Pose();
}
void MainWindow::slot_set_goal_btn()
{
    myrviz->Set_Goal_Pose();
}
void MainWindow::slot_display_Path(int state)
{
    bool enable=state>1?true:false;
    QStringList qli=Path_Color_box->currentText().split(";");
    QColor color=QColor(qli[0].toInt(),qli[1].toInt(),qli[2].toInt());
    myrviz->Display_Path(Path_Topic_box->currentText(),color,enable);
}
void MainWindow::slot_display_Map(int state)
{
    bool enable=state>1?true:false;
    myrviz->Display_Map(Map_Topic_box->currentText(),Map_Color_Scheme_box->currentText(),enable);
}
void MainWindow::slot_display_RobotModel(int state)
{
    bool enable=state>1?true:false;
    myrviz->Display_RobotModel(enable);
}
void MainWindow::slot_display_laser(int state)
{
    bool enable=state>1?true:false;
    myrviz->Display_LaserScan(Laser_Topic_box->currentText(),enable);
}
void MainWindow::slot_display_tf(int state)
{
    bool enable=state>1?true:false;
    myrviz->Display_TF(enable);
}
void MainWindow::slot_display_grid(int state)
{
    bool enable=state>1?true:false;
    QStringList qli=Grid_color_box->currentText().split(";");
    QColor color=QColor(qli[0].toInt(),qli[1].toInt(),qli[2].toInt());
    myrviz->Display_grid(cell_count_box->text().toInt(),color,enable);
}

void MainWindow::slot_treewidget_value_change(QString)
{
    myrviz->Set_fixedFrame(fixed_box->currentText());
}
void MainWindow::slot_quick_cmd_clicked()
{
    laser_cmd=new QProcess;
    laser_cmd->start("bash");
    laser_cmd->write(ui.textEdit_laser_cmd->toPlainText().toLocal8Bit()+'\n');
    connect(laser_cmd,SIGNAL(readyReadStandardError()),this,SLOT(slot_quick_output()));
    connect(laser_cmd,SIGNAL(readyReadStandardOutput()),this,SLOT(slot_quick_output()));

}
void MainWindow::slot_quick_output()
{
    ui.textEdit_quick_output->append("<font color=\"#FF0000\">"+laser_cmd->readAllStandardError()+"</font>");
    ui.textEdit_quick_output->append("<font color=\"#FFFFFF\">"+laser_cmd->readAllStandardOutput()+"</font>");
}
void MainWindow::slot_update_image(QImage im)
{
    ui.label_image->setPixmap(QPixmap::fromImage(im));
}
void MainWindow::slot_sub_image()
{
    qnode.sub_image(ui.lineEdit_image_topic->text());
}
void MainWindow::slot_update_power(float value)
{
      ui.label_power_val->setText(QString::number(value).mid(0,5)+"V");
      double n=(value-10.5)/(12.5-10.5);
      int val=n*100;
      ui.progressBar->setValue(val);
}
void MainWindow::slot_update_dashboard(float x,float y)
{
    ui.label_dir_x->setText(x>0?"正向":"反向");
    ui.label_dir_y->setText(y>0?"正向":"反向");
    speed_x_dashBoard->setValue(abs(x)*100);
    speed_y_dashBoard->setValue(abs(y)*100);

}
void MainWindow::slot_pushbtn_click()
{
  QPushButton* btn=qobject_cast<QPushButton*> (sender());
  char k=btn->text().toStdString()[0];
  bool is_all=ui.checkBox_is_all->isChecked();
  float linear=ui.label_linera->text().toFloat()*0.01;
  float angular=ui.label_raw->text().toFloat()*0.01;

  switch (k) {
    case 'i':
      qnode.set_cmd_vel(is_all?'I':'i',linear,angular);
      break;
  case 'u':
    qnode.set_cmd_vel(is_all?'U':'u',linear,angular);
    break;
  case 'o':
    qnode.set_cmd_vel(is_all?'O':'o',linear,angular);
    break;
  case 'j':
    qnode.set_cmd_vel(is_all?'J':'j',linear,angular);
    break;
  case 'l':
    qnode.set_cmd_vel(is_all?'L':'l',linear,angular);
    break;
  case 'm':
    qnode.set_cmd_vel(is_all?'M':'m',linear,angular);
    break;
  case ',':
    qnode.set_cmd_vel(is_all?'<':',',linear,angular);
    break;
  case '.':
    qnode.set_cmd_vel(is_all?'>':'.',linear,angular);
    break;
  }
}
void MainWindow::slot_linera_value_change(int value)
{
    ui.label_linera->setText(QString::number(value));
}
void MainWindow::slot_raw_value_change(int value)
{
    ui.label_raw->setText(QString::number(value));
}
MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::on_button_connect_clicked(bool check ) {
	if ( ui.checkbox_use_environment->isChecked() ) {
		if ( !qnode.init() ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
            ui.treeWidget->setEnabled(true);
            myrviz=new qrviz(ui.Layout_rviz);
		}
	} else {
		if ( ! qnode.init(ui.line_edit_master->text().toStdString(),
				   ui.line_edit_host->text().toStdString()) ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
			ui.line_edit_master->setReadOnly(true);
			ui.line_edit_host->setReadOnly(true);
			ui.line_edit_topic->setReadOnly(true);
            ui.treeWidget->setEnabled(true);
            myrviz=new qrviz(ui.Layout_rviz);
		}
	}
}


void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
	bool enabled;
	if ( state == 0 ) {
		enabled = true;
	} else {
		enabled = false;
	}
	ui.line_edit_master->setEnabled(enabled);
	ui.line_edit_host->setEnabled(enabled);
	//ui.line_edit_topic->setEnabled(enabled);
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
        ui.view_logging->scrollToBottom();
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "robot_control_ui");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    ui.line_edit_master->setText(master_url);
    ui.line_edit_host->setText(host_url);
    //ui.line_edit_topic->setText(topic_name);
    bool remember = settings.value("remember_settings", false).toBool();
    ui.checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    ui.checkbox_use_environment->setChecked(checked);
    if ( checked ) {
    	ui.line_edit_master->setEnabled(false);
    	ui.line_edit_host->setEnabled(false);
    	//ui.line_edit_topic->setEnabled(false);
    }
}
void MainWindow::slot_cmd_stop()
{
    qnode.cmd_stop();
}
void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "robot_control_ui");
    settings.setValue("master_url",ui.line_edit_master->text());
    settings.setValue("host_url",ui.line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));

}

void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();
	QMainWindow::closeEvent(event);
}

}  // namespace robot_control_ui

