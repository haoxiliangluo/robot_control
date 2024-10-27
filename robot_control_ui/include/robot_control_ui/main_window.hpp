/**
 * @file /include/robot_control_ui/main_window.hpp
 *
 * @brief Qt based gui for robot_control_ui.
 *
 * @date November 2010
 **/
#ifndef robot_control_ui_MAIN_WINDOW_H
#define robot_control_ui_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtWidgets/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include "CCtrlDashBoard.h"
#include <QImage>
#include <QProcess>
#include <QComboBox>
#include "qrviz.hpp"
#include <QSpinBox>
#include "joystick.h"
/*****************************************************************************
** Namespace
*****************************************************************************/

namespace robot_control_ui {
struct MyPose{
    double x;
    double y;
    double z;
};
/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();
    enum {
      upleft = 0,
      up,
      upright,
      left,
      stop,
      right,
      downleft,
      down,
      downright
    };
    enum LogLevel {
             Debug,
             Info,
             Warn,
             Error,
             Fatal
     };
	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

    //创建虚拟摇杆
    JoyStick *rock_widget;
public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();
	void on_button_connect_clicked(bool check );
	void on_checkbox_use_environment_stateChanged(int state);

    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically
    void slot_linera_value_change(int);
    void slot_raw_value_change(int);
    void slot_pushbtn_click();
    void slot_update_dashboard(float,float);
    void slot_update_power(float);
    void slot_update_image(QImage);
    void slot_sub_image();
    void slot_quick_cmd_clicked();
    void slot_quick_output();
    void slot_treewidget_value_change(QString);
    void slot_display_grid(int);
    void slot_display_tf(int);
    void slot_display_laser(int);
    void slot_display_RobotModel(int);
    void slot_display_Map(int);
    void slot_display_Path(int);
    void slot_set_start_btn();
    void slot_set_goal_btn();
    void slot_display_GolabalMap(int);
    void slot_display_LocalMap(int);
    void slot_update_pos(double,double,double);
    void slot_set_return_pos();
    void slot_display_marker(int);
    void slot_return();
    void slot_refresh_topic();
    void slot_cmd_stop();
    void initTopicList();
    void slot_rockKeyChange(int key);
    void slot_set_start_pose();
    void slot_set_goal_pose();
    void slot_update_pose(double x,double y,double z);
    //返航
    void slot_set_return_pose();
    void slot_return_pose();
private:
	Ui::MainWindowDesign ui;
	QNode qnode;
    CCtrlDashBoard* speed_x_dashBoard;
    CCtrlDashBoard* speed_y_dashBoard;
    QProcess *laser_cmd;
    qrviz *myrviz;
    QComboBox* fixed_box;
    QSpinBox* cell_count_box;
    QComboBox* Grid_color_box;
    QComboBox* Laser_Topic_box;
    QComboBox* Map_Topic_box;
    QComboBox* Path_Topic_box;
    QComboBox* Map_Color_Scheme_box;
    QComboBox* Path_Color_box;
    QComboBox* Golabal_CostMap_Topic_box;
    QComboBox* GolabalMapcolorScheme_box;
    QComboBox* Golabal_Planner_Topic_box;
    QComboBox* GolabalPlannercolorScheme_box;
    QComboBox* Local_CostMap_Topic_box;
    QComboBox* LocalMapcolorScheme_box;
    QComboBox* Local_Planner_Topic_box;
    QComboBox* LocalPlannercolorScheme_box;
    QComboBox *Marker_Topic_box;
};

}  // namespace robot_control_ui

#endif // robot_control_ui_MAIN_WINDOW_H
