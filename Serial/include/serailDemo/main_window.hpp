/**
 * @file /include/class1_serailDemo/main_window.hpp
 *
 * @brief Qt based gui for class1_serailDemo.
 *
 * @date November 2010
 **/
#ifndef class1_serailDemo_MAIN_WINDOW_H
#define class1_serailDemo_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtWidgets/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include "QSerialPort"
#include "QSerialPortInfo"
/*****************************************************************************
** Namespace
*****************************************************************************/

namespace class1_serailDemo {

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
    QSerialPort serial;
	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();
    void update_serial_date();//更新串口消息
    void cleanAccept();//清空接收
    void cleanSend();//清空发送
    void sendMessge();//发送数据
    void updateDataBits();  // 更新数据位
    void updateBaudRate();  // 更新波特率
    void updateStopBits();  // 更新停止位
    void updateParity();    // 更新校验位
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
    void slot_open_serial();//打开串口
    void slot_close_serial();//关闭串口
    void logMessage(const QString &message); // 在QListWidget中输出消息
    QString toHex(const QByteArray &data); // 将数据转换为Hex字符串
     void readSerialData(); // 读取串口数据

private:
	Ui::MainWindowDesign ui;
	QNode qnode;

    // 定义串口参数变量
    QSerialPort::BaudRate baudRate;     // 波特率
    QSerialPort::DataBits dataBits;     // 数据位
    QSerialPort::StopBits stopBits;     // 停止位
    QSerialPort::Parity parity;         // 校验位
    qint64 receivedDataSize; // 接收到的数据量

};

}  // namespace class1_serailDemo

#endif // class1_serailDemo_MAIN_WINDOW_H
