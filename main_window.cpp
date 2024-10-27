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
#include <QPushButton>
#include <QSerialPort>
#include <QByteArray>
#include <QDebug>
#include "../include/serailDemo/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace class1_serailDemo {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
    ,      baudRate(QSerialPort::Baud9600),  // 初始化波特率
      dataBits(QSerialPort::Data8),     // 初始化数据位
      stopBits(QSerialPort::OneStop),   // 初始化停止位
      parity(QSerialPort::NoParity) , receivedDataSize(0)   , qnode(argc,argv)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

//    ReadSettings();
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
    //测试
    QList<QSerialPortInfo> serialPortInfoList = QSerialPortInfo::availablePorts();
    foreach (const QSerialPortInfo &info, serialPortInfoList) {
        qDebug() << "串口名称:" << info.portName();
        qDebug() << "描述:" << info.description();
        qDebug() << "制造商:" << info.manufacturer();
        qDebug() << "序列号:" << info.serialNumber();
        qDebug() << "系统位置:" << info.systemLocation();
        qDebug() << "供应商ID:" << info.vendorIdentifier();
        qDebug() << "产品ID:" << info.productIdentifier();
        qDebug() << "是否为空:" << info.isNull();
        qDebug() << "是否忙碌:" << info.isBusy();
        qDebug() << "是否有效:" << info.isValid();

    }
    //初始界面属性设置
    ui.serial_port_2->setEditable(true);
    ui.buote_edit->addItem("9600");
    ui.buote_edit->addItem("19200");
    ui.buote_edit->setEditable(true);
    ui.data_bit_edit->addItem("8");
    ui.data_bit_edit->setEditable(true);
    ui.stop_bit_edit->addItem("1");
    ui.stop_bit_edit->setEditable(true);
    ui.jiaoyan_edit->addItems({"None", "Even", "Odd", "Mark", "Space"});
    ui.jiaoyan_edit->setEditable(true);
    ui.listWidget->setVerticalScrollBar(ui.verticalScrollBar_2);
    ui.textEdit->setVerticalScrollBar(ui.verticalScrollBar);
    report();

    //打开串口
    connect(ui.open_serial_btn,&QPushButton::clicked,this,&MainWindow::slot_open_serial);
    //关闭串口
    connect(ui.close_serial_btn,&QPushButton::clicked,this,&MainWindow::slot_close_serial);
    // 连接信号槽以更新参数
    connect(ui.data_bit_edit, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MainWindow::updateDataBits);
    connect(ui.buote_edit, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MainWindow::updateBaudRate);
    connect(ui.stop_bit_edit, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MainWindow::updateStopBits);
    connect(ui.jiaoyan_edit, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MainWindow::updateParity);

    connect(ui.clean_ptn_1,&QPushButton::clicked,this,&MainWindow::cleanAccept);
    connect(ui.clean_ptn_2,&QPushButton::clicked,this,&MainWindow::cleanSend);
    connect(ui.send_messger_ptn,&QPushButton::clicked,this,&MainWindow::sendMessge);
    connect(&serial,&QSerialPort::readyRead,this,&MainWindow::readSerialData);
    connect(ui.report_btn,&QPushButton::clicked,this,&MainWindow::report);
    connect(ui.save_btn,&QPushButton::clicked,this,&MainWindow::save_data);
}

MainWindow::~MainWindow() {
    if(serial.isOpen())
    {
        serial.close();
        logMessage("串口关闭成功！");
    }
}

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
void MainWindow::report()
{
    ui.serial_port_2->clear(); //清空下拉菜单

        portList = QSerialPortInfo::availablePorts(); //获取端口列表

        //添加下拉菜单
        for (QSerialPortInfo &i : portList)
        {
            ui.serial_port_2->addItem(i.portName());
        }
}
void MainWindow::sendMessge() {
    if (serial.isOpen()) {
        QString message = ui.textEdit->toPlainText();
        QByteArray data = message.toUtf8();
        if(ui.checkBox_3->isChecked())
        {
            message = data.toHex(' ').toUpper(); // 将数据转换为十六进制字符串，并用空格分隔
            serial.write(data);
        }
        else
        {
            serial.write(data);
        }
        receivedDataSize += data.size();
        if(ui.checkBox_5->isChecked())
        {

            if (receivedDataSize >= 200 * 1024)
            {
                cleanAccept();
                receivedDataSize = 0;
            }
            //            logMessage("当前数据量"+QString::number(receivedDataSize));
        }
        logMessage("发送: " + message);
        if(ui.checkBox_5->isChecked())
        {

            if (receivedDataSize >= 200 * 1024)
            {
                cleanAccept();
                receivedDataSize = 0;
            }
        }
    } else {
        logMessage("串口并没有被打开.");
        QMessageBox::warning(this, "warning！", "串口并没有被打开,脑子不要可以捐了.");
    }
}
void MainWindow::readSerialData()
{
   // serial.write("asda");
    while (serial.canReadLine()) {
        QByteArray data = serial.read(serial.bytesAvailable());
        receivedDataSize += data.size();
    if (ui.checkBox_5->isChecked() && receivedDataSize >= 200 * 1024) {
            cleanAccept();
            receivedDataSize = 0;
        }
        QString message;
        if (ui.checkBox_2->isChecked()) {
            message = toHex(data);
        } else  {
            message = QString::fromUtf8(data);
        }
        logMessage("接收: "+message);
    }
}
QString MainWindow::toHex(const QByteArray &data)
{
    QString hexString;
    for (char byte : data) {
        hexString.append(QString("%1 ").arg(static_cast<unsigned char>(byte), 2, 16, QChar('0')).toUpper());
    }
    return hexString.trimmed();
}
void MainWindow::cleanAccept() {
    ui.listWidget->clear();
}
void MainWindow::save_data()
{
    QString filename = QFileDialog::getSaveFileName(this, tr("选择保存路径"), "");
    QFile file(filename);
    file.open(QIODevice::ReadWrite | QIODevice::Text);
    //获取日志信息
    QStringList saveData;
    for(int i=0 ;i<ui.listWidget->count();i++)
    {
        QListWidgetItem* item = ui.listWidget->item(i);
        saveData << item->text();
    }

    // 将 QStringList 转换为单个 QString
    QString saveDataString = saveData.join("\n");

    // 写入日志信息
    if (file.write(saveDataString.toUtf8()) > 0) {
        // 弹出成功对话框
        QMessageBox::information(this, tr("保存数据"), tr("保存成功！"));
    } else {
        QMessageBox::warning(this, tr("保存数据"), tr("保存失败！"));
    }

}
void MainWindow::cleanSend() {
    ui.textEdit->clear();
}
void MainWindow::logMessage(const QString &message) {
    if (ui.time_Check->isChecked())
    {
        getTime();
    }
    ui.listWidget->addItem(time+message);
}
void MainWindow::updateDataBits() {
    QString dataBitsText = ui.data_bit_edit->currentText();
    if (dataBitsText == "5") {
        dataBits = QSerialPort::Data5;
    } else if (dataBitsText == "6") {
        dataBits = QSerialPort::Data6;
    } else if (dataBitsText == "7") {
        dataBits = QSerialPort::Data7;
    } else if (dataBitsText == "8") {
        dataBits = QSerialPort::Data8;
    }
}

void MainWindow::updateBaudRate() {
    QString baudRateText = ui.buote_edit->currentText();
    baudRate = static_cast<QSerialPort::BaudRate>(baudRateText.toInt());
}

void MainWindow::updateStopBits() {
    QString stopBitsText = ui.stop_bit_edit->currentText();
    if (stopBitsText == "1") {
        stopBits = QSerialPort::OneStop;
    } else if (stopBitsText == "1.5") {
        stopBits = QSerialPort::OneAndHalfStop;
    } else if (stopBitsText == "2") {
        stopBits = QSerialPort::TwoStop;
    }
}
void MainWindow::updateParity() {
    QString parityText = ui.jiaoyan_edit->currentText();
    if (parityText == "None") {
        parity = QSerialPort::NoParity;
    } else if (parityText == "Even") {
        parity = QSerialPort::EvenParity;
    } else if (parityText == "Odd") {
        parity = QSerialPort::OddParity;
    } else if (parityText == "Mark") {
        parity = QSerialPort::MarkParity;
    } else if (parityText == "Space") {
        parity = QSerialPort::SpaceParity;
    }
}


void MainWindow::update_serial_date()
{
    serial.setPortName(ui.serial_port_2->currentText());//串口名
}
void MainWindow::slot_open_serial()
{   //相当于sudo ，pkexec一般用于图形化编程
    QString command = "pkexec chmod 666 /dev/" + portList.at(ui.serial_port_2->currentIndex()).portName();
    system(command.toUtf8());
    update_serial_date();
    serial.setParity(parity);
    serial.setBaudRate(baudRate);
    serial.setDataBits(dataBits);
    serial.setStopBits(stopBits);
    if (serial.open(QIODevice::ReadWrite)) {
        logMessage("串口打开成功！");

    } else {
        logMessage("串口打开失败！");
        qDebug()<<serial.error();
    }
}
void MainWindow::slot_close_serial()
{
    if(serial.isOpen())
    {
        serial.close();
        logMessage("串口关闭成功！");
    }
}
void MainWindow::on_button_connect_clicked(bool check ) {
	if ( ui.checkbox_use_environment->isChecked() ) {
		if ( !qnode.init() ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
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
		}
	}
}
void MainWindow::getTime()
{
    this->time.clear();
    QTime now = QTime::currentTime();
    //格式化字符串，两位数字，不足补0
    QString hour = QString("%1").arg(now.hour(), 2, 10, QChar('0'));
    QString minute = QString("%1").arg(now.minute(), 2, 10, QChar('0'));
    QString second = QString("%1").arg(now.second(), 2, 10, QChar('0'));
    this->time = "[" + hour + ":" + minute + ":" + second + "] ";

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
    QSettings settings("Qt-Ros Package", "class1_serailDemo");
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

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "class1_serailDemo");
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

}  // namespace class1_serailDemo

