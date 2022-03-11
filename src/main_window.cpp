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
#include "../include/hg_gui/main_window.hpp"
#include <QDebug>

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "ros/ros.h"
#include "std_msgs/String.h"

using namespace std;
string workspace_path;  // 工作空间路径

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace hg_gui {

using namespace Qt;

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
    setWindowIcon(QIcon(":/images/HG_Logo.jpg"));
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

    setWindowTitle("RoboGenius");

    // user_name = 0  /home/pickle
    // user_name = 1  /root

	ros::init(argc, argv, "robogenius_node");    //初始化节点，向节点管理器注册
	ros::NodeHandle nh("~");    //用于launch文件传递参数

	nh.param("workspace_path", workspace_path, std::string("/home/hgsim/hg_ws"));    //从launch文件获取工作空间路径参数

    user_name_str = QStandardPaths::writableLocation(QStandardPaths::HomeLocation);

    string ws_path = workspace_path;
	const char * ws_name = ws_path.data(); // 工作空间路径

    // ws_name = "/home/hgsim/hg_ws";
    user_ws_str = user_name_str + ws_name;
    source_ws = "source " + user_ws_str;
//    qDebug() << "user_name_str" << "source "+user_name_str+"/hg_ws/devel/setup.bash\nroslaunch cobot_moveit_config demo_gazebo.launch";

    user_name = 0;
    btn_disable();

    /*运行roscore*/
    bash_cmd = new QProcess;
    bash_cmd->start("bash");
    bash_cmd->write("roscore\n");

    for(int i=0;i<5;i++)
    {
        score[i] = 0;
    }

    connect(ui.pushButton_wspeed,SIGNAL(clicked()),this,SLOT(slot_pushbtn_click()));
    connect(ui.pushButton_aspeed,SIGNAL(clicked()),this,SLOT(slot_pushbtn_click()));
    connect(ui.pushButton_sspeed,SIGNAL(clicked()),this,SLOT(slot_pushbtn_click()));
    connect(ui.pushButton_dspeed,SIGNAL(clicked()),this,SLOT(slot_pushbtn_click()));
    connect(ui.pushButton_xspeed,SIGNAL(clicked()),this,SLOT(slot_pushbtn_click()));

    connect(&qnode,&QNode::forward,this,&MainWindow::voice_forward);
    connect(&qnode,&QNode::back_off,this,&MainWindow::voice_back_off);
    connect(&qnode,&QNode::turn_left,this,&MainWindow::voice_turn_left);
    connect(&qnode,&QNode::turn_right,this,&MainWindow::voice_turn_right);
    connect(&qnode,&QNode::stop,this,&MainWindow::voice_stop);

    Init_timer();
    Scara_Widgets_element_Init();
    //on_button_connect_clicked(true);

    ssocket = new QUdpSocket(this);
    ssocket->bind(8080);
    ip = "127.0.0.1";
    port = 8089;


//    QString fileName = QFileDialog::getOpenFileName(this,tr("选择日志文件"),"",tr("TXT(*.txt)")); //选择路径
//    QIODevice::WriteOnly | QIODevice::Append |
//    QFile file("/root/result.txt");
//    if(!file.open( QIODevice::Text | QIODevice::ReadOnly))
//    {
//        std::cout << "Open failed." << std::endl;
//    }
//    while (!file.atEnd())
//    {
//        QByteArray line = file.readLine();
//        QString str(line);
//        score[count] = str.toInt();
//        count++;
//    }
//    file.close();

//    for(int i=0;i<50;i++)
//    {
//        qDebug() << *(score+i);
//    }

//    QFile file("/root/result.txt");
//    if(!file.open( QIODevice::Text | QIODevice::WriteOnly))
//    {
//        std::cout << "Open failed." << std::endl;
//    }
//    QTextStream txtOutput(&file);
//    for(int i=0;i<50;i++)
//    {
//        txtOutput << *(score+i) << endl;
//    }
//    file.close();

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

void MainWindow::sendmsg()
{
//    unsigned char buf[5];
//    for(int i=0;i<5;i++)
//    {
//        buf[i] = i;
//    }
    //  u8 *msg = buf;
    //  memcpy(msg,&JC2HCDate.Rsp,sizeof(JC2HCDate.Rsp));     //内存拷贝
    //  //CRC校验码计算
    //  Utilities::getCRC16(buf,268);
    QByteArray JCdata((char*)score,5);  //定义字节数组存储发送数据
    QString send_data = JCdata.toHex();
    qDebug() << JCdata.toHex();     //在应用程序输出处打印发送数据
    ssocket->writeDatagram(JCdata.data(),JCdata.size(),QHostAddress(ip),port);
}

void MainWindow::btn_enable()
{
    ui.pushButton_aspeed->setEnabled(true);
    ui.pushButton_dspeed->setEnabled(true);
    ui.pushButton_remote->setEnabled(true);
    ui.pushButton_sspeed->setEnabled(true);
    ui.pushButton_wspeed->setEnabled(true);
    ui.pushButton_xspeed->setEnabled(true);
    ui.pushButton_nav_map->setEnabled(true);
    ui.pushButton_savemap->setEnabled(true);
    ui.pushButton_buildmap->setEnabled(true);
    ui.pushButton_language->setEnabled(true);
    ui.pushButton_loadworld->setEnabled(true);
    ui.pushButton_time_stop->setEnabled(true);
    ui.pushButton_voice_nav->setEnabled(true);
    ui.pushButton_Unitchange->setEnabled(true);
    ui.pushButton_image_view->setEnabled(true);
    ui.pushButton_pick_check->setEnabled(true);
    ui.pushButton_time_start->setEnabled(true);
    ui.pushButton_build_world->setEnabled(true);
    ui.pushButton_getcurangle->setEnabled(true);
    ui.pushButton_place_check->setEnabled(true);
    ui.pushButton_pickandplace->setEnabled(true);
    ui.pushButton_update_score->setEnabled(true);
    ui.pushButton_getrobotstates->setEnabled(true);
    ui.pushButton_nav_map_turtle->setEnabled(true);
    ui.pushButton_savemap_turtle->setEnabled(true);
    ui.pushButton_buildmap_turtle->setEnabled(true);
    ui.pushButton_open_nav_config->setEnabled(true);
    ui.pushButton_loadworld_turtle->setEnabled(true);
    ui.pushButton_voice_navigation->setEnabled(true);
    ui.pushButton_hand_nav->setEnabled(true);
    ui.pushButton_voice_Ctrl->setEnabled(true);
}

void MainWindow::btn_disable()
{
    ui.pushButton_aspeed->setEnabled(false);
    ui.pushButton_dspeed->setEnabled(false);
    ui.pushButton_remote->setEnabled(false);
    ui.pushButton_sspeed->setEnabled(false);
    ui.pushButton_wspeed->setEnabled(false);
    ui.pushButton_xspeed->setEnabled(false);
    ui.pushButton_nav_map->setEnabled(false);
    ui.pushButton_savemap->setEnabled(false);
    ui.pushButton_buildmap->setEnabled(false);
    ui.pushButton_language->setEnabled(false);
    ui.pushButton_loadworld->setEnabled(false);
    ui.pushButton_time_stop->setEnabled(false);
    ui.pushButton_voice_nav->setEnabled(false);
    ui.pushButton_Unitchange->setEnabled(false);
    ui.pushButton_image_view->setEnabled(false);
    ui.pushButton_pick_check->setEnabled(false);
    ui.pushButton_time_start->setEnabled(false);
    ui.pushButton_build_world->setEnabled(false);
    ui.pushButton_getcurangle->setEnabled(false);
    ui.pushButton_place_check->setEnabled(false);
    ui.pushButton_pickandplace->setEnabled(false);
    ui.pushButton_update_score->setEnabled(false);
    ui.pushButton_getrobotstates->setEnabled(false);
    ui.pushButton_nav_map_turtle->setEnabled(false);
    ui.pushButton_savemap_turtle->setEnabled(false);
    ui.pushButton_buildmap_turtle->setEnabled(false);
    ui.pushButton_open_nav_config->setEnabled(false);
    ui.pushButton_loadworld_turtle->setEnabled(false);
    ui.pushButton_voice_navigation->setEnabled(false);
    ui.pushButton_hand_nav->setEnabled(false);
    ui.pushButton_voice_Ctrl->setEnabled(false);
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
    QSettings settings("Qt-Ros Package", "hg_gui");
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
    QSettings settings("Qt-Ros Package", "hg_gui");
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
    bash_killer = new QProcess;
    bash_killer->start("bash");
    bash_killer->write("kill $(ps -ef | grep roscore | grep -v grep | awk '{print $2}')\nkill $(ps -ef | grep rviz | grep -v grep | awk '{print $2}')\nkill $(ps -ef | grep demo_gazebo | grep -v grep | awk '{print $2}')\nkill $(ps -ef | grep gzserver | grep -v grep | awk '{print $2}')\nkill $(ps -ef | grep gzclient | grep -v grep | awk '{print $2}')\nkill $(ps -ef | grep gazebo_pick | grep -v grep | awk '{print $2}')\n");
    on_pushButton_close_process_clicked();
    bash_killer->waitForFinished(200);
    bash_cmd->close();
    bash_killer->close();
    bash_load_world->close();
    bash_remote->close();
}


void MainWindow::slot_pushbtn_click()
{
    char k = 0x00;
    QPushButton* btn = qobject_cast<QPushButton*> (sender());
//    qDebug() << btn->text();
    if(btn->text().toUtf8() == "前进")
    {
        k = 'w';
        //qDebug() <<k;
    }
    else if(btn->text().toUtf8() == "后退")
    {
        k = 'x';
        //qDebug() <<k;
    }
    else if(btn->text().toUtf8() == "左转")
    {
        k = 'a';
        //qDebug() <<k;
    }
    else if(btn->text().toUtf8() == "右转")
    {
        k = 'd';
        //qDebug() <<k;
    }
    else if(btn->text().toUtf8() == "停止")
    {
        k = 's';
        //qDebug() <<k;
    }
    else
    {
        k=btn->text().toStdString()[0];
        //qDebug() <<k;
    }
    switch(k)
    {
        case ('w'):
            {
                if(linear<linear_max)
                {
                    linear = linear + 0.03;
                }
                else
                {
                    linear = linear_max;
                }
                qnode.set_cmd_vel('w',linear,angular);break;
            }
        case ('a'):
            {
                if(angular<angular_max)
                {
                    angular = angular + 0.1;
                }
                else
                {
                    angular = angular_max;
                }
                angular = angular + 0.1;
                qnode.set_cmd_vel('a',linear,angular);break;
            }
        case ('d'):
            {
                if(angular>(-1*angular_max))
                {
                    angular = angular - 0.1;
                }
                else
                {
                    angular = -1*angular_max;
                }
                qnode.set_cmd_vel('d',linear,angular);break;
            }
        case ('x'):
            {
                if(linear>(-1*linear_max))
                {
                    linear = linear - 0.03;
                }
                else
                {
                    linear = -1*linear_max;
                }
                qnode.set_cmd_vel('x',linear,angular);break;
            }
        case ('s'):
            {
                linear = 0;
                angular = 0;
                qnode.set_cmd_vel('s',linear,angular);break;
            }
    }
}

void MainWindow::on_pushButton_loadworld_clicked()
{
    bash_load_world = new QProcess;
    QString load_world_cmd;
    // if(user_name == 0)
    // {
    //     load_world_cmd = "source /home/grantli/ahg_ws/devel/setup.bash\nroslaunch cobot_moveit_config demo_gazebo.launch\n";
    // }
    // else
    // {
    //     load_world_cmd = "source /root/hg_ws/devel/setup.bash\nroslaunch cobot_moveit_config demo_gazebo.launch\n";
    // }
    load_world_cmd = source_ws +"/devel/setup.bash\nroslaunch cobot_moveit_config demo_gazebo.launch\n";
    bash_load_world->start("bash");
    bash_load_world->write(load_world_cmd.toLocal8Bit());
    on_button_connect_clicked(true);
    ui.pushButton_loadworld->setEnabled(false);
}



void MainWindow::on_pushButton_remote_clicked()
{
    bash_remote = new QProcess;
    QString remote_cmd = "roslaunch motion_control Joint_look.launch\n";
    bash_remote->start("bash");
    bash_remote->write(remote_cmd.toLocal8Bit());
    on_button_connect_clicked(true);
    ui.pushButton_remote->setEnabled(false);
}

void MainWindow::on_pushButton_buildmap_clicked()
{
    bash_build_map = new QProcess;
    QString build_map_cmd;
    // if(user_name == 0)
    // {
    //     build_map_cmd = "source /home/grantli/ahg_ws/devel/setup.bash\nroslaunch turtlebot3_slam agv_slam.launch\n";
    // }
    // else
    // {
    //     build_map_cmd = "source /root/hg_ws/devel/setup.bash\nroslaunch turtlebot3_slam agv_slam.launch\n";
    // }
    build_map_cmd = source_ws + "/devel/setup.bash\nroslaunch turtlebot3_slam agv_slam.launch\n";
    bash_build_map->start("bash");
    bash_build_map->write(build_map_cmd.toLocal8Bit());
    ui.pushButton_buildmap->setEnabled(false);
}

void MainWindow::on_pushButton_nav_map_clicked()
{
    bash_nav_map = new QProcess;
    QString nav_map_cmd;
    // if(user_name == 0)
    // {
    //     nav_map_cmd = "source /home/grantli/ahg_ws/devel/setup.bash\nroslaunch turtlebot3_navigation agv_navigation.launch\n";
    // }
    // else
    // {
    //     nav_map_cmd = "source /root/hg_ws/devel/setup.bash\nroslaunch turtlebot3_navigation agv_navigation.launch\n";
    // }
    nav_map_cmd = source_ws + "/devel/setup.bash\nroslaunch turtlebot3_navigation agv_navigation.launch\n";
    bash_nav_map->start("bash");
    bash_nav_map->write(nav_map_cmd.toLocal8Bit());
    ui.pushButton_nav_map->setEnabled(false);
}

void MainWindow::on_pushButton_loadworld_turtle_clicked()
{
    bash_load_turtle_world = new QProcess;
    QString load_turtle_world_cmd;
    // if(user_name == 0)
    // {
    //     load_turtle_world_cmd = "source /home/grantli/ahg_ws/devel/setup.bash\nroslaunch all_world world_gazebo_turtlebot.launch\n";
    // }
    // else
    // {
    //     load_turtle_world_cmd = "source /root/hg_ws/devel/setup.bash\nroslaunch all_world world_gazebo_turtlebot.launch\n";
    // }
    load_turtle_world_cmd = source_ws + "/devel/setup.bash\nroslaunch all_world world_gazebo_turtlebot.launch\n";
    bash_load_turtle_world->start("bash");
    bash_load_turtle_world->write(load_turtle_world_cmd.toLocal8Bit());
    ui.pushButton_loadworld_turtle->setEnabled(false);
}

void MainWindow::on_pushButton_buildmap_turtle_clicked()
{
    bash_build_turtle_map = new QProcess;
    QString build_turtle_map_cmd;
    // if(user_name == 0)
    // {
    //     build_turtle_map_cmd = "source /home/grantli/ahg_ws/devel/setup.bash\nroslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping\n";
    // }
    // else
    // {
    //     build_turtle_map_cmd = "source /root/hg_ws/devel/setup.bash\nroslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping\n";
    // }
    build_turtle_map_cmd = source_ws + "/devel/setup.bash\nroslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping\n";
    bash_build_turtle_map->start("bash");
    bash_build_turtle_map->write(build_turtle_map_cmd.toLocal8Bit());
    ui.pushButton_buildmap_turtle->setEnabled(false);
}

void MainWindow::on_pushButton_nav_map_turtle_clicked()
{
    bash_nav_turtle_map = new QProcess;
    QString nav_turtle_map_cmd;
    // if(user_name == 0)
    // {
    //     nav_turtle_map_cmd = "source /home/grantli/ahg_ws/devel/setup.bash\nroslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/station.yaml\n";
    // }
    // else
    // {
    //     nav_turtle_map_cmd = "source /root/hg_ws/devel/setup.bash\nroslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/station.yaml\n";
    // }
    nav_turtle_map_cmd = source_ws + "/devel/setup.bash\nroslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/station.yaml\n";
    bash_nav_turtle_map->start("bash");
    bash_nav_turtle_map->write(nav_turtle_map_cmd.toLocal8Bit());
    ui.pushButton_nav_map_turtle->setEnabled(false);
}

void MainWindow::on_pushButton_getcurangle_clicked()
{
    cobot_joint_position[0] = qnode.cobot_position[0];
    cobot_joint_position[1] = qnode.cobot_position[1];
    cobot_joint_position[2] = qnode.cobot_position[2];
    cobot_joint_position[3] = qnode.cobot_position[3];
    cobot_joint_position[4] = qnode.cobot_position[4];
    cobot_joint_position[5] = qnode.cobot_position[5];
    cobot_joint_position[6] = qnode.cobot_position[6];
    ui.lineEdit_cobot_joint1->setText(QString::number(qnode.cobot_position[0],'f',2));
    ui.lineEdit_cobot_joint2->setText(QString::number(qnode.cobot_position[1],'f',2));
    ui.lineEdit_cobot_joint3->setText(QString::number(qnode.cobot_position[2],'f',2));
    ui.lineEdit_cobot_joint4->setText(QString::number(qnode.cobot_position[3],'f',2));
    ui.lineEdit_cobot_joint5->setText(QString::number(qnode.cobot_position[4],'f',2));
    ui.lineEdit_cobot_joint6->setText(QString::number(qnode.cobot_position[5],'f',2));
    ui.lineEdit_cobot_joint7->setText(QString::number(qnode.cobot_position[6],'f',2));
}

void MainWindow::on_pushButton_Unitchange_clicked()
{
    if(unit_change_flag == 0)
    {
        if(language_flag == 0)
        {
            ui.pushButton_Unitchange->setText(QString::fromUtf8("弧度"));
        }
        else
        {
            ui.pushButton_Unitchange->setText("To_Rad");
        }
        ui.lineEdit_cobot_joint1->setText(QString::number(qnode.cobot_position[0]*ToDeg,'f',2));
        ui.lineEdit_cobot_joint2->setText(QString::number(qnode.cobot_position[1]*ToDeg,'f',2));
        ui.lineEdit_cobot_joint3->setText(QString::number(qnode.cobot_position[2]*ToDeg,'f',2));
        ui.lineEdit_cobot_joint4->setText(QString::number(qnode.cobot_position[3]*ToDeg,'f',2));
        ui.lineEdit_cobot_joint5->setText(QString::number(qnode.cobot_position[4]*ToDeg,'f',2));
        ui.lineEdit_cobot_joint6->setText(QString::number(qnode.cobot_position[5]*ToDeg,'f',2));
        ui.lineEdit_cobot_joint7->setText(QString::number(qnode.cobot_position[6]*ToDeg,'f',2));
        unit_change_flag = 1;
    }
    else
    {
        if(language_flag == 1)
        {
            ui.pushButton_Unitchange->setText(QString::fromUtf8("角度"));
        }
        else
        {
            ui.pushButton_Unitchange->setText("To_Deg");
        }
        ui.lineEdit_cobot_joint1->setText(QString::number(qnode.cobot_position[0],'f',2));
        ui.lineEdit_cobot_joint2->setText(QString::number(qnode.cobot_position[1],'f',2));
        ui.lineEdit_cobot_joint3->setText(QString::number(qnode.cobot_position[2],'f',2));
        ui.lineEdit_cobot_joint4->setText(QString::number(qnode.cobot_position[3],'f',2));
        ui.lineEdit_cobot_joint5->setText(QString::number(qnode.cobot_position[4],'f',2));
        ui.lineEdit_cobot_joint6->setText(QString::number(qnode.cobot_position[5],'f',2));
        ui.lineEdit_cobot_joint7->setText(QString::number(qnode.cobot_position[6],'f',2));
        unit_change_flag = 0;
    }
}

void MainWindow::on_pushButton_language_clicked()
{
    if(language_flag == 0)
    {
        ui.pushButton_wspeed->setText(QString::fromUtf8("前进"));
        ui.pushButton_wspeed->setShortcut(QKeySequence(tr("W")));
        ui.pushButton_sspeed->setText(QString::fromUtf8("停止"));
        ui.pushButton_sspeed->setShortcut(QKeySequence(tr("S")));
        ui.pushButton_aspeed->setText(QString::fromUtf8("左转"));
        ui.pushButton_aspeed->setShortcut(QKeySequence(tr("A")));
        ui.pushButton_dspeed->setText(QString::fromUtf8("右转"));
        ui.pushButton_dspeed->setShortcut(QKeySequence(tr("D")));
        ui.pushButton_xspeed->setText(QString::fromUtf8("后退"));
        ui.pushButton_xspeed->setShortcut(QKeySequence(tr("X")));
        ui.pushButton_loadworld->setText(QString::fromUtf8("加载世界"));
        ui.pushButton_loadworld_turtle->setText(QString::fromUtf8("加载世界"));
        ui.pushButton_buildmap->setText(QString::fromUtf8("启动建图"));
        ui.pushButton_buildmap_turtle->setText(QString::fromUtf8("启动建图"));
        ui.pushButton_nav_map->setText(QString::fromUtf8("启动导航"));
        ui.pushButton_nav_map_turtle->setText(QString::fromUtf8("启动导航"));
        ui.pushButton_savemap->setText(QString::fromUtf8("保存地图"));
        ui.pushButton_savemap_turtle->setText(QString::fromUtf8("保存地图"));
        ui.label_map_path->setText(QString::fromUtf8("地图路径:"));
        ui.label_5->setText(QString::fromUtf8("       任务编号:"));
        ui.pushButton_build_world->setText(QString::fromUtf8("环境搭建"));
        ui.pushButton_open_nav_config->setText(QString::fromUtf8("导航点设置"));
        ui.pushButton_voice_navigation->setText(QString::fromUtf8("导航仿真测试"));
        ui.pushButton_hand_nav->setText(QString::fromUtf8("手动导航"));
        ui.pushButton_voice_nav->setText(QString::fromUtf8("语音导航"));
        ui.pushButton_remote->setText(QString::fromUtf8("观察姿态"));
        ui.pushButton_image_view->setText(QString::fromUtf8("相机"));
        ui.pushButton_pick_check->setText(QString::fromUtf8("抓取测试"));
        ui.pushButton_place_check->setText(QString::fromUtf8("放置测试"));
        ui.pushButton_pickandplace->setText(QString::fromUtf8("综合抓取任务"));
        ui.pushButton_voice_Ctrl->setText(QString::fromUtf8("语音交互"));
        ui.pushButton_getrobotstates->setText(QString::fromUtf8("获取机器人状态"));
        ui.pushButton_close_process->setText(QString::fromUtf8("关闭"));
        ui.pushButton_update_score->setText(QString::fromUtf8("提交评测"));
        ui.pushButton_getcurangle->setText(QString::fromUtf8("获取关节角"));
        ui.pushButton_Unitchange->setText(QString::fromUtf8("角度"));
        ui.label_cobot_joint1->setText(QString::fromUtf8("七轴关节角1:"));
        ui.label_cobot_joint2->setText(QString::fromUtf8("七轴关节角2:"));
        ui.label_cobot_joint3->setText(QString::fromUtf8("七轴关节角3:"));
        ui.label_cobot_joint4->setText(QString::fromUtf8("七轴关节角4:"));
        ui.label_cobot_joint5->setText(QString::fromUtf8("七轴关节角5:"));
        ui.label_cobot_joint6->setText(QString::fromUtf8("七轴关节角6:"));
        ui.label_cobot_joint7->setText(QString::fromUtf8("七轴关节角7:"));
        language_flag = 1;
    }
    else
    {
        ui.pushButton_wspeed->setText("w");
        ui.pushButton_sspeed->setText("s");
        ui.pushButton_aspeed->setText("a");
        ui.pushButton_dspeed->setText("d");
        ui.pushButton_xspeed->setText("x");
        ui.pushButton_wspeed->setShortcut(QKeySequence(tr("W")));
        ui.pushButton_sspeed->setShortcut(QKeySequence(tr("S")));
        ui.pushButton_aspeed->setShortcut(QKeySequence(tr("A")));
        ui.pushButton_dspeed->setShortcut(QKeySequence(tr("D")));
        ui.pushButton_xspeed->setShortcut(QKeySequence(tr("X")));
        ui.pushButton_loadworld->setText(QString::fromUtf8("Load_world"));
        ui.pushButton_loadworld_turtle->setText(QString::fromUtf8("Load_world"));
        ui.pushButton_buildmap->setText(QString::fromUtf8("Build_map"));
        ui.pushButton_buildmap_turtle->setText(QString::fromUtf8("Build_map"));
        ui.pushButton_nav_map->setText(QString::fromUtf8("Nav_map"));
        ui.pushButton_nav_map_turtle->setText(QString::fromUtf8("Nav_map"));
        ui.pushButton_savemap->setText(QString::fromUtf8("Save_map"));
        ui.pushButton_savemap_turtle->setText(QString::fromUtf8("Save_map"));
        ui.label_map_path->setText(QString::fromUtf8("Map_Path: "));
        ui.label_5->setText(QString::fromUtf8("Task_Number:"));
        ui.pushButton_build_world->setText(QString::fromUtf8("Build_world"));
        ui.pushButton_open_nav_config->setText(QString::fromUtf8("Nav_Goal"));
        ui.pushButton_voice_navigation->setText(QString::fromUtf8("Nav_world"));
        ui.pushButton_hand_nav->setText(QString::fromUtf8("Hand_Nav"));
        ui.pushButton_voice_nav->setText(QString::fromUtf8("Voice_Nav"));
        ui.pushButton_remote->setText(QString::fromUtf8("Joint_Look"));
        ui.pushButton_image_view->setText(QString::fromUtf8("Camera"));
        ui.pushButton_pick_check->setText(QString::fromUtf8("Pick_Cheak"));
        ui.pushButton_place_check->setText(QString::fromUtf8("Place_Cheak"));
        ui.pushButton_pickandplace->setText(QString::fromUtf8("Pick_Place"));
        ui.pushButton_voice_Ctrl->setText(QString::fromUtf8("Voice_Ctrl"));
        ui.pushButton_getrobotstates->setText(QString::fromUtf8("Get_States"));
        ui.pushButton_close_process->setText(QString::fromUtf8("Close"));
        ui.pushButton_update_score->setText(QString::fromUtf8("Update_Score"));
        ui.pushButton_getcurangle->setText(QString::fromUtf8("Get_Angle"));
        ui.pushButton_Unitchange->setText(QString::fromUtf8("To_Deg"));
        ui.label_cobot_joint1->setText("Cobot_Joint1:");
        ui.label_cobot_joint2->setText("Cobot_Joint2:");
        ui.label_cobot_joint3->setText("Cobot_Joint3:");
        ui.label_cobot_joint4->setText("Cobot_Joint4:");
        ui.label_cobot_joint5->setText("Cobot_Joint5:");
        ui.label_cobot_joint6->setText("Cobot_Joint6:");
        ui.label_cobot_joint7->setText("Cobot_Joint7:");
        language_flag = 0;
    }


}

void MainWindow::Init_timer()
{
    ui.lcd_display->display("00:00:00:000");
    pTimer = new QTimer;
    connect(pTimer,SIGNAL(timeout()),this,SLOT(updateDisplay()));
    //connect(pTimer,&QTimer::timeout,this,&MainWindow::updateDisplay);
}

void MainWindow::Delay_func(int seconds)
{
//    int delayTime = 5;
    QElapsedTimer timer;
    timer.start();
    while(timer.elapsed() < (seconds * 1000))
    {
    }
}

void MainWindow::Scara_Widgets_element_Init()
{
//    qDebug() << QCoreApplication::applicationDirPath();
    this->move(400,200);
    ui.plainTextEdit_script->setHidden(true);
    ui.plainTextEdit_script->resize(661,400);
    ui.tabWidget_first_cell->setCurrentWidget(ui.tab_scara_robot); // 初始化Tab页面为 Robot
    ui.tabWidget_scara_cmd->setCurrentWidget(ui.tab_scara_movej);  // 初始化 Tab 页面为 scara moveJ
    ui.plainTextEdit_user_graphic_cmd->move(400,550);
    ui.plainTextEdit_user_graphic_cmd->resize(600,131);
    ui.widget_graphic_cmd->setFixedSize(661,2000);
    QScrollArea *m_pScroll = new QScrollArea(ui.tab_scripts);
    m_pScroll->move(400,20);
    m_pScroll->resize(600,500);
    m_pScroll->setWidget(ui.widget_graphic_cmd);//给widget_2设置滚动条
}

bool MainWindow::openTextByStream(const QString &aFileName)
{
    //用 QTextStream打开文本文件
    QFile   aFile(aFileName);
    if (!aFile.exists()) //文件不存在
        return false;
    if (!aFile.open(QIODevice::ReadOnly | QIODevice::Text))
        return false;
    QTextStream aStream(&aFile); //用文本流读取文件
    //    aStream.setAutoDetectUnicode(true); //自动检测Unicode,才能正常显示文档内的汉字
    linenumber = 0;
    while(!aStream.atEnd())
    {
        aStream.readLine();
        linenumber++;
    }
//    qDebug() << linenumber;
//    qDebug() << ui->plainTextEdit_script->textCursor().position();
    aStream.seek(0);
    ui.plainTextEdit_script->setPlainText(aStream.readAll());
    aFile.close();//关闭文件
    ui.tabWidget_scara_cmd->setCurrentWidget(ui.tab_scara_movej);
    return  true;
}

bool MainWindow::openTextConfigByStrem()
{
    //用 QTextStream打开文本文件
//    QFile aFile(QCoreApplication::applicationDirPath() + "/" + open_file_name + ".txt");
    QFile aFile(open_file_path + "/" + open_file_name + ".txt");
//    qDebug() << QCoreApplication::applicationDirPath() + "/" + open_file_name + ".txt";
    if (!aFile.exists()) //文件不存在
        return false;
    if (!aFile.open(QIODevice::ReadOnly | QIODevice::Text))
        return false;
    QTextStream aStream(&aFile); //用文本流读取文件
    int count = 0;
    int count1 = 0;
    int count2 = 0;
    user_graphic_cmd_index = 0;
    //    aStream.setAutoDetectUnicode(true); //自动检测Unicode,才能正常显示文档内的汉字
    while(!aStream.atEnd())
    {
        if(count%2==0)
        {
            user_graphic_cmd_labelname[count1] = aStream.readLine();
            count1++;
        }
        else{
            user_graphic_cmd_str[count2] = aStream.readLine();
            count2++;
            user_graphic_cmd_index++;
        }
        count++;
    }
    aFile.close();//关闭文件
    return  true;
}

bool MainWindow::saveTextByStream(const QString &aFileName)
{
    //用QTextStream保存文本文件
    QFile   aFile(aFileName);
    if (!aFile.open(QIODevice::WriteOnly | QIODevice::Text))
        return false;
    QTextStream aStream(&aFile); //用文本流读取文件
    //    aStream.setAutoDetectUnicode(true); //自动检测Unicode,才能正常显示文档内的汉字
    QString str=ui.plainTextEdit_script->toPlainText(); //转换为字符串
    aStream<<str; //写入文本流
    aFile.close();//关闭文件
    return  true;
}

void MainWindow::clicklabel_savefile_Init()
{
    for(int i=0;i<user_graphic_cmd_index;i++)
    {
        ClickLabel *cmd_user = new ClickLabel(ui.widget_graphic_cmd);
        connect(cmd_user,&ClickLabel::clicked,this,&MainWindow::deal_label_enter);
        connect(cmd_user,&ClickLabel::del,this,&MainWindow::deal_label_del);
        user_graphic_cmd[i] = cmd_user;
        cmd_user->setFrameShape(QFrame::Panel);
        cmd_user->move(20,i*30);
        script_cursor_Init();
        ui.plainTextEdit_script->insertPlainText(user_graphic_cmd_str[i]);
        cmd_user->setText(user_graphic_cmd_labelname[i]);
        cmd_user->show();
    }
}

void MainWindow::deal_label_enter(int index)
{
    int str_index = 0;
    QObject *clicklabel = QObject::sender();
    for(int i=0;i<50;i++)
    {
        if(clicklabel == user_graphic_cmd[i])
            str_index = int(user_graphic_cmd[i]->pos().y()/30);
    }
    ui.plainTextEdit_user_graphic_cmd->setPlainText(user_graphic_cmd_str[str_index]);
}

void MainWindow::deal_label_del()
{
    int str_index = 0;
    QObject *clicklabel = QObject::sender();
    for(int i=0;i<50;i++)
    {
        if(clicklabel == user_graphic_cmd[i])
        {
            user_graphic_cmd[i]->close();
            str_index = i;
        }
//            user_graphic_cmd[i]->hide();
    }
    array_del(user_graphic_cmd,str_index);
    clicklabel_cmd_renew();
}

void MainWindow::array_del(ClickLabel *array[], int index)
{
    for(int i=0;i<50;i++)
    {
        if((i >= index) && (i<user_graphic_cmd_index))
        {
            array[i] = array[i+1];
        }
    }
    user_graphic_cmd_index--;
}

void MainWindow::clicklabel_cmd_renew()
{
    for(int i=0;i<user_graphic_cmd_index;i++)
    {
        user_graphic_cmd[i]->move(20,30*i);
    }
}

void MainWindow::script_cursor_Init()
{
    if(script_cursor_init_flag == 0)
    {
        QTextCursor textCursor=ui.plainTextEdit_script->textCursor();
        textCursor.setPosition(13016,QTextCursor::MoveMode::MoveAnchor);
        ui.plainTextEdit_script->setTextCursor(textCursor);
        script_cursor_init_flag++;
    }
}

bool MainWindow::saveTextConfigByStrem()
{
    //"/home/pickle/Code/Qt/build-script_assistant-Desktop_Qt_5_9_9_GCC_64bit-Debug"
//    QFile aFile(QCoreApplication::applicationDirPath() + "/" + save_file_name + ".txt");
    QFile aFile(save_file_path + "/" + save_file_name + ".txt");
    qDebug() << save_file_path + "/" + save_file_name + ".txt";
    if (!aFile.open(QIODevice::WriteOnly | QIODevice::Text))
        return false;
    QTextStream aStream(&aFile); //用文本流读取文件
    QString str_cmd = "";
    for(int i=0;i<user_graphic_cmd_index;i++)
    {
        str_cmd = str_cmd + user_graphic_cmd[i]->text() + "\n" + user_graphic_cmd_str[i];
    }
    aStream<<str_cmd; //写入文本流
    aFile.close();//关闭文件
    return  true;
}

void MainWindow::comboBox_state_switch(int rindex, int mindex)
{
    switch (mindex) {
    case(0):
        if(rindex == 0)
            int x = 0;
//            ui->tabWidget->setCurrentWidget(ui->tab_cjoint);
        else
        {
//            ui->tabWidget->setCurrentWidget(ui->tab_sjoint);
//            scara_joint_arm_enable();
        }
        break;
    case(1):
//        ui->tabWidget->setCurrentWidget(ui->tab_pose);
        break;
    case(2):
//        ui->tabWidget->setCurrentWidget(ui->tab_cartesian);
        break;
    default:break;
    }
}

void MainWindow::clicklabel_cmd_rebuild(int array[])
{
    for(int i=0;i<user_graphic_cmd_index;i++)
    {
        user_graphic_cmd[i]->move(20,array[i]*30);
    }
}

void MainWindow::user_graph_cmd_str_rebuild(int array[])
{
    ClickLabel *user_graphic_cmd_temp[user_graphic_cmd_index];
    QString user_graphic_cmd_str_temp[user_graphic_cmd_index];
    for(int i=0;i<user_graphic_cmd_index;i++)
    {
        user_graphic_cmd_str_temp[i] = user_graphic_cmd_str[i];
        user_graphic_cmd_temp[i] = user_graphic_cmd[i];
    }
    for(int i=0;i<user_graphic_cmd_index;i++)
    {
        user_graphic_cmd_str[array[i]] = user_graphic_cmd_str_temp[i];
        user_graphic_cmd[array[i]] = user_graphic_cmd_temp[i];
    }
}

void MainWindow::Robot_Scara_MoveP_LineEdit_Enable(bool enable)
{
    bool en = false;
    if(enable)
    {
        en = true;
    }
    else
    {
        en = false;
    }
    ui.lineEdit_scara_movep_posx_value->setEnabled(en);
    ui.lineEdit_scara_movep_posy_value->setEnabled(en);
    ui.lineEdit_scara_movep_posz_value->setEnabled(en);
    ui.lineEdit_scara_movep_orienx_value->setEnabled(en);
    ui.lineEdit_scara_movep_orieny_value->setEnabled(en);
    ui.lineEdit_scara_movep_orienz_value->setEnabled(en);
    ui.lineEdit_scara_movep_orienw_value->setEnabled(en);
}

void MainWindow::updateDisplay()
{
    QTime currTime = QTime::currentTime();
    int t = baseTime.msecsTo(currTime);
    QTime showTime(0,0,0,0);
    showTime = showTime.addMSecs(t);
    timeStr = showTime.toString("hh:mm:ss:zzz");
    ui.lcd_display->display(timeStr);
}

void MainWindow::on_pushButton_time_start_clicked()
{
    baseTime = baseTime.currentTime();
    pTimer->start(1);
    //重置状态
    if(ui.pushButton_time_stop->text() != "Stop")
    {
        ui.pushButton_time_stop->setText("Stop");
    }
    ui.pushButton_time_start->setEnabled(false);
}

void MainWindow::on_pushButton_time_stop_clicked()
{
    if(ui.pushButton_time_stop->text() == "Stop")
    {
        ui.pushButton_time_stop->setText("Clear");
        pTimer->stop();
    }
    else if(ui.pushButton_time_stop->text() == "Clear")
    {
        ui.pushButton_time_stop->setText("Stop");
//        ui->tb_display->clear();
        ui.lcd_display->display("00:00:00:000");
        ui.pushButton_time_start->setEnabled(true);
    }
}

void MainWindow::on_pushButton_close_process_clicked()
{
    bash_killer = new QProcess;
    bash_killer->start("bash");
    //bash_killer->write("kill $(ps -ef | grep gazebo_spawn | grep -v grep | awk '{print $2}')\nps -ef | grep gazebo_pick | grep -v grep | awk '{print $2}')\nkill $(ps -ef | grep roscore | grep -v grep | awk '{print $2}')\nkill $(ps -ef | grep rviz | grep -v grep | awk '{print $2}')\nkill $(ps -ef | grep demo_gazebo | grep -v grep | awk '{print $2}')\nkill $(ps -ef | grep gzserver | grep -v grep | awk '{print $2}')\nkill $(ps -ef | grep gzclient | grep -v grep | awk '{print $2}')\n");
	bash_killer->write("kill $(ps -ef | grep gazebo_spawn | grep -v grep | awk '{print $2}')\n");
    bash_killer->write("kill $(ps -ef | grep demo_gazebo | grep -v grep | awk '{print $2}')\n");
    bash_killer->write("kill $(ps -ef | grep agv_slam | grep -v grep | awk '{print $2}')\n");
    bash_killer->waitForFinished(200);
    bash_killer->write("kill $(ps -ef | grep pick_check | grep -v grep | awk '{print $2}')\n");
    bash_killer->write("kill $(ps -ef | grep place_check | grep -v grep | awk '{print $2}')\n");
    bash_killer->write("kill $(ps -ef | grep move_nav_hand | grep -v grep | awk '{print $2}')\n");
    bash_killer->write("kill $(ps -ef | grep Joint_look | grep -v grep | awk '{print $2}')\n");
    bash_killer->waitForFinished(200);
    bash_killer->write("kill $(ps -ef | grep agv_navigation | grep -v grep | awk '{print $2}')\n");
	bash_killer->write("kill $(ps -ef | grep gazebo_pick | grep -v grep | awk '{print $2}')\n");
    bash_killer->write("kill $(ps -ef | grep voice_nav | grep -v grep | awk '{print $2}')\n");
    bash_killer->write("kill $(ps -ef | grep voice_cmd | grep -v grep | awk '{print $2}')\n");
    bash_killer->write("kill $(ps -ef | grep gazebo_nav | grep -v grep | awk '{print $2}')\n");
    bash_killer->write("kill $(ps -ef | grep rqt_image_view | grep -v grep | awk '{print $2}')\n");
    bash_killer->waitForFinished(200);
    bash_killer->write("killall gzserver\n");
    bash_killer->write("killall gzclient\n");
	bash_killer->write("kill $(ps -ef | grep roscore | grep -v grep | awk '{print $2}')\n");
	bash_killer->write("kill $(ps -ef | grep rviz | grep -v grep | awk '{print $2}')\n");
    bash_killer->waitForFinished(200);
    bash_cmd->close();
    bash_killer->close();
    bash_load_world->close();
    bash_remote->close();
}

void MainWindow::on_pushButton_savemap_turtle_clicked()
{
    bash_save_turtle_map = new QProcess;
    QString map_path = ui.lineEdit_map_path->text();
    QString sav_turtle_map_cmd = "rosrun map_server map_saver -f ~";
    sav_turtle_map_cmd = sav_turtle_map_cmd + map_path + "\n";
    bash_save_turtle_map->start("bash");
    bash_save_turtle_map->write(sav_turtle_map_cmd.toLocal8Bit());
}

void MainWindow::on_pushButton_savemap_clicked()
{
    bash_save_map = new QProcess;
    QString map_path = ui.lineEdit_map_path->text();
    QString sav_map_cmd = "rosrun map_server map_saver -f ~";
    sav_map_cmd = sav_map_cmd + map_path + "\n";
//    qDebug() << sav_map_cmd;
    bash_save_map->start("bash");
    bash_save_map->write(sav_map_cmd.toLocal8Bit());
}

void MainWindow::on_pushButton_open_nav_config_clicked()
{
    bash_nav_config = new QProcess;
    QString nav_config_cmd;
    // if(user_name == 0)
    // {
    //     nav_config_cmd = "gedit /home/grantli/ahg_ws/src/workstation/turtlebot3/turtlebot3_navigation/config/goal_nav.yaml\n";
    // }
    // else
    // {
    //     nav_config_cmd = "gedit /root/hg_ws/src/workstation/turtlebot3/turtlebot3_navigation/config/goal_nav.yaml\n";
    // }
    string ws_path = workspace_path;
	const char * ws_name = ws_path.data(); // 工作空间路径

    nav_config_cmd = "gedit " + user_name_str + ws_name + "/src/workstation/turtlebot3/turtlebot3_navigation/config/goal_nav.yaml\n";
    bash_nav_config->start("bash");
    bash_nav_config->write(nav_config_cmd.toLocal8Bit());
}

void MainWindow::on_pushButton_getrobotstates_clicked()
{
    ui.lineEdit_robotstate_pose_x->setText(QString::number(qnode.robot_states[0],'f',5));
    ui.lineEdit_robotstate_pose_y->setText(QString::number(qnode.robot_states[1],'f',5));
    ui.lineEdit_robotstate_pose_z->setText(QString::number(qnode.robot_states[2],'f',5));
    ui.lineEdit_robotstate_orient_x->setText(QString::number(qnode.robot_states[3],'f',5));
    ui.lineEdit_robotstate_orient_y->setText(QString::number(qnode.robot_states[4],'f',5));
    ui.lineEdit_robotstate_orient_z->setText(QString::number(qnode.robot_states[5],'f',5));
    ui.lineEdit_robotstate_orient_w->setText(QString::number(qnode.robot_states[6],'f',5));
}

void MainWindow::on_pushButton_build_world_clicked()
{
    bash_build_world = new QProcess;
    QString build_world_cmd;
    // if(user_name == 0)
    // {
    //     build_world_cmd = "source /home/grantli/ahg_ws/devel/setup.bash\nroslaunch all_world gazebo_spawn.launch\n";
    // }
    // else
    // {
    //     build_world_cmd = "source /root/hg_ws/devel/setup.bash\nroslaunch all_world gazebo_spawn.launch\n";
    // }
    build_world_cmd = source_ws + "/devel/setup.bash\nroslaunch all_world gazebo_spawn.launch\n";
    bash_build_world->start("bash");
    bash_build_world->write(build_world_cmd.toLocal8Bit());
    ui.pushButton_build_world->setEnabled(false);
    score[1] = 3;
}

void MainWindow::on_pushButton_pickandplace_clicked()
{
    bash_pick_place = new QProcess;
    QString pick_place_cmd;
    // if(user_name == 0)
    // {
    //     pick_place_cmd = "source /home/grantli/ahg_ws/devel/setup.bash\nroslaunch all_world gazebo_pick.launch\n";
    // }
    // else
    // {
    //     pick_place_cmd = "source /root/hg_ws/devel/setup.bash\nroslaunch all_world gazebo_pick.launch\n";
    // }
    pick_place_cmd = source_ws + "/devel/setup.bash\nroslaunch all_world gazebo_pick.launch\n";
    bash_pick_place->start("bash");
    bash_pick_place->write(pick_place_cmd.toLocal8Bit());
    ui.pushButton_pickandplace->setEnabled(false);
}

void MainWindow::on_pushButton_image_view_clicked()
{
    bash_image_view = new QProcess;
    QString image_view_cmd = "rqt_image_view\n";
    bash_image_view->start("bash");
    bash_image_view->write(image_view_cmd.toLocal8Bit());
    ui.pushButton_image_view->setEnabled(false);
}


void MainWindow::on_pushButton_update_score_clicked()
{
//    ui.pushButton_sspeed->click();
    sendmsg();
}

void MainWindow::on_pushButton_pick_check_clicked()
{
    bash_pick_check = new QProcess;
    QString pick_check_cmd;
    // if(user_name == 0)
    // {
    //     pick_check_cmd = "source /home/grantli/ahg_ws/devel/setup.bash\nroslaunch motion_control pick_check.launch\n";
    // }
    // else
    // {
    //     pick_check_cmd = "source /root/hg_ws/devel/setup.bash\nroslaunch motion_control pick_check.launch\n";
    // }
    pick_check_cmd = source_ws + "/devel/setup.bash\nroslaunch motion_control pick_check.launch\n";
    bash_pick_check->start("bash");
    bash_pick_check->write(pick_check_cmd.toLocal8Bit());
    ui.pushButton_pick_check->setEnabled(false);
}

void MainWindow::on_pushButton_place_check_clicked()
{
    bash_place_check = new QProcess;
    QString place_check_cmd;
    // if(user_name == 0)
    // {
    //     place_check_cmd = "source /home/grantli/ahg_ws/devel/setup.bash\nroslaunch motion_control place_check.launch\n";
    // }
    // else
    // {
    //     place_check_cmd = "source /root/hg_ws/devel/setup.bash\nroslaunch motion_control place_check.launch\n";
    // }
    place_check_cmd = source_ws + "/devel/setup.bash\nroslaunch motion_control place_check.launch\n";
    bash_place_check->start("bash");
    bash_place_check->write(place_check_cmd.toLocal8Bit());
    ui.pushButton_place_check->setEnabled(false);
}

void MainWindow::on_pushButton_voice_nav_clicked()
{
    system("gnome-terminal -x bash -c 'roslaunch castlex_voice_system voice_nav.launch' &");
    ui.pushButton_voice_nav->setEnabled(false);
}

void MainWindow::on_comboBox_currentIndexChanged(int index)
{
    Task_number = index;
    score[0] = index;
    if(index == 0)
    {
        btn_disable();
    }
    else
    {
        btn_enable();
    }

    //int index = ui.comboBox->currentIndex();
}

void MainWindow::on_pushButton_voice_navigation_clicked()
{
    bash_nav_world = new QProcess;
    QString nav_world_cmd;
    // if(user_name == 0)
    // {
    //     nav_world_cmd = "source /home/grantli/ahg_ws/devel/setup.bash\nroslaunch all_world gazebo_nav.launch\n";
    // }
    // else
    // {
    //     nav_world_cmd = "source /root/hg_ws/devel/setup.bash\nroslaunch all_world gazebo_nav.launch\n";
    // }
    nav_world_cmd = source_ws + "/devel/setup.bash\nroslaunch all_world gazebo_nav.launch\n";
    bash_nav_world->start("bash");
    bash_nav_world->write(nav_world_cmd.toLocal8Bit());
    ui.pushButton_voice_navigation->setEnabled(false);
}

void MainWindow::on_pushButton_hand_nav_clicked()
{
    system("gnome-terminal -x bash -c 'roslaunch turtlebot3_navigation move_nav_hand.launch' &");
    ui.pushButton_hand_nav->setEnabled(false);
}

void MainWindow::voice_forward()
{
    ui.pushButton_wspeed->click();
}

void MainWindow::voice_back_off()
{
    ui.pushButton_xspeed->click();
}

void MainWindow::voice_turn_left()
{
    ui.pushButton_aspeed->click();
}

void MainWindow::voice_turn_right()
{
    ui.pushButton_dspeed->click();
}

void MainWindow::voice_stop()
{
    ui.pushButton_sspeed->click();
}

void MainWindow::on_pushButton_voice_Ctrl_clicked()
{
    system("gnome-terminal -x bash -c 'roslaunch castlex_voice_system voice_cmd.launch' &");
    ui.pushButton_voice_Ctrl->setEnabled(false);
}


}  // namespace hg_gui





void hg_gui::MainWindow::on_actionOpen_File_triggered()
{
    script_cursor_init_flag = 0;
    //打开文件
    QString curPath=QDir::currentPath();//获取系统当前目录
    //调用打开文件对话框打开一个文件
    QString aFileName=QFileDialog::getOpenFileName(this,"打开一个文件",curPath,
                                                   "程序文件(*.h *cpp *py);;文本文件(*.txt);;所有文件(*.*)");
    open_file_path = aFileName.left(aFileName.lastIndexOf("/"));
    open_file_name = aFileName.right(aFileName.size()-aFileName.lastIndexOf("/")-1);
    open_file_name = open_file_name.left(open_file_name.indexOf("."));
//    qDebug() << "file_name: "<< open_file_name;
    if (aFileName.isEmpty())
        return; //如果未选择文件，退出
    openTextByStream(aFileName); //打开文件
    openTextConfigByStrem();
    clicklabel_savefile_Init();
}

void hg_gui::MainWindow::on_actionSave_FIle_triggered()
{
    QString curPath=QDir::currentPath();//获取系统当前目录
    QString dlgTitle="另存为一个文件"; //对话框标题
//    QString filter="h文件(*.h);;c++文件(*.cpp);;py文件(*.py);;文本文件(*.txt);;所有文件(*.*)"; //文件过滤器
    QString filter="程序文件(*.h *cpp *py);;文本文件(*.txt);;所有文件(*.*)"; //文件过滤器
    QString aFileName=QFileDialog::getSaveFileName(this,dlgTitle,curPath,filter);
    save_file_path = aFileName.left(aFileName.lastIndexOf("/"));
//    qDebug() << "aFileName: " << aFileName;
//    qDebug()<<"last  '.' pos: "<<aFileName.lastIndexOf(".");  //查找最后一个'.'
//    qDebug()<<"last  '.' pos: "<<aFileName.lastIndexOf("/");  //查找最后一个'.'
    save_file_name = aFileName.right(aFileName.size()-aFileName.lastIndexOf("/")-1);
    save_file_name = save_file_name.left(save_file_name.indexOf("."));
//    qDebug() << "file_name: "<< save_file_name;
    if (aFileName.isEmpty())
        return;
    saveTextByStream(aFileName);
    saveTextConfigByStrem();
}

void hg_gui::MainWindow::on_comboBox_robot_ns_currentIndexChanged(int index)
{
    // robot 型号选择下拉框变化触发处理槽函数
    // 输入参数为变更后的序号
    robot_index = index;
    comboBox_state_switch(robot_index,move_cmd_index); // 根据全局变量 robot_index move_cmd_index来更新tab面板
}

void hg_gui::MainWindow::on_comboBox_movecmd_currentIndexChanged(int index)
{
    // move_cmd_index 运动命令选择下拉框变化触发处理槽函数
    // 输入参数为变更后的序号
    move_cmd_index = index;
    comboBox_state_switch(robot_index,move_cmd_index);  // 根据全局变量 robot_index move_cmd_index来更新tab面板
}

void hg_gui::MainWindow::on_pushButton_scara_movej_clicked()
{
    ClickLabel *cmd_user = new ClickLabel(ui.widget_graphic_cmd);
    connect(cmd_user,&ClickLabel::clicked,this,&MainWindow::deal_label_enter);
    connect(cmd_user,&ClickLabel::del,this,&MainWindow::deal_label_del);
    user_graphic_cmd[user_graphic_cmd_index] = cmd_user;
    cmd_user->setFrameShape(QFrame::Panel);
    cmd_user->move(20,user_graphic_cmd_index*30);
    location_xy xy_plane;
    xy_plane.x = 100;
    xy_plane.y = user_graphic_cmd_index*30;
    clicklabel_location.append(xy_plane);
    script_cursor_Init();
    double scara_joint1 = 0;
    double scara_joint2 = 0;
    double scara_joint3 = 0;
    double scara_joint4 = 0;
    QString joint_cmd = "";
    scara_joint1 = ui.lineEdit_scara_movej_joint1_value->text().toDouble();
    scara_joint2 = ui.lineEdit_scara_movej_joint2_value->text().toDouble();
    scara_joint3 = ui.lineEdit_scara_movej_joint3_value->text().toDouble();
    scara_joint4 = ui.lineEdit_scara_movej_joint4_value->text().toDouble();
    joint_cmd= QString("    tutorial.Scara_Move_J(%1,%2,%3,%4)\n").arg(scara_joint1).arg(scara_joint2).arg(scara_joint3).arg(scara_joint4);
    cmd_list<<joint_cmd;
//    qDebug() << "wkr";
    cmd_user->setText("Move_Joint");
    ui.plainTextEdit_script->insertPlainText(joint_cmd);
    cmd_user->show();
    user_graphic_cmd_str[user_graphic_cmd_index] = joint_cmd;
    user_graphic_cmd_index++;
}

void hg_gui::MainWindow::on_pushButton_scara_movep_clicked()
{
    ClickLabel *cmd_user = new ClickLabel(ui.widget_graphic_cmd);
    connect(cmd_user,&ClickLabel::clicked,this,&MainWindow::deal_label_enter);
    connect(cmd_user,&ClickLabel::del,this,&MainWindow::deal_label_del);
    user_graphic_cmd[user_graphic_cmd_index] = cmd_user;
    cmd_user->setFrameShape(QFrame::Panel);
    cmd_user->move(20,user_graphic_cmd_index*30);
    double pos_x = ui.lineEdit_scara_movep_posx_value->text().toDouble();
    double pos_y = ui.lineEdit_scara_movep_posy_value->text().toDouble();
    double pos_z = ui.lineEdit_scara_movep_posz_value->text().toDouble();
    double orien_x = ui.lineEdit_scara_movep_orienx_value->text().toDouble();
    double orien_y = ui.lineEdit_scara_movep_orieny_value->text().toDouble();
    double orien_z = ui.lineEdit_scara_movep_orienz_value->text().toDouble();
    double orien_w = ui.lineEdit_scara_movep_orienw_value->text().toDouble();
    QString pose_cmd= QString("    tutorial.Scara_Move_P(%1,%2,%3,%4,%5,%6,%7)\n").arg(pos_x).arg(pos_y).arg(pos_z).arg(orien_w).arg(orien_x).arg(orien_y).arg(orien_z);
    ui.plainTextEdit_script->insertPlainText(pose_cmd);
    cmd_user->setText("Move_Pos");
    cmd_user->show();
    user_graphic_cmd_str[user_graphic_cmd_index] = pose_cmd;
    user_graphic_cmd_index++;
}

void hg_gui::MainWindow::on_pushButton_scara_movec_clicked()
{
    ClickLabel *cmd_user = new ClickLabel(ui.widget_graphic_cmd);
    connect(cmd_user,&ClickLabel::clicked,this,&MainWindow::deal_label_enter);
    connect(cmd_user,&ClickLabel::del,this,&MainWindow::deal_label_del);
    user_graphic_cmd[user_graphic_cmd_index] = cmd_user;
    cmd_user->setFrameShape(QFrame::Panel);
    cmd_user->move(20,user_graphic_cmd_index*30);
    script_cursor_Init();
    double hight = ui.lineEdit_scara_movec_high_value->text().toDouble();
    QString Group = ui.comboBox_scara_movec_group->currentText();
    QString vision = ui.comboBox_scara_movec_vision->currentText();
    QString move_cam_cmd= QString("    tutorial.Scara_Move_Cam(%1,\"%2\",\"%3\")\n").arg(hight).arg(Group).arg(vision);
    ui.plainTextEdit_script->insertPlainText(move_cam_cmd);
    cmd_user->setText("Move_Cam");
    cmd_user->show();
    user_graphic_cmd_str[user_graphic_cmd_index] = move_cam_cmd;
    user_graphic_cmd_index++;
}

void hg_gui::MainWindow::on_pushButton_scara_rivet_clicked()
{
    ClickLabel *cmd_user = new ClickLabel(ui.widget_graphic_cmd);
    connect(cmd_user,&ClickLabel::clicked,this,&MainWindow::deal_label_enter);
    connect(cmd_user,&ClickLabel::del,this,&MainWindow::deal_label_del);
    user_graphic_cmd[user_graphic_cmd_index] = cmd_user;
    cmd_user->setFrameShape(QFrame::Panel);
    cmd_user->move(20,user_graphic_cmd_index*30);
    location_xy xy_plane;
    xy_plane.x = 100;
    xy_plane.y = user_graphic_cmd_index*30;
    clicklabel_location.append(xy_plane);
    script_cursor_Init();
    double rivet_distance = 0;
    QString joint_cmd = "";
    rivet_distance = ui.lineEdit_scara_rivet_value->text().toDouble();
    joint_cmd= QString("    tutorial.Rivet_State_Setting(%1)\n").arg(rivet_distance);
    cmd_list<<joint_cmd;
    cmd_user->setText("Rivet");
    ui.plainTextEdit_script->insertPlainText(joint_cmd);
    cmd_user->show();
    user_graphic_cmd_str[user_graphic_cmd_index] = joint_cmd;
    user_graphic_cmd_index++;
}

void hg_gui::MainWindow::on_pushButton_robot_func_delay_clicked()
{
    ClickLabel *cmd_user = new ClickLabel(ui.widget_graphic_cmd);
    connect(cmd_user,&ClickLabel::clicked,this,&MainWindow::deal_label_enter);
    connect(cmd_user,&ClickLabel::del,this,&MainWindow::deal_label_del);
    user_graphic_cmd[user_graphic_cmd_index] = cmd_user;
    cmd_user->setFrameShape(QFrame::Panel);
    cmd_user->move(20,user_graphic_cmd_index*30);
    script_cursor_Init();
    double delay_time = ui.lineEdit_robot_func_delay_time->text().toDouble();
    QString delay_cmd= QString("    time.sleep(%1)\n").arg(delay_time);
    ui.plainTextEdit_script->insertPlainText(delay_cmd);
    cmd_user->setText("Delay_cmd");
    cmd_user->show();
    user_graphic_cmd_str[user_graphic_cmd_index] = delay_cmd;
    user_graphic_cmd_index++;
}

void hg_gui::MainWindow::on_pushButton_graph_cmd_rebuild_clicked()
{
    int location_y[user_graphic_cmd_index],location_y_rec[user_graphic_cmd_index];
    int scala_arrary[user_graphic_cmd_index];
    for(int i=0;i<user_graphic_cmd_index;i++)
    {
//        qDebug() << "x: " << user_graphic_cmd[i]->pos().x() << "y: " << user_graphic_cmd[i]->pos().y();
        location_y[i] = user_graphic_cmd[i]->pos().y();
        location_y_rec[i] = user_graphic_cmd[i]->pos().y();
        scala_arrary[i] = i;
    }
    for(int i=0;i<user_graphic_cmd_index-1;i++)
    {
        for(int j=0;j<user_graphic_cmd_index-1-i;j++)
        {
            if(location_y[j] > location_y[j+1])
            {
                int temp;
                temp = location_y[j];
                location_y[j] = location_y[j+1];
                location_y[j+1] = temp;
            }
        }
    }
    for(int i=0;i<user_graphic_cmd_index;i++)
    {
        for(int j=0;j<user_graphic_cmd_index;j++)
        {
            if(location_y_rec[i] == location_y[j])
            {
                scala_arrary[i] = j;
            }
        }
    }
    clicklabel_cmd_rebuild(scala_arrary);
    user_graph_cmd_str_rebuild(scala_arrary);
}

void hg_gui::MainWindow::on_pushButton_scara_movej_getjoint_clicked()
{
    scara_joint_position[0] = qnode.scara_position[0];
    scara_joint_position[1] = qnode.scara_position[1];
    scara_joint_position[2] = qnode.scara_position[2];
    scara_joint_position[3] = qnode.scara_position[3];
    ui.lineEdit_scara_movej_joint1_value->setText(QString::number(qnode.scara_position[0],'f',4));
    ui.lineEdit_scara_movej_joint2_value->setText(QString::number(qnode.scara_position[1],'f',4));
    ui.lineEdit_scara_movej_joint3_value->setText(QString::number(qnode.scara_position[2],'f',4));
    ui.lineEdit_scara_movej_joint4_value->setText(QString::number(qnode.scara_position[3],'f',4));
}

void hg_gui::MainWindow::on_pushButton_scara_movej_getpose_clicked()
{
    scara_movep_group_index = ui.comboBox_scara_movep_group->currentIndex();
//    QDateTime current_date_time = QDateTime::currentDateTime();
//    QString current_date = current_date_time.toString("yyyy-MM-dd");
//    QString current_time = current_date_time.toString("hh:mm:ss.zzz");
//    qDebug() << current_time;
    Robot_Scara_MoveP_LineEdit_Enable(false);
//    Delay_func(2);
//    current_date_time = QDateTime::currentDateTime();
//    current_time = current_date_time.toString("hh:mm:ss.zzz");
//    qDebug() << current_time;
    switch(scara_movep_group_index)
    {
        case 0:
            ui.lineEdit_scara_movep_posx_value->setText(QString::number(qnode.robot_scara_big_grip_pose[0],'f',4));
            ui.lineEdit_scara_movep_posy_value->setText(QString::number(qnode.robot_scara_big_grip_pose[1],'f',4));
            ui.lineEdit_scara_movep_posz_value->setText(QString::number(qnode.robot_scara_big_grip_pose[2],'f',4));
            ui.lineEdit_scara_movep_orienx_value->setText(QString::number(qnode.robot_scara_big_grip_pose[3],'f',4));
            ui.lineEdit_scara_movep_orieny_value->setText(QString::number(qnode.robot_scara_big_grip_pose[4],'f',4));
            ui.lineEdit_scara_movep_orienz_value->setText(QString::number(qnode.robot_scara_big_grip_pose[5],'f',4));
            ui.lineEdit_scara_movep_orienw_value->setText(QString::number(qnode.robot_scara_big_grip_pose[6],'f',4));
            Robot_Scara_MoveP_LineEdit_Enable(true);break;
        case 1:
            ui.lineEdit_scara_movep_posx_value->setText(QString::number(qnode.robot_scara_small_grip_pose[0],'f',4));
            ui.lineEdit_scara_movep_posy_value->setText(QString::number(qnode.robot_scara_small_grip_pose[1],'f',4));
            ui.lineEdit_scara_movep_posz_value->setText(QString::number(qnode.robot_scara_small_grip_pose[2],'f',4));
            ui.lineEdit_scara_movep_orienx_value->setText(QString::number(qnode.robot_scara_small_grip_pose[3],'f',4));
            ui.lineEdit_scara_movep_orieny_value->setText(QString::number(qnode.robot_scara_small_grip_pose[4],'f',4));
            ui.lineEdit_scara_movep_orienz_value->setText(QString::number(qnode.robot_scara_small_grip_pose[5],'f',4));
            ui.lineEdit_scara_movep_orienw_value->setText(QString::number(qnode.robot_scara_small_grip_pose[6],'f',4));
            Robot_Scara_MoveP_LineEdit_Enable(true);break;
        default:break;
    }
}



void hg_gui::MainWindow::on_pushButton_scara_grip_big_clicked()
{
    ClickLabel *cmd_user = new ClickLabel(ui.widget_graphic_cmd);
    connect(cmd_user,&ClickLabel::clicked,this,&MainWindow::deal_label_enter);
    connect(cmd_user,&ClickLabel::del,this,&MainWindow::deal_label_del);
    user_graphic_cmd[user_graphic_cmd_index] = cmd_user;
    cmd_user->setFrameShape(QFrame::Panel);
    cmd_user->move(20,user_graphic_cmd_index*30);
    location_xy xy_plane;
    xy_plane.x = 100;
    xy_plane.y = user_graphic_cmd_index*30;
    clicklabel_location.append(xy_plane);
    script_cursor_Init();
    double scara_big_hand_joint1 = 0;
    double scara_big_hand_joint2 = 0;
    QString joint_cmd = "";
    scara_big_hand_joint1 = ui.lineEdit_scara_big_grip_joint1_value->text().toDouble();
    scara_big_hand_joint2 = ui.lineEdit_scara_big_grip_joint2_value->text().toDouble();
    joint_cmd= QString("    tutorial.Big_Grip_State_Setting(%1,%2)\n").arg(scara_big_hand_joint1).arg(scara_big_hand_joint2);
    cmd_list<<joint_cmd;
    cmd_user->setText("Big_Grip");
    ui.plainTextEdit_script->insertPlainText(joint_cmd);
    cmd_user->show();
    user_graphic_cmd_str[user_graphic_cmd_index] = joint_cmd;
    user_graphic_cmd_index++;
}

void hg_gui::MainWindow::on_pushButton_scara_grip_small_clicked()
{
    ClickLabel *cmd_user = new ClickLabel(ui.widget_graphic_cmd);
    connect(cmd_user,&ClickLabel::clicked,this,&MainWindow::deal_label_enter);
    connect(cmd_user,&ClickLabel::del,this,&MainWindow::deal_label_del);
    user_graphic_cmd[user_graphic_cmd_index] = cmd_user;
    cmd_user->setFrameShape(QFrame::Panel);
    cmd_user->move(20,user_graphic_cmd_index*30);
    location_xy xy_plane;
    xy_plane.x = 100;
    xy_plane.y = user_graphic_cmd_index*30;
    clicklabel_location.append(xy_plane);
    script_cursor_Init();
    double scara_small_hand_joint1 = 0;
    double scara_small_hand_joint2 = 0;
    QString joint_cmd = "";
    scara_small_hand_joint1 = ui.lineEdit_scara_small_grip_joint1_value->text().toDouble();
    scara_small_hand_joint2 = ui.lineEdit_scara_small_grip_joint2_value->text().toDouble();
    joint_cmd= QString("    tutorial.Small_Grip_State_Setting(%1,%2)\n").arg(scara_small_hand_joint1).arg(scara_small_hand_joint2);
    cmd_list<<joint_cmd;
    cmd_user->setText("Small_Grip");
    ui.plainTextEdit_script->insertPlainText(joint_cmd);
    cmd_user->show();
    user_graphic_cmd_str[user_graphic_cmd_index] = joint_cmd;
    user_graphic_cmd_index++;
}

//加载 装配台scara 仿真场景
void hg_gui::MainWindow::on_pushButton_scara_load_world_clicked()
{
    qDebug() << ui.plainTextEdit_script->textCursor().position();
    qnode.scara_sim_flag = 1;
    bash_load_scara_world = new QProcess;
    QString load_scara_world_cmd;
    load_scara_world_cmd = source_ws +"/devel/setup.bash\nroslaunch all_world scara_gazebo.launch\n";
    bash_load_scara_world->start("bash");
    bash_load_scara_world->write(load_scara_world_cmd.toLocal8Bit());
    on_button_connect_clicked(true);
    ui.pushButton_scara_load_world->setEnabled(false);
}
