/**
 * @file /include/hg_gui/main_window.hpp
 *
 * @brief Qt based gui for hg_gui.
 *
 * @date November 2010
 **/
#ifndef hg_gui_MAIN_WINDOW_H
#define hg_gui_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtWidgets/QMainWindow>
#include "ui_main_window.h"
#include "clicklabel.hpp"
#include "qnode.hpp"
#include <QProcess>
#include <math.h>
#include <QTime>
#include <QTimer>
#include <QFileDialog>
#include <QUdpSocket>
#include <QScrollArea>
#include <QDateTime>

#define ToDeg 180/M_PI
#define ToRad M_PI/180

extern int scara_movep_group_index;

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace hg_gui {

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

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();


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
    void Init_timer();

    void Delay_func(int seconds);

    void Scara_Widgets_element_Init();
    bool openTextByStream(const QString &aFileName);
    bool openTextConfigByStrem();
    bool saveTextByStream(const QString &aFileName);
    void clicklabel_savefile_Init();
    void deal_label_enter(int index);
    void deal_label_del();
    void array_del(ClickLabel *array[],int index);
    void clicklabel_cmd_renew();
    void script_cursor_Init();
    bool saveTextConfigByStrem();
    void comboBox_state_switch(int rindex,int mindex);
    void clicklabel_cmd_rebuild(int array[]);
    void user_graph_cmd_str_rebuild(int array[]);
    void Robot_Scara_MoveP_LineEdit_Enable(bool enable);

private slots:
    void on_pushButton_scara_load_world_clicked();

private slots:
    void on_pushButton_scara_grip_small_clicked();

private slots:
    void on_pushButton_scara_grip_big_clicked();

private slots:
    void on_pushButton_scara_movej_getpose_clicked();

private slots:
    void on_pushButton_scara_movej_getjoint_clicked();

private slots:
    void on_pushButton_graph_cmd_rebuild_clicked();

private slots:
    void on_pushButton_robot_func_delay_clicked();

private slots:
    void on_pushButton_scara_rivet_clicked();

private slots:
    void on_pushButton_scara_movec_clicked();

private slots:
    void on_pushButton_scara_movep_clicked();

private slots:
    void on_pushButton_scara_movej_clicked();

private slots:
    void on_comboBox_movecmd_currentIndexChanged(int index);

private slots:
    void on_comboBox_robot_ns_currentIndexChanged(int index);

private slots:
    void on_actionSave_FIle_triggered();

private slots:
    void on_actionOpen_File_triggered();

private slots:
    void on_pushButton_voice_Ctrl_clicked();

private slots:
    void on_pushButton_hand_nav_clicked();

private slots:
    void on_pushButton_voice_navigation_clicked();

private slots:
    void on_comboBox_currentIndexChanged(int index);

private slots:
    void on_pushButton_voice_nav_clicked();

private slots:
    void on_pushButton_place_check_clicked();

private slots:
    void on_pushButton_pick_check_clicked();

private slots:
    void on_pushButton_image_view_clicked();

private slots:
    void on_pushButton_update_score_clicked();

private slots:
    void on_pushButton_pickandplace_clicked();

private Q_SLOTS:
    void on_pushButton_loadworld_clicked();

    void on_pushButton_remote_clicked();

    void slot_pushbtn_click();

    void on_pushButton_buildmap_clicked();

    void on_pushButton_nav_map_clicked();

    void on_pushButton_loadworld_turtle_clicked();

    void on_pushButton_buildmap_turtle_clicked();

    void on_pushButton_nav_map_turtle_clicked();

    void on_pushButton_getcurangle_clicked();

    void on_pushButton_Unitchange_clicked();

    void on_pushButton_language_clicked();

    void updateDisplay();

    void on_pushButton_time_start_clicked();

    void on_pushButton_time_stop_clicked();

    void on_pushButton_close_process_clicked();

    void on_pushButton_savemap_turtle_clicked();

    void on_pushButton_savemap_clicked();

    void on_pushButton_open_nav_config_clicked();

    void on_pushButton_getrobotstates_clicked();

    void on_pushButton_build_world_clicked();

    void sendmsg();

    void btn_enable();

    void btn_disable();

    void voice_forward();

    void voice_back_off();

    void voice_turn_left();

    void voice_turn_right();

    void voice_stop();

private:
	Ui::MainWindowDesign ui;
    int unit_change_flag = 0;
    int language_flag = 0;
    int count = 0;
    float linear = 0;
    float angular = 0;
    float linear_max = 0.4;
    float angular_max = 1.5;
    int Task_number = 0;
    double cobot_joint_position[7];
    QString user_name_str = "";
    QString ws_name = "";
    QString user_ws_str = "";
    QString source_ws = "";
    QString ip;
    quint16 port;
	QNode qnode;
    QProcess *bash_cmd;
    QProcess *bash_killer;
    QProcess *bash_load_world;
    QProcess *bash_remote;
    QProcess *bash_build_map;
    QProcess *bash_nav_map;
    QProcess *bash_save_map;
    QProcess *bash_load_turtle_world;
    QProcess *bash_build_turtle_map;
    QProcess *bash_nav_turtle_map;
    QProcess *bash_save_turtle_map;
    QProcess *bash_nav_config;
    QProcess *bash_build_world;
    QProcess *bash_pick_place;
    QProcess *bash_image_view;
    QProcess *bash_pick_check;
    QProcess *bash_place_check;
    QProcess *bash_nav_world;
    QProcess *bash_load_scara_world;
//    QProcess *bash_voice_nav;
    QTimer *pTimer;
    QTime baseTime;
    QString timeStr;
    int user_name;
    QUdpSocket *ssocket;
    unsigned char score[5];



    //scara
    int robot_index = 0;
    int move_cmd_index = 0;
    int linenumber = 0;
    int script_cursor_init_flag = 0;
    QString open_file_path = "";
    QString save_file_path = "";
    QString open_file_name = "";
    QString save_file_name = "";
    QString user_graphic_cmd_labelname[100];
    QString user_graphic_cmd_str[100];
    ClickLabel *user_graphic_cmd[100];
    QList<QString> cmd_list;
    struct location_xy
    {
        int x;
        int y;
    };
    QList<location_xy> clicklabel_location;
    double scara_joint_position[4];
};

}  // namespace hg_gui

#endif // hg_gui_MAIN_WINDOW_H
