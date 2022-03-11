/**
 * @file /include/hg_gui/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef hg_gui_QNODE_HPP_
#define hg_gui_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>
#include <std_msgs/String.h>
#include <std_msgs/Int64.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <QDebug>
#include <gazebo_msgs/ModelStates.h>
#include <tf/transform_listener.h>

extern int scara_movep_group_index;
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace hg_gui {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);

    double cobot_position[7];
    double scara_position[4];
    double robot_states[7];
    double robot_scara_big_grip_pose[7];
    double robot_scara_small_grip_pose[7];
    int scara_sim_flag = 0;

    void set_cmd_vel(char k,float linear,float angular);
	void run();

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);

Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();
    void forward();
    void back_off();
    void turn_left();
    void turn_right();
    void stop();

private:
	int init_argc;
	char** init_argv;
    int voice_cmd;
	ros::Publisher chatter_publisher;
    ros::Publisher pub_speed;
    ros::Subscriber Jointstats_sub;
    ros::Subscriber chatter_subscriber;
    ros::Subscriber states_subscriber;
    ros::Subscriber voice_subscriber;
    QStringListModel logging_model;
//    tf::TransformListener listener;

    void chatter_callback(const std_msgs::String &msg);
    void Jointstates_callback(const sensor_msgs::JointStateConstPtr& msg);
    void Robotstates_callback(const gazebo_msgs::ModelStatesConstPtr& msg);
    void Voicestates_callback(const std_msgs::Int64& msg);
};

}  // namespace hg_gui

#endif /* hg_gui_QNODE_HPP_ */
