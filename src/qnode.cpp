/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <sstream>
#include "../include/hg_gui/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace hg_gui {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"hg_gui");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
	chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
    pub_speed = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
    chatter_subscriber = n.subscribe("chatter",1000,&QNode::chatter_callback,this);
    Jointstats_sub = n.subscribe("joint_states",1000,&QNode::Jointstates_callback,this);
    states_subscriber = n.subscribe("gazebo/model_states",1,&QNode::Robotstates_callback,this);
    voice_subscriber = n.subscribe("/nav_position",1,&QNode::Voicestates_callback,this);
	start();
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"hg_gui");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
	chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
    pub_speed = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
    chatter_subscriber = n.subscribe("chatter",1000,&QNode::chatter_callback,this);
    Jointstats_sub = n.subscribe("joint_states",10,&QNode::Jointstates_callback,this);
    states_subscriber = n.subscribe("gazebo/model_states",1,&QNode::Robotstates_callback,this);
    voice_subscriber = n.subscribe("/nav_position",1,&QNode::Voicestates_callback,this);
    start();
	return true;
}

void QNode::run() {
	ros::Rate loop_rate(1);
	int count = 0;
    tf::TransformListener listener_big;
    tf::TransformListener listener_small;
	while ( ros::ok() ) {
        if(scara_sim_flag == 1)
        {
            tf::StampedTransform transform_big;
            tf::StampedTransform transform_small;
            try{
                //得到坐标world和坐标scara_dualgrip_big_grip_ref之间的关系
                listener_big.waitForTransform("world", "scara_dualgrip_big_grip_ref", ros::Time(0), ros::Duration(3.0));
                listener_big.lookupTransform("world", "scara_dualgrip_big_grip_ref", ros::Time(0), transform_big);
                //得到坐标world和坐标scara_dualgrip_big_grip_ref之间的关系
                listener_small.waitForTransform("world", "scara_dualgrip_small_grip_ref", ros::Time(0), ros::Duration(3.0));
                listener_small.lookupTransform("world", "scara_dualgrip_small_grip_ref", ros::Time(0), transform_small);
            }
            catch (tf::TransformException &ex) {
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
            }

            robot_scara_big_grip_pose[0]=transform_big.getOrigin().x();
            robot_scara_big_grip_pose[1]=transform_big.getOrigin().y();
            robot_scara_big_grip_pose[2]=transform_big.getOrigin().z();
            robot_scara_big_grip_pose[3]=transform_big.getRotation().getX();
            robot_scara_big_grip_pose[4]=transform_big.getRotation().getY();
            robot_scara_big_grip_pose[5]=transform_big.getRotation().getZ();
            robot_scara_big_grip_pose[6]=transform_big.getRotation().getW();

            robot_scara_small_grip_pose[0]=transform_small.getOrigin().x();
            robot_scara_small_grip_pose[1]=transform_small.getOrigin().y();
            robot_scara_small_grip_pose[2]=transform_small.getOrigin().z();
            robot_scara_small_grip_pose[3]=transform_small.getRotation().getX();
            robot_scara_small_grip_pose[4]=transform_small.getRotation().getY();
            robot_scara_small_grip_pose[5]=transform_small.getRotation().getZ();
            robot_scara_small_grip_pose[6]=transform_small.getRotation().getW();
        }
		std_msgs::String msg;
		std::stringstream ss;
		ss << "hello world " << count;
		msg.data = ss.str();
		chatter_publisher.publish(msg);
//		log(Info,std::string("I sent: ")+msg.data);
		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

// 设置速度命令
void QNode::set_cmd_vel(char k,float linear,float angular)
{
    geometry_msgs::Twist vel_control;
    vel_control.linear.x = linear;
    vel_control.angular.z = angular;
    pub_speed.publish(vel_control);
}

// 已弃用
void QNode::chatter_callback(const std_msgs::String &msg)
{
//    log(Info,"I receive"+msg.data);
}

// 获取机器人关节角
void QNode::Jointstates_callback(const sensor_msgs::JointStateConstPtr& msg)
{
    int data_size = msg->name.size();
//    qDebug() << "data_size: " << data_size;
    // data_size = 10 scara
//    qDebug() << "2" <<msg->position[1];
    if(data_size == 10)
    {
        scara_position[0] = msg->position[0];
        scara_position[1] = msg->position[1];
        scara_position[2] = msg->position[2];
        scara_position[3] = msg->position[3];
    }
    if(data_size>10)
    {
        cobot_position[0] = msg->position[8];
        cobot_position[1] = msg->position[9];
        cobot_position[2] = msg->position[10];
        cobot_position[3] = msg->position[11];
        cobot_position[4] = msg->position[12];
        cobot_position[5] = msg->position[13];
        cobot_position[6] = msg->position[14];
    }
}

// 获取Gazebo中机器人状态
void QNode::Robotstates_callback(const gazebo_msgs::ModelStatesConstPtr& msg)
{
    int modelCount = msg->name.size();
//    qDebug() << "2" <<msg->position[1];
    for(int modelInd = 0; modelInd < modelCount; ++modelInd)
    {
        if(msg->name[modelInd] == "robot")
        {
            geometry_msgs::Pose pose = msg->pose[modelInd];
            robot_states[0]= pose.position.x;
            robot_states[1]= pose.position.y;
            robot_states[2]= pose.position.z;
            robot_states[3]= pose.orientation.x;
            robot_states[4]= pose.orientation.y;
            robot_states[5]= pose.orientation.z;
            robot_states[6]= pose.orientation.w;
            break;
        }
    }
}

// 语音控制回调函数
void QNode::Voicestates_callback(const std_msgs::Int64& msg)
{
//    qDebug() << "nav_position";
    std::cout << msg.data;
    voice_cmd = msg.data;
    switch (voice_cmd) {
        case(7): emit forward(); break;
        case(8): emit back_off(); break;
        case(9): emit turn_left(); break;
        case(10): emit turn_right(); break;
        case(11): emit stop(); break;
        default:emit stop();break;
    }
}

void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

}  // namespace hg_gui
