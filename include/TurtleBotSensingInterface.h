//
// Created by Gerard Canal <gcanal@iri.upc.edu> on 20/10/18.
//

#ifndef KCL_TURTLEBOT_TURTLEBOTSENSINGINTERFACE_H
#define KCL_TURTLEBOT_TURTLEBOTSENSINGINTERFACE_H
#include "ros/ros.h"
#include "kobuki_msgs/DockInfraRed.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "rosplan_knowledge_msgs/GetInstanceService.h"
#include "rosplan_knowledge_msgs/KnowledgeUpdateServiceArray.h"
#include "mongodb_store/message_store.h"
#include "diagnostic_msgs/KeyValue.h"
#include "std_msgs/Bool.h"

//#define AUTO_LOCALISED

class TurtleBotSensingInterface {
private:
    ros::NodeHandle nh_;
    ros::Subscriber dock_subs;
    ros::Subscriber pose_subs;
    ros::Subscriber localised_mock_subs;
    ros::Subscriber somebody_at_mock_subs;
    ros::Subscriber is_printer_busy_mock_subs;

    ros::ServiceClient update_kb;
    ros::ServiceClient query_instances;
    ros::ServiceClient update_knowledge;

    // Scene database
    mongodb_store::MessageStoreProxy message_store;


    // Predicates
    bool docked, changed_docked;
    bool localised, changed_localised;
    std::map<std::string, bool> robot_at;
    bool changed_robotat;
    std::map<std::string, bool> somebodyat;
    bool changed_somebodyat;
    std::map<std::string, bool> isbusy;
    bool changed_isbusy;

    // Callbacks
    void dock_cb(kobuki_msgs::DockInfraRedConstPtr msg);
    void pose_cb(geometry_msgs::PoseWithCovarianceStampedConstPtr msg);
    void localise_cb(std_msgs::BoolConstPtr msg);
    void somebodyat_cb(diagnostic_msgs::KeyValueConstPtr msg);
    void isbusy_cb(diagnostic_msgs::KeyValueConstPtr msg);

    std::string robot_name_;
public:
    TurtleBotSensingInterface(ros::NodeHandle& nh, std::string robot_name="kenny");
    ~TurtleBotSensingInterface() = default;

    void updateKB();
};


#endif //KCL_TURTLEBOT_TURTLEBOTSENSINGINTERFACE_H
