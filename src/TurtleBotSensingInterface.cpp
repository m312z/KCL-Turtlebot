//
// Created by Gerard Canal <gcanal@iri.upc.edu> on 20/10/18.
//

#include <limits>
#include <TurtleBotSensingInterface.h>


TurtleBotSensingInterface::TurtleBotSensingInterface(ros::NodeHandle &nh, std::string robot_name) : nh_(nh), message_store(nh) {
    //dock_subs =  nh.subscribe("/mobile_base/sensors/dock_ir", 1, &TurtleBotSensingInterface::dock_cb, this);
    dock_subs =  nh.subscribe("/mobile_base/sensors/core", 1, &TurtleBotSensingInterface::dock_cb, this);
    pose_subs =  nh.subscribe("/amcl_pose", 1, &TurtleBotSensingInterface::pose_cb, this);
    localised_mock_subs =  nh.subscribe("/localised_mock", 1, &TurtleBotSensingInterface::localise_cb, this);
    somebody_at_mock_subs =  nh.subscribe("/somebodyat_mock", 1, &TurtleBotSensingInterface::somebodyat_cb, this);
    is_printer_busy_mock_subs =  nh.subscribe("/isbusy_mock", 1, &TurtleBotSensingInterface::isbusy_cb, this);

    query_instances = nh.serviceClient<rosplan_knowledge_msgs::GetInstanceService>("/rosplan_knowledge_base/state/instances");
    update_knowledge = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateServiceArray>("/rosplan_knowledge_base/update_array");
    docked = true;
    robot_name_ = robot_name;
    changed_docked = changed_localised = changed_robotat = false;
}

void TurtleBotSensingInterface::dock_cb(kobuki_msgs::SensorStateConstPtr msg) {
    changed_docked = docked; // copy value
    /*docked =  msg->data[0] == kobuki_msgs::DockInfraRed::NEAR_RIGHT;
    docked &= msg->data[1] == kobuki_msgs::DockInfraRed::NEAR_CENTER;
    docked &= msg->data[2] == kobuki_msgs::DockInfraRed::NEAR_LEFT;*/
    docked = msg->charger != msg->DISCHARGING;
    changed_docked ^= docked; // XOR will be true only if both are different
}


void TurtleBotSensingInterface::pose_cb(geometry_msgs::PoseWithCovarianceStampedConstPtr msg) {
    assert(msg->header.frame_id == "map");

    rosplan_knowledge_msgs::GetInstanceService inst;
    inst.request.type_name = "waypoint";
    if (not query_instances.call(inst)) {
        ROS_ERROR("Could not call service to get instances.");
        return;
    }

    double d = std::numeric_limits<double>::max();
    std::string wpName = "";
    for (auto wit=inst.response.instances.begin(); wit!=inst.response.instances.end(); ++wit) {
        std::vector< boost::shared_ptr<geometry_msgs::PoseStamped> > results;
        message_store.queryNamed<geometry_msgs::PoseStamped>(*wit, results);
        assert(results.size() > 0);
        double vX = results[0]->pose.position.x - msg->pose.pose.position.x;
        double vY = results[0]->pose.position.y - msg->pose.pose.position.y;
        if (sqrt(vX*vX + vY*vY) < d) {
            wpName = *wit;
            d = sqrt(vX*vX + vY*vY);
        }
    }

    changed_robotat = false;
    for (auto wit=inst.response.instances.begin(); wit!=inst.response.instances.end(); ++wit) {
        bool bak = robot_at[*wit];
        robot_at[*wit] = (*wit == wpName);
        if (bak ^ robot_at[*wit]) changed_robotat = true;
    }

    #ifdef AUTO_LOCALISED
    changed_localised = localised;
    localised = (msg->pose.covariance[0] < 0.009) && (msg->pose.covariance[1] < 0.009);
    changed_localised ^= localised;
    #endif
}

void TurtleBotSensingInterface::updateKB() {
    rosplan_knowledge_msgs::KnowledgeUpdateServiceArray updatePredSrv;

    diagnostic_msgs::KeyValue robot_param;
    robot_param.value = robot_name_;
    robot_param.key = "r";

    // docked
    if (changed_docked) {

        rosplan_knowledge_msgs::KnowledgeItem item;
        item.knowledge_type = item.FACT;
        item.is_negative = not docked;
        item.attribute_name = "docked";
        item.values.push_back(robot_param);
        updatePredSrv.request.knowledge.push_back(item);
        //if (docked)
        updatePredSrv.request.update_type.push_back(rosplan_knowledge_msgs::KnowledgeUpdateServiceArray::Request::ADD_KNOWLEDGE);
        item.is_negative = docked;
	updatePredSrv.request.knowledge.push_back(item);
	updatePredSrv.request.update_type.push_back(rosplan_knowledge_msgs::KnowledgeUpdateServiceArray::Request::REMOVE_KNOWLEDGE);
    }
    if (changed_docked) {
        changed_docked = false;
        rosplan_knowledge_msgs::KnowledgeItem item;
        item.knowledge_type = item.FACT;
        item.is_negative = docked;
        item.attribute_name = "undocked";
        item.values.push_back(robot_param);
        updatePredSrv.request.knowledge.push_back(item);
        //if (not docked)
        updatePredSrv.request.update_type.push_back(rosplan_knowledge_msgs::KnowledgeUpdateServiceArray::Request::ADD_KNOWLEDGE);
	item.is_negative = not docked;
        updatePredSrv.request.knowledge.push_back(item);
	updatePredSrv.request.update_type.push_back(rosplan_knowledge_msgs::KnowledgeUpdateServiceArray::Request::REMOVE_KNOWLEDGE);
    }

    // localised
    if (changed_localised) {
        changed_localised = false;
        rosplan_knowledge_msgs::KnowledgeItem item;
        item.knowledge_type = item.FACT;
        item.is_negative = 0;
        item.attribute_name = "localised";
        item.values.push_back(robot_param);
        updatePredSrv.request.knowledge.push_back(item);
        if (localised)
            updatePredSrv.request.update_type.push_back(rosplan_knowledge_msgs::KnowledgeUpdateServiceArray::Request::ADD_KNOWLEDGE);
        else updatePredSrv.request.update_type.push_back(rosplan_knowledge_msgs::KnowledgeUpdateServiceArray::Request::REMOVE_KNOWLEDGE);
    }

    if (changed_robotat) {
        changed_robotat = false;
        // Robot at
        for (auto it = robot_at.begin(); it != robot_at.end(); ++it) {
            rosplan_knowledge_msgs::KnowledgeItem item;
            item.knowledge_type = item.FACT;
            item.is_negative = 0;
            item.attribute_name = "robot_at";
            item.values.push_back(robot_param);

            diagnostic_msgs::KeyValue wp;
            wp.value = it->first;
            wp.key = "w";
            item.values.push_back(wp);

            updatePredSrv.request.knowledge.push_back(item);
            if (it->second)
                updatePredSrv.request.update_type.push_back(
                        rosplan_knowledge_msgs::KnowledgeUpdateServiceArray::Request::ADD_KNOWLEDGE);
            else
                updatePredSrv.request.update_type.push_back(
                        rosplan_knowledge_msgs::KnowledgeUpdateServiceArray::Request::REMOVE_KNOWLEDGE);
        }
    }

    if (changed_somebodyat) {
        changed_somebodyat = false;
        for (auto it = somebodyat.begin(); it != somebodyat.end(); ++it) {
            rosplan_knowledge_msgs::KnowledgeItem item;
            item.knowledge_type = item.FACT;
            item.is_negative = 0;
            item.attribute_name = "somebody_at";

            diagnostic_msgs::KeyValue wp;
            wp.value = it->first;
            wp.key = "w";
            item.values.push_back(wp);

            updatePredSrv.request.knowledge.push_back(item);
            if (it->second)
                updatePredSrv.request.update_type.push_back(
                        rosplan_knowledge_msgs::KnowledgeUpdateServiceArray::Request::ADD_KNOWLEDGE);
            else
                updatePredSrv.request.update_type.push_back(
                        rosplan_knowledge_msgs::KnowledgeUpdateServiceArray::Request::REMOVE_KNOWLEDGE);
        }
    }

    if (changed_isbusy) {
        changed_isbusy = false;
        for (auto it = isbusy.begin(); it != isbusy.end(); ++it) {
            rosplan_knowledge_msgs::KnowledgeItem item;
            item.knowledge_type = item.FACT;
            item.is_negative = 0;
            item.attribute_name = "is_busy";

            diagnostic_msgs::KeyValue wp;
            wp.value = it->first;
            wp.key = "w";
            item.values.push_back(wp);

            updatePredSrv.request.knowledge.push_back(item);
            if (it->second)
                updatePredSrv.request.update_type.push_back(
                        rosplan_knowledge_msgs::KnowledgeUpdateServiceArray::Request::ADD_KNOWLEDGE);
            else
                updatePredSrv.request.update_type.push_back(
                        rosplan_knowledge_msgs::KnowledgeUpdateServiceArray::Request::REMOVE_KNOWLEDGE);
        }
    }

    if (not update_knowledge.call(updatePredSrv))
        ROS_INFO("KCL: (TurtleBotSensing) failed to update PDDL model in knowledge base");

}

void TurtleBotSensingInterface::localise_cb(std_msgs::BoolConstPtr msg) {
#ifndef AUTO_LOCALISED
    changed_localised = localised; // copy value
    localised = msg->data > 0;
    changed_localised ^= localised; // XOR will be true only if both are different
#endif
}

void TurtleBotSensingInterface::somebodyat_cb(diagnostic_msgs::KeyValueConstPtr msg) {
    changed_somebodyat = somebodyat[msg->key]; // copy value
    somebodyat[msg->key] = msg->value == "yes";
    changed_somebodyat ^= somebodyat[msg->key]; // XOR will be true only if both are different
}

void TurtleBotSensingInterface::isbusy_cb(diagnostic_msgs::KeyValueConstPtr msg) {
    changed_isbusy = isbusy[msg->key]; // copy value
    isbusy[msg->key] = msg->value == "yes";
    changed_isbusy ^= isbusy[msg->key]; // XOR will be true only if both are different
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "turtlebot_sensing");
    ros::NodeHandle n;

    TurtleBotSensingInterface tbsi(n);

    ros::Rate loop_rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        /*std::cout << "Is the robot docked? (1/0): ";
        bool dbkp = tbsi.docked;
        std::cin >> tbsi.docked;
        tbsi.changed_docked = tbsi.docked ^ dbkp;*/
        tbsi.updateKB();
        loop_rate.sleep();
    }
}

