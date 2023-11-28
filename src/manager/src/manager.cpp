#include "manager.h"
#include <tf/tf.h>
#include <tf/LinearMath/Matrix3x3.h>

Manager::Manager(int argc, char **argv) :
    argc_(argc),
    argv_(argv)
{
    plan_topic = "manager/plan";
    cmd_plan_topic = "manager/cmd_plan";
    cancel_plan_topic = "move_base/cancel";
    back_topic = "manager/back";
    goal_topic = "/move_base_simple/goal";
    goal_result_topic = "/move_base/result";
    map_topic = "map";

    plan_ = new Plan();
    plan_->init();

    last_id = 0;
    last_type_ = PlanType_t::STOP;
    isBusy = false;
    isSave_map_ = false;
    status_.status = status_.STOP;
}

Manager::~Manager()
{
    delete plan_;
}

bool Manager::init()
{
    ros::init(argc_, argv_, "manager");
    if (!ros::master::check()) 
        return false;
    
    ros::start();
    TopicSubPub();
    mapname_ = "/home/ubuntu/share/Dev/simulation_ws/src/simulation/map/nav";
    return true;  
}

void Manager::run()
{
    ros::Rate loop_rate(frame_rate_);
    ros::AsyncSpinner spinner(thread_num_);
    spinner.start();

    while(ros::ok()) {

        loop_rate.sleep();
    }
}

void Manager::TopicSubPub()
{
    nh = new ros::NodeHandle();

    pub_goal_ = nh->advertise<geometry_msgs::PoseStamped> (goal_topic.c_str(), 10);
    sub_goal_result_ = nh->subscribe<move_base_msgs::MoveBaseActionResult>(goal_result_topic.c_str(), 200, &Manager::actionResult_call, this);
    sub_map_ = nh->subscribe(map_topic.c_str(), 1, &Manager::mapCallback, this);
    sub_plan_ = nh->subscribe<manager_msgs::Plan> (plan_topic.c_str(), 10, &Manager::planCallback, this);
    sub_cmd_plan_ = nh->subscribe<manager_msgs::Status> (cmd_plan_topic.c_str(), 10, &Manager::cmdPlanCallback, this);
    pub_cancel_plan = nh->advertise<actionlib_msgs::GoalID> (cancel_plan_topic.c_str(), 1);
    pub_back_ = nh->advertise<std_msgs::String>(back_topic.c_str(), 1);
    set_goal_timer = nh->createTimer(ros::Duration(0.1), &Manager::set_goal_timer_callback, this);
}

void Manager::planCallback(const manager_msgs::Plan::ConstPtr &msg)
{
    addPlan(msg);
}

void Manager::addPlan(const manager_msgs::Plan::ConstPtr &plan)
{
    PlanType_t type;
    type = (PlanType_t)plan->type.status;

    geometry_msgs::PoseStamped poseStamped;
    poseStamped.pose.orientation.w = plan->pose.orientation.w;
    poseStamped.pose.orientation.x = plan->pose.orientation.x;
    poseStamped.pose.orientation.y = plan->pose.orientation.y;
    poseStamped.pose.orientation.z = plan->pose.orientation.z;
    
    poseStamped.pose.position.x = plan->pose.position.x;
    poseStamped.pose.position.y = plan->pose.position.y;
    poseStamped.pose.position.z = plan->pose.position.z;
    poseStamped.header.frame_id = "map";
    
    plan_->add(plan->id, poseStamped, type);

    ROS_INFO("success add plan %ld type:%s", plan->id, PLAN_TYPE[type]);

    stream_ << "add [" << plan->id << "] " << 
        "[" << PLAN_TYPE[type]  << "]" << " SUCCESSED";
    msg_back.data = stream_.str();
    pub_back_.publish(msg_back);
    stream_.str("");
}


void Manager::cmdPlanCallback(const manager_msgs::Status::ConstPtr &msg)
{
    manager_msgs::Status status_temp;
    status_temp.status= msg->status;

    switch (status_temp.status) {
    case status_.STOP:
        last_id = 1;
        plan_->setCurrentId(last_id);
        goalId.id = "";
        pub_cancel_plan.publish(goalId);
        ROS_INFO("Plan STOP");
   
        stream_ << "Plan STOP";
        msg_back.data = stream_.str();
        pub_back_.publish(msg_back);
        stream_.str("");

        break;
    
    case status_.START:
        last_id = 1;
        plan_->setCurrentId(last_id);
        ROS_INFO("Plan is start run");

        if (plan_->getmaxId() == 0) {
            stream_ << "Please add Plan";
            msg_back.data = stream_.str();
            pub_back_.publish(msg_back);
        } else {
            stream_ << "Plan START";
            msg_back.data = stream_.str();
            pub_back_.publish(msg_back);
        }

   
        stream_.str("");
        
        isBusy = false; 
        break;  
 
    case status_.CANCEL:
        plan_->setCurrentId(last_id);
        ROS_INFO("Plan cancel");
 
        stream_ << "Plan [" << last_id << "] " << "CANCEL";
        msg_back.data = stream_.str();
        pub_back_.publish(msg_back);
        stream_.str("");

        goalId.id = "";
        pub_cancel_plan.publish(goalId);
        break;

    case status_.CONTINUE:
        plan_->setCurrentId(last_id);
        ROS_INFO("Plan is continue");
        
        stream_ << "Plan [" << last_id << "] " << "CONTINUE";
        msg_back.data = stream_.str();
        pub_back_.publish(msg_back);
        stream_.str("");

        isBusy = false;      
        break;

    case status_.DELETEALL:
        plan_->init();

        ROS_INFO("Plan deleteAll");
        stream_ << "Plan DELETEALL";
        msg_back.data = stream_.str();
        pub_back_.publish(msg_back);
        stream_.str("");
        break;

    case status_.MAPSAVE:
        isSave_map_ = true;
        break;
    default:
        break;
    }

    status_.status = msg->status;
}

void Manager::set_goal_timer_callback(const ros::TimerEvent &e)
{
    if (status_.status != status_.START && status_.status != status_.CONTINUE)
        return;
 
    if (isBusy) 
        return;

    last_id = plan_->getCurrentId();
    if (plan_->getmaxId() == 0 || last_id > plan_->getmaxId()) 
        return;

    isBusy = true;
    
    switch (plan_->getCurrentType()) {
    case PlanType_t::STOP:
        ROS_INFO("Plan STOP [%d] successed.", last_id);
        
        stream_ << "Plan STOP [" << last_id << "] " << "successed";
        msg_back.data  = stream_.str();
        pub_back_.publish(msg_back);
        stream_.str("");

        status_.status = status_.STOP;
        break;
    
    case PlanType_t::PAUSE:
        ROS_INFO("Plan PAUSE [%d] successed.", last_id);
        plan_->setCurrentId(last_id);
        break;

    case PlanType_t::MOVE:
        pub_goal_.publish(plan_->getCurrentGoal());
        break;

    case PlanType_t::TASK:
        pub_goal_.publish(plan_->getCurrentGoal());
        /* Execute task when reached */
        break;

    default:
        break;
    }
}

void Manager::actionResult_call(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg)
{
    if (msg->status.status == actionlib_msgs::GoalStatus::SUCCEEDED) {


        switch (plan_->getCurrentType())
        {
        case PlanType_t::MOVE:
            
            plan_->setCurrentId(last_id + 1);
            
            ROS_INFO("Plan MOVE [%d] successed.", last_id);
            stream_ << "Plan MOVE [" << last_id << "] " << "successed";
            msg_back.data = stream_.str();
            pub_back_.publish(msg_back);
            stream_.str("");

            isBusy = false;
            break;

        case PlanType_t::TASK:
            /* To do Task */
            sleep(20);
            plan_->setCurrentId(last_id + 1);
            
            ROS_INFO("Plan TASK [%d] successed.", last_id);
            
            stream_ << "Plan TASK [" << last_id << "] " << "successed";
            msg_back.data = stream_.str();
            pub_back_.publish(msg_back);
            stream_.str("");

            isBusy = false;
            break;
        default:
            break;
        }
    }
}

void Manager::mapCallback(const nav_msgs::OccupancyGridConstPtr& map)
{
    if (!isSave_map_) 
        return;

    ROS_INFO("Received a %d X %d map @ %.3f m/pix",
             map->info.width,
             map->info.height,
             map->info.resolution);

    std::string mapdatafile = mapname_ + ".pgm";
    ROS_INFO("Writing map occupancy data to %s", mapdatafile.c_str());
    FILE *out = fopen(mapdatafile.c_str(), "w");
    if (!out)
    {
        ROS_ERROR("Couldn't save map file to %s", mapdatafile.c_str());
        return;
    }

    fprintf(out, "P5\n# CREATOR: Map_generator.cpp %.3f m/pix\n%d %d\n255\n",
            map->info.resolution, map->info.width, map->info.height);
    for (unsigned int y = 0; y < map->info.height; y++)
    {
        for (unsigned int x = 0; x < map->info.width; x++)
        {
            unsigned int i = x + (map->info.height - y - 1) * map->info.width;
            if (map->data[i] == 0)
            { // occ [0,0.1)
                fputc(254, out);
            }
            else if (map->data[i] == +100)
            { // occ (0.65,1]
                fputc(000, out);
            }
            else
            { // occ [0.1,0.65]
                fputc(205, out);
            }
        }
    }

    fclose(out);

    std::string mapmetadatafile = mapname_ + ".yaml";
    ROS_INFO("Writing map occupancy data to %s", mapmetadatafile.c_str());
    FILE *yaml = fopen(mapmetadatafile.c_str(), "w");

    /*
resolution: 0.100000
origin: [0.000000, 0.000000, 0.000000]
#
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
     */

    geometry_msgs::Quaternion orientation = map->info.origin.orientation;
    tf::Matrix3x3 mat(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
    double yaw, pitch, roll;
    mat.getEulerYPR(yaw, pitch, roll);

    fprintf(yaml, "image: %s\nresolution: %f\norigin: [%f, %f, %f]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n\n",
            mapdatafile.c_str(), map->info.resolution, map->info.origin.position.x, map->info.origin.position.y, yaw);

    fclose(yaml);

    ROS_INFO("Map Save Done\n");
    stream_ << "Map Save Successed";
    msg_back.data = stream_.str();
    pub_back_.publish(msg_back);
    stream_.str("");

    isSave_map_ = false;
}
