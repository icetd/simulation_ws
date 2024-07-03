#include "manager.h"
#include <tf/tf.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <functional>
#include "log.h"

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
    delete can_;
}

bool Manager::init()
{
    for (int i = 0; i < OIL_NEEDLE_NUM; i++) {
        isGetOilDone_[i] = false;
        oil_Init_done_[i] = false;
        oil_Wash_done_[i] = false;
        oil_Take_done_[i] = false;
    }
    isArucoEfficient_ = false;
    isAligMentDone_ = false;
    isCancelAlignMentDone_ = false;
    initCan();

    ros::init(argc_, argv_, "manager");
    if (!ros::master::check()) 
        return false;
    
    ros::start();
    TopicSubPub();
    mapname_ = "/home/ubuntu/share/robot_ws/src/robot/map/nav";
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

void Manager::initCan()
{
    can_ = new Can((char *)"can0");
    //can_->Init();
    //can_->SetOnCanReceiveDataCallback(std::bind(&Manager::canCallback, this, std::placeholders::_1));
    //can_->StartAutoRead();
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

    ActionType_t actionType;
    actionType = (ActionType_t)plan->action_id;

    geometry_msgs::PoseStamped poseStamped;
    poseStamped.pose.orientation.w = plan->pose.orientation.w;
    poseStamped.pose.orientation.x = plan->pose.orientation.x;
    poseStamped.pose.orientation.y = plan->pose.orientation.y;
    poseStamped.pose.orientation.z = plan->pose.orientation.z;
    
    poseStamped.pose.position.x = plan->pose.position.x;
    poseStamped.pose.position.y = plan->pose.position.y;
    poseStamped.pose.position.z = plan->pose.position.z;
    poseStamped.header.frame_id = "map";
    
    plan_->add(plan->id, poseStamped, type, actionType);
    
    // 如果是取油任务，设置对准偏移以及取油量
    if (actionType >= 1 && actionType <= 6) {
        oil_needle_capacity_[actionType] = plan->needle_capacity;
        alignment_tank_id[actionType] = plan->alignment_tank_id;
        alignment_offset_x[actionType] = plan->alignment_offset_x;
        alignment_offset_y[actionType] = plan->alignment_offset_y;
    }

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
            stream_ << "Plan [" << last_id << "] " << "START";
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
            switch (plan_->getCurrentAction()) {
            case ActionType_t::NOTING:
                break;

            case ActionType_t::GET_OIL1:
                getOilTask(1);
                break;

            case ActionType_t::GET_OIL2:
                getOilTask(2);
                break;
 
            case ActionType_t::GET_OIL3:
                getOilTask(3);
                break;

            case ActionType_t::GET_OIL4:
                getOilTask(4);
                break;

            case ActionType_t::GET_OIL5:
                getOilTask(5);
                break;

            case ActionType_t::GET_OIL6:
                getOilTask(6);
                break;

            default:
                break;
            }

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

    isSave_map_ = false;
}

void Manager::canCallback(struct can_frame rx_frame)
{
    int32_t pos = 0, step = 0, oilId = 0;
    
    uint8_t id = rx_frame.can_id >> 7;
    uint8_t cmd = rx_frame.can_id & 0x7F;

    switch (cmd) {
    case 0x0A:
        sscanf((char *)rx_frame.data, "I%d", &oilId);
        oil_Init_done_[oilId + 1] = true;        
        break;

    case 0x0B:
        sscanf((char *)rx_frame.data, "W%d", &oilId);
        oil_Wash_done_[oilId + 1] = true;
        break;

    case 0x0C:
        sscanf((char *)rx_frame.data, "T%d", &oilId);
        oil_Take_done_[oilId + 1] = true;
        break;

    case 0x0D:
        if (rx_frame.data[0] == 'A') {
            isAligMentDone_ = true;
            isCancelAlignMentDone_ = false;
            LOG(INFO, "alignment done.");
        } else if (rx_frame.data[0] == 'C') {
            isAligMentDone_ = false;
            isCancelAlignMentDone_ = true;
            LOG(INFO, "cancel alignment done.");
        }
        break;
	
    case 0x1B:
        sscanf((char *)rx_frame.data, "%d", &step);
        if (oil_Init_done_[id - 1] && id <= 7 && id >= 2) {
            oil_value_[id - 1] = - (float)step / 400;
            LOG(INFO, "id%d:%.2f", id - 1, oil_value_[id - 1]);
            
	    stream_ << "OIL [" << id - 1 << "] " << "[" << oil_value_[id - 1] <<"]";
            msg_back.data = stream_.str();
            pub_back_.publish(msg_back);
            stream_.str("");
        }
        break;
    }
}

void Manager::arucoCallback(int32_t x, int32_t y, int32_t z)
{
    x_ = x;
    y_ = y;
    z_ = z;
    isArucoEfficient_ = true;
}

void Manager::getOilTask(int id)
{
    setAlignMentTankId(alignment_tank_id[id]);
    setAlignMentOffsetX(alignment_offset_x[id]);
    setAlignMentOffsetY(alignment_offset_y[id]);
    startAlignMent(id);
}

void Manager::setAlignMentTankId(int32_t tank_id)
{
    struct can_frame tx_frame;
    tx_frame = {
        .can_id = (0x00 << 7) | 0x07,
        .can_dlc = 8};

    sprintf((char *)tx_frame.data, "t%d", tank_id);
    for (int i = 0; i < 3; i++ ) {
        can_->Transmit(&tx_frame);
        sleepMs(100);
    }
}

void Manager::setAlignMentOffsetX(int32_t offset_x)
{
    struct can_frame tx_frame;
    tx_frame = {
        .can_id = (0x00 << 7) | 0x07,
        .can_dlc = 8};

    sprintf((char *)tx_frame.data, "ox%d", offset_x);
    for (int i = 0; i < 3; i++ ) {
        can_->Transmit(&tx_frame);
        sleepMs(100);
    }
}


void Manager::setAlignMentOffsetY(int32_t offset_y)
{
    struct can_frame tx_frame;
    tx_frame = {
        .can_id = (0x00 << 7) | 0x07,
        .can_dlc = 8};

    sprintf((char *)tx_frame.data, "oy%d", offset_y);
    for (int i = 0; i < 3; i++ ) {
        can_->Transmit(&tx_frame);
        sleepMs(100);
    }
}

void Manager::startAlignMent(int id)
{
    detector_ = new Detector((char *)"/home/ubuntu/share/robot_ws/configs/640x480_camera_calibration.yml");
    detector_->setArucoCallback(std::bind(&Manager::arucoCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    detector_->Init("rtsp://127.0.0.1:8554/unicast");
    detector_->start();
    detector_->detach();
    
    isArucoEfficient_ = false;
    isAligMentDone_ = false;
    struct can_frame tx_frame;
    tx_frame = {
        .can_id = (0x00 << 7) | 0x07,
        .can_dlc = 8};

    sprintf((char *)tx_frame.data, "start");
    for (int i = 0; i < 3; i++) {
        can_->Transmit(&tx_frame);
        sleepMs(100);
    }

    do {
        if (isArucoEfficient_) {
            updateAruco(x_, y_, z_);
            isArucoEfficient_ = false;
        }
	sleepMs(10);
    } while (!isAligMentDone_);

    detector_->stop();
    setOilVaule(id, oil_needle_capacity_[id]);
    startTakeOil(id);
}

void Manager::updateAruco(int32_t x, int32_t y, int32_t z)
{
    struct can_frame tx_frame;
    tx_frame = {
        .can_id = (0x00 << 7) | 0x08,
        .can_dlc = 8};

    sprintf((char *)tx_frame.data, "x%d", x);
    can_->Transmit(&tx_frame);
    sleepMs(100);

    sprintf((char *)tx_frame.data, "y%d", y);
    can_->Transmit(&tx_frame);
    sleepMs(100);

    sprintf((char *)tx_frame.data, "z%d", z);
    can_->Transmit(&tx_frame);
    sleepMs(100);
}


void Manager::setOilVaule(uint16_t id, uint8_t value)
{
    struct can_frame tx_frame;

    switch(id) {
    case 1:
        tx_frame = {
            .can_id = (0x00 << 7)| 0x01,
            .can_dlc = 8
        };
        sprintf((char *)tx_frame.data, "s%d", value);
        for (int i = 0; i < 3; i++) {
            can_->Transmit(&tx_frame);
            sleepMs(10);
        }
        break;
    case 2:
        tx_frame = {
            .can_id = (0x00 << 7)| 0x02,
            .can_dlc = 8
        };
        sprintf((char *)tx_frame.data, "s%d", value);
        for (int i = 0; i < 3; i++) {
            can_->Transmit(&tx_frame);
            sleepMs(10);
        }
        break;
    case 3:
        tx_frame = {
            .can_id = (0x00 << 7)| 0x03,
            .can_dlc = 8
        };
        sprintf((char *)tx_frame.data, "s%d", value);
        for (int i = 0; i < 3; i++) {
            can_->Transmit(&tx_frame);
            sleepMs(10);
        }
        break;
    case 4:
        tx_frame = {
            .can_id = (0x00 << 7)| 0x04,
            .can_dlc = 8
        };
        sprintf((char *)tx_frame.data, "s%d", value);
        for (int i = 0; i < 3; i++) {
            can_->Transmit(&tx_frame);
            sleepMs(10);
        }
        break;
    case 5:
        tx_frame = {
            .can_id = (0x00 << 7)| 0x05,
            .can_dlc = 8
        };
        sprintf((char *)tx_frame.data, "s%d", value);
        for (int i = 0; i < 3; i++) {
            can_->Transmit(&tx_frame);
            sleepMs(10);
        }
        break;
    case 6:
        tx_frame = {
            .can_id = (0x00 << 7)| 0x06,
            .can_dlc = 8
        };
        sprintf((char *)tx_frame.data, "s%d", value);
        for (int i = 0; i < 3; i++) {
            can_->Transmit(&tx_frame);
            sleepMs(10);
        }
        break;
    }
}

void Manager::startTakeOil(uint16_t id)
{
    struct can_frame tx_frame;
    tx_frame = {
        .can_id = (0x00 << 7) | 0x09,
        .can_dlc = 8};
	
    oil_Init_done_[id] = false;        
    oil_Wash_done_[id] = false;        
    oil_Take_done_[id] = false;  

    sprintf((char *)tx_frame.data, "oil%d", id);
    for (int i = 0; i < 3; i++) {
        can_->Transmit(&tx_frame);
        sleepMs(10);
    }

    do {
        sleepMs(200);
    } while(!oil_Take_done_[id]);

    startCancelAlignMent(id);
}

void Manager::startCancelAlignMent(int id)
{
    struct can_frame tx_frame;
    tx_frame = {
        .can_id = (0x00 << 7) | 0x07,
        .can_dlc = 8};

    sprintf((char *)tx_frame.data, "cancel");
    for (int i = 0; i < 3; i++) {
        can_->Transmit(&tx_frame);
        sleepMs(10);
    }

    do {
        sleepMs(100);
    } while(!isCancelAlignMentDone_);
}
