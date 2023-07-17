#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
//#include <std_srvs/SetBool.h>
#include <map>
//#include <cmd_mux/ChangeControlSource.h>

class CmdMux
{
public:
    CmdMux(ros::NodeHandle& nh)
        : nh_(nh), current_priority_(0)
    {
        if (!nh_.hasParam("sources")) {
            ROS_ERROR("No sources parameter found");
            return;
        }
        // Load control sources and their priorities from the parameter server
        XmlRpc::XmlRpcValue sources;
        if (!nh_.getParam("sources", sources)) {
        std::string sources_str;
        if (nh_.getParam("sources", sources_str)) {
            ROS_ERROR("Failed to get sources as XmlRpcValue, value was: %s", sources_str.c_str());
            } 
            else {
            ROS_ERROR("Failed to get sources as XmlRpcValue or string");
            }
        return;
        }
        nh.getParam("sources", sources);
        for (int i = 0; i < sources.size(); ++i)
        {
            std::string name = static_cast<std::string>(sources[i]["name"]);
            int priority = static_cast<int>(sources[i]["priority"]);
            sources_[name] = priority;
        }

        // Create subscribers and publisher
        local_sub_ = nh_.subscribe("/cmd_local", 10, &CmdMux::cmdLocalCallback, this);
        web_sub_ = nh_.subscribe("/cmd_web", 10, &CmdMux::cmdWebCallback, this);
        pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        
        //creat sub to change source
        source_change_sub_ = nh_.subscribe("/control_source",10, &CmdMux::sourceChangeCallback, this);

        // Create service
        //source_change_srv_ = nh_.advertiseService("/control_source", &CmdMux::sourceChangeCallback, this);
    }

    void cmdLocalCallback(const geometry_msgs::Twist::ConstPtr& msg)
    {
        int priority = sources_["cmd_local"];
        if (current_source_ == "cmd_local" || priority > current_priority_)
        {
            ROS_INFO("Publishing /cmd_vel from /cmd_local");
            pub_.publish(msg);
            current_source_ = "cmd_local";
            current_priority_ = priority;
        }
        else
        {
            ROS_WARN("Ignoring /cmd_local because current control source is %s", current_source_.c_str());
        }
    }

    void cmdWebCallback(const geometry_msgs::Twist::ConstPtr& msg)
    {
        int priority = sources_["cmd_web"];
        if (current_source_ == "cmd_web" || priority > current_priority_)
        {
            ROS_INFO("Publishing /cmd_vel from /cmd_web");
            pub_.publish(msg);
            current_source_ = "cmd_web";
            current_priority_ = priority;
        }
        else
        {
            ROS_WARN("Ignoring /cmd_web because current control source is %s", current_source_.c_str());
        }
    }

    void sourceChangeCallback(const std_msgs::String::ConstPtr& msg)
    {
        if (sources_.count(msg->data) > 0)
        {
            current_source_ = msg->data;
            current_priority_ = sources_[msg->data];
            ROS_INFO("Control source changed to %s", current_source_.c_str());
        }
        else
        {
            ROS_WARN("Invalid control source: %s", msg->data.c_str());
        }
    }

/*try to created a srv but no function, need to try after
    bool sourceChangeCallback(cmd_mux::ChangeControlSource::Request& req, cmd_mux::ChangeControlSource::Response& res)
    {
        if (sources_.count(req.source) > 0)
        {
            current_source_ = req.source;
            current_priority_ = sources_[req.source;
            res.success = true;
            res.message = "Control source changed to " + req.source;
        }
        else
        {
            res.success = false;
            res.message = "Invalid control source: " + req.source;
        }
        return true;
    }
*/
private:
    ros::NodeHandle nh_;
    ros::Subscriber local_sub_;
    ros::Subscriber web_sub_;
    ros::Subscriber source_change_sub_;
    ros::Publisher pub_;
    //ros::ServiceServer source_change_srv_;
    std::map<std::string, int> sources_;
    std::string current_source_;
    int current_priority_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cmd_mux");
    ros::NodeHandle nh;
    CmdMux cmd_mux(nh);
    ros::spin();
    return 0;
}
