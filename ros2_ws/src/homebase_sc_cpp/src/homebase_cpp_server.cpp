#include <functional>
#include <memory>
#include <thread>
#include <vector>

#include "homebase_action/action/Homebase.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

// #include "mutac_msgs/msg/Identifier.hpp"
// #include "mutac_msgs/msg/Label.hpp"
// #include "mutac_msgs/msg/LabeledPoint.hpp"
// #include "mutac_msgs/msg/LabeledPath.hpp"
// #include "mutac_msgs/msg/Plan.hpp"
// #include "mutac_msgs/msg/Label.hpp"
// #include "mutac_msgs/msg/Metrics.hpp"
#include "geometry_msgs/msg/Point.hpp"

namespace hbcpp
{

class HomebaseActionServer : public rclcpp::Node
{
public:
    using Homebase = homebase_action::action::Homebase;
    using GoalHandleHomebase = rclcpp_action::ServerGoalHandle<Homebase>;

    HOMEBASE_SC_CPP_PUBLIC
    explicit HomebaseActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) :
        Node("homebase_action_server", options)
    {
        using namespace std::placeholders;

        this->action_server_ = rclcpp_action::create_server<Homebase>(
            this,
            "Homebase"
            std::bind(&HomebaseActionServer::handle_goal, this, _1, _2),
            std::bind(&HomebaseActionServer::handle_cancel, this, _1),
            std::bind(&HomebaseActionServer::handle_accepted, this, _1));

        for(int i = 0; i < 1; i++){
            desired_points.push_back(geometry_msgs::Point(0,0,0));
            drone_points.push_back(geometry_msgs::Point(0,0,0));
            distancias.push_back(0);
        }
    }

private:

    rclcpp_action::Server<Homebase>::SharedPtr action_server_;
    std::vector<float> distancias;
    std::vector<geometry_msgs::Point> desired_points;
    std::vector<geometry_msgs::Point> drone_points;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const Homebase::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Peticion de goal con orden %d ", goal->order);
        (void) uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancle(
        const std::shared_ptr<GoalHandleHomebase> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Petición para cancelar la goal");
        (void) goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleHomebase> goal_handle)
    {
        using namespace std::placeholders;
        std::thread{std::bind(&FibonacciActionServer::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleHomebase> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Ejecutando orden");
        const auto goal = goal_handle->get_goal();

        auto result = std::make_shared<Homebase::Result>();
    
        while(!canceled(goal_handle) && !drones_arrived() && rclcpp::ok())
        {
            publish_feedback_msg();
        }

        if(rclcpp::ok()){
            result->result = "Drones en base";
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal completado");
        }

    }
    
    bool canceled(const std::shared_ptr<GoalHandleHomebase> goal_handle, 
                const std::shared_pointer<Homebase::Result> res)
    {
        if (goal_handle->is_canceling())
        {
            res->result = "Goal cancelado";
            goal_handle->canceled(res);
            RCLCPP_INFO(this->get_logger(), "Goal cancelado");
            //TODO: publicar ultima posicion conocida del dron


            return;
        }
    }


    void publish_feedback_msg(const std::shared_ptr<GoalHandleHomebase> goal_handle)
    {
        auto feedback = std::make_shared<Homebase::Feedback>();
        float[drone_points.size()] dist;
        for(int i = 0; i < drone_points.size(); i++){
            dist[i] = distancias[i]; 
        }
        feedback->distance = dist;
        goal_handle->publish_feedback(feedback);
    }



    bool drones_arrived()
    {
        bool drones_arrived = true;
        for(int i = 0; i < desired_points.size(); i++)
        {
            drones_arrived &= (desired_points[i] == drone_points[i])
        }
    }

    bool operator==(const geometry_msgs::Point & p1, const geometry_msgs::Point & p2) 
    {
        return (p1->x == p2->x) && (p1->y == p2-> y) && (p1->z == p2->z);
    }


};

}