#include <functional>
#include <memory>
#include <thread>
#include "pallet_loader/comms.h"
#include "prototype/action/lift.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

/*#include "action_tutorials_cpp/visibility_control.h"*/

namespace pallet_loader
{
class PalletActionServer : public rclcpp::Node
{
public:
  using Fibonacci = prototype::action::Lift;
  using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;


 // ACTION_TUTORIALS_CPP_PUBLIC
  explicit PalletActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("pallet_action_server", options)
  {
    using namespace std::placeholders;
    RCLCPP_INFO(this->get_logger(), "Received goal request with order");
    comms_.setup("/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_75834353031351417031-if00", 115200, 10000)
    this->action_server_ = rclcpp_action::create_server<Fibonacci>(
      this,
      "Pallet_Loader",
      std::bind(&PalletActionServer::handle_goal, this, _1, _2),
      std::bind(&PalletActionServer::handle_cancel, this, _1),
      std::bind(&PalletActionServer::handle_accepted, this, _1));
    
  }

private:

  Comms comms_;
  rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Fibonacci::Goal> goal)
  {
    //RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->lift);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&PalletActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Fibonacci::Feedback>();
    auto & sequence = feedback->status;
    sequence.push_back(0);
    sequence.push_back(1);
    auto result = std::make_shared<Fibonacci::Result>();
    /*
    for (int i = 1; (i < goal->lift) && rclcpp::ok(); ++i) {
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->done = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }
      // Update sequence
      sequence.push_back(sequence[i] + sequence[i - 1]);
      // Publish feedback
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish feedback");

      loop_rate.sleep();
    }*/

   comms_.read_imu_values();

    // Check if goal is done
    if (rclcpp::ok()) {
      result->done = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }
};  // class PalletActionServer

}  // namespace pallet_loader

RCLCPP_COMPONENTS_REGISTER_NODE(pallet_loader::PalletActionServer)