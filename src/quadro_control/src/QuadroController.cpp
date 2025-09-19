#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <control_msgs/action/follow_joint_trajectory.hpp>

std::shared_ptr<rclcpp::Node> node;
bool common_goal_accepted = false;
rclcpp_action::ResultCode common_resultcode = rclcpp_action::ResultCode::UNKNOWN;
int common_action_result_code = control_msgs::action::FollowJointTrajectory_Result::SUCCESSFUL;

// Define desired goals for all 12 joints
std::vector<std::vector<double>> desired_goals = {
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},  // Initial position
    {1, 2, 1, 1, 2, 1, 1, 2, 1, 1, 2, 1},  // First movement
    {0.2, 0.3, 0.2, -0.2, 0.3, 0.2, 0.2, -0.3, 0.2, -0.2, 0.3, 0.2},  // Second movement
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}   // Return to initial
};

unsigned int ct_goals_reached = 0;

void common_goal_response(
  rclcpp_action::ClientGoalHandle
  <control_msgs::action::FollowJointTrajectory>::SharedPtr future)
{
  RCLCPP_DEBUG(
    node->get_logger(), "common_goal_response time: %f",
    rclcpp::Clock().now().seconds());
  auto goal_handle = future.get();
  if (!goal_handle) {
    common_goal_accepted = false;
    printf("Goal rejected\n");
  } else {
    common_goal_accepted = true;
    printf("Goal accepted\n");
  }
}

void common_result_response(
  const rclcpp_action::ClientGoalHandle
  <control_msgs::action::FollowJointTrajectory>::WrappedResult & result)
{
  printf("common_result_response time: %f\n", rclcpp::Clock(RCL_ROS_TIME).now().seconds());
  common_resultcode = result.code;
  common_action_result_code = result.result->error_code;
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      printf("Action goal succeeded\n");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      printf("Action goal was aborted\n");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      printf("Action goal was canceled\n");
      return;
    default:
      printf("Action goal: Unknown result code\n");
      return;
  }
}

void common_feedback(
  rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr,
  const std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Feedback> feedback)
{
  // Check if all joints are close to their desired positions
  if (ct_goals_reached < desired_goals.size()) {
    bool goal_reached = true;
    for (size_t i = 0; i < feedback->actual.positions.size(); ++i) {
      if (std::fabs(feedback->actual.positions[i] - desired_goals[ct_goals_reached][i]) > 0.05) {
        goal_reached = false;
        break;
      }
    }
    
    if (goal_reached) {
      std::cout << "Goal # " << ct_goals_reached << " reached" << std::endl;
      ct_goals_reached++;
      if (ct_goals_reached < desired_goals.size()) {
        std::cout << "Moving to next goal # " << ct_goals_reached << std::endl;
      }
    }
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  node = std::make_shared<rclcpp::Node>("trajectory_test_node");

  RCLCPP_DEBUG(node->get_logger(), "node created");

  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr action_client;
  action_client = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
    node->get_node_base_interface(),
    node->get_node_graph_interface(),
    node->get_node_logging_interface(),
    node->get_node_waitables_interface(),
    "/joint_trajectory_controller/follow_joint_trajectory");

  while (true) {
    bool response =
      action_client->wait_for_action_server(std::chrono::seconds(1));
    if (!response) {
      using namespace std::chrono_literals;
      std::this_thread::sleep_for(2000ms);
      RCLCPP_WARN(node->get_logger(), "Trying to connect to the server again");
      continue;
    } else {
      break;
    }
  }

  RCLCPP_DEBUG(node->get_logger(), "Created action server");

  // Define all 12 joint names
  std::vector<std::string> joint_names = {
    "Base-Body_Revolute-41",
    "Actuator-1_Revolute-61",
    "upper-leg-1_Revolute-68",
    "Base-Body_Revolute-62",
    "Actuator-2_Revolute-63",
    "upper-leg-2_Revolute-69",
    "Base-Body_Revolute-64",
    "Actuator-3_Revolute-65",
    "upper-leg-3_Revolute-70",
    "Base-Body_Revolute-66",
    "Actuator-4_Revolute-67",
    "upper-leg-4_Revolute-71"
  };

  // Create trajectory points
  std::vector<trajectory_msgs::msg::JointTrajectoryPoint> points;
  
  for (size_t i = 0; i < desired_goals.size(); ++i) {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.time_from_start = rclcpp::Duration::from_seconds(2.0 * (i + 1));
    point.positions = desired_goals[i];
    points.push_back(point);
  }

  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions opt;
  opt.goal_response_callback = std::bind(common_goal_response, std::placeholders::_1);
  opt.result_callback = std::bind(common_result_response, std::placeholders::_1);
  opt.feedback_callback = std::bind(common_feedback, std::placeholders::_1, std::placeholders::_2);

  control_msgs::action::FollowJointTrajectory_Goal goal_msg;
  goal_msg.goal_time_tolerance = rclcpp::Duration::from_seconds(2.0);
  goal_msg.trajectory.joint_names = joint_names;
  goal_msg.trajectory.points = points;

  RCLCPP_DEBUG(node->get_logger(), "async_send_goal");
  auto goal_handle_future = action_client->async_send_goal(goal_msg, opt);

  if (rclcpp::spin_until_future_complete(node, goal_handle_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "send goal call failed :(");
    action_client.reset();
    node.reset();
    return 1;
  }
  RCLCPP_INFO(node->get_logger(), "send goal call ok :)");

  rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr
    goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node->get_logger(), "Goal was rejected by server");
    action_client.reset();
    node.reset();
    return 1;
  }
  RCLCPP_INFO(node->get_logger(), "Goal was accepted by server");

  // Wait for the server to be done with the goal
  auto result_future = action_client->async_get_result(goal_handle);
  RCLCPP_INFO(node->get_logger(), "Waiting for result");
  if (rclcpp::spin_until_future_complete(node, result_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "get result call failed :(");
    action_client.reset();
    node.reset();
    return 1;
  }
  action_client.reset();

  if (desired_goals.size() != ct_goals_reached) {
    RCLCPP_ERROR(node->get_logger(), "Not all the goals were reached");
    rclcpp::shutdown();
    return -1;
  }

  node.reset();
  rclcpp::shutdown();

  return 0;
}