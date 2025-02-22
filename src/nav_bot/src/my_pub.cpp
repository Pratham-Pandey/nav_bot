#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
//#include "rclcpp_components/register_node_macro.hpp"

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std;


class NavigateTo : public rclcpp::Node{
  public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    explicit NavigateTo(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) : Node("navigate_to", options){
      std::cout<<"Initializing....\n";
      RCLCPP_ERROR(this->get_logger(), "Initializing....(logger)");
      // Take Goal Input
      user_input();
      // goal_x =4.6;
      // goal_y = 0.0;
      // goal_orient_z = 1.0;

      this->client_ptr_ = rclcpp_action::create_client<NavigateToPose>(this, "/navigate_to_pose");
      
      send_goal();
    }

    void send_goal(){
      using namespace std::placeholders;

      if (!this->client_ptr_->wait_for_action_server()) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        //rclcpp::shutdown();
        return;
      }
      
      // auto goal = NavigateToPose::Goal();
      NavigateToPose::Goal goal_pose = NavigateToPose::Goal();

      goal_pose.pose.header.frame_id ="map";
      
      goal_pose.pose.pose.position.x = goal_x;
      goal_pose.pose.pose.position.y = goal_y;
      goal_pose.pose.pose.orientation.w = goal_orient_z;
      goal_pose.behavior_tree = ""; 

      RCLCPP_INFO(this->get_logger(), "Sending goal");

      auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

      send_goal_options.goal_response_callback = std::bind(&NavigateTo::goal_response_callback, this, _1);
      send_goal_options.feedback_callback = std::bind(&NavigateTo::feedback_callback, this, _1, _2);
      send_goal_options.result_callback = std::bind(&NavigateTo::result_callback, this, _1);

      this->client_ptr_->async_send_goal(goal_pose, send_goal_options);  
      
      //
      // goal_msg.order = order

      // self._action_client.wait_for_server()
      // send_goal_future = self._action_client.send_goal_async(goal_msg)
      // rclpy.spin_until_future_complete(self, send_goal_future)
      
      // goal_handle = send_goal_future.result()
      // if not goal_handle.accepted:
      //     self.get_logger().info('Goal rejected')
      //     return
  
      // self.get_logger().info('Goal accepted')
      // result_future = goal_handle.get_result_async()
      // rclpy.spin_until_future_complete(self, result_future)
    }

    void goal_response_callback(const GoalHandleNavigateToPose::SharedPtr & goal_handle){
      if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
      } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
      }    
    }

    void feedback_callback(GoalHandleNavigateToPose::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback){
      std::cout<<"Feedback:\n";
      std::cout<<"\tNumber of Recoveries: "<< feedback->number_of_recoveries<<"\n";
    }

    void result_callback(const GoalHandleNavigateToPose::WrappedResult & result){
      std::cout<<"Result:\n";
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          std::cout<<"\tGoal Reached!\n";
          break;
        case rclcpp_action::ResultCode::ABORTED:
          std::cout<<"\tABORTED\n";
          break;
        case rclcpp_action::ResultCode::CANCELED:
          std::cout<<"\tCANCLED\n";
          break;
        default:
          RCLCPP_ERROR(this->get_logger(), "\tUnknown result code");
      }
      user_input();    
      send_goal();
    }

  private:

    void user_input(){
      // Take Goal Input
      cout<<"Enter Goal Coordinates:\n";
      cout << "\tEnter X Coordinate: ";
      cin >> goal_x;
      cout << "\tEnter Y Coordinate: ";
      cin >> goal_y;
      cout << "\tEnter Orientation(Z): ";
      cin >> goal_orient_z;
    }

    float goal_x;
    float goal_y;
    float goal_orient_z;

    rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;

};

int main(int argc, char ** argv){
  rclcpp::init(argc, argv);
  auto action_client = std::make_shared<NavigateTo>();
  rclcpp::spin(action_client);
  rclcpp::shutdown();

  return 0;
}