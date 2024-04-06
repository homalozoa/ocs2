#include "ocs2_ros_interfaces/command/target_trajectories_publisher.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

class Node1 : public rclcpp::Node
{
public:
  Node1(const std::string & node_name) : rclcpp::Node(node_name) {}
  void init()
  {
    publisher_ = std::make_unique<ocs2::TargetTrajPublisher>(
      this->weak_from_this(), "prefix_node", rclcpp::QoS(10));
    timer_ =
      this->create_wall_timer(std::chrono::seconds(1), std::bind(&Node1::timer_callback, this));
  }

private:
  void timer_callback()
  {
    ocs2::TargetTrajectories traj(1);
    publisher_->publish_traj(traj);
  }
  std::unique_ptr<ocs2::TargetTrajPublisher> publisher_ = nullptr;
  rclcpp::TimerBase::SharedPtr timer_;
};

class Node2 : public rclcpp_lifecycle::LifecycleNode
{
public:
  Node2(const std::string & node_name) : rclcpp_lifecycle::LifecycleNode(node_name) {}
  void init()
  {
    publisher_ =
      std::make_unique<ocs2::TargetTrajPublisher>(this->weak_from_this(), "prefix_lcnode");
    timer_ = this->create_wall_timer(std::chrono::seconds(1), [this]() -> void {
      ocs2::TargetTrajectories traj(1);
      publisher_->publish_traj(traj);
    });
  }

private:
  std::unique_ptr<ocs2::TargetTrajPublisher> publisher_ = nullptr;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Node1>("test_node");
  auto lcnode = std::make_shared<Node2>("test_lcnode");
  node->init();
  lcnode->init();
  std::cout << "Spinning" << std::endl;
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(node);
  executor->add_node(lcnode->get_node_base_interface());
  executor->spin();
  rclcpp::shutdown();
  return 0;
}
