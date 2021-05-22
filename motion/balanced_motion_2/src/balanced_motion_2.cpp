#include "balanced_motion_2/balanced_motion_2.hpp"

BalancedMotion2::BalancedMotion2() : Node("BalancedMotion2")
{
  transitioner.addMotion(&walk);
  transitioner.addMotion(&stand);
  transitioner.addMotion(&dive);
  transitioner.addMotion(&getup);

  currentMotion = &stand;

  State state;
  state.standing = true;
  aim = currentMotionAimState = state;

  sub_joint_states =
    create_subscription<nao_interfaces::msg::Joints>(
    "sensors/joints", 1,
    [this](nao_interfaces::msg::Joints::SharedPtr sensor_joints) {
      nao_interfaces::msg::Joints command = make_joints(*sensor_joints);
      pub_joints->publish(command);
    });

  sub_action =
    create_subscription<motion_msgs::msg::Action>(
    "action", 1,
    [this](motion_msgs::msg::Action::SharedPtr action) {
      aim = evaluate_aim(*action);
    });


  pub_joints = this->create_publisher<nao_interfaces::msg::Joints>("effectors/joints", 10);
}

nao_interfaces::msg::Joints BalancedMotion2::make_joints(nao_interfaces::msg::Joints & sensor_joints)
{
  // Evaluate current state
  State current = evaluate_current_state();

  auto [motion, motion_aim_state] = transitioner.findNextMotion(current, aim);
  if (motion != nullptr)
  {
    std::cout << "transitioning to: " << motion->name << std::endl;
    currentMotion->reset();
    currentMotion = motion;
    currentMotionAimState = motion_aim_state;
  }

  return currentMotion->makeJoints(currentMotionAimState);
}

State BalancedMotion2::evaluate_current_state()
{
  State state(true);
  state.standing = (currentMotion == &stand);
  state.walking = (currentMotion == &walk);
  state.diving = (currentMotion == &dive && !dive.hasFinished());
  state.on_ground = (currentMotion == &dive && dive.hasFinished());  // Or, use gyroscope or something here
  state.travelling = (currentMotion == &walk && walk.isTravelling());
  state.getting_up = (currentMotion == &getup && !getup.hasFinished());
  state.on_feet = (currentMotion == &walk || currentMotion == &stand ||
    (currentMotion == &getup && getup.hasFinished()));
  return state;
}

State BalancedMotion2::evaluate_aim(motion_msgs::msg::Action action)
{
  State state;
  if (action.action == motion_msgs::msg::Action::ACTIONWALK)
  {
    state.walking = true;
  } else if (action.action == motion_msgs::msg::Action::ACTIONSTAND) {
    state.standing = true;
  } else if (action.action == motion_msgs::msg::Action::ACTIONDIVE) {
    state.diving = true;
  } else {
    RCLCPP_ERROR(get_logger(), "Unknown action type requested: " + std::to_string(action.action));
  }
  return state;
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BalancedMotion2>());
  rclcpp::shutdown();
  return 0;
}