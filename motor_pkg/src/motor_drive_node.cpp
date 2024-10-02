/*
 * motor_node.cpp
 *
 *      Author: Chis Chun
 */
#include <motor_pkg/motor_node.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <cmath>
#include <chrono>

double wheel_base = 0.4236;          // m
double wheel_radius_teleop = 0.0575; // m
bool direction_flag1 = true;
bool direction_flag2 = true;
double topic_rpm_value1 = 0.0;
double topic_rpm_value2 = 0.0;

// pid parameter
#define TIME 0.01

double pidControl1 = 0.0;
double p_gain1 = 0.01;
double i_gain1 = 0.42;
double d_gain1 = 0.01;
// double p_gain1 = 0.0;
// double i_gain1 = 0.0;
// double d_gain1 = 0.0;
double errorGap1 = 0.0;
double realError1 = 0.0;
double accError1 = 0.0;
double pControl1 = 0.0;
double iControl1 = 0.0;
double dControl1 = 0.0;

double pidControl2 = 0.0;
double p_gain2 = 0.01;
double i_gain2 = 0.42;
double d_gain2 = 0.01;
// double p_gain2 = 0.0;
// double i_gain2 = 0.0;
// double d_gain2 = 0.0;
double errorGap2 = 0.0;
double realError2 = 0.0;
double accError2 = 0.0;
double pControl2 = 0.0;
double iControl2 = 0.0;
double dControl2 = 0.0;

// odometry parameter
double now_left_wheel_pose_rad = 0.0;  // rad
double now_right_wheel_pose_rad = 0.0; // rad
double now_left_wheel_pose = 0.0;      // m = r*theta
double now_right_wheel_pose = 0.0;     // m
double x_ = 0.0;                       //   [m]
double y_ = 0.0;                       //   [m]
double heading_ = 0.0;                 // [rad]

double left_vel = 0.0;  // m/s
double right_vel = 0.0; // m/s

double delta_distance = 0.0;
double delta_theta = 0.0;

double delta_linear = 0.0;
double delta_angular = 0.0;

// Current velocity:
double linear_ = 0.0;  //   [m/s]
double angular_ = 0.0; // [rad/s]

// Previous wheel position/state [rad]:
double left_wheel_old_pos_ = 0.0;
double right_wheel_old_pos_ = 0.0;

double dt = 0.0001;
double pulse_1 = 0.0;
double pulse_2 = 0.0;

double target_rpm1 = 0.0;
double target_rpm2 = 0.0;

double lpf_value_ = 0.0;
double prev_lpf_value_ = 0.0;
double lambda = 0.5;


double to_see_target1 =0.0;
double to_see_target2 =0.0;


void pidControlSystem(double &errorGap, double &realError, double &accError, double &pControl, double &iControl, double &dControl, double &target, double &current, double &p_gain, double &i_gain, double &d_gain, double &pidControl)
{
  errorGap = target - current - realError;
  realError = target - current;
  accError += realError;

  pControl = p_gain * realError;
  // iControl = i_gain * (accError * TIME);
  // dControl = d_gain * (errorGap / TIME);
  iControl = i_gain * accError;
  dControl = d_gain * errorGap;



  pidControl = pControl + iControl + dControl;

  // lpf_value_ = (1 - lambda)*prev_lpf_value_ + lambda*pidControl;
  // prev_lpf_value_ = lpf_value_;
  // pidControl = lpf_value_;
}

void LoadParameters(void)
{
  std::ifstream inFile("/home/ubuntu/robot_ws/src/motor_pkg/data/motor_input.txt");
  if (!inFile.is_open())
  {
    RCLCPP_ERROR(rclcpp::get_logger("motor_node"), "Unable to open the file");
    return;
  }

  int i = 0;
  std::size_t found;
  for (std::string line; std::getline(inFile, line);)
  {
    found = line.find("=");

    switch (i)
    {
    case 0:
      pwm_range = atof(line.substr(found + 2).c_str());
      break;
    case 1:
      pwm_frequency = atof(line.substr(found + 2).c_str());
      break;
    case 2:
      pwm_limit = atof(line.substr(found + 2).c_str());
      break;
    case 3:
      control_cycle = atof(line.substr(found + 2).c_str());
      break;
    case 4:
      acceleration_ratio = atof(line.substr(found + 2).c_str());
      break;
    case 5:
      wheel_radius = atof(line.substr(found + 2).c_str());
      break;
    case 6:
      robot_radius = atof(line.substr(found + 2).c_str());
      break;
    case 7:
      encoder_resolution = atof(line.substr(found + 2).c_str());
      break;
      // case :  = atof(line.substr(found+2).c_str()); break;
    }
    i += 1;
  }
  inFile.close();
}

int InitMotors(void)
{
  pinum = pigpio_start(NULL, NULL);

  if (pinum < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("motor_node"), "Setup failed");
    RCLCPP_ERROR(rclcpp::get_logger("motor_node"), "pinum is %d", pinum);
    return 1;
  }

  set_mode(pinum, motor1_dir, PI_OUTPUT);
  set_mode(pinum, motor2_dir, PI_OUTPUT);
  set_mode(pinum, motor1_pwm, PI_OUTPUT);
  set_mode(pinum, motor2_pwm, PI_OUTPUT);
  set_mode(pinum, motor1_encA, PI_INPUT);
  set_mode(pinum, motor1_encB, PI_INPUT);
  set_mode(pinum, motor2_encA, PI_INPUT);
  set_mode(pinum, motor2_encB, PI_INPUT);

  gpio_write(pinum, motor1_dir, PI_LOW);
  gpio_write(pinum, motor2_dir, PI_LOW);

  set_PWM_range(pinum, motor1_pwm, pwm_range);
  set_PWM_range(pinum, motor2_pwm, pwm_range);
  set_PWM_frequency(pinum, motor1_pwm, pwm_frequency);
  set_PWM_frequency(pinum, motor2_pwm, pwm_frequency);
  set_PWM_dutycycle(pinum, motor1_pwm, 0);
  set_PWM_dutycycle(pinum, motor1_pwm, 0);

  set_pull_up_down(pinum, motor1_encA, PI_PUD_DOWN);
  set_pull_up_down(pinum, motor1_encB, PI_PUD_DOWN);
  set_pull_up_down(pinum, motor2_encA, PI_PUD_DOWN);
  set_pull_up_down(pinum, motor2_encB, PI_PUD_DOWN);

  current_pwm1 = 0;
  current_pwm2 = 0;

  current_direction1 = true;
  current_direction2 = true;

  acceleration = pwm_limit / (acceleration_ratio);

  RCLCPP_INFO(rclcpp::get_logger("motor_node"), "Setup Fin");
  return 0;
}

void SetInterrupts(void)
{
  callback(pinum, motor1_encA, EITHER_EDGE, Interrupt1A);
  callback(pinum, motor1_encB, EITHER_EDGE, Interrupt1B);
  callback(pinum, motor2_encA, EITHER_EDGE, Interrupt2A);
  callback(pinum, motor2_encB, EITHER_EDGE, Interrupt2B);
}

void Interrupt1A(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{
  (void)pi;
  (void)user_gpio;
  (void)level;
  (void)tick;
  if (gpio_read(pinum, motor1_dir) == true)
    encoder_count_1A--;
  else
    encoder_count_1A++;
  speed_count_1++;
}

void Interrupt1B(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{
  (void)pi;
  (void)user_gpio;
  (void)level;
  (void)tick;
  if (gpio_read(pinum, motor1_dir) == true)
    encoder_count_1B--;
  else
    encoder_count_1B++;
  speed_count_1++;
}

void Interrupt2A(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{
  (void)pi;
  (void)user_gpio;
  (void)level;
  (void)tick;
  if (gpio_read(pinum, motor2_dir) == true)
    encoder_count_2A--;
  else
    encoder_count_2A++;  
    // printf("RPM1 : %10.0f    ||  RPM2 : %10.0f\n", rpm_value1, rpm_value2);

  speed_count2++;
}

void Interrupt2B(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{
  (void)pi;
  (void)user_gpio;
  (void)level;
  (void)tick;
  if (gpio_read(pinum, motor2_dir) == true)
    encoder_count_2B--;
  else
    encoder_count_2B++;
  speed_count2++;
}

int SumMotor1Encoder()
{
  encoder_count_1 = encoder_count_1A + encoder_count_1B;

  return encoder_count_1;
}

int SumMotor2Encoder()
{
  encoder_count_2 = encoder_count_2A + encoder_count_2B;

  return encoder_count_2;
}

void InitEncoders(void)
{
  encoder_count_1 = 0;
  encoder_count_2 = 0;
  encoder_count_1A = 0;
  encoder_count_1B = 0;
  encoder_count_2A = 0;
  encoder_count_2B = 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
void Initialize(void)
{
  LoadParameters();
  InitMotors();
  InitEncoders();
  SetInterrupts();

  wheel_round = 2 * M_PI * wheel_radius;
  robot_round = 2 * M_PI * robot_radius;

  switch_direction = true;
  theta_distance_flag = 0;

  RCLCPP_INFO(rclcpp::get_logger("motor_node"), "pwm_range %d", pwm_range);
  RCLCPP_INFO(rclcpp::get_logger("motor_node"), "pwm_frequency %d", pwm_frequency);
  RCLCPP_INFO(rclcpp::get_logger("motor_node"), "pwm_limit %d", pwm_limit);
  RCLCPP_INFO(rclcpp::get_logger("motor_node"), "control_cycle %f", control_cycle);
  RCLCPP_INFO(rclcpp::get_logger("motor_node"), "acceleration_ratio %d", acceleration_ratio);
  RCLCPP_INFO(rclcpp::get_logger("motor_node"), "Initialize Complete");

  printf("\033[2J");
}

void MotorController(int motor_num, bool direction, int pwm)
{
  int local_pwm = LimitPwm(pwm);

  if (motor_num == 1)
  {
    if (direction == true)
    {
      gpio_write(pinum, motor1_dir, PI_LOW);
      set_PWM_dutycycle(pinum, motor1_pwm, local_pwm);
      current_pwm1 = local_pwm;
      current_direction1 = true;
    }
    else if (direction == false)
    {
      gpio_write(pinum, motor1_dir, PI_HIGH);
      set_PWM_dutycycle(pinum, motor1_pwm, local_pwm);
      current_pwm1 = local_pwm;
      current_direction1 = false;
    }
  }

  else if (motor_num == 2)
  {
    if (direction == true)
    {
      gpio_write(pinum, motor2_dir, PI_LOW);
      set_PWM_dutycycle(pinum, motor2_pwm, local_pwm);
      current_pwm2 = local_pwm;
      current_direction2 = true;
    }
    else if (direction == false)
    {
      gpio_write(pinum, motor2_dir, PI_HIGH);
      set_PWM_dutycycle(pinum, motor2_pwm, local_pwm);
      current_pwm2 = local_pwm;
      current_direction2 = false;
    }
  }
}
void AccelController(int motor_num, bool direction, int desired_pwm)
{
  bool local_current_direction;
  int local_pwm;
  int local_current_pwm;

  if (motor_num == 1)
  {
    local_current_direction = current_direction1;
    local_current_pwm = current_pwm1;
  }
  else if (motor_num == 2)
  {
    local_current_direction = current_direction2;
    local_current_pwm = current_pwm2;
  }

  if (direction == local_current_direction)
  {
    if (desired_pwm > local_current_pwm)
    {
      local_pwm = local_current_pwm + acceleration;
      MotorController(motor_num, direction, local_pwm);
    }
    else if (desired_pwm < local_current_pwm)
    {
      local_pwm = local_current_pwm - acceleration;
      MotorController(motor_num, direction, local_pwm);
    }
    else
    {
      local_pwm = local_current_pwm;
      MotorController(motor_num, direction, local_pwm);
    }
  }
  else
  {
    if (desired_pwm >= 0)
    {
      local_pwm = local_current_pwm - acceleration;
      if (local_pwm <= 0)
      {
        local_pwm = 0;
        MotorController(motor_num, direction, local_pwm);
      }
      else
        MotorController(motor_num, local_current_direction, local_pwm);
    }
    else
    {
      local_pwm = local_current_pwm;
      MotorController(motor_num, direction, local_pwm);
    }
  }
}

int LimitPwm(int pwm)
{
  int output;
  if (pwm > pwm_limit * 2)
  {
    output = pwm_limit;
    RCLCPP_WARN(rclcpp::get_logger("motor_node"), "pwm too fast!!!");
  }
  else if (pwm > pwm_limit)
    output = pwm_limit;
  else if (pwm < 0)
  {
    output = 0;
    RCLCPP_WARN(rclcpp::get_logger("motor_node"), "trash value!!!");
  }
  else
    output = pwm;
  return output;
}

void CalculateRpm()
{
  rpm_value1 = (speed_count_1 * (60 * control_cycle)) / (encoder_resolution * 4);
  speed_count_1 = 0;

  rpm_value2 = (speed_count2 * (60 * control_cycle)) / (encoder_resolution * 4);
  speed_count2 = 0;
}

void InfoMotors()
{
  CalculateRpm();
  printf("\033[2J");
  printf("\033[1;1H");
  printf("Encoder1A : %5d    ||  Encoder2A : %5d\n", encoder_count_1A, encoder_count_2A);
  printf("Encoder1B : %5d    ||  Encoder2B : %5d\n", encoder_count_1B, encoder_count_2B);
  printf("RPM1 : %10.0f    ||  RPM2 : %10.0f\n", rpm_value1, rpm_value2);
  printf("PWM1 : %10.0d    ||  PWM2 : %10.0d\n", current_pwm1, current_pwm2);
  printf("DIR1 :%11s    ||  DIR2 :%11s\n", current_direction1 ? "CW" : "CCW", current_direction2 ? "CW" : "CCW");
  printf("x : %10.2f    ||  y : %10.2f    ||  heading : %10.2f\n", x_, y_, heading_);
  printf("ACC  :%11.0d\n", acceleration);
  // printf("target_RPM1 : %10.2f    ||  target_RPM2 : %10.2f\n", to_see_target1, to_see_target2);
  printf("\n");

  if (current_direction1 == true)
    topic_rpm_value1 = rpm_value1;
  else
    topic_rpm_value1 = -1 * rpm_value1;
  if (current_direction2 == true)
    topic_rpm_value2 = rpm_value2;
  else
    topic_rpm_value2 = -1 * rpm_value2;
}

void Calculate_Odom()
{
  pulse_1 = SumMotor1Encoder();
  pulse_2 = SumMotor2Encoder();

  now_left_wheel_pose_rad = (pulse_1 * 2 * M_PI) / (4 * encoder_resolution);
  now_left_wheel_pose = wheel_radius_teleop * now_left_wheel_pose_rad;

  now_right_wheel_pose_rad = (pulse_2 * 2 * M_PI) / (4 * encoder_resolution);
  now_right_wheel_pose = wheel_radius_teleop * now_right_wheel_pose_rad;

  left_vel = (now_left_wheel_pose - left_wheel_old_pos_);
  right_vel = (now_right_wheel_pose - right_wheel_old_pos_);

  left_wheel_old_pos_ = now_left_wheel_pose;
  right_wheel_old_pos_ = now_right_wheel_pose;

  delta_distance = (left_vel + right_vel) / 2.0;
  delta_theta = (right_vel - left_vel) / wheel_base;

  x_ += delta_distance * cos(heading_);
  y_ += delta_distance * sin(heading_);
  heading_ += delta_theta;

  delta_linear = delta_distance / dt;
  delta_angular = delta_theta / dt;
}

class MotorDrive : public rclcpp::Node
{
public:
  MotorDrive();
  double target_pwm1 = 0.0;
  double target_pwm2 = 0.0;
  double linear_velocity = 0.0;
  double angular_velocity = 0.0;
  double left_wheel_velocity = 0.0;
  double right_wheel_velocity = 0.0;
  double left_wheel_rpm = 0.0;
  double right_wheel_rpm = 0.0;
  double left_wheel_pwm = 0.0;
  double right_wheel_pwm = 0.0;
  bool target_dir1 = true;
  bool target_dir2 = true;

private:
  void TimerCallback();
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void pidGainCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void Publish_Odom();

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_pid_check_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_pid_gain_;
  rclcpp::Time last_update_time = rclcpp::Clock{}.now(); // 현재 시각 받아와서 이전시각 초기화
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

MotorDrive::MotorDrive() : Node("motor_control_node")
{
  timer_ = this->create_wall_timer(
      10ms, std::bind(&MotorDrive::TimerCallback, this));
  sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel_control", 10, std::bind(&MotorDrive::cmdVelCallback, this, std::placeholders::_1));
  odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::SensorDataQoS());
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  pub_pid_check_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("rpm", 10);
  sub_pid_gain_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("pid_gain", 10,
                                                                              std::bind(&MotorDrive::pidGainCallback, this, std::placeholders::_1));
}

void MotorDrive::TimerCallback()
{
  dt = (rclcpp::Clock{}.now() - last_update_time).seconds(); // 현재 시각과 이전 시각 차를 빼서 시간 변화량을 초 단위로 환산
  last_update_time = rclcpp::Clock{}.now();                  // 이전 시각 업데이트
  InfoMotors();
  Calculate_Odom();
  Publish_Odom();

  pidControlSystem(errorGap1, realError1, accError1, pControl1, iControl1, dControl1, left_wheel_rpm, topic_rpm_value1, p_gain1, i_gain1, d_gain1, pidControl1);
  target_pwm1 = (pidControl1 / 90) * 150;

  pidControlSystem(errorGap2, realError2, accError2, pControl2, iControl2, dControl2, right_wheel_rpm, topic_rpm_value2, p_gain2, i_gain2, d_gain2, pidControl2);
  target_pwm2 = (pidControl2 / 90) * 150;


  // target_pwm1 = 0.0;
  // target_pwm2 = 0.0;

  



  if (target_dir1 == true)
  {
    direction_flag1 = true;
    target_pwm1 = abs(target_pwm1);
  }
  else if (target_dir1 == false)
  {
    direction_flag1 = false;
    target_pwm1 = abs(target_pwm1);
  }

  if (target_dir2 == true)
  {
    direction_flag2 = true;
    target_pwm2 = abs(target_pwm2);
  }
  else if (target_dir2 == false)
  {
    direction_flag2 = false;
    target_pwm2 = abs(target_pwm2);
  }

  MotorController(1, direction_flag1, target_pwm1);
  MotorController(2, direction_flag2, target_pwm2);

  // pid확인을 위한 topic pub
  std_msgs::msg::Float64MultiArray msg;
  msg.data = {topic_rpm_value1, topic_rpm_value2, left_wheel_rpm, right_wheel_rpm};
  pub_pid_check_->publish(msg);
}

void MotorDrive::Publish_Odom()
{
  auto odom_msg = std::make_shared<nav_msgs::msg::Odometry>();

  odom_msg->header.stamp = this->now();
  odom_msg->header.frame_id = "odom";
  odom_msg->child_frame_id = "base_footprint";

  // x, y 위치 정보 할당
  odom_msg->pose.pose.position.x = x_;
  odom_msg->pose.pose.position.y = y_;
  odom_msg->pose.pose.position.z = 0.0;

  // heading을 quaternion으로 변환하여 방향 정보 할당
  tf2::Quaternion q;
  q.setRPY(0, 0, heading_);
  odom_msg->pose.pose.orientation = tf2::toMsg(q);

  // 선형 속도와 각속도 할당
  odom_msg->twist.twist.linear.x = delta_linear;
  odom_msg->twist.twist.linear.y = 0.0;
  odom_msg->twist.twist.angular.z = delta_angular;

  odom_publisher_->publish(*odom_msg);

  auto tf = geometry_msgs::msg::TransformStamped();
  tf.header.stamp = this->now();
  tf.header.frame_id = "odom";
  tf.child_frame_id = "base_footprint";
  tf.transform.translation.x = odom_msg->pose.pose.position.x;
  tf.transform.translation.y = odom_msg->pose.pose.position.y;
  tf.transform.translation.z = odom_msg->pose.pose.position.z;
  tf.transform.rotation.x = odom_msg->pose.pose.orientation.x;
  tf.transform.rotation.y = odom_msg->pose.pose.orientation.y;
  tf.transform.rotation.z = odom_msg->pose.pose.orientation.z;
  tf.transform.rotation.w = odom_msg->pose.pose.orientation.w;

  tf_broadcaster_->sendTransform(tf); // add this line
}

void MotorDrive::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  linear_velocity = msg->linear.x;
  angular_velocity = msg->angular.z;
  left_wheel_velocity = linear_velocity - (angular_velocity * wheel_base / 2);
  right_wheel_velocity = linear_velocity + (angular_velocity * wheel_base / 2);
  left_wheel_rpm = (left_wheel_velocity / (2 * M_PI * wheel_radius_teleop)) * 60;   // target_rpm1
  right_wheel_rpm = (right_wheel_velocity / (2 * M_PI * wheel_radius_teleop)) * 60; // target_rpm2
  left_wheel_pwm = (left_wheel_rpm / 90) * 150;
  right_wheel_pwm = (right_wheel_rpm / 90) * 150;
  target_pwm1 = left_wheel_pwm;
  target_pwm2 = right_wheel_pwm;

///////////////////////////////////////////여기
  to_see_target1 = left_wheel_rpm;
  to_see_target2 = right_wheel_rpm;


  if (target_pwm1 > 0)
    target_dir1 = true;
  if (target_pwm1 < 0)
    target_dir1 = false;

  if (target_pwm2 > 0)
    target_dir2 = true;
  if (target_pwm2 < 0)
    target_dir2 = false;

  // std::cout << "--------------------------------------------------" << std::endl;
  // std::cout << "target_pwm1: " << target_pwm1 << std::endl;
  // std::cout << "target_pwm2: " << target_pwm2 << std::endl;
}

void MotorDrive::pidGainCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (msg->data.size() != 6)
  {
    RCLCPP_ERROR(this->get_logger(), "Received data size is not correct for pid_gain");
    return;
  }
  // target_rpm1 = msg->data[0];
  // target_rpm1 = msg->data[1];

  p_gain1 = msg->data[0];
  i_gain1 = msg->data[1];
  d_gain1 = msg->data[2];
  p_gain2 = msg->data[3];
  i_gain2 = msg->data[4];
  d_gain2 = msg->data[5];
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  Initialize();
  rclcpp::spin(std::make_shared<MotorDrive>());

  rclcpp::shutdown();
  MotorController(1, true, 0);
  MotorController(2, true, 0);
  return 0;
}
