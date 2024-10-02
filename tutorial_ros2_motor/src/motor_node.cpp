/*
 * motor_node.cpp
 *
 *      Author: Chis Chun
 */
#include <tutorial_ros2_motor/motor_node.hpp>

#include <cmath>
#include <chrono>


// odometry parameter
double wheel_base = 0.4236;          // m
double wheel_radius_teleop = 0.0575; // m
double now_left_wheel_pose_rad = 0.0;  // rad
double now_right_wheel_pose_rad = 0.0; // rad
double now_left_wheel_pose = 0.0;      // m = r*theta
double now_right_wheel_pose = 0.0;     // m
double x_ = 0.0;                       //   [m]
double y_ = 0.0;                       //   [m]
double heading_ = 0.0;                 // [rad]

bool is_leftA_high = false;
bool is_leftB_high = false;

bool is_rightA_high = false;
bool is_rightB_high = false;


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

int i = 0; // interval


//PID Kwon
// pid parameter
#define TIME 0.01

double pidControl1 = 0.0;
double p_gain1 = 1.00;
double i_gain1 = 0.01;
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
double p_gain2 = 1.00;
double i_gain2 = 0.01;
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

double r_target_rpm = 0.0;
double l_target_rpm = 0.0;
int r_target_dir = 0;
int l_target_dir = 0;

int tmp_dir1 = 0;
int tmp_dir2 = 0;

double smooth_rpm_r = 0.0;
double smooth_rpm_l = 0.0;

void LoadParameters(void)
{
  std::ifstream inFile("/home/ubuntu/robot_ws/src/tutorial_ros2_motor/data/motor_input.txt");
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
  callback(pinum, motor1_encA, RISING_EDGE, Interrupt1A);
  callback(pinum, motor1_encB, RISING_EDGE, Interrupt1B);
  callback(pinum, motor2_encA, RISING_EDGE, Interrupt2A);
  callback(pinum, motor2_encB, RISING_EDGE, Interrupt2B);

  callback(pinum, motor1_encA, FALLING_EDGE, Interrupt1Af);
  callback(pinum, motor1_encB, FALLING_EDGE, Interrupt1Bf);
  callback(pinum, motor2_encA, FALLING_EDGE, Interrupt2Af);
  callback(pinum, motor2_encB, FALLING_EDGE, Interrupt2Bf);
}

// Interrupt part
void Interrupt1A(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{
  (void)pi;
  (void)user_gpio;
  (void)level;
  (void)tick;
  is_rightA_high = true;
  if (is_rightB_high){
    encoder_count_1A--;
    speed_count_1--;
  }else{
    encoder_count_1A++;
    speed_count_1++;
  }
}

void Interrupt1B(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{
  (void)pi;
  (void)user_gpio;
  (void)level;
  (void)tick;
  is_rightB_high = true;
  if (is_rightA_high){
    encoder_count_1B++;
    speed_count_1++;
  }else{
    encoder_count_1B--;
    speed_count_1--;
  }
}

void Interrupt2A(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{
  (void)pi;
  (void)user_gpio;
  (void)level;
  (void)tick;
  is_leftA_high = true;
  if (is_leftB_high){
    encoder_count_2A--;
    speed_count2--;
  }else{
    encoder_count_2A++;
    speed_count2++;
  }
}

void Interrupt2B(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{
  (void)pi;
  (void)user_gpio;
  (void)level;
  (void)tick;
  is_leftB_high = true;
  if (is_leftA_high){
    encoder_count_2B++;
    speed_count2++;
  }else{
    encoder_count_2B--;
    speed_count2--;
  }  
}

void Interrupt1Af(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{
  (void)pi;
  (void)user_gpio;
  (void)level;
  (void)tick;
  is_rightA_high = false;
  if (is_rightB_high){
    encoder_count_1A++;
    speed_count_1++;
  }else{
    encoder_count_1A--;
    speed_count_1--;
  }
}

void Interrupt1Bf(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{
  (void)pi;
  (void)user_gpio;
  (void)level;
  (void)tick;
  is_rightB_high = false;
  if (is_rightA_high){
    encoder_count_1B--;
    speed_count_1--;
  }
  else{
    encoder_count_1B++;
    speed_count_1++;
  }
}

void Interrupt2Af(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{
  (void)pi;
  (void)user_gpio;
  (void)level;
  (void)tick;
  is_leftA_high = false;
  if (is_leftB_high){
    encoder_count_2A++;
    speed_count2++;
  }
  else{
    encoder_count_2A--;
    speed_count2--;
  }
  
}

void Interrupt2Bf(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{
  (void)pi;
  (void)user_gpio;
  (void)level;
  (void)tick;
  is_leftB_high = false;
  if (is_leftA_high){
    encoder_count_2B--;
    speed_count2--;
  }
  else{
    encoder_count_2B++;
    speed_count2++;
  }
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

  wheel_round = 2 * PI * wheel_radius;
  robot_round = 2 * PI * robot_radius;

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
  if (motor_num == 1)
  {
    pwm = (int)pwm;
    direction = true;
    if (pwm < 0)
    {
      direction = false;
      pwm = -pwm;
    }
    int local_pwm = LimitPwm(pwm);
    if (direction == false)
    {
      gpio_write(pinum, motor1_dir, PI_HIGH);
      set_PWM_dutycycle(pinum, motor1_pwm, local_pwm);
      current_pwm1 = local_pwm;
      current_direction1 = false;
    }
    else if (direction == true)
    {
      gpio_write(pinum, motor1_dir, PI_LOW);
      set_PWM_dutycycle(pinum, motor1_pwm, local_pwm);
      current_pwm1 = local_pwm;
      current_direction1 = true;
    }
  }

  else if (motor_num == 2)
  {
    pwm = (int)pwm;
    direction = false;
    if (pwm < 0)
    {
      direction = true;
      pwm = -pwm;
    }
    int local_pwm = LimitPwm(pwm);
    if (direction == false)
    {
      gpio_write(pinum, motor2_dir, PI_LOW);
      set_PWM_dutycycle(pinum, motor2_pwm, local_pwm);
      current_pwm2 = local_pwm;
      current_direction2 = false;
    }
    else if (direction == true)
    {
      gpio_write(pinum, motor2_dir, PI_HIGH);
      set_PWM_dutycycle(pinum, motor2_pwm, local_pwm);
      current_pwm2 = local_pwm;
      current_direction2 = true;
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

void pidControlSystem(double &prevError, double &realError, double &accError, double &pControl, double &iControl, double &dControl, double &target, double &current, double &p_gain, double &i_gain, double &d_gain, double &pidControl)
{
    realError = target - current;
    accError += realError * TIME;  // Integral accumulates over time
    double errorDelta = (realError - prevError) / TIME;  // Change in error over time

    pControl = p_gain * realError;
    iControl = i_gain * accError;
    dControl = d_gain * errorDelta;

    pidControl = (int)(pControl + iControl + dControl);

    prevError = realError;  // Update previous error for next iteration
}




void SwitchTurn(int pwm1, int pwm2)
{
  int local_pwm1 = LimitPwm(pwm1);
  int local_pwm2 = LimitPwm(pwm2);
  if (switch_direction == true)
  {
    MotorController(1, switch_direction, local_pwm1);
    MotorController(2, switch_direction, local_pwm2);
    switch_direction = false;
    RCLCPP_INFO(rclcpp::get_logger("motor_node"), "true");
  }
  else
  {
    MotorController(1, switch_direction, local_pwm1);
    MotorController(2, switch_direction, local_pwm2);
    switch_direction = true;
    RCLCPP_INFO(rclcpp::get_logger("motor_node"), "false");
  }
}

void ThetaTurn(double theta, int pwm)
{
  double local_encoder;
  int local_pwm = LimitPwm(pwm);
  if (theta_distance_flag == 1)
  {
    InitEncoders();
    theta_distance_flag = 2;
  }
  SumMotor1Encoder();
  SumMotor2Encoder();
  if (theta > 0)
  {
    local_encoder = (encoder_resolution * 4 / 360) * (robot_round / wheel_round) * theta;
    MotorController(1, true, local_pwm);
    MotorController(2, true, local_pwm);
    // AccelController(1, true, local_pwm);
    // AccelController(2, true, local_pwm);
  }
  else
  {
    local_encoder = -(encoder_resolution * 4 / 360) * (robot_round / wheel_round) * theta;
    MotorController(1, false, local_pwm);
    MotorController(2, false, local_pwm);
    // AccelController(1, false, local_pwm);
    // AccelController(2, false, local_pwm);
  }

  if (encoder_count_1 > local_encoder)
  {
    InitEncoders();
    MotorController(1, true, 0);
    MotorController(2, true, 0);
    theta_distance_flag = 3;
  }
}

void DistanceGo(double distance, int pwm)
{
  double local_encoder = (encoder_resolution * 4 * distance) / wheel_round;
  int local_pwm = LimitPwm(pwm);
  bool direction = true;
  if (distance < 0)
  {
    direction = false;
    local_encoder = -local_encoder;
  }
  if (theta_distance_flag == 3)
  {
    InitEncoders();
    theta_distance_flag = 4;
  }
  SumMotor1Encoder();
  SumMotor2Encoder();
  if (encoder_count_1 < local_encoder)
  {
    if (direction == true)
    {
      MotorController(1, true, local_pwm);
      MotorController(2, false, local_pwm);
      // AccelController(1, true, local_pwm);
      // AccelController(2, false, local_pwm);
    }
    else
    {
      MotorController(1, false, local_pwm);
      MotorController(2, true, local_pwm);
      // AccelController(1, false, local_pwm);
      // AccelController(2, true, local_pwm);
    }
  }
  else
  {
    InitEncoders();
    MotorController(1, true, 0);
    MotorController(2, true, 0);
    // AccelController(1, true, 0);
    // AccelController(2, true, 0);
    theta_distance_flag = 0;
  }
}

void ThetaTurnDistanceGo(double theta, int turn_pwm, double distance, int go_pwm)
{
  if (theta_distance_flag == 0)
  {
    theta_distance_flag = 1;
  }
  else if (theta_distance_flag == 1 || theta_distance_flag == 2)
  {
    ThetaTurn(theta, turn_pwm);
  }
  else if (theta_distance_flag == 3 || theta_distance_flag == 4)
  {
    DistanceGo(distance, go_pwm);
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
  if (i == 4){
    rpm_value1 = (speed_count_1 * (60 * control_cycle / 5)) / (encoder_resolution * 4);
    speed_count_1 = 0;
    rpm_value2 = (speed_count2  * (60 * control_cycle / 5)) / (encoder_resolution * 4);
    speed_count2 =  0;
    i = 0;
  }
  i += 1;
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
void InfoMotors()
{
  CalculateRpm();
  printf("\033[2J");
  printf("\033[1;1H");
  printf("Encoder1A : %5d    ||  Encoder2A : %5d\n", encoder_count_1A, encoder_count_2A);
  printf("Encoder1B : %5d    ||  Encoder2B : %5d\n", encoder_count_1B, encoder_count_2B);
  printf("RPM1 : %f    ||  RPM2 : %f\n", rpm_value1, rpm_value2);
  printf("target RPM1 : %f    ||  target RPM2 : %f\n", r_target_rpm, l_target_rpm);
  printf("PWM1 : %10.0d    ||  PWM2 : %10.0d\n", current_pwm1, current_pwm2);
  printf("target PWM1 : %10.0d    ||  target PWM2 : %10.0d\n", pidControl1, pidControl2);
  printf("x_ : %10f    ||  y_ : %10f\n", x_, y_);
  printf("delta_theta : %10f    ||  delta_distance : %10f\n", delta_theta, delta_distance);
  printf("heading_ : %10f    " ,heading_);
  printf("delta_linear : %10f    ||  delta_angular : %10f\n", delta_linear, delta_angular);
  printf("DIR1 :%11s    ||  DIR2 :%11s\n", current_direction1 ? "CW" : "CCW", current_direction2 ? "CW" : "CCW");
  printf("ACC  :%11.0d\n", acceleration);
  printf("error gap : %f, %f", errorGap1, errorGap2);
  printf("\n");
  printf("//////////////////////////////////////////////////");
  
}

RosCommunicator::RosCommunicator()
    : Node("tutorial_ros2_motor"), count_(0)
{
  timer_ = this->create_wall_timer(
      10ms, std::bind(&RosCommunicator::TimerCallback, this));
  subscription_ = this->create_subscription<std_msgs::msg::Int64MultiArray>(
      "/tutorial/teleop", 10, std::bind(&RosCommunicator::TeleopCallback, this, _1));
}

void RosCommunicator::TimerCallback()
{
  int target_pwm1, target_pwm2;
  // MotorController(1, true, 100);
  // MotorController(2, true, 100);
  // AccelController(1, true, 100);
  // AccelController(2, true, 100);
  // SwitchTurn(100, 100);
  // ThetaTurnDistanceGo(180,100,30,110);
  InfoMotors();
  Calculate_Odom();
  pidControlSystem(errorGap1, realError1, accError1, pControl1, iControl1, dControl1, l_target_rpm, rpm_value1, p_gain1, i_gain1, d_gain1, pidControl1);

  pidControlSystem(errorGap2, realError2, accError2, pControl2, iControl2, dControl2, r_target_rpm, rpm_value2, p_gain2, i_gain2, d_gain2, pidControl2);

  MotorController(1, tmp_dir2, pidControl2);
  MotorController(2, tmp_dir1, pidControl1);

}

void RosCommunicator::TeleopCallback(const std_msgs::msg::Int64MultiArray::SharedPtr msg)
{
  
  if (msg->data[0] == 0)
    tmp_dir1 = true;
  else
    tmp_dir1 = false;
  if (msg->data[1] == 0)
    tmp_dir2 = true;
  else
    tmp_dir2 = false;
  r_target_dir = tmp_dir1;
  l_target_dir = tmp_dir2;
  r_target_rpm = msg->data[0];
  l_target_rpm = msg->data[1];
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  Initialize();
  rclcpp::spin(std::make_shared<RosCommunicator>());

  rclcpp::shutdown();
  MotorController(1, true, 0);
  MotorController(2, true, 0);
  return 0;
}
