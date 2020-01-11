#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

ros::Publisher vel_pub; // publishes twist message for velocity

ros::Subscriber roll_sub; // subscribes to roll control effort
ros::Subscriber yaw_sub; // subscribes to yaw control effort
ros::Subscriber lat_sub; // subscribes to lateral control effort
ros::Subscriber long_sub; // subscribes to longitudinal control effort
ros::Subscriber vert_sub; // subscribes to vertical control effort

ros::Subscriber roll_enable_sub; // subscribes to roll pid_enable
ros::Subscriber yaw_enable_sub; // subscribes to yaw pid_enable
ros::Subscriber lat_enable_sub; // subscribes to lateral pid_enable
ros::Subscriber long_enable_sub; // subscribes to longitudinal pid_enable
ros::Subscriber vert_enable_sub; // subscribes to vertical pid_enable

ros::Subscriber cmd_vel_sub; // subscribes to cmd_vel

double roll_ctrl(0);
double yaw_ctrl(0);
double lat_ctrl(0);
double long_ctrl(0);
double vert_ctrl(0);

bool roll_enable(false);
bool yaw_enable(false);
bool lat_enable(false);
bool long_enable(false);
bool vert_enable(false);


void rollCallback(const std_msgs::Float64::ConstPtr& data) {
  roll_ctrl = data->data;
  //publish the vector values -> build up command vector message
  geometry_msgs::Twist effortVectors;

  effortVectors.linear.x = lat_ctrl;
  effortVectors.linear.y = long_ctrl;
  effortVectors.linear.z = vert_ctrl;

  effortVectors.angular.x = yaw_ctrl;
  effortVectors.angular.y = roll_ctrl;
  effortVectors.angular.z = 0;

  vel_pub.publish(effortVectors);
}

void yawCallback(const std_msgs::Float64::ConstPtr& data) {
  yaw_ctrl = data->data;
  //publish the vector values -> build up command vector message
  geometry_msgs::Twist effortVectors;

  effortVectors.linear.x = lat_ctrl;
  effortVectors.linear.y = long_ctrl;
  effortVectors.linear.z = vert_ctrl;

  effortVectors.angular.x = yaw_ctrl;
  effortVectors.angular.y = roll_ctrl;
  effortVectors.angular.z = 0;

  vel_pub.publish(effortVectors);
}

void latCallback(const std_msgs::Float64::ConstPtr& data) {
  lat_ctrl = data->data;
  //publish the vector values -> build up command vector message
  geometry_msgs::Twist effortVectors;

  effortVectors.linear.x = lat_ctrl;
  effortVectors.linear.y = long_ctrl;
  effortVectors.linear.z = vert_ctrl;

  effortVectors.angular.x = yaw_ctrl;
  effortVectors.angular.y = roll_ctrl;
  effortVectors.angular.z = 0;

  vel_pub.publish(effortVectors);
}

void longCallback(const std_msgs::Float64::ConstPtr& data) {
  long_ctrl = data->data;
  //publish the vector values -> build up command vector message
  geometry_msgs::Twist effortVectors;

  effortVectors.linear.x = lat_ctrl;
  effortVectors.linear.y = long_ctrl;
  effortVectors.linear.z = vert_ctrl;

  effortVectors.angular.x = yaw_ctrl;
  effortVectors.angular.y = roll_ctrl;
  effortVectors.angular.z = 0;

  vel_pub.publish(effortVectors);
}

void vertCallback(const std_msgs::Float64::ConstPtr& data) {
  vert_ctrl = data->data;
  //publish the vector values -> build up command vector message
  geometry_msgs::Twist effortVectors;

  effortVectors.linear.x = lat_ctrl;
  effortVectors.linear.y = long_ctrl;
  effortVectors.linear.z = vert_ctrl;

  effortVectors.angular.x = yaw_ctrl;
  effortVectors.angular.y = roll_ctrl;
  effortVectors.angular.z = 0;

  vel_pub.publish(effortVectors);
}

void rollToggle(const std_msgs::Bool::ConstPtr& data) {
  roll_enable = data->data;
}

void yawToggle(const std_msgs::Bool::ConstPtr& data) {
  yaw_enable = data->data;
}

void latToggle(const std_msgs::Bool::ConstPtr& data) {
  lat_enable = data->data;
}

void longToggle(const std_msgs::Bool::ConstPtr& data) {
  long_enable = data->data;
}

void vertToggle(const std_msgs::Bool::ConstPtr& data) {
  vert_enable = data->data;
}

void cmdCallback(const geometry_msgs::Twist::ConstPtr& data) {
  if (!roll_enable) {
    roll_ctrl = data->angular.y;
  }
  if (!yaw_enable) {
    roll_ctrl = data->angular.x;
  }
  if (!lat_enable) {
    roll_ctrl = data->linear.x;
  }
  if (!long_enable) {
    roll_ctrl = data->linear.y;
  }
  if (!vert_enable) {
    roll_ctrl = data->linear.z;
  }
  if (!(roll_enable && yaw_enable && lat_enable && long_enable && vert_enable)){
    // publish the vector values -> build up command vector message
    // only publish if some pids are actually disabled
    geometry_msgs::Twist effortVectors;

    effortVectors.linear.x = lat_ctrl;
    effortVectors.linear.y = long_ctrl;
    effortVectors.linear.z = vert_ctrl;

    effortVectors.angular.x = yaw_ctrl;
    effortVectors.angular.y = roll_ctrl;
    effortVectors.angular.z = 0;

    vel_pub.publish(effortVectors);
  }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "effort_pub");

    ros::NodeHandle n;

    roll_sub = n.subscribe<std_msgs::Float64>("roll_motion/control_effort", 1, &rollCallback);
    yaw_sub = n.subscribe<std_msgs::Float64>("yaw_motion/control_effort", 1, &yawCallback);
    lat_sub = n.subscribe<std_msgs::Float64>("lat_motion/control_effort", 1, &latCallback);
    long_sub = n.subscribe<std_msgs::Float64>("long_motion/control_effort", 1, &longCallback);
    vert_sub = n.subscribe<std_msgs::Float64>("vert_motion/control_effort", 1, &vertCallback);

    vel_pub = n.advertise<geometry_msgs::Twist>("rov/eff_vel", 1);

    ros::spin();
  }
