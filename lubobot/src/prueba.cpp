#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/UInt16.h>
#include <tf/transform_datatypes.h>
#include <util.h>

#include <boost/numeric/ublas/matrix.hpp>
#include <sstream>

static const float axleLength = 0.235;
static const float wheelDiameter = 0.072;

static const double MAX_DBL = std::numeric_limits<double>::max();

static const double COVARIANCE[36] = {
    1e-5, 1e-5, 0.0,     0.0, 0.0,  1e-5,  // NOLINT(whitespace/braces)
    1e-5, 1e-5, 0.0,     0.0, 0.0,  1e-5,    0.0, 0.0, MAX_DBL, 0.0,
    0.0,  0.0,  0.0,     0.0, 0.0,  MAX_DBL, 0.0, 0.0, 0.0,     0.0,
    0.0,  0.0,  MAX_DBL, 0.0, 1e-5, 1e-5,    0.0, 0.0, 0.0,     1e-5};

/**
 * \brief Represents a robot pose.
 */
struct Pose {
  float x;
  float y;
  float yaw;

  /**
   * \brief 3x3 covariance matrix in row-major order.
   */
  std::vector<float> covariance;
};

typedef Pose Vel;
typedef unsigned long long timestamp_t;
typedef boost::numeric::ublas::matrix<float> Matrix;

Pose pose;
Vel vel;


uint32_t prevTicksLeft;
uint32_t prevTicksRight;
float totalLeftDist;
float totalRightDist;
bool firstOnData;
timestamp_t prevOnDataTime;

Matrix poseCovar;

float measuredLeftVel;
float measuredRightVel;
float requestedLeftVel;
float requestedRightVel;

uint16_t lwheel = 0;
uint16_t rwheel = 0;

std::string base_frame_;
std::string odom_frame_;

nav_msgs::Odometry odom_msg_;

ros::Publisher odom_pub_;

Matrix addMatrices(const Matrix& A, const Matrix& B) {
  size_t rows = A.size1();
  size_t cols = A.size2();

  assert(rows == B.size1());
  assert(cols == B.size2());

  Matrix C(rows, cols);
  for (size_t i = 0u; i < rows; i++) {
    for (size_t j = 0u; j < cols; j++) {
      const float a = A(i, j);
      const float b = B(i, j);
      if (util::willFloatOverflow(a, b)) {
        // If overflow, set to float min or max depending on direction of
        // overflow
        C(i, j) = (a < 0.0) ? std::numeric_limits<float>::min()
                            : std::numeric_limits<float>::max();
      } else {
        C(i, j) = a + b;
      }
    }
  }
  return C;
}

void onData() {
  if (firstOnData) {
    prevTicksLeft = lwheel;
    prevTicksRight = rwheel;
    prevOnDataTime = util::getTimestamp();
    firstOnData = false;
  }

  // Get current time
  util::timestamp_t curTime = util::getTimestamp();
  float dt = (curTime - prevOnDataTime) / 1000000.0;
  float deltaDist, deltaX, deltaY, deltaYaw, leftWheelDist, rightWheelDist,
      wheelDistDiff;

  // Get cumulative ticks (wraps around at 65535)
  uint16_t totalTicksLeft = lwheel;
  uint16_t totalTicksRight = rwheel;
  // Compute ticks since last update
  int ticksLeft = totalTicksLeft - prevTicksLeft;
  int ticksRight = totalTicksRight - prevTicksRight;
  prevTicksLeft = totalTicksLeft;
  prevTicksRight = totalTicksRight;

  // Handle wrap around
  if (fabs(ticksLeft) > 0.9 * util::V_3_MAX_ENCODER_TICKS) {
    ticksLeft = (ticksLeft % util::V_3_MAX_ENCODER_TICKS) + 1;
  }
  if (fabs(ticksRight) > 0.9 * util::V_3_MAX_ENCODER_TICKS) {
    ticksRight = (ticksRight % util::V_3_MAX_ENCODER_TICKS) + 1;
  }

  // Compute distance travelled by each wheel
  leftWheelDist =
      (ticksLeft / util::V_3_TICKS_PER_REV) * wheelDiameter * util::PI;
  rightWheelDist =
      (ticksRight / util::V_3_TICKS_PER_REV) * wheelDiameter * util::PI;
  deltaDist = (rightWheelDist + leftWheelDist) / 2.0;

  wheelDistDiff = rightWheelDist - leftWheelDist;
  deltaYaw = wheelDistDiff / axleLength;

  measuredLeftVel = leftWheelDist / dt;
  measuredRightVel = rightWheelDist / dt;

  // Moving straight
  if (fabs(wheelDistDiff) < util::EPS) {
    deltaX = deltaDist * cos(pose.yaw);
    deltaY = deltaDist * sin(pose.yaw);
  } else {
    float turnRadius =
        (axleLength / 2.0) * (leftWheelDist + rightWheelDist) / wheelDistDiff;
    deltaX = turnRadius * (sin(pose.yaw + deltaYaw) - sin(pose.yaw));
    deltaY = -turnRadius * (cos(pose.yaw + deltaYaw) - cos(pose.yaw));
  }

  totalLeftDist += leftWheelDist;
  totalRightDist += rightWheelDist;

  if (fabs(dt) > util::EPS) {
    vel.x = deltaDist / dt;
    vel.y = 0.0;
    vel.yaw = deltaYaw / dt;
  } else {
    vel.x = 0.0;
    vel.y = 0.0;
    vel.yaw = 0.0;
  }

  // Update covariances
  // Ref: "Introduction to Autonomous Mobile Robots" (Siegwart 2004, page 189)
  float kr = 1.0;  // TODO: Perform experiments to find these nondeterministic
                   // parameters
  float kl = 1.0;
  float cosYawAndHalfDelta = cos(pose.yaw + (deltaYaw / 2.0));  // deltaX?
  float sinYawAndHalfDelta = sin(pose.yaw + (deltaYaw / 2.0));  // deltaY?
  float distOverTwoWB = deltaDist / (axleLength * 2.0);

  Matrix invCovar(2, 2);
  invCovar(0, 0) = kr * fabs(rightWheelDist);
  invCovar(0, 1) = 0.0;
  invCovar(1, 0) = 0.0;
  invCovar(1, 1) = kl * fabs(leftWheelDist);

  Matrix Finc(3, 2);
  Finc(0, 0) =
      (cosYawAndHalfDelta / 2.0) - (distOverTwoWB * sinYawAndHalfDelta);
  Finc(0, 1) =
      (cosYawAndHalfDelta / 2.0) + (distOverTwoWB * sinYawAndHalfDelta);
  Finc(1, 0) =
      (sinYawAndHalfDelta / 2.0) + (distOverTwoWB * cosYawAndHalfDelta);
  Finc(1, 1) =
      (sinYawAndHalfDelta / 2.0) - (distOverTwoWB * cosYawAndHalfDelta);
  Finc(2, 0) = (1.0 / axleLength);
  Finc(2, 1) = (-1.0 / axleLength);
  Matrix FincT = boost::numeric::ublas::trans(Finc);

  Matrix Fp(3, 3);
  Fp(0, 0) = 1.0;
  Fp(0, 1) = 0.0;
  Fp(0, 2) = (-deltaDist) * sinYawAndHalfDelta;
  Fp(1, 0) = 0.0;
  Fp(1, 1) = 1.0;
  Fp(1, 2) = deltaDist * cosYawAndHalfDelta;
  Fp(2, 0) = 0.0;
  Fp(2, 1) = 0.0;
  Fp(2, 2) = 1.0;
  Matrix FpT = boost::numeric::ublas::trans(Fp);

  Matrix velCovar = boost::numeric::ublas::prod(invCovar, FincT);
  velCovar = boost::numeric::ublas::prod(Finc, velCovar);

  vel.covariance[0] = velCovar(0, 0);
  vel.covariance[1] = velCovar(0, 1);
  vel.covariance[2] = velCovar(0, 2);
  vel.covariance[3] = velCovar(1, 0);
  vel.covariance[4] = velCovar(1, 1);
  vel.covariance[5] = velCovar(1, 2);
  vel.covariance[6] = velCovar(2, 0);
  vel.covariance[7] = velCovar(2, 1);
  vel.covariance[8] = velCovar(2, 2);

  Matrix poseCovarTmp = boost::numeric::ublas::prod(poseCovar, FpT);
  poseCovarTmp = boost::numeric::ublas::prod(Fp, poseCovarTmp);
  poseCovar = addMatrices(poseCovarTmp, velCovar);

  pose.covariance[0] = poseCovar(0, 0);
  pose.covariance[1] = poseCovar(0, 1);
  pose.covariance[2] = poseCovar(0, 2);
  pose.covariance[3] = poseCovar(1, 0);
  pose.covariance[4] = poseCovar(1, 1);
  pose.covariance[5] = poseCovar(1, 2);
  pose.covariance[6] = poseCovar(2, 0);
  pose.covariance[7] = poseCovar(2, 1);
  pose.covariance[8] = poseCovar(2, 2);

  // Update pose
  pose.x += deltaX;
  pose.y += deltaY;
  pose.yaw = util::normalizeAngle(pose.yaw + deltaYaw);

  prevOnDataTime = curTime;
}

void publishOdom() {
  Pose pose = pose;
  Vel vel = vel;

  // Populate position info
  geometry_msgs::Quaternion quat =
      tf::createQuaternionMsgFromRollPitchYaw(0, 0, pose.yaw);
  odom_msg_.header.stamp = ros::Time::now();
  odom_msg_.pose.pose.position.x = pose.x;
  odom_msg_.pose.pose.position.y = pose.y;
  odom_msg_.pose.pose.orientation = quat;

  // Populate velocity info
  odom_msg_.twist.twist.linear.x = vel.x;
  odom_msg_.twist.twist.linear.y = vel.y;
  odom_msg_.twist.twist.angular.z = vel.yaw;

  // Update covariances
  odom_msg_.pose.covariance[0] = static_cast<double>(pose.covariance[0]);
  odom_msg_.pose.covariance[1] = pose.covariance[1];
  odom_msg_.pose.covariance[5] = pose.covariance[2];
  odom_msg_.pose.covariance[6] = pose.covariance[3];
  odom_msg_.pose.covariance[7] = pose.covariance[4];
  odom_msg_.pose.covariance[11] = pose.covariance[5];
  odom_msg_.pose.covariance[30] = pose.covariance[6];
  odom_msg_.pose.covariance[31] = pose.covariance[7];
  odom_msg_.pose.covariance[35] = pose.covariance[8];
  odom_msg_.twist.covariance[0] = vel.covariance[0];
  odom_msg_.twist.covariance[1] = vel.covariance[1];
  odom_msg_.twist.covariance[5] = vel.covariance[2];
  odom_msg_.twist.covariance[6] = vel.covariance[3];
  odom_msg_.twist.covariance[7] = vel.covariance[4];
  odom_msg_.twist.covariance[11] = vel.covariance[5];
  odom_msg_.twist.covariance[30] = vel.covariance[6];
  odom_msg_.twist.covariance[31] = vel.covariance[7];
  odom_msg_.twist.covariance[35] = vel.covariance[8];

  //   if (publish_tf_)
  //   {
  //     tf_odom_.header.stamp = ros::Time::now();
  //     tf_odom_.transform.translation.x = pose.x;
  //     tf_odom_.transform.translation.y = pose.y;
  //     tf_odom_.transform.rotation = quat;
  //     tf_broadcaster_.sendTransform(tf_odom_);
  //   }

  odom_pub_.publish(odom_msg_);
}

void LeftWheelCallback(const std_msgs::UInt16 msg) {
  lwheel = msg.data;
}

void RightWheelCallback(const std_msgs::UInt16 msg) {
  rwheel = msg.data;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Subscriber lwheel_sub = n.subscribe("lwheel", 100, LeftWheelCallback);
  ros::Subscriber rwheel_sub = n.subscribe("rwheel", 100, RightWheelCallback);
  n.param<std::string>("base_frame", base_frame_, "base_footprint");
  n.param<std::string>("odom_frame", odom_frame_, "odom");
  odom_pub_ = n.advertise<nav_msgs::Odometry>("odom", 30);
  ros::Rate loop_rate(10);
  int count = 0;

  // Set frame_id's
  //   tf_odom_.header.frame_id = odom_frame_;
  //   tf_odom_.child_frame_id = base_frame_;
  odom_msg_.header.frame_id = odom_frame_;
  odom_msg_.child_frame_id = base_frame_;
  //   joint_state_msg_.name.resize(2);
  //   joint_state_msg_.position.resize(2);
  //   joint_state_msg_.velocity.resize(2);
  //   joint_state_msg_.effort.resize(2);
  //   joint_state_msg_.name[0] = "left_wheel_joint";
  //   joint_state_msg_.name[1] = "right_wheel_joint";

  // Populate intial covariances

  pose.covariance.resize(32);
  vel.covariance.resize(32);

  for (int i = 0; i < 36; i++) {
    odom_msg_.pose.covariance[i] = COVARIANCE[i];
    odom_msg_.twist.covariance[i] = COVARIANCE[i];
  }

  firstOnData = true;

  while (ros::ok()) {
    onData();
    publishOdom();
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  return 0;
}