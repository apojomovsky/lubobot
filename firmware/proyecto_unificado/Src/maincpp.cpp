#include <cmsis_os.h>
#include <main.h>
#include <ros.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <I2Cdev.h>

#ifdef __cplusplus
#include <arm_math.h>
#include <vector>
extern "C" {
#endif

#include <maincpp.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <nav_msgs/Odometry.h>
//#include <geometry_msgs/TransformStamped.h>
#include <create2.h>
#include <MPU6050.h>

using namespace irobot;

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

Pose pose;
Vel vel;

nav_msgs::Odometry odom_msg;

ros::Publisher odom_pub("odom", &odom_msg);

//geometry_msgs::TransformStamped t;
//tf::TransformBroadcaster tfBroadcaster;

ros::NodeHandle nh;

extern UART_HandleTypeDef huart5;

irobot::create2 robot(&huart5, BRC_GPIO_Port, BRC_Pin);

geometry_msgs::Twist cmdvel_msg;

float vel_right, vel_left;
const float wheels_dist = 235.0;

void cmdvel_cb(const geometry_msgs::Twist& msg) {
	vel_right = msg.linear.x + msg.angular.z * wheels_dist / 2;
	vel_left = msg.linear.x - msg.angular.z * wheels_dist / 2;
	robot.driveVelocity(vel_right, vel_left);
}

ros::Subscriber<geometry_msgs::Twist> cmdvel_sub("cmd_vel", &cmdvel_cb);

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	nh.getHardware()->reset_rbuf();
}

uint16_t prevTicksLeft = 0;
uint16_t prevTicksRight = 0;
bool firstLoop = true;
uint32_t prevOnDataTime = 0;

float measuredLeftVel;
float measuredRightVel;

float totalLeftDist;
float totalRightDist;

geometry_msgs::Quaternion quat;

void ToQuaternion(geometry_msgs::Quaternion* quat, int16_t yaw, int16_t pitch,
		int16_t roll) // yaw (Z), pitch (Y), roll (X)
		{
	// Abbreviations for the various angular functions
	double cy = cos(yaw * 0.5);
	double sy = sin(yaw * 0.5);
	double cp = cos(pitch * 0.5);
	double sp = sin(pitch * 0.5);
	double cr = cos(roll * 0.5);
	double sr = sin(roll * 0.5);

	quat->w = cy * cp * cr + sy * sp * sr;
	quat->x = cy * cp * sr - sy * sp * cr;
	quat->y = sy * cp * sr + cy * sp * cr;
	quat->z = sy * cp * cr - cy * sp * sr;
}

float dt, deltaDist, deltaX, deltaY, deltaYaw, leftWheelDist, rightWheelDist,
		wheelDistDiff;

arm_matrix_instance_f32 invCovar;
arm_matrix_instance_f32 Finc, FincT;
arm_matrix_instance_f32 Fp, FpT;
arm_matrix_instance_f32 velCovar, tmp;
arm_matrix_instance_f32 poseCovar, poseCovarTmp, tmp2;

float32_t poseCovarData[9];
float32_t poseCovarTmpData[9];

void setup(void) {
	nh.subscribe(cmdvel_sub);
	nh.advertise(odom_pub);
//  tfBroadcaster.init(nh);
	nh.initNode();
	robot.start();
	robot.pauseStream();
	robot.goSafeMode();
	odom_msg.header.frame_id = "odom";
	odom_msg.child_frame_id = "base_link";
	vel.covariance.resize(9);
	pose.covariance.resize(9);
	for (int i = 0; i < 9; ++i) {
		poseCovarData[i] = 0;
	}
	arm_mat_init_f32(&poseCovar, 3, 3, (float32_t *) poseCovarData);
	arm_mat_init_f32(&poseCovarTmp, 3, 3, (float32_t *) poseCovarTmpData);
}

void loop(void) {
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);

	if (firstLoop) {
		prevTicksLeft = robot.readLeftEncoder();
		prevTicksRight = robot.readRightEncoder();
		prevOnDataTime = HAL_GetTick();
		vTaskDelay(100 / portTICK_PERIOD_MS);
		firstLoop = false;
		return;
	}

	uint32_t curTime = HAL_GetTick();
	dt = (curTime - prevOnDataTime) / 1000.0;
	// Get cumulative ticks (wraps around at 65535)
	uint16_t totalTicksLeft = robot.readLeftEncoder();
	uint16_t totalTicksRight = robot.readRightEncoder();
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

	leftWheelDist = (ticksLeft / util::V_3_TICKS_PER_REV) * util::WHEEL_DIAMETER
			* PI;
	rightWheelDist = (ticksRight / util::V_3_TICKS_PER_REV)
			* util::WHEEL_DIAMETER * PI;
	deltaDist = (rightWheelDist + leftWheelDist) / 2.0;

	wheelDistDiff = rightWheelDist - leftWheelDist;
	deltaYaw = wheelDistDiff / util::AXLE_LENGTH;

	// Compute distance traveled by each wheel
	measuredLeftVel = leftWheelDist / dt;
	measuredRightVel = rightWheelDist / dt;

	// Moving straight
	if (fabs(wheelDistDiff) < util::EPS) {
		deltaX = deltaDist * cos(pose.yaw);
		deltaY = deltaDist * sin(pose.yaw);
	} else {
		float turnRadius = (util::AXLE_LENGTH / 2.0)
				* (leftWheelDist + rightWheelDist) / wheelDistDiff;
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
	float kr = 1.0; // TODO: Perform experiments to find these nondeterministic parameters
	float kl = 1.0;
	float cosYawAndHalfDelta = cos(pose.yaw + (deltaYaw / 2.0)); // deltaX?
	float sinYawAndHalfDelta = sin(pose.yaw + (deltaYaw / 2.0)); // deltaY?
	float distOverTwoWB = deltaDist / (util::AXLE_LENGTH * 2.0);

	float32_t invCovarData[4];
	invCovarData[0] = kr * fabs(rightWheelDist);
	invCovarData[1] = 0.0;
	invCovarData[2] = 0.0;
	invCovarData[3] = kl * fabs(leftWheelDist);
	arm_mat_init_f32(&invCovar, 2, 2, (float32_t *) invCovarData);

	float32_t FincData[6], FincTData[6];
	FincData[0] = (cosYawAndHalfDelta / 2.0)
			- (distOverTwoWB * sinYawAndHalfDelta);
	FincData[1] = (cosYawAndHalfDelta / 2.0)
			+ (distOverTwoWB * sinYawAndHalfDelta);
	FincData[2] = (sinYawAndHalfDelta / 2.0)
			+ (distOverTwoWB * cosYawAndHalfDelta);
	FincData[3] = (sinYawAndHalfDelta / 2.0)
			- (distOverTwoWB * cosYawAndHalfDelta);
	FincData[4] = (1.0 / util::AXLE_LENGTH);
	FincData[5] = (-1.0 / util::AXLE_LENGTH);
	arm_mat_init_f32(&Finc, 3, 2, (float32_t *) FincData);
	arm_mat_init_f32(&FincT, 2, 3, (float32_t *) FincTData);
	arm_mat_trans_f32(&Finc, &FincT);

	float32_t FpData[9], FpTData[9];
	arm_mat_init_f32(&Fp, 3, 3, (float32_t *) FpData);
	arm_mat_init_f32(&FpT, 3, 3, (float32_t *) FpTData);
	FpData[0] = 1.0;
	FpData[1] = 0.0;
	FpData[2] = (-deltaDist) * sinYawAndHalfDelta;
	FpData[3] = 0.0;
	FpData[4] = 1.0;
	FpData[5] = deltaDist * cosYawAndHalfDelta;
	FpData[6] = 0.0;
	FpData[7] = 0.0;
	FpData[8] = 1.0;

	arm_mat_trans_f32(&Fp, &FpT);

	float32_t tmpData[9];
	arm_mat_init_f32(&tmp, 3, 3, (float32_t *) tmpData);

	float32_t velCovarData[9];
	arm_mat_init_f32(&velCovar, 3, 3, (float32_t *) velCovarData);

	arm_mat_mult_f32(&invCovar, &FincT, &tmp);
	arm_mat_mult_f32(&invCovar, &tmp, &velCovar);

	vel.covariance[0] = velCovar.pData[0];
	vel.covariance[1] = velCovar.pData[1];
	vel.covariance[2] = velCovar.pData[2];
	vel.covariance[3] = velCovar.pData[3];
	vel.covariance[4] = velCovar.pData[4];
	vel.covariance[5] = velCovar.pData[5];
	vel.covariance[6] = velCovar.pData[6];
	vel.covariance[7] = velCovar.pData[7];
	vel.covariance[8] = velCovar.pData[8];

	float32_t tmp2Data[9];
	arm_mat_init_f32(&tmp2, 3, 3, (float32_t *) tmp2Data);

	arm_mat_mult_f32(&poseCovar, &FpT, &tmp2);
	arm_mat_mult_f32(&Fp, &tmp2, &poseCovarTmp);
	arm_mat_add_f32(&poseCovarTmp, &velCovar, &poseCovar);

	pose.covariance[0] = poseCovar.pData[0];
	pose.covariance[1] = poseCovar.pData[1];
	pose.covariance[2] = poseCovar.pData[2];
	pose.covariance[3] = poseCovar.pData[3];
	pose.covariance[4] = poseCovar.pData[4];
	pose.covariance[5] = poseCovar.pData[5];
	pose.covariance[6] = poseCovar.pData[6];
	pose.covariance[7] = poseCovar.pData[7];
	pose.covariance[8] = poseCovar.pData[8];

	// Update pose
	pose.x += deltaX;
	pose.y += deltaY;
	pose.yaw = util::normalizeAngle(pose.yaw + deltaYaw);

	prevOnDataTime = curTime;

	ToQuaternion(&quat, 0, 0, pose.yaw);

//    odom_msg.header.stamp = HAL_GetTick();
	odom_msg.pose.pose.position.x = pose.x;
	odom_msg.pose.pose.position.y = pose.y;
	odom_msg.pose.pose.orientation = quat;

	// Populate velocity info
	odom_msg.twist.twist.linear.x = vel.x;
	odom_msg.twist.twist.linear.y = vel.y;
	odom_msg.twist.twist.angular.z = vel.yaw;

	// Update covariances
	odom_msg.pose.covariance[0] = static_cast<double>(pose.covariance[0]);
	odom_msg.pose.covariance[1] = pose.covariance[1];
	odom_msg.pose.covariance[5] = pose.covariance[2];
	odom_msg.pose.covariance[6] = pose.covariance[3];
	odom_msg.pose.covariance[7] = pose.covariance[4];
	odom_msg.pose.covariance[11] = pose.covariance[5];
	odom_msg.pose.covariance[30] = pose.covariance[6];
	odom_msg.pose.covariance[31] = pose.covariance[7];
	odom_msg.pose.covariance[35] = pose.covariance[8];
	odom_msg.twist.covariance[0] = vel.covariance[0];
	odom_msg.twist.covariance[1] = vel.covariance[1];
	odom_msg.twist.covariance[5] = vel.covariance[2];
	odom_msg.twist.covariance[6] = vel.covariance[3];
	odom_msg.twist.covariance[7] = vel.covariance[4];
	odom_msg.twist.covariance[11] = vel.covariance[5];
	odom_msg.twist.covariance[30] = vel.covariance[6];
	odom_msg.twist.covariance[31] = vel.covariance[7];
	odom_msg.twist.covariance[35] = vel.covariance[8];

//    if (publish_tf_)
//    {
//      tf_odom_.header.stamp = ros::Time::now();
//      tf_odom_.transform.translation.x = pose.x;
//      tf_odom_.transform.translation.y = pose.y;
//      tf_odom_.transform.rotation = quat;
//      tf_broadcaster_.sendTransform(tf_odom_);
//    }

	odom_pub.publish(&odom_msg);

	nh.spinOnce();
	vTaskDelay(100 / portTICK_PERIOD_MS);
}

#ifdef __cplusplus
}
#endif
