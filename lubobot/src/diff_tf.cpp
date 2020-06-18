#include "diff_tf.h"

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <sstream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "diff_tf_cpp");
    ros::NodeHandle n;
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 1000);
    ros::Rate loop_rate(10);
    int count = 0;
    while (ros::ok())
    {
        nav_msgs::Odometry msg;
        // ***************************************

        if (firstOnData)
        {
            // Initialize tick counts
            prevTicksLeft = GET_DATA(ID_LEFT_ENC);
            prevTicksRight = GET_DATA(ID_RIGHT_ENC);
            prevOnDataTime = util::getTimestamp();
            firstOnData = false;
        }

        // Get current time
        util::timestamp_t curTime = util::getTimestamp();
        float dt = (curTime - prevOnDataTime) / 1000000.0;
        float deltaDist, deltaX, deltaY, deltaYaw, leftWheelDist, rightWheelDist, wheelDistDiff;

        // Protocol versions 1 and 2 use distance and angle fields for odometry
        int16_t angleField;
        // Get cumulative ticks (wraps around at 65535)
        uint16_t totalTicksLeft = GET_DATA(ID_LEFT_ENC);
        uint16_t totalTicksRight = GET_DATA(ID_RIGHT_ENC);
        // Compute ticks since last update
        int ticksLeft = totalTicksLeft - prevTicksLeft;
        int ticksRight = totalTicksRight - prevTicksRight;
        prevTicksLeft = totalTicksLeft;
        prevTicksRight = totalTicksRight;

        // Handle wrap around
        if (fabs(ticksLeft) > 0.9 * util::V_3_MAX_ENCODER_TICKS)
        {
            ticksLeft = (ticksLeft % util::V_3_MAX_ENCODER_TICKS) + 1;
        }
        if (fabs(ticksRight) > 0.9 * util::V_3_MAX_ENCODER_TICKS)
        {
            ticksRight = (ticksRight % util::V_3_MAX_ENCODER_TICKS) + 1;
        }

        // Compute distance travelled by each wheel
        leftWheelDist = (ticksLeft / util::V_3_TICKS_PER_REV) * model.getWheelDiameter() * util::PI;
        rightWheelDist = (ticksRight / util::V_3_TICKS_PER_REV) * model.getWheelDiameter() * util::PI;
        deltaDist = (rightWheelDist + leftWheelDist) / 2.0;

        wheelDistDiff = rightWheelDist - leftWheelDist;
        deltaYaw = wheelDistDiff / model.getAxleLength();

        measuredLeftVel = leftWheelDist / dt;
        measuredRightVel = rightWheelDist / dt;

        // Moving straight
        if (fabs(wheelDistDiff) < util::EPS)
        {
            deltaX = deltaDist * cos(pose.yaw);
            deltaY = deltaDist * sin(pose.yaw);
        }
        else
        {
            float turnRadius = (model.getAxleLength() / 2.0) * (leftWheelDist + rightWheelDist) / wheelDistDiff;
            deltaX = turnRadius * (sin(pose.yaw + deltaYaw) - sin(pose.yaw));
            deltaY = -turnRadius * (cos(pose.yaw + deltaYaw) - cos(pose.yaw));
        }

        totalLeftDist += leftWheelDist;
        totalRightDist += rightWheelDist;

        if (fabs(dt) > util::EPS)
        {
            vel.x = deltaDist / dt;
            vel.y = 0.0;
            vel.yaw = deltaYaw / dt;
        }
        else
        {
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
        float distOverTwoWB = deltaDist / (model.getAxleLength() * 2.0);

        Matrix invCovar(2, 2);
        invCovar(0, 0) = kr * fabs(rightWheelDist);
        invCovar(0, 1) = 0.0;
        invCovar(1, 0) = 0.0;
        invCovar(1, 1) = kl * fabs(leftWheelDist);

        Matrix Finc(3, 2);
        Finc(0, 0) = (cosYawAndHalfDelta / 2.0) - (distOverTwoWB * sinYawAndHalfDelta);
        Finc(0, 1) = (cosYawAndHalfDelta / 2.0) + (distOverTwoWB * sinYawAndHalfDelta);
        Finc(1, 0) = (sinYawAndHalfDelta / 2.0) + (distOverTwoWB * cosYawAndHalfDelta);
        Finc(1, 1) = (sinYawAndHalfDelta / 2.0) - (distOverTwoWB * cosYawAndHalfDelta);
        Finc(2, 0) = (1.0 / model.getAxleLength());
        Finc(2, 1) = (-1.0 / model.getAxleLength());
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

        Matrix velCovar = ublas::prod(invCovar, FincT);
        velCovar = ublas::prod(Finc, velCovar);

        vel.covariance[0] = velCovar(0, 0);
        vel.covariance[1] = velCovar(0, 1);
        vel.covariance[2] = velCovar(0, 2);
        vel.covariance[3] = velCovar(1, 0);
        vel.covariance[4] = velCovar(1, 1);
        vel.covariance[5] = velCovar(1, 2);
        vel.covariance[6] = velCovar(2, 0);
        vel.covariance[7] = velCovar(2, 1);
        vel.covariance[8] = velCovar(2, 2);

        Matrix poseCovarTmp = ublas::prod(poseCovar, FpT);
        poseCovarTmp = ublas::prod(Fp, poseCovarTmp);
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

        // ***************************************
        odom_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }
    return 0;
}