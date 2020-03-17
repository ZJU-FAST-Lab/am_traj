#include <eigen3/Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PolynomialTrajectory.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Joy.h>

const int _DIM_x = 0;
const int _DIM_y = 1;
const int _DIM_z = 2;

using namespace std;

class TrajectoryServer
{
private:
    // Subscribers
    ros::Subscriber _odom_sub;
    ros::Subscriber _traj_sub;
    ros::Subscriber _joy_sub;

    // publishers
    ros::Publisher _cmd_pub;
    ros::Publisher _vis_cmd_pub;
    ros::Publisher _vis_vel_pub;
    ros::Publisher _vis_acc_pub;
    ros::Publisher _vis_traj_pub;
    ros::Publisher _vis_traj_points;
    ros::Publisher _vis_rate_pub;

    // joy state
    int jstate = -1;
    int JOY_MANUAL;
    int JOY_AUTO;

    // configuration for trajectory
    int _n_segment = 0;
    int _traj_id = 0;
    uint32_t _traj_flag = 0;
    Eigen::VectorXd _time;
    Eigen::MatrixXd _coef[3];
    vector<int> _order;

    double _vis_traj_width = 0.2;
    double mag_coeff;
    ros::Time _final_time = ros::TIME_MIN;
    ros::Time _start_time = ros::TIME_MAX;
    double _start_yaw = 0.0, _final_yaw = 0.0;

    // state of the server
    //enum ServerState{INIT, TRAJ, HOVER} state = INIT;
    enum ServerState
    {
        INIT,
        TRAJ,
        HOVER
    } state = INIT;
    ;
    nav_msgs::Odometry _odom;
    quadrotor_msgs::PositionCommand _cmd;
    geometry_msgs::PoseStamped _vis_cmd;
    geometry_msgs::Vector3 _vis_vaj_rate;

    visualization_msgs::Marker _vis_vel, _vis_acc;
    visualization_msgs::Marker _vis_traj;

    sensor_msgs::PointCloud2 traj_pts;
    pcl::PointCloud<pcl::PointXYZ> traj_pts_pcd;

public:
    TrajectoryServer(ros::NodeHandle &handle)
    {
        _odom_sub =
            handle.subscribe("odometry", 50, &TrajectoryServer::rcvOdometryCallback, this, ros::TransportHints().tcpNoDelay());

        _traj_sub =
            handle.subscribe("trajectory", 2, &TrajectoryServer::rcvTrajectoryCallabck, this, ros::TransportHints().tcpNoDelay());

        _joy_sub =
            handle.subscribe("joy", 1, &TrajectoryServer::rcvJoyCallback, this, ros::TransportHints().tcpNoDelay());

        handle.param("joymap_manual", JOY_MANUAL, 6);
        handle.param("joymap_auto", JOY_AUTO, 7);

        _cmd_pub =
            handle.advertise<quadrotor_msgs::PositionCommand>("position_command", 50);

        _vis_cmd_pub =
            handle.advertise<geometry_msgs::PoseStamped>("desired_position", 50);

        _vis_vel_pub =
            handle.advertise<visualization_msgs::Marker>("desired_velocity", 50);

        _vis_acc_pub =
            handle.advertise<visualization_msgs::Marker>("desired_acceleration", 50);

        _vis_traj_pub =
            handle.advertise<visualization_msgs::Marker>("trajectory_vis", 1);

        _vis_traj_points =
            handle.advertise<sensor_msgs::PointCloud2>("trajectory_vis_points", 1);
        _vis_rate_pub =
            handle.advertise<geometry_msgs::Vector3>("/position_cmd_vaj_rate", 1);

        double pos_gain[3] = {5.7, 5.7, 6.2};
        double vel_gain[3] = {3.4, 3.4, 4.0};
        setGains(pos_gain, vel_gain);

        _vis_traj.header.stamp = ros::Time::now();
        _vis_traj.header.frame_id = "odom";

        _vis_traj.ns = "trajectory/trajectory";
        _vis_traj.id = 0;
        _vis_traj.type = visualization_msgs::Marker::SPHERE_LIST;
        _vis_traj.action = visualization_msgs::Marker::ADD;
        _vis_traj.scale.x = _vis_traj_width;
        _vis_traj.scale.y = _vis_traj_width;
        _vis_traj.scale.z = _vis_traj_width;
        _vis_traj.pose.orientation.x = 0.0;
        _vis_traj.pose.orientation.y = 0.0;
        _vis_traj.pose.orientation.z = 0.0;
        _vis_traj.pose.orientation.w = 1.0;
        _vis_traj.color.r = 0.0;
        _vis_traj.color.g = 0.0;
        _vis_traj.color.b = 1.0;
        _vis_traj.color.a = 0.3;
        _vis_traj.points.clear();
    }

    void setGains(double pos_gain[3], double vel_gain[3])
    {
        _cmd.kx[_DIM_x] = pos_gain[_DIM_x];
        _cmd.kx[_DIM_y] = pos_gain[_DIM_y];
        _cmd.kx[_DIM_z] = pos_gain[_DIM_z];

        _cmd.kv[_DIM_x] = vel_gain[_DIM_x];
        _cmd.kv[_DIM_y] = vel_gain[_DIM_y];
        _cmd.kv[_DIM_z] = vel_gain[_DIM_z];
    }

    bool cmd_flag = false;
    void rcvOdometryCallback(const nav_msgs::Odometry &odom)
    {
        if (odom.child_frame_id == "X" || odom.child_frame_id == "O")
            return;
        // #1. store the odometry
        _odom = odom;
        _vis_cmd.header = _odom.header;
        _vis_cmd.header.frame_id = "odom";

        if (state == INIT && fabs(_odom.pose.pose.position.z - 1.0) < 0.1)
            cmd_flag = true;

        if (state == INIT)
        {
            //ROS_WARN("[TRAJ SERVER] Pub initial pos command");
            _cmd.position = _odom.pose.pose.position;

            _cmd.header.stamp = _odom.header.stamp;
            _cmd.header.frame_id = "odom";
            _cmd.trajectory_flag = _traj_flag;

            _cmd.velocity.x = 0.0;
            _cmd.velocity.y = 0.0;
            _cmd.velocity.z = 0.0;

            _cmd.acceleration.x = 0.0;
            _cmd.acceleration.y = 0.0;
            _cmd.acceleration.z = 0.0;
            _cmd_pub.publish(_cmd);

            _vis_cmd.pose.position.x = _cmd.position.x;
            _vis_cmd.pose.position.y = _cmd.position.y;
            _vis_cmd.pose.position.z = _cmd.position.z;
            _vis_cmd_pub.publish(_vis_cmd);

            return;
        }

        // #2. try to calculate the new state
        if (state == TRAJ && ((odom.header.stamp - _start_time).toSec() / mag_coeff > (_final_time - _start_time).toSec()))
        {
            state = HOVER;
            _traj_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_COMPLETED;
        }

        // #3. try to publish command
        pubPositionCommand();
    }

    void rcvTrajectoryCallabck(const quadrotor_msgs::PolynomialTrajectory &traj)
    {
        if (jstate == -1)
            return;
        //ROS_WARN("[SERVER] Recevied The Trajectory with %.3lf.", _start_time.toSec());
        //ROS_WARN("[SERVER] Now the odom time is : ");
        // #1. try to execuse the action

        if (traj.action == quadrotor_msgs::PolynomialTrajectory::ACTION_ADD)
        {
            jstate = 1;
            ROS_WARN("[SERVER] Loading the trajectory.");
            if ((int)traj.trajectory_id < _traj_id)
                return;

            state = TRAJ;
            _traj_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
            _traj_id = traj.trajectory_id;
            _n_segment = traj.num_segment;
            _final_time = _start_time = traj.header.stamp;
            _time.resize(_n_segment);

            _order.clear();
            for (int idx = 0; idx < _n_segment; ++idx)
            {
                _final_time += ros::Duration(traj.time[idx]);
                _time(idx) = traj.time[idx];
                _order.push_back(traj.order[idx]);
            }

            _start_yaw = traj.start_yaw;
            _final_yaw = traj.final_yaw;
            mag_coeff = traj.mag_coeff;

            int max_order = *max_element(begin(_order), end(_order));

            _coef[_DIM_x] = Eigen::MatrixXd::Zero(max_order + 1, _n_segment);
            _coef[_DIM_y] = Eigen::MatrixXd::Zero(max_order + 1, _n_segment);
            _coef[_DIM_z] = Eigen::MatrixXd::Zero(max_order + 1, _n_segment);

            //ROS_WARN("stack the coefficients");
            int shift = 0;
            for (int idx = 0; idx < _n_segment; ++idx)
            {
                int order = traj.order[idx];

                for (int j = 0; j < (order + 1); ++j)
                {
                    _coef[_DIM_x](j, idx) = traj.coef_x[shift + j];
                    _coef[_DIM_y](j, idx) = traj.coef_y[shift + j];
                    _coef[_DIM_z](j, idx) = traj.coef_z[shift + j];
                }

                shift += (order + 1);
            }
        }
        else if (traj.action == quadrotor_msgs::PolynomialTrajectory::ACTION_ABORT)
        {
            ROS_WARN("[SERVER] Aborting the trajectory.");
            state = HOVER;
            _traj_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_COMPLETED;
        }
        else if (traj.action == quadrotor_msgs::PolynomialTrajectory::ACTION_WARN_IMPOSSIBLE)
        {
            state = HOVER;
            _traj_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_IMPOSSIBLE;
        }
    }

    void rcvJoyCallback(const sensor_msgs::Joy &joy)
    {
        if (joy.buttons.at(JOY_MANUAL) && !joy.buttons.at(JOY_AUTO))
        {
            jstate = -1;
            state = HOVER;
        }

        if (joy.buttons.at(JOY_AUTO) && !joy.buttons.at(JOY_MANUAL))
        {
            jstate = 0;
            state = INIT;
        }
    }

    void pubPositionCommand()
    {
        // #1. check if it is right state
        if (state == INIT)
            return;
        if (state == HOVER || jstate == -1)
        {
            if (_cmd.header.frame_id != "odom")
            {
                _cmd.position = _odom.pose.pose.position;
            }

            _cmd.header.stamp = _odom.header.stamp;
            _cmd.header.frame_id = "odom";
            _cmd.trajectory_flag = _traj_flag;

            _cmd.velocity.x = 0.0;
            _cmd.velocity.y = 0.0;
            _cmd.velocity.z = 0.0;

            _cmd.acceleration.x = 0.0;
            _cmd.acceleration.y = 0.0;
            _cmd.acceleration.z = 0.0;

            _cmd.yaw_dot = 0.0;
        }
        // #2. locate the trajectory segment
        if (state == TRAJ && jstate == 1)
        {
            _cmd.header.stamp = _odom.header.stamp;

            _cmd.header.frame_id = "odom";
            _cmd.trajectory_flag = _traj_flag;
            _cmd.trajectory_id = _traj_id;

            double t = max(0.0, (_odom.header.stamp - _start_time).toSec()) / mag_coeff;

            //cout<<"t: "<<t<<endl;

            // #3. calculate the desired states
            //ROS_WARN("[SERVER] the time : %.3lf\n, n = %d, m = %d", t, _n_order, _n_segment);
            for (int idx = 0; idx < _n_segment; ++idx)
            {
                if (t > _time[idx] && idx + 1 < _n_segment)
                {
                    t -= _time[idx];
                }
                else
                {
                    t /= _time[idx];

                    _cmd.position.x = 0.0;
                    _cmd.position.y = 0.0;
                    _cmd.position.z = 0.0;
                    _cmd.velocity.x = 0.0;
                    _cmd.velocity.y = 0.0;
                    _cmd.velocity.z = 0.0;
                    _cmd.acceleration.x = 0.0;
                    _cmd.acceleration.y = 0.0;
                    _cmd.acceleration.z = 0.0;

                    int cur_order = _order[idx];
                    int cur_poly_num = cur_order + 1;

                    for (int i = 0; i < cur_poly_num; i++)
                    {
                        _cmd.position.x += _coef[_DIM_x].col(idx)(i) * pow(t, i);
                        _cmd.position.y += _coef[_DIM_y].col(idx)(i) * pow(t, i);
                        _cmd.position.z += _coef[_DIM_z].col(idx)(i) * pow(t, i);

                        if (i < (cur_poly_num - 1))
                        {
                            _cmd.velocity.x += (i + 1) * _coef[_DIM_x].col(idx)(i + 1) * pow(t, i) / _time[idx];

                            _cmd.velocity.y += (i + 1) * _coef[_DIM_y].col(idx)(i + 1) * pow(t, i) / _time[idx];

                            _cmd.velocity.z += (i + 1) * _coef[_DIM_z].col(idx)(i + 1) * pow(t, i) / _time[idx];
                        }

                        if (i < (cur_poly_num - 2))
                        {
                            _cmd.acceleration.x += (i + 2) * (i + 1) * _coef[_DIM_x].col(idx)(i + 2) * pow(t, i) / _time[idx] / _time[idx];

                            _cmd.acceleration.y += (i + 2) * (i + 1) * _coef[_DIM_y].col(idx)(i + 2) * pow(t, i) / _time[idx] / _time[idx];

                            _cmd.acceleration.z += (i + 2) * (i + 1) * _coef[_DIM_z].col(idx)(i + 2) * pow(t, i) / _time[idx] / _time[idx];
                        }
                    }

                    //_cmd.yaw = _start_yaw + (_final_yaw - _start_yaw) * t / ((_final_time - _start_time).toSec() + 1e-9);
                    _cmd.yaw = atan2(_cmd.velocity.y, _cmd.velocity.x);

                    //calculate proportional term of yaw_dot
                    double k_p = 1.5;
                    double yaw_dot_p = 0.0;
                    tf::Quaternion quat(_odom.pose.pose.orientation.x, _odom.pose.pose.orientation.y, _odom.pose.pose.orientation.z, _odom.pose.pose.orientation.w);
                    tf::Matrix3x3 rotM(quat);
                    double roll, pitch, yaw;
                    rotM.getRPY(roll, pitch, yaw);
                    const double pi = 3.14159265358979;
                    double deltaYaw = yaw - _cmd.yaw;
                    if (deltaYaw <= -pi)
                    {
                        deltaYaw += 2 * pi;
                    }
                    if (deltaYaw > pi)
                    {
                        deltaYaw -= 2 * pi;
                    }
                    yaw_dot_p = -k_p * deltaYaw;

                    //calculate derivative term of yaw_dot
                    double k_d = 0.25;
                    double yaw_dot_d = 0.0;
                    double v_horiz_sqr_norm = _cmd.velocity.y * _cmd.velocity.y + _cmd.velocity.x * _cmd.velocity.x;
                    yaw_dot_d = v_horiz_sqr_norm == 0 ? 0 : k_d * (-_cmd.velocity.y * _cmd.acceleration.x + _cmd.velocity.x * _cmd.acceleration.y) / v_horiz_sqr_norm;

                    //synthesis yaw_dot
                    _cmd.yaw_dot = yaw_dot_p + yaw_dot_d;

                    if (fabs(_cmd.yaw_dot) > pi / 3)
                    {
                        _cmd.yaw_dot /= fabs(_cmd.yaw_dot);
                        _cmd.yaw_dot *= pi / 3;
                    }
                    
                    if(_cmd.velocity.z*_cmd.velocity.z * 2.0 > _cmd.velocity.y*_cmd.velocity.y + _cmd.velocity.x*_cmd.velocity.x)
                    {
                        _cmd.yaw = yaw;
                        _cmd.yaw_dot = 0.0;
                    }

                    break;
                }
            }
        }

        // #4. just publish
        _cmd_pub.publish(_cmd);

        _vis_vaj_rate.x = sqrt(_cmd.velocity.x * _cmd.velocity.x + _cmd.velocity.y * _cmd.velocity.y + _cmd.velocity.z * _cmd.velocity.z);
        _vis_vaj_rate.y = sqrt(_cmd.acceleration.x * _cmd.acceleration.x + _cmd.acceleration.y * _cmd.acceleration.y + _cmd.acceleration.z * _cmd.acceleration.z);
        _vis_vaj_rate.z = 0.0;
        _vis_rate_pub.publish(_vis_vaj_rate);

        _vis_cmd.header = _cmd.header;
        _vis_cmd.pose.position.x = _cmd.position.x;
        _vis_cmd.pose.position.y = _cmd.position.y;
        _vis_cmd.pose.position.z = _cmd.position.z;

        tf::Quaternion q_ = tf::createQuaternionFromYaw(_cmd.yaw);
        geometry_msgs::Quaternion odom_quat;
        tf::quaternionTFToMsg(q_, odom_quat);
        _vis_cmd.pose.orientation = odom_quat;
        _vis_cmd_pub.publish(_vis_cmd);

        _vis_vel.ns = "vel";
        _vis_vel.id = 0;
        _vis_vel.header.frame_id = "odom";
        _vis_vel.type = visualization_msgs::Marker::ARROW;
        _vis_vel.action = visualization_msgs::Marker::ADD;
        _vis_vel.color.a = 1.0;
        _vis_vel.color.r = 0.0;
        _vis_vel.color.g = 1.0;
        _vis_vel.color.b = 0.0;

        _vis_vel.header.stamp = _odom.header.stamp;
        _vis_vel.points.clear();

        geometry_msgs::Point pt;
        pt.x = _cmd.position.x;
        pt.y = _cmd.position.y;
        pt.z = _cmd.position.z;

        _vis_traj.points.push_back(pt);
        _vis_traj_pub.publish(_vis_traj);

        pcl::PointXYZ point(pt.x, pt.y, pt.z);
        traj_pts_pcd.points.push_back(point);
        traj_pts_pcd.width = traj_pts_pcd.points.size();
        traj_pts_pcd.height = 1;
        traj_pts_pcd.is_dense = true;

        pcl::toROSMsg(traj_pts_pcd, traj_pts);
        traj_pts.header.frame_id = "odom";
        _vis_traj_points.publish(traj_pts);

        _vis_vel.points.push_back(pt);

        pt.x = _cmd.position.x + _cmd.velocity.x;
        pt.y = _cmd.position.y + _cmd.velocity.y;
        pt.z = _cmd.position.z + _cmd.velocity.z;

        _vis_vel.points.push_back(pt);

        _vis_vel.scale.x = 0.2;
        _vis_vel.scale.y = 0.4;
        _vis_vel.scale.z = 0.4;

        _vis_vel_pub.publish(_vis_vel);

        _vis_acc.ns = "acc";
        _vis_acc.id = 0;
        _vis_acc.header.frame_id = "odom";
        _vis_acc.type = visualization_msgs::Marker::ARROW;
        _vis_acc.action = visualization_msgs::Marker::ADD;
        _vis_acc.color.a = 1.0;
        _vis_acc.color.r = 1.0;
        _vis_acc.color.g = 1.0;
        _vis_acc.color.b = 0.0;

        _vis_acc.header.stamp = _odom.header.stamp;

        _vis_acc.points.clear();
        pt.x = _cmd.position.x;
        pt.y = _cmd.position.y;
        pt.z = _cmd.position.z;

        _vis_acc.points.push_back(pt);

        pt.x = _cmd.position.x + _cmd.acceleration.x;
        pt.y = _cmd.position.y + _cmd.acceleration.y;
        pt.z = _cmd.position.z + _cmd.acceleration.z;

        _vis_acc.points.push_back(pt);

        _vis_acc.scale.x = 0.2;
        _vis_acc.scale.y = 0.4;
        _vis_acc.scale.z = 0.4;

        _vis_acc_pub.publish(_vis_acc);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "traj_server_node");
    ros::NodeHandle handle("~");

    TrajectoryServer server(handle);

    ros::spin();

    return 0;
}
