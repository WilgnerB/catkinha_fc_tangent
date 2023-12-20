#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <angles/angles.h>
#include <cmath>
#include <Eigen/Dense> 
#include <sensor_msgs/LaserScan.h>
#include <utility>  // Add this line to include the necessary header for std::std::pair
#include <visualization_msgs/Marker.h>
#include <vector>

std::pair<double, double> line;
#define _x_   0
#define _y_   1
#define _a_   2

#define PI    3.14159265359
#define INF   100000.0

// Helper
double myAbs(double a) { return a >= 0 ? a : -a; }

double sign(double a) { return a >= 0 ? 1.0 : (a < 0.0 ? -1.0 : 0.0); }

double rad2deg(double a) { return 180.0/PI * a; }
// Function to calculate the Euclidean distance between two points
double calculateDistance(double x1, double y1, double x2, double y2) {
    return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
}
double roll, pitch, yaw;
double laser_d = 5.;
double D_MAX = 1.0;  // Maximum distance for obstacle sensing
double Ts = 0.1;
double Rx = 0.;
double Ry = 0.;
double Gx = 0.;
double Gy = 0.;
double d_folowed = INF;
double d_folowed_at = INF;
double d_reach = INF; 

// Variables
geometry_msgs::Pose current_position;
geometry_msgs::Pose target;
// variables
double po[3];   // current pose
double sp[3];   // setpoint
double min_point[3];   // min
int obstacle_detected = 0;
geometry_msgs::Pose discontinuity_point;
double minimum_distance = std::numeric_limits<double>::infinity();
std::vector<geometry_msgs::Pose> explored_edge;
double vl, va;  // desired velocities
// Initial speeds
float v = 0.4;
float w = 0;
Eigen::Vector2d goal(8.0, 2.0);
Eigen::Vector2d robotkinha(8.0, 2.0);
double linear_vel, angular_vel; // linear and angular velocity commands for robot
double Vmax = 10; //Max velocity for the robot
double Vtot; //Max velocity for the robot
double d;
double u1, u2;
// Control parameters
double kp = 11.0;
double kp1 = 1.0;
double kp2 = 1.0;
double x_d, y_d, theta_d; // desired position and orientation of robot on lemniscate

void UpScan(const sensor_msgs::LaserScan& scan);
double getAngVel(void) { return va; }
double getLinVel(void) { return vl; }

double vdmax, vamax, atoli, atole, dtoli, dtole;    // parameters

constexpr static double wid = 0.5;     // a bit more then half the robot's width

// laser scan info
std::vector<Eigen::Vector3d> obstacles;  // Updated obstacle positions
constexpr static size_t samples = 270;
double bmin, bmax, binc, rmin, rmax;
double ranges_laser[samples], angles_laser[samples];
// Reading from the sensors
int detected_front = 0;
int detected_right = 0;
int detected_left = 0;
int detected_left_45 = 0;
int detected_right_45 = 0;

// Callbacks
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {

    double x, y, theta; // current position and orientation of robot
    // Get current position of robot
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;

    // Get current orientation of robot
    tf::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, theta); // convert quaternion to Euler angles
    current_position.position.x = x;
    current_position.position.y = y;
    current_position.position.z = theta;

    yaw = theta;
    po[_x_] = x;
    po[_y_] = y;
    po[_a_] = yaw;
}

// Initialize min_range to a large value
double min_range = INF;

void UpScan(const sensor_msgs::LaserScan& laserScan)
{
    // Clear previous obstacle positions
    min_range = INF;
    obstacles.clear();
    for(size_t i = 0; i < samples; i++)
    {
        ranges_laser[i] = laserScan.ranges[i];
        if (laserScan.ranges[i] < min_range) {
            min_range = laserScan.ranges[i];
        }
        if (laserScan.ranges[i] < D_MAX) {
            double angle = laserScan.angle_min + i * laserScan.angle_increment;
            double obs_x = std::cos(angle) * laserScan.ranges[i];
            double obs_y = std::sin(angle) * laserScan.ranges[i];

            // Transform the obstacle position to the map frame
            double map_x = po[_x_] + obs_x * std::cos(yaw) - obs_y * std::sin(yaw);
            double map_y = po[_y_] + obs_x * std::sin(yaw) + obs_y * std::cos(yaw);

            // Store obstacle position
            obstacles.push_back(Eigen::Vector3d(map_x, map_y, 0.0));
        }
    }
    // Reading from the sensors
    detected_front = int(ranges_laser[135] < 0.3);
    detected_right = int(ranges_laser[45] < 0.5);
    detected_right_45 = int(ranges_laser[90] < 0.5);
    detected_left = int(ranges_laser[225] < 0.5);
    detected_left_45 = int(ranges_laser[180] < 0.5);
    obstacle_detected = (detected_front || detected_right || detected_left);
    if(obstacle_detected){
        double angle = laserScan.angle_min + 135 * laserScan.angle_increment;
        double obs_x = std::cos(angle) * laserScan.ranges[135];
        double obs_y = std::sin(angle) * laserScan.ranges[135];

        // Transform the obstacle position to the map frame
        double map_x = po[_x_] + obs_x * std::cos(yaw) - obs_y * std::sin(yaw);
        double map_y = po[_y_] + obs_x * std::sin(yaw) + obs_y * std::cos(yaw);
        goal << map_x, map_y;
        d_folowed_at = calculateDistance(map_x, map_y, sp[_x_], sp[_y_]);
    }
}

// Functions
bool findDiscontinuityPoint() {
    // list free tangents
    // angles_laser, ranges_laser, and corrections
    size_t numTan = 0;
    double angl[samples], rang[samples], corr[samples];

    for(size_t i = 0; i < samples-1; i++)
    {
        double delta = ranges_laser[i] - ranges_laser[i+1];

        // ROS_INFO("delta: %f", delta);
        // ROS_INFO("ranges_laser[%f]: %f", i * 1., ranges_laser[i]);
        // ROS_INFO("ranges_laser[%f]: %f", i * 1., ranges_laser[i+1]);
        // ROS_INFO("wid: %f", wid);
        // ROS_INFO("rmax: %f", rmax);
        // ROS_INFO("rmin: %f", rmin);

        if( delta > 2*wid || (ranges_laser[i] > rmax && ranges_laser[i+1] < rmin) )
        {
            // 'closing' discontinuity
            angl[numTan] = angles_laser[i];
            rang[numTan] = ranges_laser[i+1];
            corr[numTan] = -asin(wid/ranges_laser[i+1]);
            numTan++;
        }
        else if( delta < -2*wid || (ranges_laser[i] < rmax && ranges_laser[i+1] > rmin) )
        {
            // 'opening' discontinuity
            angl[numTan] = angles_laser[i+1];
            rang[numTan] = ranges_laser[i];
            corr[numTan] = asin(wid/ranges_laser[i]);
            numTan++;
        }
    }

    // select best tangent
    double an, ra, co;
    double min_distance = INF; // Initialize to positive infinity

    if( numTan > 0 )
    {
        an = INF;
        ra = 0.0;
        co = 0.0;
        // selects tangent which is in the nearest direction
        for(size_t i = 0; i < numTan; i++)
        {
            // ROS_INFO("    tan: angl: %.3f, corr: %.3f, rang: %.3f", rad2deg(angl[i]), rad2deg(corr[i]), rang[i]);

            // if( myAbs(angl[i] - da) < myAbs(an - da))
            // {
            an = angl[i];
            ra = rang[i];
            co = corr[i];
            //isAnyTanSel = true;
            double obs_tan_x = ra*cos(an + co);
            double obs_tan_y = ra*sin(an + co);
            // double obs_tan_x = ra*cos(an);
            // double obs_tan_y = ra*sin(an);

            // Transform the obstacle position to the map frame
            double map_tan_x = po[_x_] + obs_tan_x * std::cos(yaw) - obs_tan_y * std::sin(yaw);
            double map_tan_y = po[_y_] + obs_tan_x * std::sin(yaw) + obs_tan_y * std::cos(yaw);

            // Calculate Euclidean distance to the specified point
            double distance_to_target = calculateDistance(map_tan_x, map_tan_y,  sp[_x_], sp[_y_]);
            if (distance_to_target < min_distance) {
                min_distance = distance_to_target;
                an = angl[i];
                ra = rang[i];
                co = corr[i];
                //isAnyTanSel = true;
                discontinuity_point.position.x = map_tan_x;
                discontinuity_point.position.y = map_tan_y;
                discontinuity_point.position.z = co;
            }


            // if( myAbs(angl[i] - da) < myAbs(an - da))
            // {
            //     an = angl[i];
            //     ra = rang[i];
            //     co = corr[i];
            //     isAnyTanSel = true;
            // }
        }
        ROS_INFO("Selected tangent: angl: %.3f, corr: %.3f, rang: %.3f", rad2deg(an), rad2deg(co), ra);
        return true;
    }
    // else
    // {
    //     Rx = po[_x_];
    //     Ry = po[_y_];
    //     Gx = goal.x();
    //     Gy = goal.y();
    //     // y = a*x + b
    //     line = discretizeLine(Rx, Ry, Gx, Gy);
    //     discontinuity_point.position.x = line.first;
    //     discontinuity_point.position.y = line.second;
    //     return discontinuity_point;
    // }
    return false;

}

// Control parameters
double k_att = 1.2;
double k_rep = 0.05;
double attractive_potential(const Eigen::Vector2d& q, const Eigen::Vector2d& goal) {
    return k_att * (goal - q).norm();
}

double repulsive_potential(const Eigen::Vector2d& q, const Eigen::Vector3d& obs, double R = D_MAX) {
    Eigen::Vector2d v = q - obs.head<2>();
    double d = v.norm();

    if (d <= R) {
        return 0.5 * k_rep * std::pow((1.0 / d - 1.0 / R), 2);
    } else {
        return 0.0;
    }
}

double total_potential(const Eigen::Vector2d& q, const Eigen::Vector2d& goal, const std::vector<Eigen::Vector3d>& obstacles) {
    double U_att = attractive_potential(q, goal);
    double U_rep = 0.0;

    for (const auto& obs : obstacles) {
        U_rep += repulsive_potential(q, obs);
    }

    return U_att + U_rep;
}

Eigen::Vector2d potential_gradient(const Eigen::Vector2d& q, const Eigen::Vector2d& goal, const std::vector<Eigen::Vector3d>& obstacles) {
    const double delta = 0.01; // Small value for numerical differentiation

    double gradient_x = (total_potential(q + Eigen::Vector2d(delta, 0.0), goal, obstacles) - total_potential(q, goal, obstacles)) / delta;
    double gradient_y = (total_potential(q + Eigen::Vector2d(0.0, delta), goal, obstacles) - total_potential(q, goal, obstacles)) / delta;

    return Eigen::Vector2d(gradient_x, gradient_y);
}

Eigen::Vector2d l_goal;

void MovePotential(){
    Eigen::Vector2d robot_position;
    robot_position << po[_x_], po[_y_];

    Eigen::Vector2d control_input = -kp * potential_gradient(robot_position, goal, obstacles);
    double dt = 1. / 10.;
    double dx = (goal.x() - l_goal.x()) / dt;
    double dy = (goal.y() - l_goal.y()) / dt;
    //Feedback Linearization
    u1 = 0 + control_input[0];
    u2 = 0 + control_input[1];
    l_goal << goal;
        
    // feedback linearization
    Eigen::Matrix2d A;
    A << cos(yaw), -d*sin(yaw),
            sin(yaw), d*cos(yaw);

    Eigen::Vector2d vw = A.inverse() * Eigen::Vector2d(u1,u2);


    // ROS_INFO("x_d: %f y_d: %f", goal.x(), goal.y());
    // ROS_INFO("x_robot: %f y_robot: %f", po[_x_], po[_y_]);
    u1 = kp1*vw[0];
    u2 = kp1*vw[1];
    Vtot = sqrt(pow(u1, 2) + pow(u2, 2));


    if (Vtot >= Vmax){
        u1 = u1 * Vmax / Vtot;
        u2 = u2 * Vmax / Vtot;
    }

    // Apply inverse dynamics controllers to errors and generate velocity commands for robot
    vl = u1;
    va = u2;

}

bool CheckGoalDir(double goal_dir, double goal_dist)
{
    // Calculate the angle between robotPose and sp
    // Process laser scan data to obtain obstacle positions
    double x = po[_x_];
    double y = po[_y_];
    double err_x = sp[_x_] - po[_x_];
    double err_y = sp[_y_] - po[_y_];
    double angle_to_goal = atan2(sp[_y_] - po[_y_], sp[_x_] - po[_x_]);

    // if (goal_dist < 0.5)
    //     return true;


    // for(size_t i = 0; i < samples; i++)
    // {
    //     if( goal_dir > 0.0 && bmin+goal_dir > angles_laser[i] ) continue;

    //     if( goal_dir < 0.0 && bmax+goal_dir < angles_laser[i] ) continue;

    //     if( goal_dist > ranges_laser[i] &&
    //         myAbs(ranges_laser[i]*sin(angles_laser[i] - goal_dir)) <= wid )
    //         return true;
    // }

    // return false;
    for(size_t i = 0; i < samples - 1; i++)
    {
        // if( goal_dist > ranges_laser[i] &&
        //     myAbs(ranges_laser[i]*sin(angles_laser[i] - goal_dir)) <= wid )
        //     return false;
        
        //ROS_INFO("  to laser: ranges_laser[i]: %.3f, ranges_laser[i + 1]: %.3f", ranges_laser[i], ranges_laser[i + 1]);
        if( ranges_laser[i] > laser_d - 0.1 && ranges_laser[i + 1] > laser_d - 0.1){
            double obs_x_0 = ranges_laser[i]*cos(angles_laser[i]);
            double obs_y_0 = ranges_laser[i]*sin(angles_laser[i]);

            // Transform the obstacle position to the map frame
            double map_x_0 = po[_x_] + obs_x_0 * std::cos(yaw) - obs_y_0 * std::sin(yaw);
            double map_y_0 = po[_y_] + obs_x_0 * std::sin(yaw) + obs_y_0 * std::cos(yaw);

            double obs_x_1 = ranges_laser[i+1]*cos(angles_laser[i+1]);
            double obs_y_1 = ranges_laser[i+1]*sin(angles_laser[i+1]);

            // Transform the obstacle position to the map frame
            double map_x_1 = po[_x_] + obs_x_1 * std::cos(yaw) - obs_y_1 * std::sin(yaw);
            double map_y_1 = po[_y_] + obs_x_1 * std::sin(yaw) + obs_y_1 * std::cos(yaw);

            double angle_to_laser0 = atan2(map_y_0 - po[_y_], map_x_0 - po[_x_]);
            double angle_to_laser1 = atan2(map_y_1 - po[_y_], map_x_1 - po[_x_]);
            //ROS_INFO("  to laser: angle_to_laser0: %.3f, angle_to_goal: %.3f, angle_to_laser1: %.3f", angle_to_laser0, angle_to_goal, angle_to_laser1);

            if (angle_to_goal > angle_to_laser0 && angle_to_goal < angle_to_laser1 && !detected_front) {
                ROS_INFO("  to laser: angle_to_laser0: %.3f, angle_to_goal: %.3f, angle_to_laser1: %.3f", angle_to_laser0, angle_to_goal, angle_to_laser1);
                return true;
                break;
            }

        }

    }
    return false;
}

//Feedback Linearization
void feedbackLinearization(double dl, double da){
    u1 = dl;
    u2 = da;

        
    // feedback linearization
    Eigen::Matrix2d A;
    A << cos(po[_a_]), -d*sin(po[_a_]),
            sin(po[_a_]), d*cos(po[_a_]);

    Eigen::Vector2d vw = A.inverse() * Eigen::Vector2d(u1,u2);

    // Apply inverse dynamics controllers to errors and generate velocity commands for robot
    linear_vel = kp1*vw[0];
    angular_vel = kp2*vw[1];

    Vtot = sqrt(pow(linear_vel, 2) + pow( angular_vel, 2));

    if (Vtot >= Vmax){
        linear_vel = linear_vel * Vmax / Vtot;
        angular_vel = angular_vel * Vmax / Vtot;
    }

    vl = linear_vel;
    va = angular_vel;
}
double f_x(double a, double b, double t){
    return a*t + b;
}

// Define the time step
const double dt = Ts;

// Function to calculate the discretized line between the robot and goal
std::pair<double, double> discretizeLine(double x1, double y1, double x2, double y2) {
    double dx = (x2 - x1) / dt;
    double dy = (y2 - y1) / dt;
    //ROS_INFO("U1: %f U2: %f", kp * (x1 + dx  - x1), kp * (y1 + dy - y1));
    return std::make_pair(kp * (x1 + dx  - x1) , kp * (y1 + dy - y1));
}

int following = 0;

void moveFollowingEdgeObstacle() {
        // delta 'dl' and 'da' to goal
        double dx = sp[_x_] - po[_x_];
        double dy = sp[_y_] - po[_y_];
        double dl = hypot(dx,dy);
        double da = atan2(dy,dx) - po[_a_];
        // Initial speeds
        v = 0.46;
        w = 0;

        // Control
        if (detected_front) {
            v = 0;
            w = M_PI * (1./2.); // 45 degrees in radians
            following = true;
        } else {
            if (detected_right) {
                w = M_PI * (1./16.); // 15 degrees in radians
            } else if (following) {
                v = 0.1;
                w = -M_PI * (1./6.); // -30 degrees in radians
            }
        }
        double distD = 1.;

        double obs_x_0 = distD*cos(w);
        double obs_y_0 = distD*sin(w);

        double map_x_0 = po[_x_] + obs_x_0 * std::cos(yaw) - obs_y_0 * std::sin(yaw);
        double map_y_0 = po[_y_] + obs_x_0 * std::sin(yaw) + obs_y_0 * std::cos(yaw);
        ROS_INFO("     folow_pose: x: %.3f, y: %.3f, rang: %.3f", map_x_0, map_y_0, wid);
        goal << map_x_0, map_y_0;
        if (std::isnan(goal.x()) || dl < 0.5){
            goal << sp[_x_], sp[_y_];
        }

        // MovePotential();
        Rx = po[_x_];
        Ry = po[_y_];
        Gx = goal.x();
        Gy = goal.y();
        // y = a*x + b
        line = discretizeLine(Rx, Ry, Gx, Gy);


        feedbackLinearization(line.first, line.second);
        // feedbackLinearization(v, w);
}

void switchToMoveToTheTarget() {
    Rx = po[_x_];
    Ry = po[_y_];
    Gx = sp[_x_];
    Gy = sp[_y_];
    // y = a*x + b
    line = discretizeLine(Rx, Ry, Gx, Gy);

    feedbackLinearization(line.first, line.second);
}

int save_d_follow = 1;
int followEdge = 0;

enum class Action { Stop,
                    GoGoal,
                    GoTangent,
                    GoFollowingEdge};

void moveToTheTarget(ros::Publisher pub, geometry_msgs::Twist vel) {
    while (ros::ok()) {
        // delta 'dl' and 'da' to goal
        double dx = sp[_x_] - po[_x_];
        double dy = sp[_y_] - po[_y_];
        double dl = hypot(dx,dy);
        double da = atan2(dy,dx) - po[_a_];
        Action action = Action::Stop;
        //d_folowed = dl;

        if(CheckGoalDir(da,dl)){
            action = Action::GoGoal;
            followEdge = 0;
            d_folowed = INF;
        }else{
            action = Action::GoGoal;
            if(obstacle_detected){
                action = Action::GoTangent;
                if(save_d_follow){
                    d_folowed = d_folowed_at;
                    save_d_follow = 0;
                }

                if(detected_front){
                    if(dl > d_folowed){
                        action = Action::GoFollowingEdge;
                        followEdge = 1;
                    }
                } else {
                    action = Action::GoTangent;
                    followEdge = 0;
                }
            }
        }

        if (CheckGoalDir(da,dl)){
            action = Action::GoGoal;
            save_d_follow = 1;
            followEdge = 0;
            d_folowed = INF;
        } 
        if (followEdge){
            action = Action::GoFollowingEdge;
            if(dl < d_folowed){
                save_d_follow = 1;
                followEdge = 0;
                action = Action::GoGoal;
            }
        }
        // action = Action::GoFollowingEdge;

        switch( action )
        {

            case Action::GoGoal:
                ROS_INFO("  GoGoal");
                Rx = po[_x_];
                Ry = po[_y_];
                Gx = sp[_x_];
                Gy = sp[_y_];
                // y = a*x + b
                line = discretizeLine(Rx, Ry, Gx, Gy);
                ROS_INFO("obstacle_detected : %d", obstacle_detected);
                feedbackLinearization(line.first, line.second);
                break;
            case Action::GoTangent:
                findDiscontinuityPoint();
                Rx = po[_x_];
                Ry = po[_y_];
                Gx = goal.x();
                Gy = goal.y();
                // y = a*x + b
                line = discretizeLine(Rx, Ry, Gx, Gy);

                feedbackLinearization(line.first, line.second);
                
                ROS_INFO("  GoTangent");
                break;
            case Action::GoFollowingEdge:
                moveFollowingEdgeObstacle();
                ROS_INFO("  moveFollowingEdgeObstacle");
                break;
            case Action::Stop:
            default:
                dl = 0.0;
                da = 0.0;
                vl = dl;
                va = da;
                ROS_INFO("  Stop");
                break;
        }
        if (min_range < 0.2) { MovePotential();}
        if (std::isnan(vl) || std::isnan(va)) {
            va = 0;
            vl = 0;
        }
        vel.angular.z = getAngVel();
        vel.linear.x = getLinVel();
        pub.publish(vel);

        ros::spinOnce();
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "catkinha_fc_tangent");

    ros::param::get("~kp", kp); // half-width of lemniscate
    ros::param::get("~kp1", kp1); // half-width of lemniscate
    ros::param::get("~kp2", kp2); // half-width of lemniscate
    ros::param::get("~x_goal", x_d); // half-width of lemniscate
    ros::param::get("~y_goal", y_d); // half-width of lemniscate
    ros::param::get("~d", d); // half-width of lemniscate
    ros::NodeHandle nh;

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Subscriber sub1 = nh.subscribe("base_pose_ground_truth", 10, odomCallback);
    ros::Subscriber sub2 = nh.subscribe("base_scan", 10, UpScan);
    ros::Rate rate( 1 / Ts);


    geometry_msgs::Twist vel_msg;
    sp[_x_] = x_d;
    sp[_y_] = y_d;
    sp[_a_] = 0;

    // current pose
    po[_x_] = 0.0;
    po[_y_] = 0.0;
    po[_a_] = 0.0;

    // desired velocity
    vl = 0.0;
    va = 0.0;

    // controller parameters
    vdmax = 2.0;
    vamax = 1.0;
    atoli = 0.1;
    atole = 0.5;
    dtoli = 0.1;
    dtole = 0.5;


    // laser scan info
    bmin = -(3./4.)*PI;
    bmax = (3./4.)*PI;
    binc = (bmax - bmin)/(samples-1);
    rmin = 1.;
    rmax = laser_d - 0.7;
    for(size_t i = 0; i < samples; i++)
    {
        ranges_laser[i] = 0.0;
        angles_laser[i] = bmin + i*binc;
    }
    geometry_msgs::Twist vel;

    moveToTheTarget(vel_pub, vel);

    return 0;
}