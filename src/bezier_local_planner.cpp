#include "bezier_local_planner/bezier_local_planner.h"
#include <Eigen/Core>
#include <chrono>

namespace bezier_local_planner
{
BezierLocalPlanner::BezierLocalPlanner(base_local_planner::CostmapModel* costmap_model,
                                       const std::vector<geometry_msgs::Point>& footprint_spec, double inscribed_radius,
                                       double circumscribed_radius, const BezierConfig& cfg, ros::NodeHandle nh)
  : costmap_model_(costmap_model)
  , footprint_spec_(footprint_spec)
  , inscribed_radius_(inscribed_radius)
  , circumscribed_radius_(circumscribed_radius)
  , cfg_(&cfg)
{
  initializeCandidateTargets();

  feasible_candidate_paths_pub_ = nh.advertise<nav_msgs::Path>("feasible_candidate_paths", 1);
  all_candidate_paths_pub_ = nh.advertise<nav_msgs::Path>("all_candidate_paths", 1);
  tracked_path_pub_ = nh.advertise<nav_msgs::Path>("tracked_path", 1);
}

BezierLocalPlanner::~BezierLocalPlanner()
{
}

void BezierLocalPlanner::initializeCandidateTargets()
{
  double dy = 1.0 / 10;
  for (int i = 0; i <= 20; i++)
  {
    Pose2D pose;
    pose.y += (-1.0 + i * dy);
    candidate_targets_.push_back(pose);
  }
}

bool BezierLocalPlanner::computeVelocityCommands(const Velocity& robot_vel, const Pose2D& robot_pose,
                                                 const std::vector<Pose2D>& global_plan,
                                                 unsigned char const* const* costmap, int size_x, int size_y,
                                                 double resolution, double origin_x, double origin_y, Velocity& cmd_vel)
{
  return true;
}

bool BezierLocalPlanner::isPathFeasible(const std::vector<Pose2D>& path)
{
  // Number of poses along the path that should be verified
  int look_ahead_idx = (int)path.size() - 1;

  for (int i = 0; i <= look_ahead_idx; ++i)
  {
    if (costmap_model_->footprintCost(path[i].x, path[i].y, path[i].theta, footprint_spec_, inscribed_radius_,
                                      circumscribed_radius_) == -1)
    {
      return false;
    }
    // Checks if the distance between two poses is higher than the robot radius or the orientation diff is bigger than
    // the specified threshold and interpolates in that case. (if obstacles are pushing two consecutive poses away, the
    // center between two consecutive poses might coincide with the obstacle ;-)!
    if (i < look_ahead_idx)
    {
      double delta_rot = NormalizeAngle(path[i + 1].theta - path[i].theta);
      Eigen::Vector2d delta_dist(path[i + 1].x - path[i].x, path[i + 1].y - path[i].y);
      if (fabs(delta_rot) > M_PI || delta_dist.norm() > inscribed_radius_)
      {
        int n_additional_samples =
            std::max(std::ceil(fabs(delta_rot) / M_PI), std::ceil(delta_dist.norm() / inscribed_radius_)) - 1;

        Eigen::Vector2d intermediate_position(path[i].x, path[i].y);
        double intermediate_theta = path[i].theta;
        for (int step = 0; step < n_additional_samples; ++step)
        {
          intermediate_position = intermediate_position + delta_dist / (n_additional_samples + 1.0);
          intermediate_theta = NormalizeAngle(intermediate_theta + delta_rot / (n_additional_samples + 1.0));
          if (costmap_model_->footprintCost(intermediate_position.x(), intermediate_position.y(), intermediate_theta,
                                            footprint_spec_, inscribed_radius_, circumscribed_radius_) == -1)
          {
            return false;
          }
        }
      }
    }
  }
  return true;
}

void BezierLocalPlanner::publishBezierPaths(const std::vector<std::vector<Pose2D> >& bezier_paths,
                                            ros::Publisher publisher)
{
  if (bezier_paths.empty())
    return;

  nav_msgs::Path gui_path;
  gui_path.header.frame_id = cfg_->map_frame;
  gui_path.header.stamp = ros::Time::now();
  geometry_msgs::PoseStamped pose;

  for (int i = 0; i < (int)bezier_paths.size(); i++)
  {
    for (int j = 0; j < (int)bezier_paths[i].size(); j++)
    {
      pose.pose.position.x = bezier_paths[i][j].x;
      pose.pose.position.y = bezier_paths[i][j].y;
      pose.pose.orientation = tf::createQuaternionMsgFromYaw(bezier_paths[i][j].theta);
      gui_path.poses.push_back(pose);
    }

    for (int j = (int)bezier_paths[i].size() - 1; j >= 0; j--)
    {
      pose.pose.position.x = bezier_paths[i][j].x;
      pose.pose.position.y = bezier_paths[i][j].y;
      pose.pose.orientation = tf::createQuaternionMsgFromYaw(bezier_paths[i][j].theta);
      gui_path.poses.push_back(pose);
    }
  }

  publisher.publish(gui_path);
}

void BezierLocalPlanner::publishBezierPath(const std::vector<Pose2D>& bezier_path, ros::Publisher publisher)
{
  nav_msgs::Path gui_path;
  gui_path.header.frame_id = cfg_->map_frame;
  gui_path.header.stamp = ros::Time::now();
  geometry_msgs::PoseStamped pose;

  for (int i = 0; i < (int)bezier_path.size(); i++)
  {
    pose.pose.position.x = bezier_path[i].x;
    pose.pose.position.y = bezier_path[i].y;
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(bezier_path[i].theta);
    gui_path.poses.push_back(pose);
  }

  publisher.publish(gui_path);
}

}  // namespace bezier_local_planner