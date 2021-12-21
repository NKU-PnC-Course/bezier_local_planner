#ifndef BEZIER_LOCAL_PLANNER_BEZIER_LOCAL_PLANNER_H
#define BEZIER_LOCAL_PLANNER_BEZIER_LOCAL_PLANNER_H

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <base_local_planner/costmap_model.h>
#include <tf/tf.h>
#include <ros/publisher.h>
#include <nav_msgs/Path.h>
#include "bezier_local_planner/utils.h"

namespace bezier_local_planner
{
class BezierLocalPlanner
{
public:
  BezierLocalPlanner(base_local_planner::CostmapModel* costmap_model,
                     const std::vector<geometry_msgs::Point>& footprint_spec, double inscribed_radius,
                     double circumscribed_radius, const BezierConfig& cfg, ros::NodeHandle nh);
  virtual ~BezierLocalPlanner();

  bool computeVelocityCommands(const Velocity& robot_vel, const Pose2D& robot_pose,
                               const std::vector<Pose2D>& global_plan, unsigned char const* const* costmap, int size_x,
                               int size_y, double resolution, double origin_x, double origin_y, Velocity& cmd_vel);

private:
  /**
   * @brief Check whether the planned path is feasible or not.
   *
   * This method currently checks only that the path, or a part of the path is collision free.
   * Obstacles are here represented as costmap instead of the internal ObstacleContainer.
   * @return \c true, if the robot footprint along the first part of the path intersects with
   *         any obstacle in the costmap, \c false otherwise.
   */
  bool isPathFeasible(const std::vector<Pose2D>& path);

  // publish multi bezier curves
  void publishBezierPaths(const std::vector<std::vector<Pose2D> >& bezier_paths, ros::Publisher publisher);
  // publish single bezier curve
  void publishBezierPath(const std::vector<Pose2D>& bezier_path, ros::Publisher publisher);
  void initializeCandidateTargets();

private:
  const BezierConfig* cfg_;  //!< Config class that stores and manages all related parameters

  base_local_planner::CostmapModel* costmap_model_;   //!< Pointer to the costmap model
  std::vector<geometry_msgs::Point> footprint_spec_;  //!< The specification of the footprint of the robot in world
                                                      //!< coordinates
  double inscribed_radius_;                           //!< The radius of the inscribed circle of the robot
  double circumscribed_radius_;                       //!< The radius of the circumscribed circle of the robot

  std::vector<Pose2D> candidate_targets_;

  ros::Publisher all_candidate_paths_pub_, feasible_candidate_paths_pub_, tracked_path_pub_;
};

}  // namespace bezier_local_planner

#endif  // BEZIER_LOCAL_PLANNER_BEZIER_LOCAL_PLANNER_H