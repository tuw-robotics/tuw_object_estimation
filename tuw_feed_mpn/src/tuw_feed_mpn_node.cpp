//
// Created by felix on 14.02.19.
//

#include <tuw_feed_mpn/tuw_feed_mpn_node.h>

using namespace tuw;

FeedMpnNode::FeedMpnNode(ros::NodeHandle &nh) : nh_(nh), tf_buffer_(ros::Duration(50, 0)), tf_listener_(tf_buffer_)
{
  latest_pose_ = nullptr;
  trackedId_ = -1.0;

  world_frame_ = std::string("map");
  robot_frame_ = std::string("p3dx/laser_front_right");
  nav_goal_topic_ = std::string("nav_goal");
  std::string person_route_ = std::string("person_route");
  pub_topic_ = nh_.advertise<geometry_msgs::PoseStamped>(nav_goal_topic_, 1);
  pub_track_points_ = nh_.advertise<visualization_msgs::Marker>(std::string("track_points"), 10);
  pub_track_lines_ = nh_.advertise<visualization_msgs::Marker>(std::string("track_lines"), 10);
  config_.distance = 1.0;
  config_.delay = 0;
  config_.timeout = 10;
  timeout_time_ = ros::Duration(config_.timeout);

  sub_tracks_ = nh_.subscribe("/r0/tracked_people", 1, &FeedMpnNode::callbackObjectDetection, this);
  reconfigureFnc_ = boost::bind(&FeedMpnNode::callbackParameters, this, _1, _2);
  reconfigureServer_.setCallback(reconfigureFnc_);
  triggered_ = true;
  init_radius_ = 10.0;
}

void FeedMpnNode::callbackParameters(tuw_feed_mpn::tuw_feed_mpnConfig &config, uint32_t level)
{
  config_.distance = config.save_distance;
  config_.rate = config.rate;
  config_.timeout = config.timeout;
  config_.auto_reinitialize = config.auto_reinitialize;
  timeout_time_ = ros::Duration(config_.timeout);
}

void FeedMpnNode::appendToRouteSegments(geometry_msgs::PoseStampedPtr &poseStamped)
{
  if (track_points_.points.size() == 0)
  {
    track_points_.header = poseStamped->header;
    track_points_.id = 0;
    track_points_.type = visualization_msgs::Marker::POINTS;
    track_points_.ns = "person_trajectory";
    track_points_.scale.x = 0.1;
    track_points_.scale.y = 0.1;
    track_points_.color.g = 1;
    track_points_.color.a = 1;

    track_lines_.header = poseStamped->header;
    track_lines_.id = 1;
    track_lines_.ns = "person_trajectory";
    track_lines_.scale.x = 0.1;
    track_lines_.scale.y = 0.1;
    track_lines_.type = visualization_msgs::Marker::LINE_STRIP;
    track_lines_.color.b = 1.0;
    track_lines_.color.a = 1.0;
  }
  track_points_.points.push_back(poseStamped->pose.position);
  track_lines_.points.push_back(poseStamped->pose.position);
}

void FeedMpnNode::callbackObjectDetection(const tuw_object_msgs::ObjectDetection &tracks)
{
  ROS_INFO("getting tracks");
  ros::Duration duration = ros::Time::now() - last_track_update_;
  std::cout << "duration since last üpdate " << duration.sec << "s" << std::endl;
  if (duration > timeout_time_)
  {
    ROS_INFO("timeout");
    if (config_.auto_reinitialize)
    {
      triggered_ = true;
      trackedId_ = INVALID_ID;
      ROS_INFO("auto-reinitialize");
    } else
    {
      ROS_INFO("auto-reinitialize deactivated: track is lost");
      trackedId_ = INVALID_ID;
    }
  }
  if (!triggered_ && trackedId_ == INVALID_ID)
  {
    ROS_INFO("not triggered therefore do nothing");
    return;
  } else if (trackedId_ != INVALID_ID) //initialized: do tracking
  {
    ROS_INFO("find matching track and update pose");
    for (auto &&object : tracks.objects)
    {
      if (object.object.ids[0] == trackedId_)
      {
        auto &pose_rf = object.object.pose;
        latest_pose_.reset(new geometry_msgs::PoseStamped());
        latest_pose_->header.stamp = ros::Time::now();
        latest_pose_->header.frame_id = tracks.header.frame_id;
        latest_pose_->pose = pose_rf;
        appendToRouteSegments(latest_pose_);
        last_track_update_ = ros::Time::now();
      }
    }
  } else //first time: do initialization
  {
    ROS_INFO("init sequence");
    if (!initAndCheck(tracks))
    {
      ROS_INFO("init not sucessful, try standing not too close to the robot");
    } else
    {
      last_track_update_ = ros::Time::now();
    }
  }
}

bool FeedMpnNode::initAndCheck(const tuw_object_msgs::ObjectDetection &tracks)
{
  std::vector<float> dists(tracks.objects.size());
  uint32_t tidx = 0;
  uint32_t tidx_chosen = 0;
  for (auto &&object : tracks.objects)
  {
    auto &pose_rf = object.object.pose;
    geometry_msgs::PoseStamped pose_rf_stamped;
    pose_rf_stamped.pose = pose_rf;
    pose_rf_stamped.header = tracks.header;

    //geometry_msgs::TransformStamped stamped_tf = tf_buffer_.lookupTransform(
    //    world_frame_, robot_frame_, ros::Time( 0 ));

    //geometry_msgs::PointStamped point_wf;
    //point_wf.header = tracks.header;
    //point_wf.point = pose_wf.position;
    //geometry_msgs::PointStamped point_rf;
    //tf_buffer_.transform(point_wf, point_rf, robot_frame_);

    double xx = pose_rf_stamped.pose.position.x * pose_rf_stamped.pose.position.x;
    double yy = pose_rf_stamped.pose.position.y * pose_rf_stamped.pose.position.y;
    double zz = pose_rf_stamped.pose.position.z * pose_rf_stamped.pose.position.z;
    double euclidean = sqrt(xx + yy + zz);

    dists[tidx++] = euclidean;
    if (euclidean < init_radius_)
    {
      tidx_chosen = tidx - 1;
      trackedId_ = object.object.ids[0];
      latest_pose_.reset(new geometry_msgs::PoseStamped(pose_rf_stamped));
      geometry_msgs::PointStamped point_rf;
      ROS_INFO("From %d tracks %d has been chosen", static_cast<int>(tracks.objects.size()),
               static_cast<int>(trackedId_));
      ROS_INFO("Distance to robot is: %lf", dists[tidx_chosen]);
      return true;
    }

  }
  return false;
}

void FeedMpnNode::triggerFeedingProcess(std_msgs::Float64 &init_radius_trigger)
{
  init_radius_ = init_radius_trigger.data;
  triggered_ = true;
}

bool FeedMpnNode::checkSaveDistance(geometry_msgs::PoseStampedPtr &poseStamped)
{
  auto &position = poseStamped->pose.position;
  double xx = position.x * position.x;
  double yy = position.y * position.y;

  return std::sqrt(xx + yy) > config_.distance;
}

void FeedMpnNode::publish()
{
  if (latest_pose_ && checkSaveDistance(latest_pose_))
  {
    pub_topic_.publish(latest_pose_);
    pub_track_points_.publish(track_points_);
    pub_track_lines_.publish(track_lines_);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tuw_feed_mpn_node");

  ros::NodeHandle nh;
  FeedMpnNode feeder(nh);
  ros::Rate r(feeder.getRate());

  while (ros::ok())
  {
    //rate can change via reconfigure
    r = ros::Rate(feeder.getRate());
    feeder.publish();
    r.sleep();
    ros::spinOnce();
  }
}
