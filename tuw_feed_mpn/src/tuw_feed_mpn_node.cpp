//
// Created by felix on 14.02.19.
//

#include <tuw_feed_mpn/tuw_feed_mpn_node.h>

using namespace tuw;

FeedMpnNode::FeedMpnNode(ros::NodeHandle &nh) : nh_(nh), tf_buffer_(ros::Duration(50, 0)), tf_listener_(tf_buffer_)
{
  latest_pose_ = nullptr;
  trackedId_ = -1.0;

  nh_.param<int>("rate", rate_, 1);

  world_frame_ = std::string("map");
  robot_frame_ = std::string("r0/laser0");
  nav_goal_topic_ = std::string("nav_goal");
  pub_topic_ = nh_.advertise<geometry_msgs::PoseStamped>(nav_goal_topic_, 1);
}

void FeedMpnNode::callbackObjectDetection(tuw_object_msgs::ObjectDetection &tracks)
{
  if (!triggered_ && trackedId_ == INVALID_ID)
  {
    return;
  } else if (trackedId_ != INVALID_ID) //initialized: do tracking
  {

    for (auto &&object : tracks.objects)
    {
      if (object.object.ids[0] == trackedId_)
      {
        auto &pose_wf = object.object.pose;
        latest_pose_.reset(new geometry_msgs::PoseStamped());
        latest_pose_->header.stamp = ros::Time::now();
        latest_pose_->header.frame_id = tracks.header.frame_id;
        latest_pose_->pose = pose_wf;
      }
    }

  } else //first time: do initialization
  {
    initAndCheck(tracks);
  }
}

bool FeedMpnNode::initAndCheck(tuw_object_msgs::ObjectDetection &tracks)
{
  std::vector<float> dists(tracks.objects.size());
  uint32_t tidx=0;
  uint32_t tidx_chosen=0;
  for (auto &&object : tracks.objects)
  {

    auto &pose_wf = object.object.pose;
    geometry_msgs::PoseStamped pose_wf_stamped;
    pose_wf_stamped.pose = pose_wf;
    pose_wf_stamped.header = tracks.header;

    //geometry_msgs::TransformStamped stamped_tf = tf_buffer_.lookupTransform(
    //    world_frame_, robot_frame_, ros::Time( 0 ));

    geometry_msgs::PointStamped point_wf;
    point_wf.header = tracks.header;
    point_wf.point = pose_wf.position;
    geometry_msgs::PointStamped point_rf;
    tf_buffer_.transform(point_wf, point_rf, robot_frame_);

    double xx = point_rf.point.x * point_rf.point.x;
    double yy = point_rf.point.y * point_rf.point.y;
    double zz = point_rf.point.z * point_rf.point.z;
    double euclidean = sqrt(xx + yy + zz);

    dists[tidx++] = euclidean;
    if (euclidean < init_radius_)
    {
      tidx_chosen = tidx - 1;
      trackedId_ = object.object.ids[0];
      latest_pose_.reset(new geometry_msgs::PoseStamped(pose_wf_stamped));
      break;
    }

  }

  ROS_INFO("From %d tracks %d has been chosen", tracks.objects.size(), trackedId_);
  ROS_INFO("Distance to robot is: %d", dists[tidx_chosen]);
  return true;
}

void FeedMpnNode::triggerFeedingProcess(std_msgs::Float64 &init_radius_trigger)
{
  init_radius_ = init_radius_trigger.data;
  triggered_ = true;
}

void FeedMpnNode::publish()
{
  if (latest_pose_)
  {
    pub_topic_.publish(latest_pose_);
  }
}

int main(int argc, char **argv)
{
  ros::NodeHandle nh;


  FeedMpnNode feeder(nh);
  ros::Rate r(feeder.getRate());

  while (ros::ok())
  {
    feeder.publish();
  }
}
