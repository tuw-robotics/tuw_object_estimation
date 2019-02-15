//
// Created by felix on 14.02.19.
//

#ifndef TUW_FEED_MPN_NODE_H
#define TUW_FEED_MPN_NODE_H

#include <tuw_object_msgs/ObjectDetection.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/convert.h>
#include <tf2/time_cache.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <message_filters/subscriber.h>

namespace tuw
{

  class FeedMpnNode
  {
  public:

    /**
     *
     */
    FeedMpnNode(ros::NodeHandle &nh);

    /**
     *
     * Get tracked objects and feed to mpn at the specified rate
     *
     * @param track
     */
    void callbackObjectDetection(tuw_object_msgs::ObjectDetection &tracks);

    /**
     *
     * @param init_radius_trigger
     */
    void triggerFeedingProcess(std_msgs::Float64 &init_radius_trigger);

    /**
     * Select the first track that is closest to the robot
     *
     * @param tracks
     * @return
     */
    bool initAndCheck(tuw_object_msgs::ObjectDetection &tracks);

    /**
     * Publishes the last known pose of the object.
     *
     */
    void publish();

  private:

    int INVALID_ID = -1;
    ros::NodeHandle nh_;
    double init_radius_;
    bool triggered_;
    int trackedId_;
    std::string world_frame_;
    std::string robot_frame_;
    std::string nav_goal_topic_;

    tf2_ros::TransformListener tf_listener_;
    tf2_ros::Buffer tf_buffer_;
    geometry_msgs::PoseStampedPtr latest_pose_;
    ros::Publisher pub_topic_;
    int rate_;
    //Pose

  };
};
#endif //PROJECT_FEED_MPN_NODE_H
