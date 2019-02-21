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
#include <dynamic_reconfigure/server.h>
#include <message_filters/subscriber.h>
#include <tuw_feed_mpn/tuw_feed_mpnConfig.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

namespace tuw
{

  class FeedMpnNode
  {
  public:

    struct FeedingConfig {
      double distance;
      double delay;
      double timeout;
      int rate;
      bool auto_reinitialize;
    };

    /**
     *
     */
    FeedMpnNode(ros::NodeHandle &nh);

    /**
     *
     * Callback for Parameterserver
     *
     * @param config
     * @param level
     */
    void callbackParameters(tuw_feed_mpn::tuw_feed_mpnConfig &config, uint32_t level);

    /**
     *
     * Get tracked objects and feed to mpn at the specified rate
     *
     * @param track
     */
    void callbackObjectDetection(const tuw_object_msgs::ObjectDetection &tracks);

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
    bool initAndCheck(const tuw_object_msgs::ObjectDetection &tracks);

    /**
     * Publishes the last known pose of the object.
     *
     */
    void publish();

    /**
     *
     * @param poseStamped check if it is save to forward a goal to the mpn.
     */
    bool checkSaveDistance(geometry_msgs::PoseStampedPtr &poseStamped);

    /**
     * get the publisher rate
     *
     * @return
     */
    int getRate()
    {
      return config_.rate;
    }

    /**
     * Used for visualization -> make message out of route the person has travelled
     *
     * @param poseStamped
     */
    void appendToRouteSegments(geometry_msgs::PoseStampedPtr &poseStamped);

  private:
    //ros
    ros::Subscriber sub_tracks_;
    ros::Publisher pub_topic_;
    ros::Publisher pub_track_points_;
    ros::Publisher pub_track_lines_;
    visualization_msgs::Marker track_points_;
    visualization_msgs::Marker track_lines_;
    ros::Time last_track_update_;
    ros::Duration timeout_time_;

    //reconfigure
    dynamic_reconfigure::Server<tuw_feed_mpn::tuw_feed_mpnConfig> reconfigureServer_;
    dynamic_reconfigure::Server<tuw_feed_mpn::tuw_feed_mpnConfig>::CallbackType reconfigureFnc_;

    FeedingConfig config_;

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
    tf::TransformListener tf1_listener_;
    geometry_msgs::PoseStampedPtr latest_pose_;
    int rate_;
    //Pose

  };
};
#endif //PROJECT_FEED_MPN_NODE_H
