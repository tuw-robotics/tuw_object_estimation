/***************************************************************************
 *   Software License Agreement (BSD License)                              *
 *   Copyright (C) 2017 by                                                 *
 *   Klaus Buchegger <klaus.buchegger@student.tuwien.ac.at>                *
 *   Florian Beck    <florian.beck@tuwien.ac.at>                           *
 *                                                                         *
 *   Redistribution and use in source and binary forms, with or without    *
 *   modification, are permitted provided that the following conditions    *
 *   are met:                                                              *
 *                                                                         *
 *   1. Redistributions of source code must retain the above copyright     *
 *      notice, this list of conditions and the following disclaimer.      *
 *   2. Redistributions in binary form must reproduce the above copyright  *
 *      notice, this list of conditions and the following disclaimer in    *
 *      the documentation and/or other materials provided with the         *
 *      distribution.                                                      *
 *   3. Neither the name of the copyright holder nor the names of its      *
 *      contributors may be used to endorse or promote products derived    *
 *      from this software without specific prior written permission.      *
 *                                                                         *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS   *
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT     *
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS     *
 *   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE        *
 *   COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,  *
 *   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,  *
 *   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;      *
 *   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER      *
 *   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT    *
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY *
 *   WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           *
 *   POSSIBILITY OF SUCH DAMAGE.                                           *
 ***************************************************************************/

#include <algorithm>
#include <assert.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tuw_tracking_heatmap/tuw_tracking_heatmap_node.h>
#include <boost/foreach.hpp>
#include <grid_map_cv/grid_map_cv.hpp>
#include <geometry_msgs/PoseArray.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/math/distributions/normal.hpp>

#define foreach BOOST_FOREACH

using namespace grid_map;
using namespace tuw;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tracking_heatmap_node");  /// initializes the ros node with default name
  ros::NodeHandle n;

  TrackingHeatmapNode heatmapNode(n);
  heatmapNode.publish();
  while (ros::ok())
  {
    ros::spin();
  }
  return 0;
}

TrackingHeatmapNode::TrackingHeatmapNode(ros::NodeHandle& n) : n_(n), n_param_("~")
{
  ROS_INFO("initializing tracking heatmap node");
  
  
  subTrackings_ = n.subscribe("/human_detections", 50, &TrackingHeatmapNode::trackingCallback, this);
  subGlobalMap_ = n.subscribe("/map", 1, &TrackingHeatmapNode::globalMapCallback, this);
  pubGridMap_ = n.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
  pubVectorField_.emplace_back(n.advertise<visualization_msgs::MarkerArray>("/vector_field_north", 1, true));
  pubVectorField_.emplace_back(n.advertise<visualization_msgs::MarkerArray>("/vector_field_east", 1, true));
  pubVectorField_.emplace_back(n.advertise<visualization_msgs::MarkerArray>("/vector_field_south", 1, true));
  pubVectorField_.emplace_back(n.advertise<visualization_msgs::MarkerArray>("/vector_field_west", 1, true));
  pubVectorField_.emplace_back(n.advertise<visualization_msgs::MarkerArray>("/vector_field_forces", 1, true));
  pubHistTrajectories_ = n.advertise<tuw_object_msgs::ObjectWithCovarianceArrayArray>("trajectory_array", 10, true);

  timer_ = n_.createTimer(ros::Duration(0.1), boost::bind(&TrackingHeatmapNode::timerCallback, this, _1));
  savingTimer_ = n_.createTimer(ros::Duration(60), boost::bind(&TrackingHeatmapNode::savingTimerCallback, this, _1));

  n_param_.param("grid_map_bagfile", grid_map_bagfile_, std::string("heatmap_bagfile"));
  n_param_.param("grid_map_bagfile_out", grid_map_bagfile_out_, std::string("heatmap_bagfile_out"));

  n_param_.param("apply_blur", apply_blur_, false);
  n_param_.param("ksize", ksize_, 15);

  n_param_.param("likelihood_field_sigma", likelihood_field_sigma_, 5.0);

  n_param_.param("compute_social_forces", compute_social_forces_, false);
  n_param_.param("a_obj", a_obj_, 10.0);
  n_param_.param("b_obj", b_obj_, 0.01);
  n_param_.param("c_obj", c_obj_, 600.0);
  n_param_.param("r_pers", r_pers_, 0.2);
  n_param_.param("border_length", border_length_, 10);
  n_param_.param("scale", scale_, 1e+10);
  n_param_.param("person_weight", person_weight_, 0.1);

  n_param_.param("image_output_dir", image_output_dir_, std::string("."));

  // check whether a bagfile is provided
  // if it is start post processing, i.e. apply blur, precompute stuff
  // otherwise, initialize new grid map and calculate initial likelihood field
  // before recording tracks into the all_dirs layer
  try
  {
    rosbag::Bag bag;
    bag.open(grid_map_bagfile_, rosbag::bagmode::Read);
    rosbag::View view(bag, rosbag::TopicQuery("heatmap_grid_map"));
    foreach (rosbag::MessageInstance const m, view)
    {
      GridMapRosConverter::fromMessage(*m.instantiate<grid_map_msgs::GridMap>(), map_);
    }
    bag.close();
    ROS_INFO("loaded heatmap backup");

    if (!map_.exists("sum"))
      map_.add("sum", 0.0);

    // normalize heatmap to values of likelihood field
    float max = map_["likelihood_field"].maxCoeff();
    float min = map_["likelihood_field"].minCoeff();

    std::cout << "likelihood min max: " << min << ", " << max << " all_dirs min max: " << map_["all_dirs"].minCoeff()
              << ", " << map_["all_dirs"].maxCoeff() << std::endl;

    for (size_t row = 0; row < map_["all_dirs"].rows(); row++)
    {
      for (size_t col = 0; col < map_["all_dirs"].cols(); col++)
      {
        map_["all_dirs"](row, col) = (map_["all_dirs"](row, col) - min) / (max - min);
      }
    }

    map_["sum"] = map_["likelihood_field"] + map_["all_dirs"];

    // add layer which is a sum of all_dirs and the initial likelihood_field
    cv::Mat all_dirs, all_dirs_and_initial_heatmap_image, likelihood_field_image, result;
    grid_map::GridMapCvConverter::toImage<float, 1>(map_, "sum", CV_32FC1, 0, map_["sum"].maxCoeff(), result);
    result.convertTo(result, CV_8UC3, 255.0);
    cv::applyColorMap(result, all_dirs_and_initial_heatmap_image, cv::COLORMAP_JET);
    cv::imwrite(image_output_dir_ + "result.jpg", result, std::vector<int>({ CV_IMWRITE_JPEG_QUALITY, 100 }));
    cv::imwrite(image_output_dir_ + "all_dirs_and_initial_heatmap.jpg", all_dirs_and_initial_heatmap_image,
                std::vector<int>({ CV_IMWRITE_JPEG_QUALITY, 100 }));

    if (apply_blur_)
    {
      ROS_INFO("applying Gaussian blur");
      // blur grid map
      cv::Mat originalImage, blurredImage;
      grid_map::GridMapCvConverter::toImage<unsigned short, 1>(map_, "all_dirs", CV_16UC1, 0.0, 30.0, originalImage);
      cv::GaussianBlur(originalImage, blurredImage, cv::Size(ksize_, ksize_), 0.0, 0.0);
      grid_map::GridMapCvConverter::addLayerFromImage<unsigned short, 1>(blurredImage, "all_dirs_blur", map_, 0.0, 30.0);

        for (size_t row = 0; row < map_["all_dirs_blur"].rows(); row++)
        {
          for (size_t col = 0; col < map_["all_dirs_blur"].cols(); col++)
          {
            map_["all_dirs"](row, col) = (map_["all_dirs_blur"](row, col) - min) / (max - min);
          }
        }

        map_.add("sum_blur", map_["likelihood_field"] + map_["all_dirs_blur"]);

        // add layer which is a sum of all_dirs and the initial likelihood_field
        grid_map::GridMapCvConverter::toImage<float, 1>(map_, "sum_blur", CV_32FC1, 0, map_["sum_blur"].maxCoeff(), result);
        result.convertTo(result, CV_8UC3, 255.0);
        cv::applyColorMap(result, all_dirs_and_initial_heatmap_image, cv::COLORMAP_JET);
        cv::imwrite(image_output_dir_ + "result_blur.jpg", result, std::vector<int>({ CV_IMWRITE_JPEG_QUALITY, 100 }));
        cv::imwrite(image_output_dir_ + "all_dirs_and_initial_heatmap_blur.jpg", all_dirs_and_initial_heatmap_image,
                    std::vector<int>({ CV_IMWRITE_JPEG_QUALITY, 100 }));
    }

    cv::namedWindow("Display window 1", cv::WINDOW_NORMAL);
    cv::imshow("Display window 1", all_dirs_and_initial_heatmap_image);
    cv::waitKey(500);

    //precomputeVectorField();
    if (compute_social_forces_)
    {
      computeSocialForces();
    }
  }
  catch (rosbag::BagIOException e)
  {
    std::vector<std::string> layers = { "north_x", "north_y", "east_x", "east_y", "south_x", "south_y", "west_x",
                                        "west_y", "all_dirs", "global_map", "likelihood_field", "f_soc_phys_x",
                                        "f_soc_phys_y", "angle_0", "angle_1", "angle_2", "angle_3", "angle_4",
                                        "angle_5", "angle_6", "angle_7", "binary_map", "bresenham_visited", "sum" };

    map_ = GridMap(layers);
    map_.setFrameId("map");
    map_.setGeometry(Length(5.0, 5.0), 0.01, Position::Zero());
    clearMap();
  }

  layers_.emplace_back(&map_["north_x"]);
  layers_.emplace_back(&map_["north_y"]);
  layers_.emplace_back(&map_["east_x"]);
  layers_.emplace_back(&map_["east_y"]);
  layers_.emplace_back(&map_["south_x"]);
  layers_.emplace_back(&map_["south_y"]);
  layers_.emplace_back(&map_["west_x"]);
  layers_.emplace_back(&map_["west_y"]);
  layers_.emplace_back(&map_["all_dirs"]);
  layers_.emplace_back(&map_["likelihood_field"]);
  layers_.emplace_back(&map_["f_soc_phys_x"]);
  layers_.emplace_back(&map_["f_soc_phys_y"]);
  layers_.emplace_back(&map_["angle_0"]);
  layers_.emplace_back(&map_["angle_1"]);
  layers_.emplace_back(&map_["angle_2"]);
  layers_.emplace_back(&map_["angle_3"]);
  layers_.emplace_back(&map_["angle_4"]);
  layers_.emplace_back(&map_["angle_5"]);
  layers_.emplace_back(&map_["angle_6"]);
  layers_.emplace_back(&map_["angle_7"]);
  layers_.emplace_back(&map_["binary_map"]);
  layers_.emplace_back(&map_["bresenham_visited"]);
}

void TrackingHeatmapNode::clearMap(void)
{
  for (GridMapIterator it(map_); !it.isPastEnd(); ++it)
  {
    for (auto layer : layers_)
    {
      (*layer)(getLinearIndexFromIndex(*it, map_.getSize())) = 0;
    }
  }
}

void TrackingHeatmapNode::publish(void)
{
  grid_map_msgs::GridMap message;
  std::vector<std::string> layers = { "all_dirs", "global_map", "likelihood_field", "sum" };
  map_.setTimestamp(ros::Time::now().toNSec());
  GridMapRosConverter::toMessage(map_, layers, message);
  hist_trajectories_array_of_arrays_.header.stamp = ros::Time::now();
  pubHistTrajectories_.publish(hist_trajectories_array_of_arrays_);
  pubGridMap_.publish(message);
}

void TrackingHeatmapNode::globalMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& _globalMap)
{
  double resolution = _globalMap->info.resolution;
  double lengthX = _globalMap->info.width * resolution;
  double lengthY = _globalMap->info.height * resolution;
  Position origin =
      Position(_globalMap->info.origin.position.x + lengthX / 2, _globalMap->info.origin.position.y + lengthY / 2);
  if (abs(lengthX - map_.getLength().x()) > resolution * 5 || abs(lengthY - map_.getLength().y()) > resolution * 5)
  {
    ROS_INFO("received global map size differs too much from internal map, creating a new internal map");
    map_.setGeometry(Length(lengthX, lengthY), resolution * 5, origin);
    ROS_INFO("Set map size to %f x %f m (%i x %i cells).", map_.getLength().x(), map_.getLength().y(),
             map_.getSize()(0), map_.getSize()(1));
    ROS_INFO("lengthX %f, lengthY %f, resolution %f, origin %f;%f", lengthX, lengthY, resolution * 5,
             _globalMap->info.origin.position.x + lengthX / 2, _globalMap->info.origin.position.y + lengthY / 2);

    if (!grid_map::GridMapRosConverter::fromOccupancyGrid(*_globalMap, "global_map", map_))
      ROS_ERROR("adding global_map layer failed");

    clearMap();
    computeInitialLikelihoodField();
    // createPathCrossingMap();
    // createPathObstacleMap();
  }

  map_.setTimestamp(ros::Time::now().toNSec());
  ROS_INFO("map backup");
  grid_map_msgs::GridMap message;
  GridMapRosConverter::toMessage(map_, message);
  rosbag::Bag bag;
  bag.open(grid_map_bagfile_out_, rosbag::bagmode::Write);
  bag.write("heatmap_grid_map", ros::Time::now(), message);
  bag.close();
  ROS_INFO("map backup done");
}

void TrackingHeatmapNode::trackingCallback(const tuw_object_msgs::ObjectDetection::ConstPtr& _detection)
{
  tuw_object_msgs::Object object;
  std_msgs::Header header = _detection->header;
  for (auto it = _detection->objects.begin(); it != _detection->objects.end(); it++)
  {
    //hist_trajectories_[it->object.ids[0]].objects.emplace_back(*it);
    //hist_trajectories_[it->object.ids[0]].header = header;
    object = (*it).object;
    double speed =
        std::sqrt(object.twist.linear.x * object.twist.linear.x + object.twist.linear.y * object.twist.linear.y);
    if (speed > 0.2)
    {
      geometry_msgs::PoseStamped sourcePose;
      geometry_msgs::PoseStamped targetPose;
      sourcePose.pose = object.pose;
      sourcePose.header = header;
      try
      {
        tf_listener_.transformPose(tf::resolve(n_.getNamespace(), "/map"), sourcePose, targetPose);
        double roll, pitch, yaw;
        int direction_layer;
        tf::Matrix3x3(tf::Quaternion(targetPose.pose.orientation.x, targetPose.pose.orientation.y,
                                     targetPose.pose.orientation.z, targetPose.pose.orientation.w))
            .getRPY(roll, pitch, yaw);

        Position position = Position(targetPose.pose.position.x, targetPose.pose.position.y);
        auto search = lastPoses_.find(object.ids[0]);
        if (search != lastPoses_.end())
        {
          Position start = Position(lastPoses_[object.ids[0]].position.x, lastPoses_[object.ids[0]].position.y);
          Index si, pi;
          map_.getIndex(start, si);
          map_.getIndex(position, pi);
          if (si[0] != pi[0] && si[1] != pi[1])
          {
            lastPoses_[object.ids[0]] = targetPose.pose;
            grid_map::LineIterator lineIt(map_, start, position);
            for (++lineIt; !lineIt.isPastEnd(); ++lineIt)
            {
              (*layers_[8])(getLinearIndexFromIndex(*lineIt, map_.getSize())) += 1.0 * person_weight_;
            }
          }
        }
        else
        {
          Index ind;
          map_.getIndex(position, ind);
          (*layers_[8])(getLinearIndexFromIndex(ind, map_.getSize())) += 1.0 * person_weight_;
          lastPoses_.emplace(object.ids[0], targetPose.pose);
        }
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
      }
    }
  }
}

void TrackingHeatmapNode::timerCallback(const ros::TimerEvent& event)
{
  for(auto&& map_it = hist_trajectories_.begin(); map_it != hist_trajectories_.end(); map_it++)
  {
    hist_trajectories_array_of_arrays_.objects_array.emplace_back(map_it->second);
  }
  hist_trajectories_array_of_arrays_.header.frame_id = "map";
  
  cv::Mat all_dirs_image, all_dirs_heatmap_image;
  /*grid_map::GridMapCvConverter::toImage<float, 1>(map_, "all_dirs", CV_32FC1, 0, map_["all_dirs"].maxCoeff(),
                                                  all_dirs_image);
  all_dirs_image.convertTo(all_dirs_image, CV_8UC3, 255.0);
  cv::applyColorMap(all_dirs_image, all_dirs_heatmap_image, cv::COLORMAP_JET);
  cv::imwrite(image_output_dir_, all_dirs_image, std::vector<int>({ CV_IMWRITE_JPEG_QUALITY, 100 }));
  cv::imwrite(image_output_dir_, all_dirs_heatmap_image, std::vector<int>({ CV_IMWRITE_JPEG_QUALITY, 100 }));
  cv::namedWindow("Display window", cv::WINDOW_NORMAL);
  cv::imshow("Display window", all_dirs_heatmap_image);
  cv::waitKey(500);*/

  publish();
}

void TrackingHeatmapNode::savingTimerCallback(const ros::TimerEvent& event)
{
  ROS_INFO("map backup");
  
  grid_map_msgs::GridMap message;
  GridMapRosConverter::toMessage(map_, message);
  rosbag::Bag bag;
  bag.open(grid_map_bagfile_out_, rosbag::bagmode::Write);
  bag.write("heatmap_grid_map", ros::Time::now(), message);
  bag.write("hist_trajectories", ros::Time::now(), hist_trajectories_array_of_arrays_);
  bag.close();
  ROS_INFO("map backup done");
}

void TrackingHeatmapNode::precomputeVectorField(void)
{
  ROS_INFO("precomputing vector field");
  int i = 0;
  // This Matrix defines the x,y coordinates (pixel) of the Polygon
  // Polygon is defined by 4 points (see Polygon.png)
  // Array constains x,y (innermost {}) offsets of points "back", "left",
  // "front", "right" per row, each row is one direction
  // back, left, front, right
  int idx_offs[4][4][2] = { { { 0, -1 }, { -10, 0 }, { 0, -4 }, { 10, 0 } },  // east
                            { { -1, 0 }, { 0, 10 }, { -4, 0 }, { 0, -10 } },  // south
                            { { 0, 1 }, { -10, 0 }, { 0, 4 }, { 10, 0 } },    // west
                            { { 1, 0 }, { 0, 10 }, { 4, 0 }, { 0, -10 } } };  // north
  std_msgs::ColorRGBA colors[4];
  colors[0].a = 1;
  colors[1].a = 1;
  colors[2].a = 1;
  colors[3].a = 1;
  colors[0].r = 1;
  colors[0].g = 0;
  colors[0].b = 0;
  colors[1].r = 0;
  colors[1].g = 1;
  colors[1].b = 0;
  colors[2].r = 0;
  colors[2].g = 0;
  colors[2].b = 1;
  colors[3].r = 1;
  colors[3].g = 1;
  colors[3].b = 0;
  for (int i = 0; i < 4; i++)
  {
    // Leave out border pixel so PolygonIterator doesn't get stuck
    int j = 0;
    visualization_msgs::MarkerArray vectorField;
    for (SubmapIterator it(map_, Index(10, 10), map_.getSize() - Size(10, 10)); !it.isPastEnd(); ++it)
    {
      Index curr_pos, back, left, front, right;
      curr_pos = front = back = left = right = *it;
      back[0] += idx_offs[i][0][0];
      back[1] += idx_offs[i][0][1];
      left[0] += idx_offs[i][1][0];
      left[1] += idx_offs[i][1][1];
      front[0] += idx_offs[i][2][0];
      front[1] += idx_offs[i][2][1];
      right[0] += idx_offs[i][3][0];
      right[1] += idx_offs[i][3][1];
      Position p_curr_pos, p_front, p_back, p_left, p_right;
      map_.getPosition(curr_pos, p_curr_pos);
      map_.getPosition(back, p_back);
      map_.getPosition(front, p_front);
      map_.getPosition(left, p_left);
      map_.getPosition(right, p_right);
      Polygon poly;
      poly.addVertex(p_back);
      poly.addVertex(p_left);
      poly.addVertex(p_front);
      poly.addVertex(p_right);
      double best_weight = 0;
      Index best_target(0, 0);
      for (PolygonIterator pit(map_, poly); !pit.isPastEnd(); ++pit)
      {
        double weight = (*layers_[8])(grid_map::getLinearIndexFromIndex(*pit, map_.getSize()));
        if (weight > best_weight)
        {
          best_weight = weight;
          best_target = *pit;
        }
      }
      if (best_weight > 0)
      {
        double dx = (*it)[0] - best_target[0];
        double dy = (*it)[1] - best_target[1];
        (*layers_[i * 2])(grid_map::getLinearIndexFromIndex(*it, map_.getSize())) = dx * map_.getResolution();
        (*layers_[i * 2 + 1])(grid_map::getLinearIndexFromIndex(*it, map_.getSize())) = dy * map_.getResolution();
        geometry_msgs::Pose curr_pose;
        curr_pose.position.x = p_curr_pos[0];
        curr_pose.position.y = p_curr_pos[1];
        dx *= map_.getResolution();
        dy *= map_.getResolution();
        double yaw = atan2(dy, dx);
        curr_pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/map";
        marker.header.stamp = ros::Time::now();
        marker.id = j;
        marker.type = 0;
        marker.pose = curr_pose;
        marker.scale.x = sqrt(dx * dx + dy * dy);
        marker.scale.y = 0.01;
        marker.scale.z = 0.01;
        marker.color = colors[i];
        vectorField.markers.emplace_back(marker);
        j++;
      }
    }
    pubVectorField_[i].publish(vectorField);
  }
  ROS_INFO("vector field computed and published");
  ROS_INFO("map backup");
  grid_map_msgs::GridMap message;
  GridMapRosConverter::toMessage(map_, message);
  rosbag::Bag bag;
  bag.open(grid_map_bagfile_out_, rosbag::bagmode::Write);
  bag.write("heatmap_grid_map", ros::Time::now(), message);
  bag.close();
  ROS_INFO("map backup done");
}

void TrackingHeatmapNode::computeInitialLikelihoodField(void)
{
  ROS_INFO("computing initial likelihood field");
  cv::Mat global_map_image, global_map_image_likely, global_map_image_inv_likely, likelihood_field, distance_field,
      distance_field_meters, likelihood_field_inv, binary_map_image, heatmap_image, heatmap_image_inv;
  grid_map::GridMapCvConverter::toImage<unsigned char, 1>(map_, "global_map", CV_8UC1, 100, 0, global_map_image);
  likelihood_field.create(global_map_image.size(), CV_32FC1);
  likelihood_field_inv.create(global_map_image.size(), CV_32FC1);
  distance_field.create(global_map_image.size(), CV_32FC1);
  distance_field_meters.create(global_map_image.size(), CV_32FC1);
  global_map_image_likely.create(global_map_image.size(), CV_32FC3);
  global_map_image_inv_likely.create(global_map_image.size(), CV_32FC3);

  cv::cvtColor(global_map_image, global_map_image_likely, CV_GRAY2RGB);
  cv::cvtColor(global_map_image, global_map_image_inv_likely, CV_GRAY2RGB);

  double min, max;

  cv::imwrite(image_output_dir_ + "global_image_XX.jpg", global_map_image, std::vector<int>({ CV_IMWRITE_JPEG_QUALITY, 100 }));

  cv::distanceTransform(global_map_image, distance_field, CV_DIST_L2, CV_DIST_MASK_5);
  cv::imwrite(image_output_dir_ + "distance_field_image_XX.jpg", distance_field, std::vector<int>({ CV_IMWRITE_JPEG_QUALITY, 100 }));
  distance_field_meters = distance_field * map_.getResolution();  // map resolution
  boost::math::normal normal_likelihood_field = boost::math::normal(0, likelihood_field_sigma_);

  for (int r = 0; r < likelihood_field.rows; r++)
  {
    for (int c = 0; c < likelihood_field.cols; c++)
    {
      likelihood_field.at<float>(r, c) = pdf(normal_likelihood_field, distance_field_meters.at<float>(r, c));
    }
  }

  cv::minMaxLoc(likelihood_field, &min, &max);

  cv::subtract(cv::Scalar::all(max), likelihood_field, likelihood_field_inv);

  for (int r = 0; r < likelihood_field.rows; r++)
  {
    for (int c = 0; c < likelihood_field.cols; c++)
    {
      cv::Vec3b& p = global_map_image_likely.at<cv::Vec3b>(r, c);
      cv::Vec3b& p1 = global_map_image_inv_likely.at<cv::Vec3b>(r, c);

      // only overwrite white pixels
      if ((likelihood_field.at<float>(r, c) > 0.0) && (p[0] == 255) && (p[1] == 255) && (p[2] == 255))
      {
        p[0] = 255 - (unsigned char)(likelihood_field.at<float>(r, c) * 255);
        p[1] = 255;
        p[2] = 255;
      }

      if ((likelihood_field_inv.at<float>(r, c) > 0.0) && (p1[0] == 255) && (p1[1] == 255) && (p1[2] == 255))
      {
        p1[0] = 255 - (unsigned char)(likelihood_field_inv.at<float>(r, c) * 255);
        p1[1] = 255;
        p1[2] = 255;
      }
    }
  }

  grid_map::GridMapCvConverter::addLayerFromImage<float, 1>(likelihood_field_inv, "likelihood_field", map_);

  cv::threshold(global_map_image, binary_map_image, 0.5, 1.0, cv::THRESH_BINARY);
  grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 1>(binary_map_image, "binary_map", map_);

  map_["likelihood_field"] = (map_["binary_map"].array() == 0.0).select(0.0, map_["likelihood_field"]);

  likelihood_field.convertTo(likelihood_field, CV_8UC3, 255.0);
  likelihood_field_inv.convertTo(likelihood_field_inv, CV_8UC3, 255.0);
  binary_map_image.convertTo(binary_map_image, CV_8UC3, 255.0);
  cv::applyColorMap(likelihood_field_inv, heatmap_image, cv::COLORMAP_JET);
  cv::applyColorMap(likelihood_field, heatmap_image_inv, cv::COLORMAP_JET);

  // write to file for documentation purposes
  // global_map_image, likelihood_field, distance_field, likelihood_field_inv, binary_map_image;
  cv::imwrite(image_output_dir_ + "global_image.jpg", global_map_image, std::vector<int>({ CV_IMWRITE_JPEG_QUALITY, 100 }));
  cv::imwrite(image_output_dir_ + "likelihood_field.jpg", likelihood_field, std::vector<int>({ CV_IMWRITE_JPEG_QUALITY, 100 }));
  cv::imwrite(image_output_dir_ + "distance_field.jpg", distance_field, std::vector<int>({ CV_IMWRITE_JPEG_QUALITY, 100 }));
  cv::imwrite(image_output_dir_ + "likelihood_field_inv.jpg", likelihood_field_inv, std::vector<int>({ CV_IMWRITE_JPEG_QUALITY, 100 }));
  cv::imwrite(image_output_dir_ + "global_map_image_likely.jpg", global_map_image_likely, std::vector<int>({ CV_IMWRITE_JPEG_QUALITY, 100 }));
  cv::imwrite(image_output_dir_ + "global_map_image_inv_likely.jpg", global_map_image_inv_likely, std::vector<int>({ CV_IMWRITE_JPEG_QUALITY, 100 }));
  cv::imwrite(image_output_dir_ + "binary_map_image.jpg", binary_map_image, std::vector<int>({ CV_IMWRITE_JPEG_QUALITY, 100 }));
  cv::imwrite(image_output_dir_ + "heatmap_image.jpg", heatmap_image, std::vector<int>({ CV_IMWRITE_JPEG_QUALITY, 100 }));
  cv::imwrite(image_output_dir_ + "heatmap_image_inv.jpg", heatmap_image_inv, std::vector<int>({ CV_IMWRITE_JPEG_QUALITY, 100 }));

  ROS_INFO("computing initial likelihood field done");
}

void TrackingHeatmapNode::computeSocialForces()
{
  ROS_INFO("compute social forces");
  visualization_msgs::MarkerArray vectorField;
  int marker_id = 0;
  // go through all non occupied pixels in map
  // output in map still has to be scaled by the anisotropic factor
  // the factor needs intended direction and is therefore not precomputed
  // see People Tracking with Human Motion Prediction from Social Forces
  for (SubmapIterator iterator(map_, Index(border_length_, border_length_),
                               map_.getSize() - Size(2 * border_length_, 2 * border_length_));
       !iterator.isPastEnd(); ++iterator)
  {
    Index curr_idx = Index(*iterator);
    Position position;
    map_.getPosition(curr_idx, position);
    map_.at("f_soc_phys_x", curr_idx) = 0;
    map_.at("f_soc_phys_y", curr_idx) = 0;
    if (map_.at("binary_map", curr_idx) != 0.0)
    {
      // iterate over rectangle around current index
      // first mark every pixel in the current rectangle as unvisited
      for (SubmapIterator rect_iterator(map_, curr_idx - Index(border_length_ / 2, border_length_ / 2),
                                        Size(border_length_, border_length_));
           !rect_iterator.isPastEnd(); ++rect_iterator)
      {
        map_.at("bresenham_visited", *rect_iterator) = 0;
      }

      for (SubmapIterator rect_iterator(map_, curr_idx - Index(border_length_ / 2, border_length_ / 2),
                                        Size(border_length_, border_length_));
           !rect_iterator.isPastEnd(); ++rect_iterator)
      {
        if (map_.at("bresenham_visited", *rect_iterator) == 0)
        {
          int x0, y0, x1, y1, dx, dy, sx, sy, err, e2, aborted, found;
          x0 = (*rect_iterator)[0];
          y0 = (*rect_iterator)[1];
          x1 = (*iterator)[0];
          y1 = (*iterator)[1];
          //             std::cout << "from: " << (*rect_iterator).transpose() << " to: "<<(*iterator).transpose() <<
          //             std::endl;
          aborted = 0;
          found = 0;
          double d = 0;
          Position n(0, 0);
          // Bresenham stuff
          //-------
          dx = abs(x1 - x0);
          sx = x0 < x1 ? 1 : -1;
          dy = -abs(y1 - y0);
          sy = y0 < y1 ? 1 : -1;
          err = dx + dy;
          //-------
          //             std::cout << "enter while loop" << std::endl;
          while (1)
          {
            Index rect_iter_idx = Index(x0, y0);
            //                 std::cout <<"..idx:" <<rect_iter_idx.transpose() << std::endl;
            // If along the line from the checked starting pixel to the center a visited pixel is found, we can ignore
            // the effect of the starting pixel on the forces
            if (map_.at("bresenham_visited", rect_iter_idx) != 0)
            {
              aborted = 1;
              break;
            }

            // Mark pixel along the line as visited
            // Don't mark the center pixel
            if (rect_iter_idx[0] != (*iterator)[0] && rect_iter_idx[1] != (*iterator)[1])
            {
              map_.at("bresenham_visited", rect_iter_idx) = 1;
            }

            Position rect_position;
            map_.getPosition(rect_iter_idx, rect_position);

            if (map_.at("binary_map", rect_iter_idx) == 0.0)
            {
              d = (position - rect_position).squaredNorm();
              n = (position - rect_position);
              n.normalize();
              found = 1;
              //                     std::cout << "found" << std::endl;
            }
            else
            {
              //                     std::cout << "not using pixel:"<<rect_iter_idx.transpose()<<std::endl;
            }

            // Bresenham stuff
            //-------
            if (x0 == x1 && y0 == y1)
              break;
            e2 = 2 * err;
            if (e2 > dy)
            {
              err += dy;
              x0 += sx;
            }  // e_xy+e_x > 0
            if (e2 < dx)
            {
              err += dx;
              y0 += sy;
            }  // e_xy+e_y < 0
               //-------
          }
          if (!aborted && found)
          {
            Position f_soc = (a_obj_ * exp((r_pers_ - d) / (b_obj_)) / (pow(10, scale_))) * n;
            //                 std::cout << "fsoc: "   << f_soc.transpose() << std::endl;

            Position f_phys;
            if (r_pers_ - d > 0)
            {
              f_phys = (c_obj_ * (r_pers_ - d) * n) / (pow(10, scale_));
              //                     std::cout << "fphys: " << f_phys << std::endl;
            }
            else
            {
              f_phys = Position(0, 0);
            }

            map_.at("f_soc_phys_x", curr_idx) += (f_soc(0) + f_phys(0));
            map_.at("f_soc_phys_y", curr_idx) += (f_soc(1) + f_phys(1));
          }
        }
      }
    }
    else
    {
      continue;
    }
    if (map_.at("f_soc_phys_x", curr_idx) != 0 || map_.at("f_soc_phys_y", curr_idx) != 0)
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "/map";
      marker.header.stamp = ros::Time::now();
      marker.id = marker_id;
      marker_id++;
      marker.type = 0;
      marker.pose.position.x = position[0];
      marker.pose.position.y = position[1];
      double f_soc_x = map_.at("f_soc_phys_x", curr_idx);
      double f_soc_y = map_.at("f_soc_phys_y", curr_idx);
      double yaw = atan2(f_soc_y, f_soc_x);
      marker.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
      marker.scale.x = sqrt(f_soc_x * f_soc_x + f_soc_y * f_soc_y);
      marker.scale.y = 0.002;
      marker.scale.z = 0.002;
      marker.color.a = 1;
      marker.color.r = 0.5;
      marker.color.g = 0.5;
      marker.color.b = 0.5;
      vectorField.markers.emplace_back(marker);
    }
  }
  pubVectorField_[4].publish(vectorField);
  ROS_INFO("Published vector field forces");
}
