// Monocular ORB-SLAM3 node for the simulated TurtleBot3.
//
// Replaces ros2_orb_slam3's mono_node_cpp entirely. That node was written to be
// fed by a Python driver over a String handshake, and it publishes no pose at
// all. We link the same orb_slam3_lib but drive it directly:
//
//   * subscribe to the bridged camera topic - no handshake, no timestep topic
//   * publish nav_msgs/Odometry, nav_msgs/Path and a map -> camera TF
//
// Frame convention is the part worth reading twice. ORB-SLAM3 returns Tcw
// (world-to-camera) in the camera-optical frame: X right, Y down, Z forward.
// REP-103 wants X forward, Y left, Z up. See publishPose().

#include <memory>
#include <deque>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include "System.h"
#include "ImuTypes.h"

class MonoSlamNode : public rclcpp::Node
{
public:
  MonoSlamNode()
  : Node("vslam_mono")
  {
    voc_file_ = declare_parameter<std::string>("voc_file", "");
    settings_file_ = declare_parameter<std::string>("settings_file", "");
    image_topic_ = declare_parameter<std::string>("image_topic", "/camera/image");
    pose_scale_ = declare_parameter<double>("pose_scale", 1.0);
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    camera_frame_ = declare_parameter<std::string>("camera_frame", "orbslam3_camera");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    show_viewer_ = declare_parameter<bool>("show_viewer", true);

    // Visual-inertial mode. Output becomes metrically scaled, so pose_scale
    // is ignored. Requires the IMU block in the settings file (tb3_imu.yaml).
    use_imu_ = declare_parameter<bool>("use_imu", false);
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/imu");

    // Nav2 needs map -> odom, not map -> camera. See publishMapToOdom().
    publish_map_odom_ = declare_parameter<bool>("publish_map_odom", true);
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    camera_link_frame_ = declare_parameter<std::string>("camera_link_frame", "camera_link");
    transform_tolerance_ = declare_parameter<double>("transform_tolerance", 0.1);

    if (voc_file_.empty() || settings_file_.empty()) {
      RCLCPP_FATAL(get_logger(), "voc_file and settings_file are required");
      throw std::runtime_error("missing voc_file/settings_file");
    }

    RCLCPP_INFO(get_logger(), "vocabulary: %s", voc_file_.c_str());
    RCLCPP_INFO(get_logger(), "settings:   %s", settings_file_.c_str());
    RCLCPP_INFO(get_logger(), "pose_scale: %.4f", pose_scale_);

    // Loading the vocabulary takes a few seconds.
    const auto sensor = use_imu_ ? ORB_SLAM3::System::IMU_MONOCULAR
                                 : ORB_SLAM3::System::MONOCULAR;
    RCLCPP_INFO(get_logger(), "mode: %s", use_imu_ ? "IMU_MONOCULAR" : "MONOCULAR");
    slam_ = std::make_unique<ORB_SLAM3::System>(
      voc_file_, settings_file_, sensor, show_viewer_);

    pose_pub_ = create_publisher<nav_msgs::msg::Odometry>("/orbslam3/pose", 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/orbslam3/path", 10);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    path_.header.frame_id = map_frame_;

    // Sensor QoS: best-effort, tolerates the bridge's reliable publisher and
    // drops stale frames rather than queueing them behind SLAM.
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, rclcpp::SensorDataQoS(),
      std::bind(&MonoSlamNode::onImage, this, std::placeholders::_1));

    if (use_imu_) {
      // 200 Hz: fast enough to keep up with the 30 Hz camera without adding
      // latency. Single-threaded executor serialises this against tracking.
      sync_timer_ = create_wall_timer(
        std::chrono::milliseconds(5),
        std::bind(&MonoSlamNode::processBuffered, this));
      imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, rclcpp::SensorDataQoS(),
        std::bind(&MonoSlamNode::onImu, this, std::placeholders::_1));
      RCLCPP_INFO(get_logger(), "fusing IMU from %s", imu_topic_.c_str());
    }

    if (publish_map_odom_) {
      map_odom_timer_ = create_wall_timer(
        std::chrono::milliseconds(33),
        std::bind(&MonoSlamNode::broadcastMapToOdom, this));
    }

    RCLCPP_INFO(get_logger(), "tracking images from %s", image_topic_.c_str());
  }

  ~MonoSlamNode() override
  {
    if (slam_) {
      slam_->Shutdown();
    }
  }

private:
  void onImu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    imu_buffer_.push_back(*msg);
  }

  // Everything buffered up to this frame's timestamp. ORB-SLAM3 integrates
  // these between the previous image and this one, so they must be handed over
  // in order and then dropped.
  std::vector<ORB_SLAM3::IMU::Point> drainImu(double image_time)
  {
    std::vector<ORB_SLAM3::IMU::Point> pts;
    std::lock_guard<std::mutex> lock(imu_mutex_);
    auto it = imu_buffer_.begin();
    while (it != imu_buffer_.end()) {
      const double t = rclcpp::Time(it->header.stamp).seconds();
      if (t > image_time) {
        break;
      }
      pts.emplace_back(
        it->linear_acceleration.x, it->linear_acceleration.y, it->linear_acceleration.z,
        it->angular_velocity.x, it->angular_velocity.y, it->angular_velocity.z, t);
      ++it;
    }
    imu_buffer_.erase(imu_buffer_.begin(), it);
    return pts;
  }

  void onImage(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(msg);
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge: %s", e.what());
      return;
    }

    // In visual-inertial mode a frame cannot be tracked until the IMU stream
    // has caught up past its timestamp. Callbacks arrive out of order, so
    // tracking here directly yields "Empty IMU measurements vector!!!" and the
    // local mapper eventually sets the bad-IMU flag and resets the map.
    // Buffer instead; processBuffered() drains once the IMU covers the frame.
    if (use_imu_) {
      std::lock_guard<std::mutex> lock(image_mutex_);
      image_buffer_.push_back({cv_ptr, msg->header});
      if (image_buffer_.size() > 60) {
        image_buffer_.pop_front();   // IMU stalled; do not grow without bound
      }
      return;
    }

    trackFrame(cv_ptr->image, msg->header, {});
  }

  // Pop frames whose timestamp the IMU stream has passed, newest-last.
  void processBuffered()
  {
    while (true) {
      cv_bridge::CvImageConstPtr img;
      std_msgs::msg::Header hdr;
      double t = 0.0;
      {
        std::lock_guard<std::mutex> ilock(image_mutex_);
        if (image_buffer_.empty()) {
          return;
        }
        t = rclcpp::Time(image_buffer_.front().second.stamp).seconds();
        {
          std::lock_guard<std::mutex> mlock(imu_mutex_);
          // Wait until the IMU has a sample at or beyond this frame.
          if (imu_buffer_.empty() ||
            rclcpp::Time(imu_buffer_.back().header.stamp).seconds() < t)
          {
            return;
          }
        }
        img = image_buffer_.front().first;
        hdr = image_buffer_.front().second;
        image_buffer_.pop_front();
      }

      const auto imu = drainImu(t);
      if (imu.empty()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000, "frame with no IMU samples, skipping");
        continue;
      }
      trackFrame(img->image, hdr, imu);
    }
  }

  void trackFrame(
    const cv::Mat & image, const std_msgs::msg::Header & header,
    const std::vector<ORB_SLAM3::IMU::Point> & imu)
  {
    const double stamp = rclcpp::Time(header.stamp).seconds();
    const Sophus::SE3f Tcw = use_imu_
      ? slam_->TrackMonocular(image, stamp, imu)
      : slam_->TrackMonocular(image, stamp);

    // eTrackingState OK == 2. Publishing while LOST emits identity poses,
    // which look like the robot teleported back to the origin.
    if (slam_->GetTrackingState() != 2) {
      if (was_tracking_) {
        RCLCPP_WARN(get_logger(), "tracking lost");
        was_tracking_ = false;
      }
      return;
    }
    if (!was_tracking_) {
      RCLCPP_INFO(get_logger(), "tracking");
      was_tracking_ = true;
    }

    publishPose(Tcw, header.stamp);
  }

  // Order matters:
  //   1. invert Tcw to get the camera pose in the world
  //   2. scale the translation (monocular SLAM has no metric scale)
  //   3. rotate optical -> ROS
  // Skipping step 3 is the classic failure: the trajectory drives sideways or
  // into the floor.
  void publishPose(const Sophus::SE3f & Tcw, const builtin_interfaces::msg::Time & stamp)
  {
    const Sophus::SE3f Twc = Tcw.inverse();

    Eigen::Matrix3f R_opt2ros;
    R_opt2ros << 0.0f, 0.0f, 1.0f,
                -1.0f, 0.0f, 0.0f,
                 0.0f, -1.0f, 0.0f;

    const Eigen::Vector3f t =
      R_opt2ros * (static_cast<float>(pose_scale_) * Twc.translation());
    const Eigen::Matrix3f R = R_opt2ros * Twc.rotationMatrix() * R_opt2ros.transpose();
    const Eigen::Quaternionf q(R);

    geometry_msgs::msg::PoseStamped ps;
    ps.header.stamp = stamp;
    ps.header.frame_id = map_frame_;
    ps.pose.position.x = t.x();
    ps.pose.position.y = t.y();
    ps.pose.position.z = t.z();
    ps.pose.orientation.x = q.x();
    ps.pose.orientation.y = q.y();
    ps.pose.orientation.z = q.z();
    ps.pose.orientation.w = q.w();

    nav_msgs::msg::Odometry odom;
    odom.header = ps.header;
    odom.child_frame_id = camera_frame_;
    odom.pose.pose = ps.pose;
    pose_pub_->publish(odom);

    path_.header.stamp = stamp;
    path_.poses.push_back(ps);
    path_pub_->publish(path_);

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header = ps.header;
      tf.child_frame_id = camera_frame_;
      tf.transform.translation.x = t.x();
      tf.transform.translation.y = t.y();
      tf.transform.translation.z = t.z();
      tf.transform.rotation = ps.pose.orientation;
      tf_broadcaster_->sendTransform(tf);
    }

    if (publish_map_odom_) {
      Eigen::Isometry3d T_map_cam = Eigen::Isometry3d::Identity();
      T_map_cam.translate(Eigen::Vector3d(t.x(), t.y(), t.z()));
      T_map_cam.rotate(Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z()));
      updateMapToOdom(T_map_cam);
    }
  }

  // Nav2 expects the localisation source to publish map -> odom: a *correction*
  // on top of wheel odometry, not the robot pose itself. Publishing
  // map -> base_footprint directly would fight the odom -> base_footprint that
  // the DiffDrive plugin already broadcasts, giving base_footprint two parents.
  //
  //   map->odom = map->camera * (base->camera)^-1 * (odom->base)^-1
  //
  // base->camera and odom->base both come from TF, so the mounting offsets
  // stay in one place (the static publishers in sim.launch.py).
  //
  // This only recomputes the correction. Broadcasting happens on a timer - see
  // broadcastMapToOdom().
  void updateMapToOdom(const Eigen::Isometry3d & T_map_cam)
  {
    geometry_msgs::msg::TransformStamped base_cam_msg;
    geometry_msgs::msg::TransformStamped odom_base_msg;
    try {
      base_cam_msg = tf_buffer_->lookupTransform(
        base_frame_, camera_link_frame_, tf2::TimePointZero);
      odom_base_msg = tf_buffer_->lookupTransform(
        odom_frame_, base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "map->odom skipped, TF not ready: %s", e.what());
      return;
    }

    const Eigen::Isometry3d T_base_cam = tf2::transformToEigen(base_cam_msg);
    const Eigen::Isometry3d T_odom_base = tf2::transformToEigen(odom_base_msg);

    T_map_odom_ = T_map_cam * T_base_cam.inverse() * T_odom_base.inverse();
    have_map_odom_ = true;
  }

  // Broadcast the correction continuously and post-dated, the way AMCL does.
  //
  // Publishing only on tracked frames leaves gaps in the TF buffer, and Nav2
  // then fails with "Lookup would require extrapolation into the past". A
  // steady stream stamped slightly in the future keeps lookups inside the
  // buffer. Holding the last correction while tracking is briefly lost is also
  // what AMCL does - odom carries the robot until vision recovers.
  void broadcastMapToOdom()
  {
    if (!have_map_odom_) {
      return;
    }
    geometry_msgs::msg::TransformStamped out = tf2::eigenToTransform(T_map_odom_);
    out.header.stamp = now() + rclcpp::Duration::from_seconds(transform_tolerance_);
    out.header.frame_id = map_frame_;
    out.child_frame_id = odom_frame_;
    tf_broadcaster_->sendTransform(out);
  }

  std::string voc_file_, settings_file_, image_topic_, map_frame_, camera_frame_;
  std::string odom_frame_, base_frame_, camera_link_frame_;
  double pose_scale_{1.0};
  bool publish_tf_{true};
  bool publish_map_odom_{true};
  bool show_viewer_{true};
  bool was_tracking_{false};
  double transform_tolerance_{0.1};
  Eigen::Isometry3d T_map_odom_{Eigen::Isometry3d::Identity()};
  bool have_map_odom_{false};
  rclcpp::TimerBase::SharedPtr map_odom_timer_;
  bool use_imu_{false};
  std::string imu_topic_;
  std::deque<sensor_msgs::msg::Imu> imu_buffer_;
  std::mutex imu_mutex_;
  std::deque<std::pair<cv_bridge::CvImageConstPtr, std_msgs::msg::Header>> image_buffer_;
  std::mutex image_mutex_;
  rclcpp::TimerBase::SharedPtr sync_timer_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::unique_ptr<ORB_SLAM3::System> slam_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  nav_msgs::msg::Path path_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MonoSlamNode>());
  rclcpp::shutdown();
  return 0;
}
