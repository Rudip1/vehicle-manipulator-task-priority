// ArUco detector: finds the markers of known objects in the camera image, estimates their pose, and publishes the
// centre of each object's top face in the odometry frame. Each marker sits on a vertical side face of an upright
// object: the object's vertical axis is `depth` behind the marker centre and its top face `rise` above it.
//
// In:  camera/image, camera/camera_info, tf (camera_optical_frame -> odom)
// Out: aruco/marker_<id> (PoseStamped), aruco/target_<id> (PointStamped, median of the recent detections), in odom
//
// Measured in the odometry frame at the time of the image, a target is consistent with the controller's own pose
// estimate: odometry drift accumulated before the detection cancels out.

#include <cv_bridge/cv_bridge.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <deque>
#include <map>
#include <stdexcept>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

class ArucoNode : public rclcpp::Node {
  public:
    ArucoNode() : Node("aruco_detector"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_) {
        const auto ids = declare_parameter("marker_ids", std::vector<int64_t>{1, 2});
        const auto depths = declare_parameter("depths", std::vector<double>{0.035, 0.06});
        const auto rises = declare_parameter("rises", std::vector<double>{0.035, 0.04});
        marker_size_ = declare_parameter("marker_size", 0.05);
        if (depths.size() != ids.size() || rises.size() != ids.size())
            throw std::invalid_argument("marker_ids, depths and rises must have the same length");
        target_frame_ = declare_parameter("target_frame", "odom");
        window_ = declare_parameter("window", 15);
        max_range_ = declare_parameter("max_range", 2.5);
        dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
        params_ = cv::aruco::DetectorParameters::create();
        params_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;

        info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            "camera/camera_info", 5, [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr m) {
                K_ = cv::Mat(3, 3, CV_64F, const_cast<double*>(m->k.data())).clone();
                D_ = cv::Mat(1, int(m->d.size()), CV_64F, const_cast<double*>(m->d.data())).clone();
            });
        image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "camera/image", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::Image::ConstSharedPtr m) { on_image(m); });
        for (size_t i = 0; i < ids.size(); ++i) {
            Target t;
            t.depth = depths[i];
            t.rise = rises[i];
            const std::string id = std::to_string(ids[i]);
            t.marker_pub = create_publisher<geometry_msgs::msg::PoseStamped>("aruco/marker_" + id, 10);
            t.target_pub = create_publisher<geometry_msgs::msg::PointStamped>("aruco/target_" + id, 10);
            targets_[int(ids[i])] = std::move(t);
        }
    }

  private:
    void on_image(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
        if (K_.empty()) return;
        const cv::Mat gray = cv_bridge::toCvShare(msg, "mono8")->image;
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::aruco::detectMarkers(gray, dictionary_, corners, ids, params_);
        for (size_t i = 0; i < ids.size(); ++i) {
            const auto it = targets_.find(ids[i]);
            if (it == targets_.end()) continue;
            Target& target = it->second;
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(std::vector<std::vector<cv::Point2f>>{corners[i]}, marker_size_, K_, D_,
                                                 rvecs, tvecs);
            const Eigen::Vector3d t(tvecs[0][0], tvecs[0][1], tvecs[0][2]);
            if (t.norm() > max_range_) continue;
            cv::Mat Rcv;
            cv::Rodrigues(rvecs[0], Rcv);
            Eigen::Matrix3d R;
            for (int r = 0; r < 3; ++r)
                for (int c = 0; c < 3; ++c) R(r, c) = Rcv.at<double>(r, c);
            Eigen::Isometry3d T_cam_marker = Eigen::Isometry3d::Identity();
            T_cam_marker.linear() = R;
            T_cam_marker.translation() = t;
            Eigen::Isometry3d T_target_cam;
            try {
                T_target_cam = tf2::transformToEigen(
                    tf_buffer_.lookupTransform(target_frame_, msg->header.frame_id, msg->header.stamp,
                                               rclcpp::Duration::from_seconds(0.1)));
            } catch (const tf2::TransformException& e) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "no transform: %s", e.what());
                return;
            }
            const Eigen::Isometry3d T = T_target_cam * T_cam_marker;
            publish_marker(target, T, msg->header.stamp);
            // The marker's z axis points out of the face; the object's axis is `depth` behind the marker centre
            // (measured horizontally: the object stands upright) and its top `rise` above.
            Eigen::Vector3d normal = T.linear().col(2);
            normal.z() = 0.0;
            if (normal.norm() < 0.5) continue;  // marker seen from above or below: not a side face
            normal.normalize();
            target.history.push_back(T.translation() - target.depth * normal + Eigen::Vector3d(0, 0, target.rise));
            while (static_cast<int>(target.history.size()) > window_) target.history.pop_front();
            publish_target(target, msg->header.stamp);
        }
    }

    struct Target {
        double depth = 0.0, rise = 0.0;
        std::deque<Eigen::Vector3d> history;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr marker_pub;
        rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_pub;
    };

    void publish_marker(Target& target, const Eigen::Isometry3d& T, const rclcpp::Time& stamp) {
        geometry_msgs::msg::PoseStamped p;
        p.header.stamp = stamp;
        p.header.frame_id = target_frame_;
        p.pose = tf2::toMsg(T);
        target.marker_pub->publish(p);
    }

    void publish_target(Target& target, const rclcpp::Time& stamp) {
        // Median per axis: robust to the odd bad pose estimate at grazing angles.
        auto median = [&target](int axis) {
            std::vector<double> v;
            for (const auto& p : target.history) v.push_back(p(axis));
            std::nth_element(v.begin(), v.begin() + long(v.size() / 2), v.end());
            return v[v.size() / 2];
        };
        geometry_msgs::msg::PointStamped p;
        p.header.stamp = stamp;
        p.header.frame_id = target_frame_;
        p.point.x = median(0);
        p.point.y = median(1);
        p.point.z = median(2);
        target.target_pub->publish(p);
    }

    int window_ = 15;
    double marker_size_ = 0.05, max_range_ = 2.5;
    std::map<int, Target> targets_;
    std::string target_frame_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> params_;
    cv::Mat K_, D_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArucoNode>());
    rclcpp::shutdown();
    return 0;
}
