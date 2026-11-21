// Pick-and-place mission as a behaviour tree (BehaviorTree.CPP 4). The tree is read from an XML file
// (config/pick_place.xml); this program provides its action nodes:
//
//   SetSuction  on="true|false"                     attach or release the box (gripper topics), wait for the state
//   WaitForTarget id="1" top="{bb}" add="x;y;z"     top of a marked object from fresh ArUco detections (or a
//                                                   fixed parameter), plus `add`
//   MoveEE      target="{bb}" offset="x;y;z"        suction cup to target + offset (MoveTo action, POSITION)
//   MovePosture posture="t1;t2;t3;t4"               arm posture (MoveTo action, POSTURE)
//   FaceTarget  target="{bb}"                       turn the base on the spot to face target (MoveTo, HEADING)
//
// Positions are Eigen::Vector3d on the blackboard, written in XML as "x;y;z". Rough positions of the objects, used
// to drive near them before looking, come from the parameters pick_guess and place_guess ({pick_guess},
// {place_guess}).

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>

#include <Eigen/Core>
#include <atomic>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <map>
#include <mutex>
#include <stdexcept>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>
#include <vehicle_manipulator_task_priority_interfaces/action/move_to.hpp>

using MoveTo = vehicle_manipulator_task_priority_interfaces::action::MoveTo;

namespace BT {
template <>
inline Eigen::Vector3d convertFromString(StringView str) {
    const auto parts = splitString(str, ';');
    if (parts.size() != 3) throw RuntimeError("expected x;y;z");
    return {convertFromString<double>(parts[0]), convertFromString<double>(parts[1]),
            convertFromString<double>(parts[2])};
}
template <>
inline std::vector<double> convertFromString(StringView str) {
    std::vector<double> out;
    for (const auto& p : splitString(str, ';')) out.push_back(convertFromString<double>(p));
    return out;
}
}  // namespace BT

/// ROS side shared by all tree nodes: the action client, the gripper topics and the latest box detection.
struct Ros {
    rclcpp::Node::SharedPtr node;
    rclcpp_action::Client<MoveTo>::SharedPtr move_to;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr attach, detach;
    std::mutex mutex;
    struct Target {
        std::optional<Eigen::Vector3d> top;
        int count = 0;
        bool fixed = false;
    };
    std::map<int, Target> targets;
    std::vector<rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr> target_subs;
    std::string suction_state;
};

/// Common part of MoveEE and MovePosture: send a goal, poll until the result arrives.
class MoveBase : public BT::StatefulActionNode {
  public:
    MoveBase(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<Ros> ros)
        : StatefulActionNode(name, config), ros_(std::move(ros)) {}

  protected:
    virtual std::optional<MoveTo::Goal> make_goal() = 0;

    BT::NodeStatus onStart() override {
        const auto goal = make_goal();
        if (!goal || !ros_->move_to->wait_for_action_server(std::chrono::seconds(5))) return BT::NodeStatus::FAILURE;
        done_ = false;
        success_ = false;
        rclcpp_action::Client<MoveTo>::SendGoalOptions opts;
        opts.result_callback = [this](const rclcpp_action::ClientGoalHandle<MoveTo>::WrappedResult& r) {
            success_ = r.code == rclcpp_action::ResultCode::SUCCEEDED && r.result->success;
            RCLCPP_INFO(ros_->node->get_logger(), "%s: %s (error %.4f, %.1f s)", name().c_str(),
                        r.result ? r.result->message.c_str() : "no result", r.result ? r.result->final_error : -1.0,
                        r.result ? r.result->duration : -1.0);
            done_ = true;
        };
        ros_->move_to->async_send_goal(*goal, opts);
        return BT::NodeStatus::RUNNING;
    }
    BT::NodeStatus onRunning() override {
        if (!done_) return BT::NodeStatus::RUNNING;
        return success_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
    void onHalted() override { ros_->move_to->async_cancel_all_goals(); }

    std::shared_ptr<Ros> ros_;
    std::atomic<bool> done_{false}, success_{false};
};

class MoveEE : public MoveBase {
  public:
    using MoveBase::MoveBase;
    static BT::PortsList providedPorts() {
        return {BT::InputPort<Eigen::Vector3d>("target"), BT::InputPort<Eigen::Vector3d>("offset", "x;y;z added to target (default 0)"),
                BT::InputPort<double>("timeout", 0.0, "[s], 0 = controller default"),
                BT::InputPort<double>("tolerance", 0.0, "[m], 0 = controller default")};
    }

  protected:
    std::optional<MoveTo::Goal> make_goal() override {
        const auto target = getInput<Eigen::Vector3d>("target");
        const Eigen::Vector3d offset = getInput<Eigen::Vector3d>("offset").value_or(Eigen::Vector3d::Zero());
        if (!target) return std::nullopt;
        const Eigen::Vector3d p = *target + offset;
        MoveTo::Goal g;
        g.type = MoveTo::Goal::POSITION;
        g.position.x = p.x();
        g.position.y = p.y();
        g.position.z = p.z();
        g.timeout = getInput<double>("timeout").value_or(0.0);
        g.tolerance = getInput<double>("tolerance").value_or(0.0);
        RCLCPP_INFO(ros_->node->get_logger(), "%s: cup to (%.3f, %.3f, %.3f)", name().c_str(), p.x(), p.y(), p.z());
        return g;
    }
};

class MovePosture : public MoveBase {
  public:
    using MoveBase::MoveBase;
    static BT::PortsList providedPorts() { return {BT::InputPort<std::vector<double>>("posture")}; }

  protected:
    std::optional<MoveTo::Goal> make_goal() override {
        const auto posture = getInput<std::vector<double>>("posture");
        if (!posture || posture->size() != 4) return std::nullopt;
        MoveTo::Goal g;
        g.type = MoveTo::Goal::POSTURE;
        g.posture = *posture;
        return g;
    }
};

class FaceTarget : public MoveBase {
  public:
    using MoveBase::MoveBase;
    static BT::PortsList providedPorts() { return {BT::InputPort<Eigen::Vector3d>("target")}; }

  protected:
    std::optional<MoveTo::Goal> make_goal() override {
        const auto target = getInput<Eigen::Vector3d>("target");
        if (!target) return std::nullopt;
        MoveTo::Goal g;
        g.type = MoveTo::Goal::HEADING;
        g.position.x = target->x();
        g.position.y = target->y();
        g.tolerance = 0.05;
        return g;
    }
};

class SetSuction : public BT::StatefulActionNode {
  public:
    SetSuction(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<Ros> ros)
        : StatefulActionNode(name, config), ros_(std::move(ros)) {}
    static BT::PortsList providedPorts() { return {BT::InputPort<bool>("on")}; }

    BT::NodeStatus onStart() override {
        on_ = getInput<bool>("on").value_or(false);
        started_ = ros_->node->now();
        (on_ ? ros_->attach : ros_->detach)->publish(std_msgs::msg::Empty());
        RCLCPP_INFO(ros_->node->get_logger(), "%s: suction %s", name().c_str(), on_ ? "on" : "off");
        return BT::NodeStatus::RUNNING;
    }
    BT::NodeStatus onRunning() override {
        {
            std::lock_guard<std::mutex> lock(ros_->mutex);
            if (ros_->suction_state == (on_ ? "attached" : "detached")) return BT::NodeStatus::SUCCESS;
        }
        // The gripper reports only changes: releasing an already released box gives no message.
        if ((ros_->node->now() - started_).seconds() > 1.0) return on_ ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
        return BT::NodeStatus::RUNNING;
    }
    void onHalted() override {}

  private:
    std::shared_ptr<Ros> ros_;
    bool on_ = false;
    rclcpp::Time started_;
};

class WaitForTarget : public BT::StatefulActionNode {
  public:
    WaitForTarget(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<Ros> ros)
        : StatefulActionNode(name, config), ros_(std::move(ros)) {}
    static BT::PortsList providedPorts() {
        return {BT::InputPort<int>("id"), BT::OutputPort<Eigen::Vector3d>("top"),
                BT::InputPort<Eigen::Vector3d>("add", "x;y;z added to the measured top (default 0)"),
                BT::InputPort<int>("detections", 10, "fresh detections needed"),
                BT::InputPort<double>("timeout", 20.0, "[s]")};
    }
    BT::NodeStatus onStart() override {
        started_ = ros_->node->now();
        id_ = getInput<int>("id").value_or(-1);
        std::lock_guard<std::mutex> lock(ros_->mutex);
        count_at_start_ = ros_->targets[id_].count;
        return BT::NodeStatus::RUNNING;
    }
    BT::NodeStatus onRunning() override {
        std::lock_guard<std::mutex> lock(ros_->mutex);
        const Ros::Target& t = ros_->targets[id_];
        const int fresh = t.count - count_at_start_;
        if (t.top && (t.fixed || fresh >= getInput<int>("detections").value_or(10))) {
            const Eigen::Vector3d top = *t.top + getInput<Eigen::Vector3d>("add").value_or(Eigen::Vector3d::Zero());
            setOutput("top", top);
            RCLCPP_INFO(ros_->node->get_logger(), "%s: marker %d top at (%.3f, %.3f, %.3f)%s", name().c_str(), id_,
                        t.top->x(), t.top->y(), t.top->z(), t.fixed ? " (fixed)" : "");
            return BT::NodeStatus::SUCCESS;
        }
        if ((ros_->node->now() - started_).seconds() > getInput<double>("timeout").value_or(20.0)) {
            RCLCPP_WARN(ros_->node->get_logger(), "%s: marker %d not seen (%d fresh detections)", name().c_str(), id_,
                        fresh);
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::RUNNING;
    }
    void onHalted() override {}

  private:
    std::shared_ptr<Ros> ros_;
    rclcpp::Time started_;
    int id_ = -1, count_at_start_ = 0;
};

template <class T>
void register_node(BT::BehaviorTreeFactory& factory, const char* id, const std::shared_ptr<Ros>& ros) {
    factory.registerBuilder<T>(id, [ros](const std::string& name, const BT::NodeConfig& config) {
        return std::make_unique<T>(name, config, ros);
    });
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto ros = std::make_shared<Ros>();
    ros->node = std::make_shared<rclcpp::Node>("mission");
    auto& node = ros->node;
    const std::string tree_file = node->declare_parameter("tree", std::string());
    const std::vector<double> pick_guess = node->declare_parameter("pick_guess", std::vector<double>{1.2, 0.4, 0.15});
    const std::vector<double> place_guess = node->declare_parameter("place_guess", std::vector<double>{-0.3, 1.2, 0.08});
    const std::vector<int64_t> target_ids = node->declare_parameter("target_ids", std::vector<int64_t>{1, 2});
    const bool use_aruco = node->declare_parameter("use_aruco", true);
    const double tick_rate = node->declare_parameter("tick_rate", 20.0);

    ros->move_to = rclcpp_action::create_client<MoveTo>(node, "move_to");
    ros->attach = node->create_publisher<std_msgs::msg::Empty>("suction/attach", 10);
    ros->detach = node->create_publisher<std_msgs::msg::Empty>("suction/detach", 10);
    auto status_pub = node->create_publisher<std_msgs::msg::String>("mission/status", rclcpp::QoS(1).transient_local());
    auto state_sub = node->create_subscription<std_msgs::msg::String>(
        "suction/state", 10, [ros](std_msgs::msg::String::ConstSharedPtr m) {
            std::lock_guard<std::mutex> lock(ros->mutex);
            ros->suction_state = m->data;
        });
    for (int64_t id64 : target_ids) {
        const int id = int(id64);
        if (!use_aruco) {  // fallback without the camera: the fixed positions of the world
            const auto p = node->declare_parameter("fixed_target_" + std::to_string(id), std::vector<double>{});
            if (p.size() != 3) throw std::invalid_argument("use_aruco:=false needs fixed_target_<id> = [x, y, z]");
            ros->targets[id].top = Eigen::Vector3d(p[0], p[1], p[2]);
            ros->targets[id].fixed = true;
            continue;
        }
        ros->target_subs.push_back(node->create_subscription<geometry_msgs::msg::PointStamped>(
            "aruco/target_" + std::to_string(id), 10, [ros, id](geometry_msgs::msg::PointStamped::ConstSharedPtr m) {
                std::lock_guard<std::mutex> lock(ros->mutex);
                ros->targets[id].top = Eigen::Vector3d(m->point.x, m->point.y, m->point.z);
                ++ros->targets[id].count;
            }));
    }
    if (!use_aruco) RCLCPP_INFO(node->get_logger(), "using the fixed target positions instead of ArUco");

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner([&executor] { executor.spin(); });

    BT::BehaviorTreeFactory factory;
    register_node<MoveEE>(factory, "MoveEE", ros);
    register_node<MovePosture>(factory, "MovePosture", ros);
    register_node<FaceTarget>(factory, "FaceTarget", ros);
    register_node<SetSuction>(factory, "SetSuction", ros);
    register_node<WaitForTarget>(factory, "WaitForTarget", ros);

    int exit_code = 0;
    try {
        BT::Tree tree = factory.createTreeFromFile(tree_file);
        tree.rootBlackboard()->set("pick_guess", Eigen::Vector3d(pick_guess[0], pick_guess[1], pick_guess[2]));
        tree.rootBlackboard()->set("place_guess", Eigen::Vector3d(place_guess[0], place_guess[1], place_guess[2]));
        const auto t0 = node->now();
        std_msgs::msg::String status;
        status.data = "running";
        status_pub->publish(status);
        BT::NodeStatus result = BT::NodeStatus::RUNNING;
        rclcpp::WallRate rate(tick_rate);
        while (rclcpp::ok() && result == BT::NodeStatus::RUNNING) {
            result = tree.tickOnce();
            rate.sleep();
        }
        const double elapsed = (node->now() - t0).seconds();
        status.data = result == BT::NodeStatus::SUCCESS ? "succeeded" : "failed";
        status_pub->publish(status);
        RCLCPP_INFO(node->get_logger(), "mission %s after %.1f s (sim time)", status.data.c_str(), elapsed);
        exit_code = result == BT::NodeStatus::SUCCESS ? 0 : 1;
        std::this_thread::sleep_for(std::chrono::seconds(1));  // let the status reach latched subscribers
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "mission error: %s", e.what());
        exit_code = 2;
    }
    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return exit_code;
}
