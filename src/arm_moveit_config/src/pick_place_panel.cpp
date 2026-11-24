#include <chrono>
#include <memory>
#include <thread>
#include <vector>

#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QLabel>
#include <QProgressBar>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <arm_task_interfaces/action/pick_place.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rviz_common/panel.hpp>
#include <yaml-cpp/yaml.h>

namespace arm_moveit_config
{

class PickPlacePanel : public rviz_common::Panel
{
  using Action = arm_task_interfaces::action::PickPlace;
  using Goal = Action::Goal;
  using GoalHandle = rclcpp_action::ClientGoalHandle<Action>;

public:
  explicit PickPlacePanel(QWidget * parent = nullptr, bool warehouse = false) : rviz_common::Panel(parent)
  {
    auto * layout = new QVBoxLayout(this);
    if (warehouse) {
      auto * title = new QLabel("Factory", this);
      title->setStyleSheet("font-size: 17px; font-weight: bold; color: #38c9cb; margin: 8px 0;");
      layout->addWidget(title);
      auto * orders = new QLabel(
        "Blue box: 2 blue\nRed box: 2 red\nGreen tray: 2 green", this);
      orders->setWordWrap(true);
      layout->addWidget(orders);
    }
    settings_ = new QWidget(this);
    auto * form = new QFormLayout(settings_);
    start_ = new QPushButton(warehouse ? "Start Orders" : "Run Pick && Place", this);
    if (warehouse) {
      start_->setStyleSheet("padding: 10px; background: #167d83; color: white; font-weight: bold;");
    }
    stop_ = new QPushButton("Stop", this);
    progress_ = new QProgressBar(this);
    status_ = new QLabel("Waiting for the pick-and-place server...", this);
    status_->setWordWrap(true);
    start_->setEnabled(false);
    stop_->setEnabled(false);
    layout->addWidget(settings_);
    layout->addWidget(start_);
    layout->addWidget(stop_);
    layout->addWidget(progress_);
    layout->addWidget(status_);
    layout->addStretch();

    try {
      const auto share = ament_index_cpp::get_package_share_directory("arm_moveit_config");
      const auto defaults = YAML::LoadFile(share +
        (warehouse ? "/config/warehouse.yaml" : "/config/shelf_pick_place.yaml"))["motion"];
      auto add_input = [this, form, &defaults](const char * label, const char * key,
          double Goal::* field, const char * suffix, double scale, double minimum,
          double maximum, double step) {
          auto * input = new QDoubleSpinBox;
          input->setDecimals(1);
          input->setRange(minimum, maximum);
          input->setSingleStep(step);
          input->setSuffix(suffix);
          input->setValue(defaults[key].as<double>() * scale);
          input->setKeyboardTracking(false);
          form->addRow(label, input);
          inputs_.push_back({input, field, scale});
          return input;
        };
      add_input("Speed", "velocity_scale", &Goal::velocity_scale, " %", 100, 1, 100, 5)
        ->setToolTip("Percentage of the configured joint velocity limits.");
      add_input("Acceleration", "acceleration_scale", &Goal::acceleration_scale,
        " %", 100, 1, 100, 5);
      add_input("Approach distance", "approach", &Goal::approach, " mm", 1000, 1, 500, 10);
      add_input("Lift distance", "lift", &Goal::lift, " mm", 1000, 1, 500, 10);
      add_input("Shelf withdrawal", "withdraw_clearance", &Goal::withdraw_clearance,
        " mm", 1000, 0, 500, 10);
      add_input("Transfer height", "transfer_z", &Goal::transfer_z, " mm", 1000, 1, 1500, 10)
        ->setToolTip("TCP height above the robot base when moving above the box.");
      add_input("Release clearance", "place_clearance", &Goal::place_clearance,
        " mm", 1000, 0, 100, 1);
      settings_valid_ = true;
    } catch (const std::exception & error) {
      status_->setText("Cannot load motion settings: " + QString::fromUtf8(error.what()));
      settings_->setEnabled(false);
    }

    node_ = std::make_shared<rclcpp::Node>(
      "pick_place_panel", rclcpp::NodeOptions().use_global_arguments(false));
    client_ = rclcpp_action::create_client<Action>(node_, warehouse ? "/warehouse_order" : "/shelf_pick_place");
    executor_.add_node(node_);
    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, [this]() {
      if (!rclcpp::ok()) {
        timer_->stop();
        start_->setEnabled(false);
        stop_->setEnabled(false);
        return;
      }
      // All action callbacks run on the Qt thread and can safely update widgets.
      executor_.spin_some();
      const bool ready = client_->action_server_is_ready();
      start_->setEnabled(settings_valid_ && !busy_ && ready);
      if (settings_valid_ && !busy_ && ready != server_ready_) {
        status_->setText(ready ? "Adjust motion settings, then click to run the full cycle." :
          "Waiting for the pick-and-place server...");
      }
      server_ready_ = ready;
    });
    timer_->start(50);
    connect(start_, &QPushButton::clicked, this, [this]() {startTask();});
    connect(stop_, &QPushButton::clicked, this, [this]() {
      stop_requested_ = true;
      stop_->setEnabled(false);
      status_->setText("Stopping; waiting for motion cancellation and scene cleanup...");
      requestCancel();
    });
  }

  ~PickPlacePanel() override
  {
    timer_->stop();
    stop_requested_ = true;
    if (rclcpp::ok()) {
      requestCancel();
    }
    // Also handle closing the panel while the goal acknowledgement is pending.
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (busy_ && rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      executor_.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    executor_.remove_node(node_);
  }

private:
  void startTask()
  {
    if (busy_ || !client_->action_server_is_ready()) {
      return;
    }
    Goal goal;
    goal.use_default_motion = false;
    for (const auto & input : inputs_) {
      goal.*(input.field) = input.widget->value() / input.scale;
    }
    busy_ = true;
    stop_requested_ = cancel_sent_ = false;
    start_->setEnabled(false);
    stop_->setEnabled(true);
    settings_->setEnabled(false);
    progress_->setRange(0, 0);
    status_->setText("Submitting pick-and-place task...");
    rclcpp_action::Client<Action>::SendGoalOptions options;
    options.goal_response_callback = [this](GoalHandle::SharedPtr handle) {
        goal_ = handle;
        if (!handle) {
          finish("Task rejected: another task is busy or the settings are invalid.", false);
        } else if (stop_requested_) {
          requestCancel();
        }
      };
    options.feedback_callback = [this](GoalHandle::SharedPtr,
        const std::shared_ptr<const Action::Feedback> feedback) {
        if (!stop_requested_) {
        status_->setText(QString("%1: %2 (%3/%4)")
            .arg(QString::fromStdString(feedback->phase),
              QString::fromStdString(feedback->stage).replace('_', ' '))
            .arg(feedback->completed_stages).arg(feedback->total_stages));
        }
        progress_->setRange(0, static_cast<int>(feedback->total_stages));
        progress_->setValue(static_cast<int>(feedback->completed_stages));
      };
    options.result_callback = [this](const GoalHandle::WrappedResult & result) {
        const bool success = result.code == rclcpp_action::ResultCode::SUCCEEDED &&
          result.result && result.result->success;
        finish(result.result ? QString::fromStdString(result.result->message) :
          "Task ended without a result; inspect the server.", success);
      };
    try {
      client_->async_send_goal(goal, options);
    } catch (const std::exception & error) {
      finish(QString::fromUtf8(error.what()), false);
    }
  }

  void requestCancel()
  {
    if (!goal_ || cancel_sent_) {
      return;
    }
    cancel_sent_ = true;
    try {
      client_->async_cancel_goal(goal_, [this](auto response) {
        if (response->return_code != 0 && busy_) {
          cancel_sent_ = false;
          stop_->setEnabled(true);
          status_->setText("Cancellation was not accepted; waiting for the task result.");
        }
      });
    } catch (const std::exception & error) {
      cancel_sent_ = false;
      stop_->setEnabled(true);
      status_->setText("Could not send cancellation: " + QString::fromUtf8(error.what()));
    }
  }

  void finish(const QString & message, bool success)
  {
    busy_ = false;
    goal_.reset();
    stop_->setEnabled(false);
    settings_->setEnabled(settings_valid_);
    progress_->setRange(0, 1);
    progress_->setValue(success ? 1 : 0);
    status_->setText(message);
  }

  struct MotionInput
  {
    QDoubleSpinBox * widget;
    double Goal::* field;
    double scale;
  };
  std::vector<MotionInput> inputs_;
  QWidget * settings_;
  QPushButton * start_;
  QPushButton * stop_;
  QProgressBar * progress_;
  QLabel * status_;
  QTimer * timer_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<Action>::SharedPtr client_;
  GoalHandle::SharedPtr goal_;
  bool settings_valid_ = false;
  bool server_ready_ = false;
  bool busy_ = false;
  bool stop_requested_ = false;
  bool cancel_sent_ = false;
};

class WarehousePanel : public PickPlacePanel
{
public:
  explicit WarehousePanel(QWidget * parent = nullptr) : PickPlacePanel(parent, true) {}
};

}  // namespace arm_moveit_config

PLUGINLIB_EXPORT_CLASS(arm_moveit_config::PickPlacePanel, rviz_common::Panel)
PLUGINLIB_EXPORT_CLASS(arm_moveit_config::WarehousePanel, rviz_common::Panel)
