#include <csignal>
#include <vector>

#include <QDebug>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QLabel>
#include <QProcess>
#include <QPushButton>
#include <QVBoxLayout>

#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/panel.hpp>
#include <yaml-cpp/yaml.h>

namespace arm_moveit_config
{

class PickPlacePanel : public rviz_common::Panel
{
public:
  explicit PickPlacePanel(QWidget * parent = nullptr) : rviz_common::Panel(parent)
  {
    auto * layout = new QVBoxLayout(this);
    auto * settings = new QWidget(this);
    auto * form = new QFormLayout(settings);
    auto * start = new QPushButton("Run Pick && Place", this);
    auto * stop = new QPushButton("Stop", this);
    stop->setEnabled(false);
    auto * status = new QLabel("Adjust motion settings, then click to run the full cycle.", this);
    status->setWordWrap(true);
    layout->addWidget(settings);
    layout->addWidget(start);
    layout->addWidget(stop);
    layout->addWidget(status);
    layout->addStretch();
    process_ = new QProcess(this);
    process_->setProcessChannelMode(QProcess::MergedChannels);
    const auto prefix = ament_index_cpp::get_package_prefix("arm_moveit_config");
    process_->setProgram(QString::fromStdString(prefix + "/lib/arm_moveit_config/shelf_pick_place.py"));
    try {
      const auto share = ament_index_cpp::get_package_share_directory("arm_moveit_config");
      const auto defaults = YAML::LoadFile(share + "/config/shelf_pick_place.yaml")["motion"];
      auto add_input = [this, form, &defaults](const char * label, const char * key,
          const char * suffix, double scale, double minimum, double maximum, double step) {
          auto * input = new QDoubleSpinBox;
          input->setDecimals(1);
          input->setRange(minimum, maximum);
          input->setSingleStep(step);
          input->setSuffix(suffix);
          input->setValue(defaults[key].as<double>() * scale);
          input->setKeyboardTracking(false);
          form->addRow(label, input);
          inputs_.push_back({key, input, scale});
          return input;
        };
      add_input("Speed", "velocity_scale", " %", 100.0, 1.0, 100.0, 5.0)
        ->setToolTip("Percentage of the configured joint velocity limits.");
      add_input("Acceleration", "acceleration_scale", " %", 100.0, 1.0, 100.0, 5.0)
        ->setToolTip("Percentage of the configured joint acceleration limits.");
      add_input("Approach distance", "approach", " mm", 1000.0, 1.0, 500.0, 10.0);
      add_input("Lift distance", "lift", " mm", 1000.0, 1.0, 500.0, 10.0);
      add_input("Shelf withdrawal", "withdraw_clearance", " mm", 1000.0, 0.0, 500.0, 10.0);
      add_input("Transfer height", "transfer_z", " mm", 1000.0, 1.0, 1500.0, 10.0)
        ->setToolTip("TCP height above the robot base when moving above the box.");
      add_input("Release clearance", "place_clearance", " mm", 1000.0, 0.0, 100.0, 1.0)
        ->setToolTip("Gap between the payload and the box bottom before release.");
    } catch (const std::exception & error) {
      status->setText("Cannot load motion settings: " + QString::fromUtf8(error.what()));
      settings->setEnabled(false);
      start->setEnabled(false);
    }
    connect(start, &QPushButton::clicked, this, [this, start, status, settings]() {
      if (process_->state() != QProcess::NotRunning) {
        return;
      }
      start->setEnabled(false);
      settings->setEnabled(false);
      QStringList arguments{"--mode", "execute", "--once", "--ros-args"};
      for (const auto & input : inputs_) {
        arguments << "-p" << QString("motion.%1:=%2").arg(
          input.parameter, QString::number(input.widget->value() / input.scale, 'f', 6));
      }
      process_->setArguments(arguments);
      task_error_.clear();
      stop_requested_ = false;
      status->setText("Planning and running the pick-and-place cycle...");
      process_->start();
    });
    connect(process_, &QProcess::started, this, [stop]() {stop->setEnabled(true);});
    connect(stop, &QPushButton::clicked, this, [this, stop, status]() {
      stop_requested_ = true;
      stop->setEnabled(false);
      status->setText("Stopping; waiting for motion cancellation...");
      requestStop();
    });
    connect(process_, &QProcess::readyReadStandardOutput, this, [this, status]() {
      while (process_->canReadLine()) {
        const auto line = QString::fromUtf8(process_->readLine()).trimmed();
        qInfo().noquote() << line;
        const QString stage_marker = "Executing physical stage: ";
        const QString error_marker = "Physical task stopped: ";
        const auto stage_offset = line.indexOf(stage_marker);
        const auto error_offset = line.indexOf(error_marker);
        if (stage_offset >= 0 && !stop_requested_) {
          status->setText("Running: " + line.mid(stage_offset + stage_marker.size()));
        }
        if (error_offset >= 0) {
          task_error_ = line.mid(error_offset + error_marker.size());
        }
      }
    });
    connect(process_, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished), this,
      [this, start, stop, status, settings](int code, QProcess::ExitStatus exit_status) {
        start->setEnabled(true);
        stop->setEnabled(false);
        settings->setEnabled(true);
        if (stop_requested_ && code == 130 && exit_status == QProcess::NormalExit) {
          status->setText("Stopped by user. Click to run again.");
        } else if (code == 0 && exit_status == QProcess::NormalExit) {
          status->setText("Complete. Click to run again.");
        } else {
          status->setText(task_error_.isEmpty() ?
            "Task stopped. See the terminal for details." : "Stopped: " + task_error_);
        }
      });
    connect(process_, &QProcess::errorOccurred, this,
      [this, start, stop, status, settings](QProcess::ProcessError error) {
        stop->setEnabled(false);
        if (error == QProcess::FailedToStart) {
          status->setText(process_->errorString());
          start->setEnabled(true);
          settings->setEnabled(true);
        }
      });
  }

  ~PickPlacePanel() override
  {
    if (process_->state() != QProcess::NotRunning) {
      // Send SIGINT directly to the task so it can cancel its active trajectory.
      requestStop();
      if (!process_->waitForFinished(12000)) {
        process_->kill();
        process_->waitForFinished(1000);
      }
    }
  }

private:
  void requestStop()
  {
    const auto pid = process_->processId();
    if (pid > 0) {
      ::kill(static_cast<pid_t>(pid), SIGINT);
    }
  }

  struct MotionInput
  {
    QString parameter;
    QDoubleSpinBox * widget;
    double scale;
  };
  std::vector<MotionInput> inputs_;
  QProcess * process_;
  QString task_error_;
  bool stop_requested_ = false;
};

}  // namespace arm_moveit_config

PLUGINLIB_EXPORT_CLASS(arm_moveit_config::PickPlacePanel, rviz_common::Panel)
