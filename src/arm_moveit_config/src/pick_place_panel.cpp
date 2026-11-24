#include <csignal>

#include <QDebug>
#include <QLabel>
#include <QProcess>
#include <QPushButton>
#include <QVBoxLayout>

#include <ament_index_cpp/get_package_prefix.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/panel.hpp>

namespace arm_moveit_config
{

class PickPlacePanel : public rviz_common::Panel
{
public:
  explicit PickPlacePanel(QWidget * parent = nullptr) : rviz_common::Panel(parent)
  {
    auto * layout = new QVBoxLayout(this);
    auto * start = new QPushButton("Run Pick && Place", this);
    auto * status = new QLabel("Click to start the full pick-and-place cycle.", this);
    status->setWordWrap(true);
    layout->addWidget(start);
    layout->addWidget(status);
    layout->addStretch();
    process_ = new QProcess(this);
    process_->setProcessChannelMode(QProcess::MergedChannels);
    const auto prefix = ament_index_cpp::get_package_prefix("arm_moveit_config");
    process_->setProgram(QString::fromStdString(prefix + "/lib/arm_moveit_config/shelf_pick_place.py"));
    process_->setArguments({"--mode", "execute", "--once"});
    connect(start, &QPushButton::clicked, this, [this, start, status]() {
      if (process_->state() != QProcess::NotRunning) {
        return;
      }
      start->setEnabled(false);
      task_error_.clear();
      status->setText("Planning and running the pick-and-place cycle...");
      process_->start();
    });
    connect(process_, &QProcess::readyReadStandardOutput, this, [this, status]() {
      while (process_->canReadLine()) {
        const auto line = QString::fromUtf8(process_->readLine()).trimmed();
        qInfo().noquote() << line;
        const QString stage_marker = "Executing physical stage: ";
        const QString error_marker = "Physical task stopped: ";
        const auto stage_offset = line.indexOf(stage_marker);
        const auto error_offset = line.indexOf(error_marker);
        if (stage_offset >= 0) {
          status->setText("Running: " + line.mid(stage_offset + stage_marker.size()));
        }
        if (error_offset >= 0) {
          task_error_ = line.mid(error_offset + error_marker.size());
        }
      }
    });
    connect(process_, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished), this,
      [this, start, status](int code, QProcess::ExitStatus exit_status) {
        start->setEnabled(true);
        status->setText(code == 0 && exit_status == QProcess::NormalExit ?
          "Complete. Click to run again." : (task_error_.isEmpty() ?
          "Task stopped. See the terminal for details." : "Stopped: " + task_error_));
      });
    connect(process_, &QProcess::errorOccurred, this,
      [this, start, status](QProcess::ProcessError error) {
        if (error == QProcess::FailedToStart) {
          status->setText(process_->errorString());
          start->setEnabled(true);
        }
      });
  }

  ~PickPlacePanel() override
  {
    if (process_->state() != QProcess::NotRunning) {
      // Send SIGINT directly to the task so it can cancel its active trajectory.
      const auto pid = process_->processId();
      if (pid > 0) {
        ::kill(static_cast<pid_t>(pid), SIGINT);
      } else {
        process_->kill();
      }
      if (!process_->waitForFinished(5000)) {
        process_->kill();
        process_->waitForFinished(1000);
      }
    }
  }

private:
  QProcess * process_;
  QString task_error_;
};

}  // namespace arm_moveit_config

PLUGINLIB_EXPORT_CLASS(arm_moveit_config::PickPlacePanel, rviz_common::Panel)
