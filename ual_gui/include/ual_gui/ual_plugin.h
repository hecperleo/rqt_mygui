#ifndef ual_gui_ual_plugin_H
#define ual_gui_ual_plugin_H

// C++
#include <string>
// ROS
#include <ros/ros.h>
#include "std_msgs/Empty.h"
#include "std_msgs/String.h"
#include <std_msgs/Int8.h>
#include "std_srvs/Empty.h"
#include "geometry_msgs/Twist.h"
// QT & RQT
#include <rqt_gui_cpp/plugin.h>
#include <ual_gui/ui_ual_plugin.h>
#include <QWidget>
#include <QBasicTimer>
#include <QString>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFile>
#include <QMessageBox>
#include <QTextStream>
#include <QButtonGroup>
// TEST
#include "sensor_msgs/PointCloud2.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"

#include <uav_abstraction_layer/ual.h>
#include <uav_abstraction_layer/GoToWaypoint.h>
#include <uav_abstraction_layer/Land.h>
#include <uav_abstraction_layer/TakeOff.h>
#include <uav_abstraction_layer/SetVelocity.h>

namespace ual_gui
{

class cloudSignal : public QObject
{
  Q_OBJECT

public:
  cloudSignal(Ui::UalPluginWidget ui)
  {
    m_value = 0;
    ui_ = ui;
  }

  int value() const { return m_value; }

public slots:
  void setValue(int value);

signals:
  void valueChanged(int newValue);

private:
  int m_value;
  Ui::UalPluginWidget ui_;
};

class UalPlugin : public rqt_gui_cpp::Plugin
{
  Q_OBJECT
public:
  UalPlugin();
  virtual void initPlugin(qt_gui_cpp::PluginContext &context);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings &plugin_settings, qt_gui_cpp::Settings &instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings &plugin_settings, const qt_gui_cpp::Settings &instance_settings);
  Ui::UalPluginWidget ui_;
  // --------------------------------------------------------------------------
protected slots:
  virtual void press_takeOff();
  virtual void press_land();
  virtual void press_goToWaypoint();
  virtual void press_setVelocity();

protected:
  // UAL //
  virtual void state_callback(const std_msgs::String msg);
  virtual void pose_callback(const geometry_msgs::PoseStamped);
  virtual void velocity_callback(const geometry_msgs::TwistStamped);
  // UAL //

  // Comment in to signal that the plugin has a way to configure it
  // bool hasConfiguration() const;
  // void triggerConfiguration();
private:
  QWidget *widget_;
  // Node
  ros::NodeHandle n_;
  // Subscriber
  // UAL //
  ros::ServiceClient srvTakeOff, srvLand, srvGoToWaypoint, srvSetVelocity;
  uav_abstraction_layer::TakeOff take_off;
  uav_abstraction_layer::Land land;
  uav_abstraction_layer::GoToWaypoint go_to_waypoint;
  uav_abstraction_layer::SetVelocity set_velocity;
  geometry_msgs::TwistStamped vel;
  geometry_msgs::PoseStamped wp;
  // UAL //
  ros::Subscriber state_sub, pose_sub, velocity_sub;
  // ros::Publisher resolution;
  // --------------------------------------------------------------------------
};
} // namespace ual_gui
#endif // ual_gui_ual_plugin_H
