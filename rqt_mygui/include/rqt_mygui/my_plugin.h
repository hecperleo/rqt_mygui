#ifndef RQT_MYGUI_MY_PLUGIN_H
#define RQT_MYGUI_MY_PLUGIN_H

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
#include <rqt_mygui/ui_my_plugin.h>
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
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"

namespace rqt_mygui
{

class cloudSignal : public QObject
{
  Q_OBJECT

public:
  cloudSignal() { m_value = 0; }

  int value() const { return m_value; }

public slots:
  void setValue(int value);

signals:
  void valueChanged(int newValue);

private:
  int m_value;
};

class MyPlugin : public rqt_gui_cpp::Plugin
{
  Q_OBJECT
public:
  MyPlugin();
  virtual void initPlugin(qt_gui_cpp::PluginContext &context);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings &plugin_settings, qt_gui_cpp::Settings &instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings &plugin_settings, const qt_gui_cpp::Settings &instance_settings);
  Ui::MyPluginWidget ui_;
  // --------------------------------------------------------------------------
protected slots:
  virtual void click_pushButton();

protected:
  virtual void velodyne_callback(const sensor_msgs::PointCloud2 &cloud);
  virtual void gps_callback(const sensor_msgs::NavSatFix msg);
  //virtual void resolution_callback(const std_msgs::Int8 msg);
  virtual void imuEuler_callback(const sensor_msgs::Imu msg);
  virtual void imuQuat_callback(const sensor_msgs::Imu msg);
  virtual void test(QString niz);
  virtual void update_list();
  virtual void resolution_pub();

  // Comment in to signal that the plugin has a way to configure it
  // bool hasConfiguration() const;
  // void triggerConfiguration();
private:
  QWidget *widget_;
  // Node
  ros::NodeHandle n_;
  // Subscriber
  ros::Subscriber velodyne_sub, resolution_sub, imuEuler_sub, imuQuat_sub, gps_sub;
  ros::Publisher resolution;
  // Variables
  //std::vector<double> vectorT;
  //bool flagSubvectorT;
  int resolucion;
  float cloudWidth;
  bool flagFirstItem;
  double updateTime, updateTimeVelodyne, updateTimeGps, updateTimeImuEuler, updateTimeImuQuat, updateTimeList;
  cloudSignal cloudUpdate;
  // --------------------------------------------------------------------------
};
} // namespace rqt_mygui
#endif // RQT_MYGUI_MY_PLUGIN_H
