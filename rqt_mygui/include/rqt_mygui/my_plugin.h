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
#include "euler_from_quaternion/Euler.h"


namespace rqt_mygui
{

class MyPlugin : public rqt_gui_cpp::Plugin
{
  Q_OBJECT
public:
  MyPlugin();
  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings,qt_gui_cpp::Settings& instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings,const qt_gui_cpp::Settings& instance_settings);
// --------------------------------------------------------------------------
protected slots:
  virtual void click_pushButton();

protected:
  virtual void velodyne_callback(const sensor_msgs::PointCloud2& cloud);
  //virtual void resolution_callback(const std_msgs::Int8 msg);
  virtual void imu_callback(const euler_from_quaternion::Euler& msg);
  virtual void test(QString niz);
  virtual void update_list();
  //virtual void update_resolution();

  // Comment in to signal that the plugin has a way to configure it
  // bool hasConfiguration() const;
  // void triggerConfiguration();
private:
  Ui::MyPluginWidget ui_;
  QWidget* widget_;
  // Node
  ros::NodeHandle n_;
  // Subscriber
  ros::Subscriber velodyne_sub, resolution_sub, imu_sub;
  ros::Publisher resolution;
  // Variables
  //std::vector<double> vectorT;
  //bool flagSubvectorT;
  int resolucion;
  float cloudWidth;
  double begin;
  float time;
// --------------------------------------------------------------------------
};
}  // namespace rqt_mygui
#endif  // RQT_MYGUI_MY_PLUGIN_H
