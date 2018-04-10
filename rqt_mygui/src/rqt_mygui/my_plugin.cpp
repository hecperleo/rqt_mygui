#include "rqt_mygui/my_plugin.h"
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <QObject>
#include <QMetaObject>
#include <Qt>
#include <ros/ros.h>
// TEST
#include "sensor_msgs/PointCloud2.h"
#include <std_msgs/Int8.h>
#include <geometry_msgs/PoseStamped.h>
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"

namespace rqt_mygui
{

MyPlugin::MyPlugin()
    : rqt_gui_cpp::Plugin(), widget_(0)
{
  // Constructor is called first before initPlugin function, needless to say.

  // give QObjects reasonable names
  setObjectName("MyPlugin");
}

void MyPlugin::initPlugin(qt_gui_cpp::PluginContext &context)
{
  // access standalone command line arguments
  QStringList argv = context.argv();
  // create QWidget
  widget_ = new QWidget();
  // extend the widget with all attributes and children from UI file
  ui_.setupUi(widget_);
  cloudUpdate = new cloudSignal(ui_);
  // add widget to the user interface
  context.addWidget(widget_);

  ros::start();
  updateTime = 0.5;
  updateTimeList = ros::Time::now().toSec();
  updateTimeVelodyne = ros::Time::now().toSec();
  updateTimeGps = ros::Time::now().toSec();
  updateTimeImuEuler = ros::Time::now().toSec();
  updateTimeImuQuat = ros::Time::now().toSec();

  resolucion = 1;
  flagFirstItem = false;
  // CONNECT
  connect(ui_.pushButton, SIGNAL(pressed()), this, SLOT(click_pushButton()));
  connect(cloudUpdate, SIGNAL(valueChanged(int)), this, SLOT(setValue(int)));
  // SUBSCRIBERS
  velodyne_sub = n_.subscribe("velodyne_points", 0, &MyPlugin::velodyne_callback, this);
  gps_sub = n_.subscribe("fix", 0, &MyPlugin::gps_callback, this);
  imuEuler_sub = n_.subscribe("imu_euler_deg", 0, &MyPlugin::imuEuler_callback, this);
  imuQuat_sub = n_.subscribe("imu/data", 0, &MyPlugin::imuQuat_callback, this);
  //resolution_sub = n_.subscribe("resolution", 0, &MyPlugin::resolution_callback, this);
  // PUBLISHER
  resolution = n_.advertise<std_msgs::Int8>("resolution", 1);
}

void MyPlugin::shutdownPlugin()
{
  // unregister all publishers here
  resolution.shutdown();
  n_.shutdown();
}

void MyPlugin::saveSettings(qt_gui_cpp::Settings &plugin_settings,
                            qt_gui_cpp::Settings &instance_settings) const
{
  // instance_settings.setValue(k, v)
}

void MyPlugin::restoreSettings(const qt_gui_cpp::Settings &plugin_settings,
                               const qt_gui_cpp::Settings &instance_settings)
{
  // v = instance_settings.value(k)
}

// --------------------------------------------------------------------------

void MyPlugin::click_pushButton()
{
  switch (resolucion)
  {
  case 1:
    resolucion++;
    break;
  case 2:
    resolucion--;
    break;
  }
  ui_.label_resolutionValue->setText(QString(" ") + QString::number(resolucion, 'f', 0));
  //ui_.list_movil->item(0)->setText(QString("Cloud = ") + QString::number(cloudWidth, 'f', 0));
  MyPlugin::resolution_pub();
}

void cloudSignal::setValue(int value)
{
  emit valueChanged(value);

  /*if ((ros::Time::now().toSec() - 1.0) >= updateItem)
  {
    ROS_INFO("1.0");
    for (int i = 0; i < ui_.list_movil->count(); ++i)
    {
      //QListWidgetItem *item = ui_.list_movil->item(i);
      //Do stuff!
      //item->setText(QString("-"));
      //ui_.list_movil->item(i)->setText(QString::number(updateItem, 'f', 0));
    }
    //ui_.list_movil->item(0)->setText(QString::number(ros::Time::now().toSec()));
    updateItem = ros::Time::now().toSec();
  }*/
  //ui_.list_movil->update();
  //QApplication::processEvents();
}

void MyPlugin::test(QString niz)
{
  //ui_.list_movil->item(0)->setText(QString("Cloud = ") + QString::number(cloudWidth, 'f', 0));
  //ui_.label_resolutionValue->setText(niz);
}

void MyPlugin::velodyne_callback(const sensor_msgs::PointCloud2 &cloud)
{
  if ((ros::Time::now().toSec() - updateTimeVelodyne) >= updateTime)
  {
    ui_.label_widthValue->setText(QString::number(cloud.width, 'f', 1));
    updateTimeVelodyne = ros::Time::now().toSec();
  }
  ui_.label_updateValue->setText(QString::number((ros::Time::now().toSec() - updateTimeList), 'f', 1));
  cloudWidth = cloud.width;

  MyPlugin::update_list();

  cloudUpdate->setValue(cloudWidth);
}

void MyPlugin::gps_callback(const sensor_msgs::NavSatFix msg)
{
  if ((ros::Time::now().toSec() - updateTimeGps) >= updateTime)
  {
    ui_.label_gpsLatitude->setText(QString::number(msg.latitude, 'f', 7));
    ui_.label_gpsLongitude->setText(QString::number(msg.longitude, 'f', 7));
    ui_.label_gpsAltitude->setText(QString::number(msg.altitude, 'f', 7));
    //ui_.list_movil->item(0)->isSelected();
    //ui_.list_movil->item(0)->setText(QString("Cloud = ") + QString::number(cloudWidth, 'f', 0));

    updateTimeGps = ros::Time::now().toSec();
  }
}

/*void MyPlugin::resolution_callback(const std_msgs::Int8 msg){
  ui_.label_resolutionValue->setText(QString("Resolucion ") + QString::number(msg.data, 'f', 0));
}*/

void MyPlugin::imuEuler_callback(const sensor_msgs::Imu msg)
{
  if ((ros::Time::now().toSec() - updateTimeImuEuler) >= updateTime)
  {
    ui_.label_rollValue->setText(QString::number(msg.orientation.x, 'f', 2));
    ui_.label_pitchValue->setText(QString::number(msg.orientation.y, 'f', 2));
    ui_.label_yawValue->setText(QString::number(msg.orientation.z, 'f', 2));
    updateTimeImuEuler = ros::Time::now().toSec();
  }
}

void MyPlugin::imuQuat_callback(const sensor_msgs::Imu msg)
{
  if ((ros::Time::now().toSec() - updateTimeImuQuat) >= updateTime)
  {
    ui_.label_quatValueX->setText(QString::number(msg.orientation.x, 'f', 4));
    ui_.label_quatValueY->setText(QString::number(msg.orientation.y, 'f', 4));
    ui_.label_quatValueZ->setText(QString::number(msg.orientation.z, 'f', 4));
    ui_.label_quatValueW->setText(QString::number(msg.orientation.w, 'f', 4));
    updateTimeImuQuat = ros::Time::now().toSec();
  }
}

void MyPlugin::update_list()
{
  if (flagFirstItem == false)
  {
    for (int i = 0; i < 4; i++)
    {
      QListWidgetItem *lwi = new QListWidgetItem();
      lwi->setSizeHint(QSize(200, 20));
      lwi->setTextAlignment(Qt::AlignCenter);
      lwi->setText(QString("Obstáculo ") + QString::number(i));
      ui_.list_movil->addItem(lwi);
    }
    flagFirstItem = true;
  }
  if ((ros::Time::now().toSec() - 1.0) >= updateItem && flagFirstItem == true)
  {
    ROS_INFO("1.0");
    for (int i = 0; i < ui_.list_movil->count(); ++i)
    {
      ui_.list_movil->item(i)->setText(QString::number(updateItem, 'f', 0));
      ui_.list_movil->update();
    }
    updateItem = ros::Time::now().toSec();
  }
}
//ui_.list_movil->item(0)->setText(QString("Cloud = ") + QString::number(cloudWidth, 'f', 0));
/*if (flagFirstItem == true)
  {
    ui_.list_movil->item(0)->setText(QString("time = "));
    updateTimeList = ros::Time::now().toSec();
  }*/

void MyPlugin::resolution_pub()
{
  std_msgs::Int8 msg;
  msg.data = resolucion;
  resolution.publish(msg);
}

// --------------------------------------------------------------------------

/*bool hasConfiguration() const
{
  return true;
}

void triggerConfiguration()
{
  // Usually used to open a dialog to offer the user a set of configuration
}*/

} // namespace rqt_mygui
//PLUGINLIB_DECLARE_CLASS(rqt_mygui, MyPlugin, rqt_mygui::MyPlugin, rqt_gui_cpp::Plugin)
PLUGINLIB_EXPORT_CLASS(rqt_mygui::MyPlugin, rqt_gui_cpp::Plugin)
