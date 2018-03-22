#include "rqt_mygui/my_plugin.h"
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <QObject>
#include <QMetaObject>
#include <Qt>
#include <ros/ros.h>
//#include "nav_msgs/Path.h"
#include "sensor_msgs/PointCloud2.h"
#include <std_msgs/Int8.h>

namespace rqt_mygui
{

MyPlugin::MyPlugin()
  : rqt_gui_cpp::Plugin()
  , widget_(0)
{
  // Constructor is called first before initPlugin function, needless to say.

  // give QObjects reasonable names
  setObjectName("MyPlugin");
}

void MyPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
{
  // access standalone command line arguments
  QStringList argv = context.argv();
  // create QWidget
  widget_ = new QWidget();
  // extend the widget with all attributes and children from UI file
  ui_.setupUi(widget_);
  // add widget to the user interface
  context.addWidget(widget_);

  ros::start();
  double begin = ros::Time::now().toSec();
  resolucion = 1;
  // CONNECT
  connect(ui_.pushButton, SIGNAL(pressed()), this, SLOT(click_pushButton()));
  // SUBSCRIBERS
  velodyne_sub   = n_.subscribe("velodyne_points", 0, &MyPlugin::velodyne_callback, this);
  resolution_sub = n_.subscribe("resolution", 0, &MyPlugin::resolution_callback, this);
  // PUBLISHER
  resolution 	= n_.advertise<std_msgs::Int8>("resolution", 1);
}

void MyPlugin::shutdownPlugin()
{
  // unregister all publishers here
  resolution.shutdown();
  n_.shutdown();
}

void MyPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings,
    qt_gui_cpp::Settings& instance_settings) const
{
  // instance_settings.setValue(k, v)
}

void MyPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
    const qt_gui_cpp::Settings& instance_settings)
{
  // v = instance_settings.value(k)
}

// --------------------------------------------------------------------------

void MyPlugin::click_pushButton(){
  switch(resolucion){
    case 1: resolucion++; break;
    case 2: resolucion--; break;
  }
}

void MyPlugin::test(QString niz){
	ui_.label_3->setText(niz);
}

void MyPlugin::velodyne_callback(const sensor_msgs::PointCloud2& cloud){
  ui_.label_widthValue->setText(QString::number(cloud.width, 'f', 1));
  ui_.label_heightValue->setText(QString::number((ros::Time::now().toSec()-begin), 'f', 1)); 
  cloudWidth = cloud.width;
  MyPlugin::update_list();
  MyPlugin::update_resolution();
}

void MyPlugin::resolution_callback(const std_msgs::Int8 msg){
  ui_.label_3->setText(QString("Resolucion ") + QString::number(msg.data, 'f', 0));
}

void MyPlugin::update_list(){
  QListWidgetItem* lwi = new QListWidgetItem(); 
  lwi->setSizeHint(QSize(200, 20));             
  lwi->setTextAlignment(Qt::AlignCenter);
  if((ros::Time::now().toSec()-begin) >= 2.0){
    if (cloudWidth > 27300){
      lwi->setText(QString::number(cloudWidth, 'f', 1));
      ui_.list_movil->addItem(lwi);
    } else if(cloudWidth < 27000){
      ui_.list_movil->takeItem(0);
    }
    begin = ros::Time::now().toSec();
  }
}

void MyPlugin::update_resolution(){
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

}  // namespace rqt_mygui
//PLUGINLIB_DECLARE_CLASS(rqt_mygui, MyPlugin, rqt_mygui::MyPlugin, rqt_gui_cpp::Plugin)
PLUGINLIB_EXPORT_CLASS(rqt_mygui::MyPlugin, rqt_gui_cpp::Plugin)
