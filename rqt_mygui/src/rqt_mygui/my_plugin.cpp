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
#include "euler_from_quaternion/Euler.h"
#include <geometry_msgs/PoseStamped.h>

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
  double updateTime = ros::Time::now().toSec();
  resolucion = 1;
  // CONNECT
  connect(ui_.pushButton, SIGNAL(pressed()), this, SLOT(click_pushButton()));
  // SUBSCRIBERS
  velodyne_sub   = n_.subscribe("velodyne_points", 0, &MyPlugin::velodyne_callback, this);
  xyzVelodyne_sub   = n_.subscribe("velodyne_xyz", 0, &MyPlugin::xyzVelodyne_callback, this);
  //resolution_sub = n_.subscribe("resolution", 0, &MyPlugin::resolution_callback, this);
  imu_sub        = n_.subscribe("Euler_RPY", 0, &MyPlugin::imu_callback, this);
  // PUBLISHER
  resolution 	   = n_.advertise<std_msgs::Int8>("resolution", 1);
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
  ui_.label_resolutionValue->setText(QString("Resolución ") + QString::number(resolucion, 'f', 0));
  MyPlugin::resolution_pub();
}

void MyPlugin::test(QString niz){
	//ui_.label_resolutionValue->setText(niz);
}

void MyPlugin::velodyne_callback(const sensor_msgs::PointCloud2& cloud){
  ui_.label_widthValue->setText(QString::number(cloud.width, 'f', 1));
  ui_.label_updateValue->setText(QString::number((ros::Time::now().toSec()-updateTime), 'f', 1)); 
  cloudWidth = cloud.width;
  MyPlugin::update_list();
}

void MyPlugin::xyzVelodyne_callback(const geometry_msgs::PoseStamped& msg){
  ui_.label_xVelodyneValue->setText(QString::number(msg.pose.position.x, 'f', 2));
  ui_.label_yVelodyneValue->setText(QString::number(msg.pose.position.y, 'f', 2));
  ui_.label_zVelodyneValue->setText(QString::number(msg.pose.position.z, 'f', 2));
}

/*void MyPlugin::resolution_callback(const std_msgs::Int8 msg){
  ui_.label_resolutionValue->setText(QString("Resolucion ") + QString::number(msg.data, 'f', 0));
}*/

void MyPlugin::imu_callback(const euler_from_quaternion::Euler& msg){
  ui_.label_rollValue->setText(QString::number(msg.roll, 'f', 2));
  ui_.label_pitchValue->setText(QString::number(msg.pitch, 'f', 2));
  ui_.label_yawValue->setText(QString::number(msg.yaw, 'f', 2));
}

void MyPlugin::update_list(){
  QListWidgetItem* lwi = new QListWidgetItem();           // Crea el item "lwi"
  lwi->setSizeHint(QSize(200, 20));                       // Se le da tamaño a lwi
  lwi->setTextAlignment(Qt::AlignCenter);                 // Todo lo escrito en lwi va a estar centrado
  if((ros::Time::now().toSec()-updateTime) >= 2.0){       // Si han pasado 2 segundos
    if (cloudWidth > 27300){                              // Si la nube de puntos es mayor de 27300
      lwi->setText(QString::number(cloudWidth, 'f', 1));  // Introduce el valor de la nube de puntos en lwi
      ui_.list_movil->addItem(lwi);                       // Añade lwi a la lista "list_movil"
    } else if(cloudWidth < 27000){                        // Si es menor de 27000
      ui_.list_movil->takeItem(0);                        // Borra el primer lwi que se ha añadido
      //ui_.list_movil->sortOrder();
      ui_.list_movil->sortItems(Qt::AscendingOrder);
    }
    updateTime = ros::Time::now().toSec();
  }
}

void MyPlugin::resolution_pub(){
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
