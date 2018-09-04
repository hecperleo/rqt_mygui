#include "ual_gui/ual_plugin.h"
#include <pluginlib/class_list_macros.h>

namespace ual_gui
{

UalPlugin::UalPlugin()
    : rqt_gui_cpp::Plugin(), widget_(0)
{
  // Constructor is called first before initPlugin function, needless to say.

  // give QObjects reasonable names
  setObjectName("UalPlugin");
}

void UalPlugin::initPlugin(qt_gui_cpp::PluginContext &context)
{
  // access standalone command line arguments
  QStringList argv = context.argv();
  // create QWidget
  widget_ = new QWidget();
  // extend the widget with all attributes and children from UI file
  ui_.setupUi(widget_);
  // add widget to the user interface
  context.addWidget(widget_);
  // ROS START
  ros::start();
  // CONNECT
  connect(ui_.pushButton_takeOff, SIGNAL(pressed()), this, SLOT(press_takeOff()));
  connect(ui_.pushButton_land, SIGNAL(pressed()), this, SLOT(press_land()));
  connect(ui_.pushButton_goToWaypoint, SIGNAL(pressed()), this, SLOT(press_goToWaypoint()));
  connect(ui_.pushButton_setVelocity, SIGNAL(pressed()), this, SLOT(press_setVelocity()));
  // SUBSCRIBERS
  state_sub = n_.subscribe("/uav_1/ual/state", 0, &UalPlugin::state_callback, this);
  pose_sub = n_.subscribe("/uav_1/ual/pose", 0, &UalPlugin::pose_callback, this);
  velocity_sub = n_.subscribe("/uav_1/ual/velocity", 0, &UalPlugin::velocity_callback, this);
  // PUBLISHERS
  // SERVICES
  srvSetVelocity = n_.serviceClient<uav_abstraction_layer::SetVelocity>("/uav_1/ual/set_velocity");
  srvTakeOff = n_.serviceClient<uav_abstraction_layer::TakeOff>("/uav_1/ual/take_off");
  srvLand = n_.serviceClient<uav_abstraction_layer::Land>("/uav_1/ual/land");
  srvGoToWaypoint = n_.serviceClient<uav_abstraction_layer::GoToWaypoint>("/uav_1/ual/go_to_waypoint");
}

void UalPlugin::shutdownPlugin()
{
  // unregister all publishers here
  n_.shutdown();
}

void UalPlugin::saveSettings(qt_gui_cpp::Settings &plugin_settings,
                             qt_gui_cpp::Settings &instance_settings) const
{
  // instance_settings.setValue(k, v)
}

void UalPlugin::restoreSettings(const qt_gui_cpp::Settings &plugin_settings,
                                const qt_gui_cpp::Settings &instance_settings)
{
  // v = instance_settings.value(k)
}

void UalPlugin::state_callback(const std_msgs::String msg)
{
  QString txt = QString::fromStdString(msg.data);
  ui_.label_State->setText(txt);
}

void UalPlugin::pose_callback(const geometry_msgs::PoseStamped msg)
{
  ui_.getPosePx->setText(QString::number(msg.pose.position.x, 'f', 2));
  ui_.getPosePy->setText(QString::number(msg.pose.position.y, 'f', 2));
  ui_.getPosePz->setText(QString::number(msg.pose.position.z, 'f', 2));
  ui_.getPoseOx->setText(QString::number(msg.pose.orientation.x, 'f', 2));
  ui_.getPoseOy->setText(QString::number(msg.pose.orientation.y, 'f', 2));
  ui_.getPoseOz->setText(QString::number(msg.pose.orientation.z, 'f', 2));
  ui_.getPoseOw->setText(QString::number(msg.pose.orientation.w, 'f', 2));
}

void UalPlugin::velocity_callback(const geometry_msgs::TwistStamped msg)
{
  ui_.getVelLx->setText(QString::number(msg.twist.linear.x, 'f', 2));
  ui_.getVelLy->setText(QString::number(msg.twist.linear.y, 'f', 2));
  ui_.getVelLz->setText(QString::number(msg.twist.linear.z, 'f', 2));
  ui_.getVelAx->setText(QString::number(msg.twist.angular.x, 'f', 2));
  ui_.getVelAy->setText(QString::number(msg.twist.angular.y, 'f', 2));
  ui_.getVelAz->setText(QString::number(msg.twist.angular.z, 'f', 2));
}

void UalPlugin::press_takeOff()
{
  double height = ui_.setTakeOffHeight->text().toDouble();
  take_off.request.height = height;
  take_off.request.blocking = false;
  srvTakeOff.call(take_off);
}

void UalPlugin::press_land()
{
  // land.request.twist.linear.x = 0.0;
  // land.request.twist.linear.y = 0.0;
  // land.request.twist.linear.z = 0.0;
  // land.request.twist.angular.x = 0.0;
  // land.request.twist.angular.y = 0.0;
  // land.request.twist.angular.z = 0.0;
  land.request.blocking = false;
  srvLand.call(land);
}

void UalPlugin::press_goToWaypoint()
{
  double pPx = ui_.setPosePx->text().toDouble();
  double pPy = ui_.setPosePy->text().toDouble();
  double pPz = ui_.setPosePz->text().toDouble();
  double pOx = ui_.setPoseOx->text().toDouble();
  double pOy = ui_.setPoseOy->text().toDouble();
  double pOz = ui_.setPoseOz->text().toDouble();
  double pOw = ui_.setPoseOw->text().toDouble();

  wp.header.frame_id = "map";
  wp.pose.position.x = pPx;
  wp.pose.position.y = pPy;
  wp.pose.position.z = pPz;
  wp.pose.orientation.x = pOx;
  wp.pose.orientation.y = pOy;
  wp.pose.orientation.z = pOz;
  wp.pose.orientation.w = pOw;

  go_to_waypoint.request.waypoint = wp;
  srvGoToWaypoint.call(go_to_waypoint);
}

void UalPlugin::press_setVelocity()
{
  double vLx = ui_.setVelLx->text().toDouble();
  double vLy = ui_.setVelLy->text().toDouble();
  double vLz = ui_.setVelLz->text().toDouble();
  double vAx = ui_.setVelAx->text().toDouble();
  double vAy = ui_.setVelAy->text().toDouble();
  double vAz = ui_.setVelAz->text().toDouble();

  vel.header.frame_id = "map";
  vel.twist.linear.x = vLx;
  vel.twist.linear.y = vLy;
  vel.twist.linear.z = vLz;
  vel.twist.angular.x = vAx;
  vel.twist.angular.y = vAy;
  vel.twist.angular.z = vAz;

  set_velocity.request.velocity = vel;
  srvSetVelocity.call(set_velocity);
}

/*bool hasConfiguration() const
{
  return true;
}

void triggerConfiguration()
{
  // Usually used to open a dialog to offer the user a set of configuration
}*/

} // namespace ual_gui

PLUGINLIB_EXPORT_CLASS(ual_gui::UalPlugin, rqt_gui_cpp::Plugin)
