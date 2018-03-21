#include "rqt_mygui/my_plugin.h"
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <QObject>
#include <QMetaObject>
#include <Qt>
#include <ros/ros.h>
#include "nav_msgs/Path.h"

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
  flagSubvectorT = true;
  resolucion = 1;
  std::cout << "[ TEST] " << '\n';

  // CONNECT
  connect(ui_.pushButton, SIGNAL(pressed()), this, SLOT(click_pushButton()));
  // SUBSCRIBERS
  setup_sub = n_.subscribe("vectorT", 0, &MyPlugin::setup_callback, this);
}

void MyPlugin::shutdownPlugin()
{
  // unregister all publishers here
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
  std::cout << "[ TEST] vector = ";
  for (int i = 0; i<vectorT.size(); i++){
    std::cout << vectorT[i] << " ";
  }
  std::cout << '\n';
  switch(resolucion){
    case 1: test("Resolucion 1"); resolucion++; break;
    case 2: test("Resolucion 2"); resolucion--; break;
  }
}

void MyPlugin::test(QString niz){
	ui_.label_3->setText(niz);
}

void MyPlugin::setup_callback(const nav_msgs::Path& msg){
  if(flagSubvectorT == true){
		for(int p = 0; p<msg.poses.size(); p++){
			vectorT.push_back(msg.poses.at(p).pose.position.x);
    }
  }
  flagSubvectorT = false;
  return;
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
