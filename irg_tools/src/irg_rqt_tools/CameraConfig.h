#ifndef irg_rqt_tools_CameraConfig_h
#define irg_rqt_tools_CameraConfig_h

#include <memory>

#include <QLineEdit>
#include <QString>
#include <QWidget>
#include <rclcpp/rclcpp.hpp>
#include <rqt_gui_cpp/plugin.h>

#include "ui_CameraConfig.h"
#include "CustomLineEdit.h"

namespace irg_rqt_tools {

class CameraConfig : public rqt_gui_cpp::Plugin
{
  Q_OBJECT

public:
  CameraConfig();

  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void shutdownPlugin();

  virtual void saveSettings(qt_gui_cpp::Settings& pluginSettings, 
                            qt_gui_cpp::Settings& instanceSettings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings& pluginSettings, 
                               const qt_gui_cpp::Settings& instanceSettings);

protected slots:
  // Response to a change in the Camera dropdown list
  virtual void onCameraChanged(int index);

  // Responses to various button clicks
  virtual void onRefreshCamerasClicked();
  virtual void onRestoreDefaultsClicked();
  virtual void onGetAllParametersClicked();

  // Responses to edit control focus changes
  virtual void onADCFocusChange(bool hasFocus);
  virtual void onEnergyConversionFocusChange(bool hasFocus);
  virtual void onExposureFocusChange(bool hasFocus);
  virtual void onGainFocusChange(bool hasFocus);
  virtual void onGammaFocusChange(bool hasFocus);
  virtual void onReadNoiseFocusChange(bool hasFocus);
  virtual void onShotNoiseFocusChange(bool hasFocus);

protected:
  void addCamera(const QString& cameraName);
  void displayError(const QString& errorMsg, QString detailedInfo = "");
  void getAllParameters();
  bool getDoubleParameter(const QString& param_name, double *value);
  double getValueFromLineEdit(QLineEdit *lineEdit);
  void selectCamera(const QString& cameraName);
  void setDoubleParameter(const QString& param_name, double value);
  void stripTrailingZeros(QString& s);
  void updateCameraList();
  void updateLineEdit(const char *paramName, CustomLineEdit *lineEdit, bool hasFocus);

protected:
  // The UI that results from reading the .ui file
  Ui::CameraConfigWidget m_ui;

  // The top-level widget for the UI
  std::unique_ptr<QWidget> m_widget;

  // A (non-spinning) ROS node used to make parameter calls
  rclcpp::Node::SharedPtr m_paramClientNode;

  // The parameters client for the currently-selected camera
  rclcpp::AsyncParametersClient::SharedPtr m_currentCamera;

  // The executor on which to make the parameter service calls
  rclcpp::executors::SingleThreadedExecutor m_executor;

  // The timeout to use on parameter-related service calls
  std::chrono::seconds m_serviceTimeout{1};

  // Parameter default values in case the user wants to restore back to default settings
  static const QString m_defaultADCBits;
  static const QString m_defaultEnergyConversion;
  static const QString m_defaultExposure;
  static const QString m_defaultGain;
  static const QString m_defaultGamma;
  static const QString m_defaultReadNoise;
  static const QString m_defaultShotNoise;

  // The max number of digits past the decimal to show in the edit control
  static const int m_significantDigits;
};

}

#endif // irg_rqt_tools_CameraConfig_h
