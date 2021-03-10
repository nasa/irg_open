#ifndef irg_rqt_tools_CameraConfig_h
#define irg_rqt_tools_CameraConfig_h

#include <memory>

#include <QString>
#include <QWidget>
#include <rclcpp/rclcpp.hpp>
#include <rqt_gui_cpp/plugin.h>

#include "ui_CameraConfig.h"

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

    // Responses to the various button clicks
    virtual void onRefreshCamerasClicked();
    virtual void onRestoreDefaultsClicked();
    virtual void onSetADCClicked();
    virtual void onSetEnergyConversionClicked();
    virtual void onSetExposureClicked();
    virtual void onSetGainClicked();
    virtual void onSetGammaClicked();
    virtual void onSetReadNoiseClicked();
    virtual void onSetShotNoiseClicked();

  protected:
    void addCamera(const QString& cameraName);
    void displayError(const QString& errorMsg, QString detailedInfo = "");
    void selectCamera(const QString& cameraName);
    void setDoubleParameter(const QString& param_name, double value);
    void updateCameraList();

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

    // Parameter default values in case the user wants to restore back to default settings
    static const QString m_defaultADCBits;
    static const QString m_defaultEnergyConversion;
    static const QString m_defaultExposure;
    static const QString m_defaultGain;
    static const QString m_defaultGamma;
    static const QString m_defaultReadNoise;
    static const QString m_defaultShotNoise;
  };
}

#endif // irg_rqt_tools_CameraConfig_h
