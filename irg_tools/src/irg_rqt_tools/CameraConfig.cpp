#include "CameraConfig.h"

#include <chrono>
#include <string>
#include <vector>

#include <pluginlib/class_list_macros.hpp>
#include <QComboBox>
#include <QMessageBox>
#include <QPushButton>

#include "Utils.h"

using namespace std::chrono_literals;

// Create a class with default constructor so that it can be used in a QVariant
class ParametersClient
{
public:
  rclcpp::AsyncParametersClient::SharedPtr m_parametersClient;
};

Q_DECLARE_METATYPE(ParametersClient)

namespace irg_rqt_tools {

  const QString CameraConfig::m_defaultADCBits = "12.0";
  const QString CameraConfig::m_defaultEnergyConversion = "1.0";
  const QString CameraConfig::m_defaultExposure = "1.0";
  const QString CameraConfig::m_defaultGain = "1.0";
  const QString CameraConfig::m_defaultGamma = "1.0";
  const QString CameraConfig::m_defaultReadNoise = "0.64";
  const QString CameraConfig::m_defaultShotNoise = "0.09";

  CameraConfig::CameraConfig() : rqt_gui_cpp::Plugin()
  {
    setObjectName("CameraConfig");

    // Create a node to be used for the parameter calls
    m_paramClientNode = std::make_shared<rclcpp::Node>("CameraConfigClient");
  }

  void CameraConfig::initPlugin(qt_gui_cpp::PluginContext& context)
  {
    m_widget = std::make_unique<QWidget>();
    m_ui.setupUi(m_widget.get());

    int serialNum = context.serialNumber();
    if (serialNum > 1) {
      m_widget->setWindowTitle(m_widget->windowTitle() + " " + QString::number(serialNum) + ")");
    }

    // Don't allow editing of the camera name in the combo box
    m_ui.camera_combo->setEditable(false);

    // Use a validator on all of the line edit controls to limit them to double values
    QDoubleValidator *validator = new QDoubleValidator();
    validator->setNotation(QDoubleValidator::StandardNotation);

    m_ui.adc_bits_edit->setValidator(validator);
    m_ui.energy_conversion_edit->setValidator(validator);
    m_ui.exposure_edit->setValidator(validator);
    m_ui.gain_edit->setValidator(validator);
    m_ui.gamma_edit->setValidator(validator);
    m_ui.read_noise_edit->setValidator(validator);
    m_ui.shot_noise_edit->setValidator(validator);

    // Set placeholder values in case the edit controls are empty
    m_ui.exposure_edit->setPlaceholderText(m_defaultExposure);
    m_ui.energy_conversion_edit->setPlaceholderText(m_defaultEnergyConversion);
    m_ui.read_noise_edit->setPlaceholderText(m_defaultReadNoise);
    m_ui.shot_noise_edit->setPlaceholderText(m_defaultShotNoise);
    m_ui.gain_edit->setPlaceholderText(m_defaultGain);
    m_ui.gamma_edit->setPlaceholderText(m_defaultGamma);
    m_ui.adc_bits_edit->setPlaceholderText(m_defaultADCBits);

    // Set the icons for all of the 'set' and 'scan' buttons
    m_ui.refresh_cameras_button->setIcon(QIcon::fromTheme("view-refresh"));
    m_ui.refresh_cameras_button->setText("");
    m_ui.set_adc_bits_button->setIcon(QIcon::fromTheme("document-save"));
    m_ui.set_adc_bits_button->setText("");
    m_ui.set_energy_conversion_button->setIcon(QIcon::fromTheme("document-save"));
    m_ui.set_energy_conversion_button->setText("");
    m_ui.set_exposure_button->setIcon(QIcon::fromTheme("document-save"));
    m_ui.set_exposure_button->setText("");
    m_ui.set_gain_button->setIcon(QIcon::fromTheme("document-save"));
    m_ui.set_gain_button->setText("");
    m_ui.set_gamma_button->setIcon(QIcon::fromTheme("document-save"));
    m_ui.set_gamma_button->setText("");
    m_ui.set_read_noise_button->setIcon(QIcon::fromTheme("document-save"));
    m_ui.set_read_noise_button->setText("");
    m_ui.set_shot_noise_button->setIcon(QIcon::fromTheme("document-save"));
    m_ui.set_shot_noise_button->setText("");

    // Connect the slots and signals
    connect(m_ui.camera_combo, SIGNAL(currentIndexChanged(int)), this, SLOT(onCameraChanged(int)));
    connect(m_ui.refresh_cameras_button, SIGNAL(clicked()), this, SLOT(onRefreshCamerasClicked()));
    connect(m_ui.restore_defaults_button, SIGNAL(clicked()),  this, SLOT(onRestoreDefaultsClicked()));
    connect(m_ui.set_adc_bits_button, SIGNAL(clicked()),  this, SLOT(onSetADCClicked()));
    connect(m_ui.set_energy_conversion_button, SIGNAL(clicked()),  this, SLOT(onSetEnergyConversionClicked()));
    connect(m_ui.set_exposure_button, SIGNAL(clicked()),  this, SLOT(onSetExposureClicked()));
    connect(m_ui.set_gain_button, SIGNAL(clicked()),  this, SLOT(onSetGainClicked()));
    connect(m_ui.set_gamma_button, SIGNAL(clicked()),  this, SLOT(onSetGammaClicked()));
    connect(m_ui.set_read_noise_button, SIGNAL(clicked()),  this, SLOT(onSetReadNoiseClicked()));
    connect(m_ui.set_shot_noise_button, SIGNAL(clicked()),  this, SLOT(onSetShotNoiseClicked()));

    context.addWidget(m_widget.get());
    updateCameraList();
  }

  void CameraConfig::shutdownPlugin()
  {
  }

  void CameraConfig::saveSettings(qt_gui_cpp::Settings& /*pluginSettings*/, qt_gui_cpp::Settings& instanceSettings) const
  {
    instanceSettings.setValue("current_camera", m_ui.camera_combo->currentText());
  }

  void CameraConfig::restoreSettings(const qt_gui_cpp::Settings& /*pluginSettings*/, const qt_gui_cpp::Settings& instanceSettings)
  {
    auto currentCamera = instanceSettings.value("current_camera", "").toString();

    // If there is a camera specified in the settings, try to select it in the combo box. If it happens
    // to not be present, we'll just be left with the default selection
    if (!currentCamera.isEmpty()) {
      selectCamera(currentCamera);
    }
  }

  void CameraConfig::onCameraChanged(int index)
  {
    // Nothing to do if the combobox was cleared
    if (index == -1) {
      return;
    }

    QString cameraName = m_ui.camera_combo->currentText();
    QVariant variant = m_ui.camera_combo->itemData(index);

    ParametersClient parametersClient = qvariant_cast<ParametersClient>(variant);
    m_currentCamera = parametersClient.m_parametersClient;

    auto future = m_currentCamera->get_parameters(
      {
        "adc_bits",
        "energy_conversion",
        "exposure",
        "gain",
        "gamma",
        "read_noise",
        "shot_noise"
      });

    auto result = rclcpp::executors::spin_node_until_future_complete(m_executor, m_paramClientNode, future, 2s);

    if (result != rclcpp::FutureReturnCode::SUCCESS) {
      QString errMsg("Failed to get parameters for camera: ");
      displayError(errMsg + cameraName);
      return;
    }

    auto parameters = future.get();

    for (auto & parameter : parameters) {
      auto value = QString::fromStdString(parameter.value_to_string());

      // Strip any trailing 0s, except for the last one
      while (value.back() =='0') {
        value.chop(1);
      }
      if (value.back() =='.') {
        value.append('0');
      }

      if (parameter.get_name() == "adc_bits") {
        m_ui.adc_bits_edit->setText(value);
      } else if (parameter.get_name() == "energy_conversion") {
        m_ui.energy_conversion_edit->setText(value);
      } else if (parameter.get_name() == "exposure") {
        m_ui.exposure_edit->setText(value);
      } else if (parameter.get_name() == "gain") {
        m_ui.gain_edit->setText(value);
      } else if (parameter.get_name() == "gamma") {
        m_ui.gamma_edit->setText(value);
      } else if (parameter.get_name() == "read_noise") {
        m_ui.read_noise_edit->setText(value);
      } else if (parameter.get_name() == "shot_noise") {
        m_ui.shot_noise_edit->setText(value);
      }
    }
  }

  void CameraConfig::onRefreshCamerasClicked()
  {
    updateCameraList();
  }

  void CameraConfig::onRestoreDefaultsClicked()
  {
    auto future = m_currentCamera->set_parameters(
      {
        rclcpp::Parameter("adc_bits", m_defaultADCBits.toDouble()),
        rclcpp::Parameter("energy_conversion", m_defaultEnergyConversion.toDouble()),
        rclcpp::Parameter("exposure", m_defaultExposure.toDouble()),
        rclcpp::Parameter("gain", m_defaultGain.toDouble()),
        rclcpp::Parameter("gamma", m_defaultGamma.toDouble()),
        rclcpp::Parameter("read_noise", m_defaultReadNoise.toDouble()),
        rclcpp::Parameter("shot_noise", m_defaultShotNoise.toDouble())
      });

    auto result = rclcpp::executors::spin_node_until_future_complete(m_executor, m_paramClientNode, future, 2s);

    if (result != rclcpp::FutureReturnCode::SUCCESS) {
      displayError("Failed to restore default parameter values");
      return;
    }

    // Update the edit controls to reflect the parameter changes
    m_ui.adc_bits_edit->setText(m_defaultADCBits);
    m_ui.energy_conversion_edit->setText(m_defaultEnergyConversion);
    m_ui.exposure_edit->setText(m_defaultExposure);
    m_ui.gain_edit->setText(m_defaultGain);
    m_ui.gamma_edit->setText(m_defaultGamma);
    m_ui.read_noise_edit->setText(m_defaultReadNoise);
    m_ui.shot_noise_edit->setText(m_defaultShotNoise);
  }

  void CameraConfig::onSetADCClicked()
  {
    setDoubleParameter("adc_bits", getValueFromLineEdit(m_ui.adc_bits_edit));
  }

  void CameraConfig::onSetEnergyConversionClicked()
  {
    setDoubleParameter("energy_conversion", getValueFromLineEdit(m_ui.energy_conversion_edit));
  }

  void CameraConfig::onSetExposureClicked()
  {
    setDoubleParameter("exposure", getValueFromLineEdit(m_ui.exposure_edit));
  }

  void CameraConfig::onSetGainClicked()
  {
    setDoubleParameter("gain", getValueFromLineEdit(m_ui.gain_edit));
  }

  void CameraConfig::onSetGammaClicked()
  {
    setDoubleParameter("gamma", getValueFromLineEdit(m_ui.gamma_edit));
  }

  void CameraConfig::onSetReadNoiseClicked()
  {
    setDoubleParameter("read_noise", getValueFromLineEdit(m_ui.read_noise_edit));
  }

  void CameraConfig::onSetShotNoiseClicked()
  {
    setDoubleParameter("shot_noise", getValueFromLineEdit(m_ui.shot_noise_edit));
  }

  void CameraConfig::addCamera(const QString& cameraName)
  {
    ParametersClient parametersClient;
    parametersClient.m_parametersClient = std::make_shared<rclcpp::AsyncParametersClient>(m_paramClientNode, cameraName.toStdString());

    QVariant associatedData;
    associatedData.setValue(parametersClient);

    m_ui.camera_combo->addItem(cameraName, associatedData);
  }

  void CameraConfig::displayError(const QString& errorMsg, QString detailedInfo)
  {
      QMessageBox msgBox(m_widget.get());
      msgBox.setWindowTitle("Error");
      msgBox.setText(errorMsg);
      if (!detailedInfo.isEmpty()) {
        msgBox.setDetailedText(detailedInfo);
      }
      msgBox.exec();
  }

  double CameraConfig::getValueFromLineEdit(QLineEdit *lineEdit)
  {
    QString text = lineEdit->text();
    if (text.isEmpty()) {
      text = lineEdit->placeholderText();
    }
    return text.toDouble();
  }

  void CameraConfig::selectCamera(const QString& cameraName)
  {
    auto index = m_ui.camera_combo->findText(cameraName);
    if (index != -1) {
      m_ui.camera_combo->setCurrentIndex(index);
    }
  }

  void CameraConfig::setDoubleParameter(const QString& paramName, double value)
  {
    auto future = m_currentCamera->set_parameters(
      {
        rclcpp::Parameter(paramName.toStdString(), value)
      });

    auto result = rclcpp::executors::spin_node_until_future_complete(m_executor, m_paramClientNode, future, 2s);

    if (result != rclcpp::FutureReturnCode::SUCCESS) {
      displayError("Failed to set camera parameter");
      m_ui.exposure_edit->setFocus(Qt::OtherFocusReason);
    }
  }

  void CameraConfig::updateCameraList()
  {
    auto currentCamera = m_ui.camera_combo->currentText();
    m_ui.camera_combo->clear();

    QStringList cameraNames = Utils::findMatchingNodeNames(node_, ".*CameraSim.*");

    if (cameraNames.isEmpty()) {
        displayError("No cameras found!");

        m_ui.adc_bits_edit->setEnabled(false);
        m_ui.energy_conversion_edit->setEnabled(false);
        m_ui.exposure_edit->setEnabled(false);
        m_ui.gain_edit->setEnabled(false);
        m_ui.gamma_edit->setEnabled(false);
        m_ui.read_noise_edit->setEnabled(false);
        m_ui.restore_defaults_button->setEnabled(false);
        m_ui.set_adc_bits_button->setEnabled(false);
        m_ui.set_energy_conversion_button->setEnabled(false);
        m_ui.set_exposure_button->setEnabled(false);
        m_ui.set_gain_button->setEnabled(false);
        m_ui.set_gamma_button->setEnabled(false);
        m_ui.set_read_noise_button->setEnabled(false);
        m_ui.set_shot_noise_button->setEnabled(false);
        m_ui.shot_noise_edit->setEnabled(false);
    } else {
      for (auto cameraName : cameraNames) {
        addCamera(cameraName);
      }

      m_ui.adc_bits_edit->setEnabled(true);
      m_ui.energy_conversion_edit->setEnabled(true);
      m_ui.exposure_edit->setEnabled(true);
      m_ui.gain_edit->setEnabled(true);
      m_ui.gamma_edit->setEnabled(true);
      m_ui.read_noise_edit->setEnabled(true);
      m_ui.restore_defaults_button->setEnabled(true);
      m_ui.set_adc_bits_button->setEnabled(true);
      m_ui.set_energy_conversion_button->setEnabled(true);
      m_ui.set_exposure_button->setEnabled(true);
      m_ui.set_gain_button->setEnabled(true);
      m_ui.set_gamma_button->setEnabled(true);
      m_ui.set_read_noise_button->setEnabled(true);
      m_ui.set_shot_noise_button->setEnabled(true);
      m_ui.shot_noise_edit->setEnabled(true);
    }
  }

}

PLUGINLIB_EXPORT_CLASS(irg_rqt_tools::CameraConfig, rqt_gui_cpp::Plugin)
