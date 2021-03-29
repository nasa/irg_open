#include "CameraConfig.h"

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <pluginlib/class_list_macros.hpp>
#include <QComboBox>
#include <QMessageBox>
#include <QPushButton>

#include "CustomLineEdit.h"
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

const char *CameraConfig::m_nameADCBits = "adc_bits";
const char *CameraConfig::m_nameEnergyConversion = "energy_conversion";
const char *CameraConfig::m_nameExposure = "exposure";
const char *CameraConfig::m_nameGain = "gain";
const char *CameraConfig::m_nameGamma = "gamma";
const char *CameraConfig::m_nameReadNoise = "read_noise";
const char *CameraConfig::m_nameShotNoise = "shot_noise";

const QString CameraConfig::m_defaultADCBits = "12.0";
const QString CameraConfig::m_defaultEnergyConversion = "1.0";
const QString CameraConfig::m_defaultExposure = "1.0";
const QString CameraConfig::m_defaultGain = "1.0";
const QString CameraConfig::m_defaultGamma = "1.0";
const QString CameraConfig::m_defaultReadNoise = "0.64";
const QString CameraConfig::m_defaultShotNoise = "0.09";

const int CameraConfig::m_significantDigits = 6;

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

  // Set the icons for the 'scan' button
  m_ui.refresh_cameras_button->setIcon(QIcon::fromTheme("view-refresh"));
  m_ui.refresh_cameras_button->setText("");

  // Connect the slots and signals
  connect(m_ui.camera_combo, SIGNAL(currentIndexChanged(int)), this, SLOT(onCameraChanged(int)));
  connect(m_ui.refresh_cameras_button, SIGNAL(clicked()), this, SLOT(onRefreshCamerasClicked()));
  connect(m_ui.restore_defaults_button, SIGNAL(clicked()),  this, SLOT(onRestoreDefaultsClicked()));
  connect(m_ui.get_all_parameters_button, SIGNAL(clicked()),  this, SLOT(onGetAllParametersClicked()));

  connect(m_ui.adc_bits_edit, SIGNAL(focusChanged(bool)), this, SLOT(onADCFocusChange(bool)));
  connect(m_ui.energy_conversion_edit, SIGNAL(focusChanged(bool)), this, SLOT(onEnergyConversionFocusChange(bool)));
  connect(m_ui.exposure_edit, SIGNAL(focusChanged(bool)), this, SLOT(onExposureFocusChange(bool)));
  connect(m_ui.gain_edit, SIGNAL(focusChanged(bool)), this, SLOT(onGainFocusChange(bool)));
  connect(m_ui.gamma_edit, SIGNAL(focusChanged(bool)), this, SLOT(onGammaFocusChange(bool)));
  connect(m_ui.read_noise_edit, SIGNAL(focusChanged(bool)), this, SLOT(onReadNoiseFocusChange(bool)));
  connect(m_ui.shot_noise_edit, SIGNAL(focusChanged(bool)), this, SLOT(onShotNoiseFocusChange(bool)));

  connect(m_ui.adc_bits_edit, SIGNAL(editingFinished()), this, SLOT(onADCEditingFinished()));
  connect(m_ui.energy_conversion_edit, SIGNAL(editingFinished()), this, SLOT(onEnergyConversionEditingFinished()));
  connect(m_ui.exposure_edit, SIGNAL(editingFinished()), this, SLOT(onExposureEditingFinished()));
  connect(m_ui.gain_edit, SIGNAL(editingFinished()), this, SLOT(onGainEditingFinished()));
  connect(m_ui.gamma_edit, SIGNAL(editingFinished()), this, SLOT(onGammaEditingFinished()));
  connect(m_ui.read_noise_edit, SIGNAL(editingFinished()), this, SLOT(onReadNoiseEditingFinished()));
  connect(m_ui.shot_noise_edit, SIGNAL(editingFinished()), this, SLOT(onShotNoiseEditingFinished()));

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
  // to not be present this time, we'll just use the default selection
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

  getAllParameters();
}

void CameraConfig::onRefreshCamerasClicked()
{
  updateCameraList();
}

void CameraConfig::onRestoreDefaultsClicked()
{
  auto future = m_currentCamera->set_parameters(
    {
      rclcpp::Parameter(m_nameADCBits, m_defaultADCBits.toDouble()),
      rclcpp::Parameter(m_nameEnergyConversion, m_defaultEnergyConversion.toDouble()),
      rclcpp::Parameter(m_nameExposure, m_defaultExposure.toDouble()),
      rclcpp::Parameter(m_nameGain, m_defaultGain.toDouble()),
      rclcpp::Parameter(m_nameGamma, m_defaultGamma.toDouble()),
      rclcpp::Parameter(m_nameReadNoise, m_defaultReadNoise.toDouble()),
      rclcpp::Parameter(m_nameShotNoise, m_defaultShotNoise.toDouble())
    });

  auto result = rclcpp::executors::spin_node_until_future_complete(m_executor,
                                                                   m_paramClientNode,
                                                                   future,
                                                                   m_serviceTimeout);

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

void CameraConfig::onGetAllParametersClicked()
{
  getAllParameters();
}

void CameraConfig::stripTrailingZeros(QString& s)
{
  // Strip any trailing 0s, except for the last one
  while (!s.isEmpty() && s[s.size()] =='0') {
    s.chop(1);
  }
  if (!s.isEmpty() && s[s.size()] =='.') {
    s.append('0');
  }
}

void CameraConfig::updateLineEdit(const char *paramName, CustomLineEdit *lineEdit, bool hasFocus)
{
  if (hasFocus) {
    // On entry into the edit control, get the current paraemeter value and populate the control
    double value = 0.0;
    if (getDoubleParameter(paramName, &value)) {
      QString strValue = QString::number(value, 'f', m_significantDigits);
      stripTrailingZeros(strValue);
      lineEdit->setText(strValue);
    } else {
      // If we couldn't get the parameter value, let the user know
      QString errMsg("Failed to get parameter: ");
      displayError(errMsg + paramName);
    }
  } else {
    // When leaving the control, set the parameter
    setDoubleParameter(paramName, getValueFromLineEdit(lineEdit));
  }
}

void CameraConfig::onADCFocusChange(bool hasFocus)
{
  updateLineEdit(m_nameADCBits, m_ui.adc_bits_edit, hasFocus);
}

void CameraConfig::onEnergyConversionFocusChange(bool hasFocus)
{
  updateLineEdit(m_nameEnergyConversion, m_ui.energy_conversion_edit, hasFocus);
}

void CameraConfig::onExposureFocusChange(bool hasFocus)
{
  updateLineEdit(m_nameExposure, m_ui.exposure_edit, hasFocus);
}

void CameraConfig::onGainFocusChange(bool hasFocus)
{
  updateLineEdit(m_nameGain, m_ui.gain_edit, hasFocus);
}

void CameraConfig::onGammaFocusChange(bool hasFocus)
{
  updateLineEdit(m_nameGamma, m_ui.gamma_edit, hasFocus);
}

void CameraConfig::onReadNoiseFocusChange(bool hasFocus)
{
  updateLineEdit(m_nameReadNoise, m_ui.read_noise_edit, hasFocus);
}

void CameraConfig::onShotNoiseFocusChange(bool hasFocus)
{
  updateLineEdit(m_nameShotNoise, m_ui.shot_noise_edit, hasFocus);
}

void CameraConfig::onADCEditingFinished()
{
  updateLineEdit(m_nameADCBits, m_ui.adc_bits_edit, false);
}

void CameraConfig::onEnergyConversionEditingFinished()
{
  updateLineEdit(m_nameEnergyConversion, m_ui.energy_conversion_edit, false);
}

void CameraConfig::onExposureEditingFinished()
{
  updateLineEdit(m_nameExposure, m_ui.exposure_edit, false);
}

void CameraConfig::onGainEditingFinished()
{
  updateLineEdit(m_nameGain, m_ui.gain_edit, false);
}

void CameraConfig::onGammaEditingFinished()
{
  updateLineEdit(m_nameGamma, m_ui.gamma_edit, false);
}

void CameraConfig::onReadNoiseEditingFinished()
{
  updateLineEdit(m_nameReadNoise, m_ui.read_noise_edit, false);
}

void CameraConfig::onShotNoiseEditingFinished()
{
  updateLineEdit(m_nameShotNoise, m_ui.shot_noise_edit, false);
}

void CameraConfig::addCamera(const QString& cameraName)
{
  ParametersClient parametersClient;
  parametersClient.m_parametersClient = std::make_shared<rclcpp::AsyncParametersClient>(m_paramClientNode, cameraName.toStdString());

  // As we add the service clients, make sure the services are ready
  if (!parametersClient.m_parametersClient->wait_for_service(m_serviceTimeout)) {
    QString errMsg("Parameter service not found for camera: ");
    displayError(errMsg + cameraName);
    return;
  }

  QVariant associatedData;
  associatedData.setValue(parametersClient);

  m_ui.camera_combo->addItem(cameraName, associatedData);
}

void CameraConfig::getAllParameters()
{
  auto future = m_currentCamera->get_parameters(
    {
      m_nameADCBits,
      m_nameEnergyConversion,
      m_nameExposure,
      m_nameGain,
      m_nameGamma,
      m_nameReadNoise,
      m_nameShotNoise
    });

  auto result = rclcpp::executors::spin_node_until_future_complete(m_executor,
                                                                   m_paramClientNode,
                                                                   future,
                                                                   m_serviceTimeout);

  if (result != rclcpp::FutureReturnCode::SUCCESS) {
    // Occasionally, the get_parameter call fails at start-up because the node we're using
    // for the service calls (which can't already be spinning) is not completely initialized
    // with full discovery information. So, give it one retry. This seems to work better
    // than extending the timeout
    result = rclcpp::executors::spin_node_until_future_complete(m_executor,
                                                                m_paramClientNode,
                                                                future,
                                                                m_serviceTimeout);
    if (result != rclcpp::FutureReturnCode::SUCCESS) {
      QString errMsg("Failed to get parameters for camera: ");
      displayError(errMsg + m_ui.camera_combo->currentText());
      return;
    }
  }

  auto parameters = future.get();

  for (auto & parameter : parameters) {
    auto value = QString::fromStdString(parameter.value_to_string());
    stripTrailingZeros(value);

    if (parameter.get_name() == m_nameADCBits) {
      m_ui.adc_bits_edit->setText(value);
    } else if (parameter.get_name() == m_nameEnergyConversion) {
      m_ui.energy_conversion_edit->setText(value);
    } else if (parameter.get_name() == m_nameExposure) {
      m_ui.exposure_edit->setText(value);
    } else if (parameter.get_name() == m_nameGain) {
      m_ui.gain_edit->setText(value);
    } else if (parameter.get_name() == m_nameGamma) {
      m_ui.gamma_edit->setText(value);
    } else if (parameter.get_name() == m_nameReadNoise) {
      m_ui.read_noise_edit->setText(value);
    } else if (parameter.get_name() == m_nameShotNoise) {
      m_ui.shot_noise_edit->setText(value);
    }
  }
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

bool CameraConfig::getDoubleParameter(const QString& paramName, double *value)
{
  auto future = m_currentCamera->get_parameters({paramName.toStdString()});
  auto result = rclcpp::executors::spin_node_until_future_complete(m_executor,
                                                                   m_paramClientNode,
                                                                   future,
                                                                   m_serviceTimeout);

  if (result != rclcpp::FutureReturnCode::SUCCESS) {
    return false;
  }

  auto parameters = future.get();
  *value = parameters[0].as_double();
  return true;
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

  auto result = rclcpp::executors::spin_node_until_future_complete(m_executor,
                                                                   m_paramClientNode,
                                                                   future,
                                                                   m_serviceTimeout);

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
      m_ui.get_all_parameters_button->setEnabled(false);
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
    m_ui.get_all_parameters_button->setEnabled(true);
    m_ui.shot_noise_edit->setEnabled(true);
  }
}

}

PLUGINLIB_EXPORT_CLASS(irg_rqt_tools::CameraConfig, rqt_gui_cpp::Plugin)
