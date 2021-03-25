#ifndef irg_rqt_tools_CustomLineEdit_h
#define irg_rqt_tools_CustomLineEdit_h

#include <QLineEdit>

class CustomLineEdit : public QLineEdit
{
  Q_OBJECT

public:
  CustomLineEdit(QWidget *parent = 0);
  ~CustomLineEdit() = default;

signals:
  void focussed(bool hasFocus);

protected:
  virtual void focusInEvent(QFocusEvent *e);
  virtual void focusOutEvent(QFocusEvent *e);
};

#endif // irg_rqt_tools_CustomLineEdit_h
