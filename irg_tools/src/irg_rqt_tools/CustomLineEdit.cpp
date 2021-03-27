#include "CustomLineEdit.h"

CustomLineEdit::CustomLineEdit(QWidget *parent)
 : QLineEdit(parent)
{
}

void CustomLineEdit::focusInEvent(QFocusEvent *e)
{
  QLineEdit::focusInEvent(e);
  emit(focusChanged(true));
}
