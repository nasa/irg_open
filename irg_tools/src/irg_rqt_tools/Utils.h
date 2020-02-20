#ifndef irg_rqt_tools_Utils_h
#define irg_rqt_tools_Utils_h

#include <QString>
#include <QStringList>

namespace irg_rqt_tools {

  class Utils {
  public:
    static QStringList getAllTopics(const QString& messageType);
    static QStringList getPublishedTopics(const QString& messageType);
  };

}

#endif // irg_rqt_tools_Utils_h
