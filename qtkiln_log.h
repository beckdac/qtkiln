#ifndef _QTKILN_LOG_H_
#define _QTKILN_LOG_H_

#include "Arduino.h"

class QTKilnLog
{
  public:
    QTKilnLog();

    void begin(void);
    void print(const char *fmt, ...);
    void debug(uint16_t priority, const char *fmt, ...);
    void warn(const char *fmt, ...);
    void error(const char *fmt, ...);

  private:
    unsigned long _lastMsgTime = 0;	// last time a message was sent
};

#endif
