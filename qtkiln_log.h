#ifndef _QTKILN_LOG_H_
#define _QTKILN_LOG_H_

#include "Arduino.h"

class QTKilnLog
{
  public:
    QTKilnLog();

    void begin(void);
    void print();
    void debug();
    void warn();
    void error();

  private:
    unsigned long _lastMsgTime = 0;	// last time a message was sent
};

#endif
