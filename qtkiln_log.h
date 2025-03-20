#ifndef _QTKILN_LOG_H_
#define _QTKILN_LOG_H_

#include "Arduino.h"

#define QTKILN_LOG_INITIAL_BUF_SIZE	255
#define QTKILN_LOG_BUF_SIZE_INC		127

#define QTKLOG_DBG_PRIO_ALWAYS	0
#define QTKLOG_DBG_PRIO_HIGH	1
#define QTKLOG_DBG_PRIO_MED	100
#define QTKLOG_DBG_PRIO_LOW	1000

class QTKilnLog
{
  public:
    QTKilnLog(bool enableSerial=true, bool enableMQTT=false, bool enableSyslog=false);

    void begin(void);
    void print(const char *fmt, ...);
    void debug(uint16_t priority, const char *fmt, ...);
    void warn(const char *fmt, ...);
    void error(const char *fmt, ...);

    void serialOutputEnable(bool enable);
    void mqttOutputEnable(bool enable);
    void syslogOutputEnable(bool enable);

    bool isSerialOutputEnabled();
    bool isMqttOutputEnabled();
    bool isSyslogOutputEnabled();

    void setDebugPriorityCutoff(uint16_t priority);
    uint16_t getDebugPriorityCutoff(void);

    uint16_t getReallocationCount(void);

  private:
    unsigned long _lastMsgTime = 0;	// last time a message was sent
    bool _serialOutputEnabled = false;
    bool _mqttOutputEnabled = false;
    bool _syslogOutputEnabled = false;
    uint16_t _debugPriorityCutoff = 0;
    uint16_t _reallocationCount = 0;
    char *_format(const char *fmt, va_list args);
};

#endif
