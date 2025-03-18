#include "Arduino.h"
#include "qtkiln_log.h"


QTKilnLog::QTKilnLog(void) {
  _lastMsgTime = 0;
}

void QTKilnLog::begin(void) {
}

void QTKilnLog::print(const char *fmt, ...) {
}

void QTKilnLog::debug(uint16_t priority, const char *fmt, ...) {
}

void QTKilnLog::warn(const char *fmt, ...) {
}

void QTKilnLog::error(const char *fmt, ...) {
}
