#include "Arduino.h"
#include "qtkiln_log.h"


QTKilnLog::QTKilnLog(bool enableSerial, bool enableMqtt, bool enableSyslog) {
  _lastMsgTime = 0;
  serialOutputEnable(enableSerial);
  mqttOutputEnable(enableMqtt);
  syslogOutputEnable(enableSyslog);
  _debugPriorityCutoff = 0;
}

void QTKilnLog::serialOutputEnable(bool enable) {
  _serialOutputEnabled = enable;
}

void QTKilnLog::mqttOutputEnable(bool enable) {
  _mqttOutputEnabled = enable;
}

void QTKilnLog::syslogOutputEnable(bool enable) {
  _syslogOutputEnabled = enable;
}

bool QTKilnLog::isSerialOutputEnabled(void) {
  return _serialOutputEnabled;
}

bool QTKilnLog::isMqttOutputEnabled(void) {
  return _mqttOutputEnabled;
}

bool QTKilnLog::isSyslogOutputEnabled(void) {
  return _syslogOutputEnabled;
}

void QTKilnLog::setDebugPriorityCutoff(uint16_t priority) {
  _debugPriorityCutoff = priority;
}

uint16_t QTKilnLog::getDebugPriorityCutoff(void) {
  return _debugPriorityCutoff;
}

uint16_t QTKilnLog::getReallocationCount(void) {
  return _reallocationCount;
}

void QTKilnLog::begin(void) {
}

char *QTKilnLog::_format(const char *fmt, va_list args) {
  char *buf;
  size_t buflen, len;

  buflen = QTKILN_LOG_INITIAL_BUF_SIZE;
  buf = (char *)malloc(buflen + 1);

  len = vsnprintf(buf, buflen + 1, fmt, args);
  if (len > buflen) {
    _reallocationCount++;
    // free and reallocate larger and try again
    free(buf);
    buflen += QTKILN_LOG_BUF_SIZE_INC;
    buf = (char *)malloc(buflen + 1);
    len = vsnprintf(buf, buflen + 1, fmt, args);
  }
 
  return buf; 
}

void QTKilnLog::print(const char *fmt, ...) {
  va_list args;
  char *buf;

  if (!_serialOutputEnabled && !_mqttOutputEnabled && !_syslogOutputEnabled)
    return;

  _lastMsgTime = millis();

  va_start(args, fmt);
  buf = _format(fmt, args);
  va_end(args);

  if (_serialOutputEnabled)
    Serial.println(buf);

  free(buf);
}

void QTKilnLog::debug(uint16_t priority, const char *fmt, ...) {
  va_list args;
  char *buf, *tmp;
  unsigned long now;

  if (!_serialOutputEnabled && !_mqttOutputEnabled && !_syslogOutputEnabled)
    return;

  if (priority > _debugPriorityCutoff)
    return;

  va_start(args, fmt);
  tmp = _format(fmt, args);
  va_end(args);

  now = millis();
  _lastMsgTime = now;

  if (_serialOutputEnabled) {
    // allocate a larger buff to hold the DEBUG tag, time, colon
    // and spaces; 64 is a very large over estimate for this
    buf = (char *)malloc(strlen(tmp) + 64);
    snprintf(buf, strlen(tmp) + 64, "%s %lu : %s", "DEBUG", now, tmp);
    Serial.println(buf);
  }

  free(tmp);
  free(buf);
}

void QTKilnLog::warn(const char *fmt, ...) {
  va_list args;
  char *buf, *tmp;
  unsigned long now;

  if (!_serialOutputEnabled && !_mqttOutputEnabled && !_syslogOutputEnabled)
    return;

  va_start(args, fmt);
  tmp = _format(fmt, args);
  va_end(args);

  now = millis();
  _lastMsgTime = now;

  if (_serialOutputEnabled) {
    // allocate a larger buff to hold the WARN tag, time, colon
    // and spaces; 64 is a very large over estimate for this
    buf = (char *)malloc(strlen(tmp) + 64);
    snprintf(buf, strlen(tmp) + 64, "%s %lu : %s", "WARN", now, tmp);
    Serial.println(buf);
  }

  free(tmp);
  free(buf);
}

void QTKilnLog::error(const char *fmt, ...) {
  va_list args;
  char *buf, *tmp;
  unsigned long now;

  if (!_serialOutputEnabled && !_mqttOutputEnabled && !_syslogOutputEnabled)
    return;

  va_start(args, fmt);
  tmp = _format(fmt, args);
  va_end(args);

  now = millis();
  _lastMsgTime = now;

  if (_serialOutputEnabled) {
    // allocate a larger buff to hold the WARN tag, time, colon
    // and spaces; 64 is a very large over estimate for this
    buf = (char *)malloc(strlen(tmp) + 64);
    snprintf(buf, strlen(tmp) + 64, "%s %lu : %s", "WARN", now, tmp);
    Serial.println(buf);
  }

  free(tmp);
  free(buf);

  // give time for buffers to clear
  delay(500);
  esp_restart();
}
