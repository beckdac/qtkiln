#ifndef _QTKILN_PROGRAM_H_
#define _QTKILN_PROGRAM_H_

#include "Arduino.h"

class QTKilnProgram
{
  public:
    QTKilnProgram::QTKilnProgram();
    QTKilnProgram::begin();
    QTKilnProgram::loop();
    QTKilnProgram::set(const char *name, const char *program);
    struct QTKilnProgramStruct *QTKilnProgram::get(const char *name);
    QTKilnProgram::run(struct QTKilnProgramStruct *program);
    QTKilnProgram::start();
    QTKilnProgram::stop();
    QTKilnProgram::pause();

  private:
    bool QTKilnProgram::_verifyProgram(const char *program);
};

#endif
