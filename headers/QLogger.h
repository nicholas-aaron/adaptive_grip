#ifndef QLOGGER__H
#define QLOGGER__H

#include "Logger.h"

   #include <QString> // TODO figure out include_dirs for cmake
   #include <QTextEdit> // TODO figure out include_dirs for cmake

class QLogger : public Logger {

   public:

   QTextEdit * textEdit;

   QLogger(QTextEdit * _textEdit);

   // Overloaded method
   void log(std::string str);
   void log(std::stringstream &);
};

#endif
