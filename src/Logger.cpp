#include "Logger.h"

Logger::Logger()
{}

void 
Logger::log(std::string str)
{
   std::cout << str << std::endl;
}

void 
Logger::log(std::stringstream &ss){
   std::cout << ss.str() << std::endl;
}
