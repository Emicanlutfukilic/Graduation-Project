#include <SD.h>
#include "Arduino.h"
void CreateLogFile(File LogFile);
void WriteLog(String log, File LogFile, String TimeStamp="");
extern String fileName;