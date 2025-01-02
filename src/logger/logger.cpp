
#include "logger.h"
String fileName = "";

void CreateLogFile(File LogFile)
{
  
  // SD karttaki en son log dosyasını bul
  File root = SD.open("/");
  if (root)
  {
    while (true)
    {
      File entry = root.openNextFile();
      if (!entry)
        break; // okunacak başka dosya yok
      if (entry.isDirectory())
        continue;                               // klasörlerle işimiz yok
      String tempstring = String(entry.name()); // dosya bulundu;
      if (fileName.endsWith(".txt"))
      {
        if (fileName.startsWith("Log_"))                                                     // txt dosyası ise
          fileName = tempstring.substring(tempstring.indexOf('_'), tempstring.length() - 4); // içinde bulunan sayıyı al
      }
    }
  }
  root.close();
  if (fileName == "")
  {
    fileName = "Log_0.txt";
  }
  else
  {
    fileName = "Log_" + String(fileName.toInt() + 1) + ".txt";
  }
  LogFile = SD.open(fileName, FILE_WRITE); // log dosyasını oluştur ve aç
  LogFile.println("Log başlangıcı");
  LogFile.close();
}
void WriteLog(String log, File LogFile, String TimeStamp)
{
  LogFile = SD.open(fileName, FILE_WRITE); // log dosyasını oluştur ve aç
  if(TimeStamp != "")
    LogFile.print(TimeStamp + " ");
  LogFile.print(log);
  LogFile.close();
}
