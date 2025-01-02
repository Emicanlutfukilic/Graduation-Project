
#include <Servo.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <SoftwareSerial.h>

#include "src/packets.h"             // paketlerin structları
#include "src/logger/logger.h"       // log dosyası işlemleri
#include "src/bno055/bno055_stm32.h" // bno055 sensörü
#include "src/bmp280/bmp280.c"       // bmp280 sensörü
//----------------------

// definitions
#define BUFFER_SIZE 512
#define ALANLAT 38.40f
#define ALANLON 33.70f
#define NS 'N'
#define EW 'E'
// fonksiyon definitions
void decodeGPS(char *sent);

void sendLoraData(packet_t *data);
// KURTARMA SERVO PINLERİ
#define UstKPin PB5
#define AltKPin PB4

// SPİ pinleri
#define MOSIPin PB15
#define MISOPin PB14
#define SCKPin PB10
#define SDCs PA4

// i2c pinleri
#define SCLPin PB6
#define SDAPin PB7

// kurtarma sistemi servolarının tanımlanması
Servo UstKurtarma, AltKurtarma;

// SD karttaki logun dosyası
File LogFile;
bool sdFail = false;

// mesaj paketleri
packet_t packet;

// Uart haberleşme bağlantıları ve interruptlar
HardwareSerial Serial1(PA10, PA9);

// pinler aynı zamanda usb serialini(Usart2) kullandığı için
// donanımdaki uart yerine yazılım uartını kullanıyoruz
SoftwareSerial Serial3(PA3, PA2);

// sabitler
String Buffer = "";
const double coeff = 288.15 / 0.0065;
const double copre = pow(101325, (-1 / 5.256));
const double rad2deg = 180 / M_PI;
const double copow = 1 / 5.256;
BMP280_HandleTypedef bmp280 = {
    .addr = BMP280_I2C_ADDRESS_0};
int pre_ms = 0, lift_ms = 0;
float pre_alt = 0;

// GPS fonksiyonu
uint8_t ent; // uarttan gelen veri char
String gpsMessage;
// gps mesajlarını deşifre eden fonksiyon
// arduino Serialı uartın RX interruptunu kullanarak
// mesajları tutuyor.
// o yüzden direkt serialda veri var mı diye okuma yapabiliyoruz
void readGpsBuffer()
{
  if (Serial1.available() > 0)
  {
    ent = Serial1.read();
  }
  else
  {
    return;
  }
  if (ent != '\n')
  {
    gpsMessage += ent;
  }
  else
  {
    decodeGPS((char *)gpsMessage.c_str());
    gpsMessage = "";
  }
}

void setup()
{

  packet.durum = 1;
  // usb seriali(usart2 aynı zamanda Serial2)
  
  Serial2.begin(115200);
  // gps bağlantısı
  Serial1.begin(9600);
  // lora bağlantısı
  Serial3.begin(9600);

  HAL_UART_Receive_IT(Serial1.getHandle(), &ent, 1);
  Serial2.println("code Start");

  // servoların pinleri
  UstKurtarma.attach(UstKPin);
  AltKurtarma.attach(AltKPin);

  // SPI ayarlanması
  SPI.setMOSI(MOSIPin);
  SPI.setMISO(MISOPin);
  SPI.setSCLK(SCKPin);
  pinMode(SDCs, OUTPUT);
  digitalWrite(SDCs, HIGH);
  SPI.begin();
  // Sd kartın başlatılması
  int a = 0;
  while (!SD.begin(SDCs))
  {
    delay(10);
    Serial2.println("sd fail");
    a += 1;
    if (a >= 3)
    {
      sdFail = true;
      break;
    }
  }

  // i2c hattını başlat
  Wire.setSDA(SDAPin);
  Wire.setSCL(SCLPin);
  Wire.begin();
  // log dosyasını oluştur
  if (!sdFail)
    CreateLogFile(LogFile);

  // sensörleri başlat
  bno055_assignI2C(Wire.getHandle());
  if (bno055_setup())
  {
    Serial2.println("BNO055 initialisation failed");
    delay(200);
  }
  bno055_setOperationModeNDOF();

  bmp280.i2c = Wire.getHandle();
  bmp280_init_default_params(&bmp280.params);
  if (!bmp280_init(&bmp280, &bmp280.params))
  {
    Serial2.println("BMP280 initialisation failed");
    delay(200);
  }
}

void loop()
{
  // Serial2.println("main loop");
  readGpsBuffer(); // gps mesajlarını oku
  // delay(5000);
  // i2cScan();
  packet.accel = bno055_getVectorAccelerometer();
  packet.gyro = bno055_getVectorGyroscope();
  packet.qnion = bno055_getVectorQuaternion();

  packet.speed = 1000 * (packet.bmp_alt - pre_alt) / (millis() - pre_ms);
  pre_ms = millis();
  pre_alt = packet.bmp_alt;

  // packet.angle = rad2deg * quaternionAngle(packet.qnion, ground_normal);
  packet.angle = rad2deg * atan2(sqrt(packet.accel.x * packet.accel.x + packet.accel.y * packet.accel.y), packet.accel.z);
  packet.bmp_alt = coeff * (1 - pow(packet.pressure, copow) * copre);

  if (packet.accel.z < -20 && packet.durum == 0) // motorun çalıştırılması
  {
    packet.durum = 1;
    lift_ms = millis();
  }
  // kalkış üstünden 10 saniye sonra, ivme sensörü 9 m/s^2 üzerinde ve basınç 1000 m üzerinde
  if (lift_ms + 10000 > millis() && packet.accel.z > 9 && packet.bmp_alt > 1000 && packet.durum == 1)
  {
    packet.durum = 2;
  }

  //  toplam ivme 1 m/s^2 altında ve 2500 m üzerinde serbest düşüşe yakın ilk paraşütün patatılması
  if (sqrt(packet.accel.z * packet.accel.z + packet.accel.y * packet.accel.y + packet.accel.x * packet.accel.x) < 1 && packet.bmp_alt > 2500 && packet.durum == 2)
  {
    packet.durum = 3;
    UstKurtarma.write(180); // continuous servo
  }
  //  yükseklik 600 m altında 2. paraşütün patatılması
  if (packet.bmp_alt < 600 && packet.durum == 3)
  {
    packet.durum = 4;
    AltKurtarma.write(180); // continuous servo
  }

  // test lora denemesi
  int size = sprintf((char *)Buffer.c_str(), "$AR,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d\n",
                     packet.accel.x, packet.accel.y, packet.accel.z,
                     packet.gyro.x, packet.gyro.y, packet.gyro.z,
                     packet.qnion.x, packet.qnion.y, packet.qnion.z,
                     packet.latitude, packet.longitude, packet.altitude,
                     packet.angle, packet.bmp_alt, packet.speed,
                     packet.temperature, packet.pressure, packet.durum);
  Serial3.print(Buffer); // loraya gönderim

  // sendLoraData(&packet); //asıl lora gönderimi

  if (!sdFail)
  {
    WriteLog(Buffer, LogFile,String(millis())); // sd karta yazım
  }
}

// i2c hattındaki cihazları taramak için
void i2cScan()
{
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++)
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.

    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial2.print("I2C device found at address 0x");
      if (address < 16)
        Serial2.print("0");
      Serial2.println(address, HEX);

      nDevices++;
    }
    else if (error == 4)
    {
      Serial2.print("Unknown error at address 0x");
      if (address < 16)
        Serial2.print("0");
      Serial2.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial2.println("No I2C devices found");
  else
    Serial2.println("done");
}

// NMea0183 mesaj protokolünden
// enlem ve boylamı ondalık dereceye çevirir
float nmea_to_dec(float deg_coord, char nsew)
{
  int degree = (int)(deg_coord / 100);
  float minutes = deg_coord - degree * 100;
  float dec_deg = minutes / 60;
  float decimal = degree + dec_deg;
  if (nsew == 'S' || nsew == 'W')
  { // return negative
    decimal *= -1;
  }
  return decimal;
}

void decodeGPS(char *sent)
{
  char *ptr;
  int i = 0;
  float lat, lon;
  ptr = strtok(sent, ",");

  do
  {
    switch (i)
    {
    case 0:
      if (strcmp(ptr, "$GNGGA"))
      {
        return;
      }

      break;
    case 2:
      lat = atof(ptr);

      break;
    case 4:
      lon = atof(ptr);

      break;
    case 6:
      if ('1' != ptr[0])
      {
        return;
      }
      packet.latitude = nmea_to_dec(lat, NS);
      packet.longitude = nmea_to_dec(lon, EW);
      break;
    case 11:
      packet.altitude = atof(ptr);

      break;
    default:
      break;
    }
    i++;
  } while ((ptr = strtok(NULL, ",")) != NULL);
}

void sendLoraData(packet_t *data)
{
  rocket_t rocket;
  rocket.header = 99;
  rocket.lat = (uint16_t)((data->latitude - ALANLAT) * 1000000);
  rocket.lon = (uint16_t)((data->longitude - ALANLON) * 1000000);
  rocket.alt = (uint16_t)data->altitude;
  rocket.p_alt = (uint16_t)data->bmp_alt;
  rocket.acc_x = (int8_t)(data->accel.x * 20);
  rocket.acc_y = (int8_t)(data->accel.y * 20);
  rocket.acc_z = (int8_t)(data->accel.y * 10);
  rocket.gyro_x = (int8_t)(data->gyro.x * 20);
  rocket.gyro_y = (int8_t)(data->gyro.y * 20);
  rocket.gyro_z = (int8_t)(data->gyro.z * 20);
  rocket.crc = 170;
  String mes = "";
  memcpy(&mes, &rocket, sizeof(rocket_t));
  for (int i = 0; i < sizeof(rocket_t); i++)
  {
    Serial3.write(mes[i]);
  }
}