#include <stdint.h>
#include "src/bno055/bno055.h"
#include "src/bmp280/bmp280.h"

typedef struct
{
    bno055_vector_t accel,
        gyro,
        qnion;
    float pressure,
        temperature,
        latitude,
        longitude,
        altitude,
        bmp_alt,
        speed,
        angle;
    uint8_t durum;
} packet_t;
typedef struct
{
    uint8_t header;
    uint16_t lat,
        lon,
        alt,
        p_alt;
    int8_t acc_x,
        acc_y,
        acc_z,
        gyro_x,
        gyro_y,
        gyro_z;
    uint8_t crc;
} rocket_t;
typedef union
{
    rocket_t rocket;
    uint8_t arr[sizeof(rocket_t)];
} conv_t;