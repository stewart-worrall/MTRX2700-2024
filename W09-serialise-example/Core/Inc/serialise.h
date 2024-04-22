#ifndef SERIALISE_HEADER
#define SERIALISE_HEADER

#include <stdint.h>
#include <string.h>
#include <stdbool.h>

// Constants
#define SENTINEL_1 0xAA
#define SENTINEL_2 0x55

// Enum for message types
typedef enum {
    SENSOR_DATA = 0,
    LED_STATE = 1,
    BUTTON_AND_STATUS = 2,
    STRING_PACKET = 3,
} MessageType;

// Sensor data struct
typedef struct {
    int32_t acc_x, acc_y, acc_z;
    int32_t gyro_x, gyro_y, gyro_z;
    uint32_t lidar_i2c, lidar_pwm;
} SensorData;

// LED state struct
typedef union {
    uint8_t led_byte;
    struct {
        uint8_t led0 : 1;
        uint8_t led1 : 1;
        uint8_t led2 : 1;
        uint8_t led3 : 1;
        uint8_t led4 : 1;
        uint8_t led5 : 1;
        uint8_t led6 : 1;
        uint8_t led7 : 1;
    } led_bits;
} LEDState;

// Button and microcontroller status struct
typedef struct {
    uint8_t button_state : 1;
    uint8_t mcu_status : 7;
} ButtonAndStatus;

// Variable length string packet struct
typedef struct {
    uint8_t length;
    char *data;
} StringPacket;

// Union of data types
typedef union {
    SensorData sensor_data;
    LEDState led_state;
    ButtonAndStatus button_and_status;
    StringPacket string_packet;
} Data;

// Header structure
typedef struct {
    uint8_t sentinel1;
    uint8_t sentinel2;
    uint16_t message_type;
    uint16_t data_length;
} Header;

// Function to pack data into a buffer for transmission
uint16_t pack_buffer(uint8_t *buffer, MessageType message_type, Data *data);
// Function to unpack the buffer and check for sentinel bytes
bool unpack_buffer(const uint8_t *buffer, Data *output_data, MessageType *output_message_type, uint16_t *output_data_length);


#endif
