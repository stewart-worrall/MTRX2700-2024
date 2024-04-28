#include "serialise.h"

// Function to pack data into a buffer for transmission
uint16_t pack_buffer(uint8_t *buffer, MessageType message_type, Data *data) {
    Header header = {SENTINEL_1, SENTINEL_2, message_type, 0};
    uint16_t buffer_idx = 0;
    uint16_t data_length = 0;

    switch (message_type) {
        case SENSOR_DATA:
            data_length = sizeof(SensorData);
            break;
        case LED_STATE:
            data_length = sizeof(LEDState);
            break;
        case BUTTON_AND_STATUS:
            data_length = sizeof(ButtonAndStatus);
            break;
        case STRING_PACKET:
            data_length = data->string_packet.length;
            break;
    }

    header.data_length = data_length;

    // Copy header to buffer
    memcpy(buffer, &header, sizeof(Header));
    buffer_idx += sizeof(Header);

    // Copy data to buffer
    memcpy(buffer + buffer_idx, data, data_length);
    buffer_idx += data_length;

    return buffer_idx;
}


// Function to unpack the buffer and check for sentinel bytes
bool unpack_buffer(const uint8_t *buffer, Data *output_data, MessageType *output_message_type, uint16_t *output_data_length) {
    Header header;
    uint16_t buffer_idx = sizeof(Header);

    // Copy header from buffer
    memcpy(&header, buffer, sizeof(Header));

    // Check sentinel bytes
    if (header.sentinel1 != SENTINEL_1 || header.sentinel2 != SENTINEL_2) {
        return false;
    }

    *output_message_type = header.message_type;
    *output_data_length = header.data_length;

    // Copy data from buffer
    switch (*output_message_type) {
        case SENSOR_DATA:
            memcpy(&output_data->sensor_data, buffer + buffer_idx, sizeof(SensorData));
            break;
        case LED_STATE:
            memcpy(&output_data->led_state, buffer + buffer_idx, sizeof(LEDState));
            break;
        case BUTTON_AND_STATUS:
            memcpy(&output_data->button_and_status, buffer + buffer_idx, sizeof(ButtonAndStatus));
            break;
        case STRING_PACKET:
            output_data->string_packet.length = *output_data_length;
            output_data->string_packet.data = (char *)(buffer + buffer_idx);
            break;
    }

    return true;
}
