#pragma once

#include "AP_Proximity_config.h"

#if AP_PROXIMITY_LIGHTWARE_MR72_ENABLED

#include "AP_Proximity_Backend_Serial.h"

#define PROXIMITY_MR72_TIMEOUT_MS            200   // requests timeout after 0.2 seconds
#define PROXIMITY_MR72_PAYLOAD_LEN_MAX       256   // maximum payload size we can accept (in some configurations sensor may send as large as 1023)
#define PROXIMITY_MR72_LEN        20   // combine this many readings together to improve efficiency

class AP_Proximity_MR72_Serial : public AP_Proximity_Backend_Serial
{

public:
    // constructor
    using AP_Proximity_Backend_Serial::AP_Proximity_Backend_Serial;

    uint16_t rxspace() const override {
        return 1280;
    };

    // update state
    void update(void) override;

    // get maximum and minimum distances (in meters) of sensor
    float distance_max() const override { return 100.0f; }
    float distance_min() const override { return 0.20f; }

private:

    // initialise sensor
    void initialise();

    // restart sensor and re-init our state
    void restart_sensor();

    // check and process replies from sensor
    void process_replies();

    // process one byte received on serial port
    // state is stored in msg structure.  when a full package is received process_message is called
    void parse_byte(uint8_t b);

    // process the latest message held in the msg structure
    void process_message();

    void MR72_ReceiveDataAnl(uint8_t *data_buffer, uint8_t datalen);

    // internal variables
    uint32_t _last_request_ms;              // system time of last request
    uint32_t _last_reply_ms;                // system time of last valid reply
    uint32_t _last_restart_ms;              // system time we restarted the sensor
    uint32_t _last_distance_received_ms;    // system time of last distance measurement received from sensor
    AP_Proximity_Boundary_3D::Face _face;   // face of _face_distance
    float _face_distance;                   // shortest distance (in meters) on face
    float _face_yaw_deg;                    // yaw angle (in degrees) of shortest distance on face
    bool _face_distance_valid;              // true if face has at least one valid distance
    uint16_t sector1;
    uint16_t sector2;
    uint16_t sector3;

    enum class ParseState {
        HEADER_L = 0,
        HEADER_H,
        CRC
    };

    // structure holding latest message contents
    struct {
        ParseState state;       // state of incoming message processing
        uint16_t payload_len;   // latest message payload length (1+ bytes in payload)
        uint8_t payload[PROXIMITY_MR72_PAYLOAD_LEN_MAX];   // payload
        uint16_t payload_recv;  // number of message's payload bytes received so far
        uint8_t crc;        // crc low byte
        uint8_t crc_expected;  // latest message's expected crc
    } _msg;
    // convert buffer to uint32, uint16
    uint32_t buff_to_uint32(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3) const;
    uint16_t buff_to_uint16(uint8_t b0, uint8_t b1) const;


};

#endif // AP_PROXIMITY_LIGHTWARE_MR72_ENABLED
