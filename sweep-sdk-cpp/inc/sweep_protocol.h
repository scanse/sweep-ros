#pragma once

#define SWEEP_RESPONSE_TYPE_DATA                'D'
#define SWEEP_RESPONSE_TYPE_INFO                'I'
#define SWEEP_RESPONSE_TYPE_MOTOR                'M'
#define SWEEP_RESPONSE_TYPE_LIDAR                'L'
#define SWEEP_RESPONSE_TYPE_RESET                'R'

#define SWEEP_RESPONSE_TYPE_DATA_START            'S'
#define SWEEP_RESPONSE_TYPE_DATA_STOP            'X'

#define SWEEP_RESPONSE_TYPE_INFO_MOTOR            'M'
#define SWEEP_RESPONSE_TYPE_INFO_VERSION        'V'
#define SWEEP_RESPONSE_TYPE_INFO_DEVICE            'D'

#define SWEEP_RESPONSE_TYPE_LIDAR_POWER            'P'
#define SWEEP_RESPONSE_TYPE_LIDAR_SETTINGS        'S'
#define SWEEP_RESPONSE_TYPE_LIDAR_INFO            'I'

#define SWEEP_RESPONSE_SCAN_NODE_SIZE 7

typedef struct _sweep_cmd_packet_t
{
    uint8_t cmdByte1;
    uint8_t cmdByte2;
    uint8_t cmdParamTerm;
}__attribute__((packed)) sweep_cmd_packet_t;

typedef struct _sweep_cmd_param_packet_t
{
    uint8_t cmdByte1;
    uint8_t cmdByte2;
    uint8_t cmdParamByte1;
    uint8_t cmdParamByte2;
    uint8_t cmdParamTerm;
}__attribute__((packed)) sweep_cmd_param_packet_t;

typedef struct _sweep_response_header_t
{
    uint8_t cmdByte1;
    uint8_t cmdByte2;
    uint8_t cmdStatusByte1;
    uint8_t cmdStatusByte2;
    uint8_t cmdSum;
    uint8_t term1;

}__attribute__((packed)) sweep_response_header_t;

typedef struct _sweep_response_param_t
{
    uint8_t cmdByte1;
    uint8_t cmdByte2;
    uint8_t cmdParamByte1;
    uint8_t cmdParamByte2;
    uint8_t term1;
    uint8_t cmdStatusByte1;
    uint8_t cmdStatusByte2;
    uint8_t cmdSum;
    uint8_t term2;

}__attribute__((packed)) sweep_response_param_t;

typedef struct _sweep_response_scan_packet_t
{
    uint8_t sync_error;
    uint16_t angle;
    uint16_t distance;
    uint8_t signal_strength;
    uint8_t checksum;
}__attribute__((packed)) sweep_response_scan_packet_t;

typedef struct _sweep_response_info_device_t
{
    uint8_t cmdByte1;
    uint8_t cmdByte2;
    uint8_t bit_rate[6];
    uint8_t laser_state;
    uint8_t mode;
    uint8_t diagnostic;
    uint8_t motor_speed[2];
    uint8_t sample_rate[4];
    uint8_t term;

}__attribute__((packed)) sweep_response_info_device_t;

typedef struct _sweep_response_info_version_t
{
    uint8_t cmdByte1;
    uint8_t cmdByte2;
    uint8_t model[5];
    uint8_t protocol_major;
    uint8_t protocol_min;
    uint8_t firmware_major;
    uint8_t firmware_minor;
    uint8_t hardware_version;
    uint8_t serial_no[8];
    uint8_t term;

}__attribute__((packed)) sweep_response_info_version_t;

typedef struct _sweep_response_info_motor_t
{
    uint8_t cmdByte1;
    uint8_t cmdByte2;
    uint8_t motor_speed[2];
    uint8_t term;

}__attribute__((packed)) sweep_response_info_motor_t;