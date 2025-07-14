#include "core_pins.h"
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <cmath>
#include <cstdlib>
#include <utility/imumaths.h>

const unsigned short crc16_tab[] = { 0x0000, 0x1021, 0x2042, 0x3063, 0x4084,
    0x50a5, 0x60c6, 0x70e7, 0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad,
    0xe1ce, 0xf1ef, 0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7,
    0x62d6, 0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485, 0xa56a,
    0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d, 0x3653, 0x2672,
    0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4, 0xb75b, 0xa77a, 0x9719,
    0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc, 0x48c4, 0x58e5, 0x6886, 0x78a7,
    0x0840, 0x1861, 0x2802, 0x3823, 0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948,
    0x9969, 0xa90a, 0xb92b, 0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50,
    0x3a33, 0x2a12, 0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b,
    0xab1a, 0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
    0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49, 0x7e97,
    0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70, 0xff9f, 0xefbe,
    0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78, 0x9188, 0x81a9, 0xb1ca,
    0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f, 0x1080, 0x00a1, 0x30c2, 0x20e3,
    0x5004, 0x4025, 0x7046, 0x6067, 0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d,
    0xd31c, 0xe37f, 0xf35e, 0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214,
    0x6277, 0x7256, 0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c,
    0xc50d, 0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c, 0x26d3,
    0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634, 0xd94c, 0xc96d,
    0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab, 0x5844, 0x4865, 0x7806,
    0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3, 0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e,
    0x8bf9, 0x9bd8, 0xabbb, 0xbb9a, 0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1,
    0x1ad0, 0x2ab3, 0x3a92, 0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b,
    0x9de8, 0x8dc9, 0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0,
    0x0cc1, 0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
    0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0 };

typedef enum {
    COMM_FW_VERSION = 65,
    COMM_JUMP_TO_BOOTLOADER=66,
    COMM_ERASE_NEW_APP=67,
    COMM_WRITE_NEW_APP_DATA=68,
    COMM_GET_VALUES=69, //Get motor operating parameters
    COMM_SET_DUTY=70, //The motor operates in duty cycle mode
    COMM_SET_CURRENT=71, //The motor operates in current loop mode
    COMM_SET_CURRENT_BRAKE=72, //The motor operates in current bake mode
    COMM_SET_RPM=73, //The motor operates in velocity loop mode
    COMM_SET_POS=74, //The motor operates in position loop mode
    COMM_SET_HANDBRAKE=75, //The motor operates om handbrake current loop mode
    COMM_SET_DETECT=76, //The motor provides real-time feedback on the current
    COMM_ROTOR_POSITION=87,//The motor feedbacks the current position
    COMM_GET_VALUES_SETUP=16,//The motor requires instructions based on one or more
    COMM_SET_POS_SPD=60, // The motor operates in position-velocity loop mode
    COMM_SET_POS_MULTI=61, //Set the motor movement to single circle motion mode
    COMM_SET_POS_SINGLE=62, // Set the motor movement to multiple circles motion mode,
    COMM_SET_POS_UNLIMITED=63, //Save
    COMM_SET_POS_ORIGIN=64, //Set the motor’s origin
} COMM_PACKET_ID;

int32_t DEADZONE = 6;

uint16_t SAMPLERATE_DELAY_MS = 100;
long MOTOR_SERIAL_BAUD_RATE = 19200;

int32_t DEGREES_TO_ENCODER_TICKS = 1000;
int32_t ERPM_TO_SPEED = 10;

int32_t MAX_SPEED_ERPM = 13000;
int32_t MAX_ACCELERATION_ERPM_PER_SECOND_SQRD = 14000;

float DEGREES_OFFSET = 0; // negative is counter clockwise

float FAULT_DEGREES = 60;

// #define DEBUG

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

bool enabled_motor = false;

void setup(void)
{
    #ifdef DEBUG
    Serial.begin(115200);
    #endif
    Serial1.begin(MOTOR_SERIAL_BAUD_RATE, SERIAL_8N1);

    while (!Serial) delay(10);

    if (!bno.begin())
    {
        Serial.print("no BNO055 detected");
        while (1);
    }

    delay(5000);

    enabled_motor = true;

}

uint16_t calculate_checksum(unsigned char *buf, unsigned int len) {
    unsigned int i;
    uint16_t cksum = 0;
    for (i = 0; i < len; i++) {
        cksum = crc16_tab[(((cksum >> 8) ^ *buf++) & 0xFF)] ^ (cksum << 8);
    }
    return cksum;
}

void get_temperature() {
    uint8_t to_write[] = {0x02, 0x05, 0x32, 0x00, 0x00, 0x00, 0x01, 0x58, 0x4C, 0x03 };
    Serial1.write(to_write, sizeof(to_write));
}

void set_position(float commanded_position_degrees) {

    int32_t commanded_position = (commanded_position_degrees + DEGREES_OFFSET) * DEGREES_TO_ENCODER_TICKS;
    int32_t commanded_speed = MAX_SPEED_ERPM;
    int32_t commanded_acceleration = MAX_ACCELERATION_ERPM_PER_SECOND_SQRD;

    uint8_t to_write[18] = {0};

    to_write[0] = 0xAA; // Frame identifier
    to_write[1] = 1 + 4 + 4 + 4; // Data length
    to_write[2] = COMM_SET_POS_SPD; // Data identifier
    
    to_write[3] = ((commanded_position >> 24)); // bits 25–32
    to_write[4] = ((commanded_position >> 16)); // bits 17–24
    to_write[5] = ((commanded_position >> 8)); // bits 9–16
    to_write[6] = (commanded_position); // bits 1-8
    
    to_write[7] = (commanded_speed >> 24) & 0xFF;
    to_write[8] = (commanded_speed >> 16) & 0xFF;
    to_write[9] = (commanded_speed >> 8) & 0xFF;
    to_write[10] = commanded_speed & 0xFF;

    to_write[11] = (commanded_acceleration >> 24) & 0xFF;
    to_write[12] = (commanded_acceleration >> 16) & 0xFF;
    to_write[13] = (commanded_acceleration >> 8) & 0xFF;
    to_write[14] = commanded_acceleration & 0xFF;
    
    uint16_t crc = calculate_checksum(to_write + 2, 1+4+4+4);  // CRC over command + data
    
    to_write[15] = (crc >> 8) & 0xFF;
    to_write[16] = crc & 0xFF;

    to_write[17] = 0xBB;

    Serial1.write(to_write, sizeof(to_write));

    Serial.print("Sending: ");
    for (int i = 0; i<14; i++) {
        Serial.printf("%02x ", to_write[i]);
        // Serial1.write(to_write[i]); // Send command to motor via serial
    }
    Serial.print("\n");

}


void loop(void)
{
    sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
    bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

    Serial.println();
    Serial.print("Gyro x=");
    Serial.print(orientationData.orientation.x);
    Serial.print(" y=");
    Serial.print(orientationData.orientation.y);
    Serial.print(" z=");
    Serial.print(orientationData.orientation.z);

    Serial.print("Gradient");
    Serial.print("direction of gradient=");
    float desired_orientation = atan2(orientationData.orientation.y, orientationData.orientation.z) * (180.0 / 3.14159);
    Serial.print(desired_orientation);

    if (abs(orientationData.orientation.y) > FAULT_DEGREES || abs(orientationData.orientation.z) > FAULT_DEGREES) {
        Serial.print("faulting. max angle !!!!!!!!!!!!!!!!!!!!!");
        enabled_motor = false;
    }

    // Writing orientation code
    
    if (Serial.read() == ' ') {
        enabled_motor = !enabled_motor;
    }

    if (enabled_motor == true) {
        Serial.print("motor is active!");
        if (abs(orientationData.orientation.y) > DEADZONE || abs(orientationData.orientation.z) > DEADZONE) {
            Serial.print("commanding position");
            set_position(desired_orientation);
        }
        get_temperature();
    }

    // while (Serial1.available()) {
    //     uint8_t b = Serial1.read();
    //     if (b == 0x03) {
    //         Serial.printf("\n");
    //     }
    //     Serial.printf("%02X ", b);
    // }




    delay(SAMPLERATE_DELAY_MS);
}

