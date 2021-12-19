#ifndef ATR_WEARABLE
#define ATR_WEARABLE

// Sub-device IDs
#define IMU_SENSOR        1
#define HEART_RATE_SENSOR 2
#define VIBRATION_MOTOR   3


// XML Codes
#define XML_TAG_MESSAGE ('0')
#define XML_TAG_HEADER ('1')
#define XML_TAG_SENDER ('2')
#define XML_TAG_RECEIVER ('3')
#define XML_TAG_TIMESTAMP ('4')
#define XML_TAG_SUB_DEVICE_ID ('5')
#define XML_TAG_MESSAGE_TYPE ('6')
#define XML_TAG_COMMAND ('7')
#define XML_TAG_OPCODE ('8')

#define CONVERSIONG 3.9

// Opcodes
#define XML_TAG_CMD_IMU ("10")
#define XML_TAG_DATA_IMU ("11")




#define IMU_DEVICE_CMD (4)
#define IMU_DEVICE_CMD_CH ('4')



// PIN Configurations

// Vibration Motor
#define VIBRATION_MOTOR_PIN A3


//OLED
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define OLED_RESET    -1
#define SCREEN_ADDRESS 0x3C


#endif
