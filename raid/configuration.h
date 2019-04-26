#ifndef configuration_h
#define configuration_h

#define DEBUG false

//Mode Definitions
#define DEBUG_MODE 0
#define STANDBY_MODE 1
#define BT_MODE 2
#define LEARN_MODE 3
#define MISSION_MODE 4
#define ESTOP_MODE 5

#define ARUCO_ID 1

#define GPSLOG_FILENAME "TRACK.txt"

#define GPS_WAYPOINT_MIN_ERROR_M 2

//User Params
#define WALK_SPEED 1700
#define HEADING_OFFSET -20

//Robot Intrinsics
#define WHEEL_BASE 0
#define WHEEL_TRACK 544
#define WHEEL_DIAMETER 230

#define CPR 1200
#define RPM 500

#define WHEEL_CIRCUMFERENCE (WHEEL_DIAMETER * PI)

#define MAX_VELOCITY WHEEL_CIRCUMFERENCE * RPM / 60.0

//Sensor Intrinsics
//axis conventions: +z up, +y forward, +x right
//angle conventions: +h right, +p up, +r right
//angles in radians
//displacements in mm
#define FRONT_TOF_1_R 0
#define FRONT_TOF_1_P (11.75 * PI) / 180.0
#define FRONT_TOF_1_H (-40.5 * PI) / 180.0
#define FRONT_TOF_1_X 0
#define FRONT_TOF_1_Y 0
#define FRONT_TOF_1_Z 0

#define FRONT_TOF_2_R 0
#define FRONT_TOF_2_P (11.75 * PI) / 180.0
#define FRONT_TOF_2_H (-13.5 * PI) / 180.0
#define FRONT_TOF_2_X 0
#define FRONT_TOF_2_Y 0
#define FRONT_TOF_2_Z 0

#define FRONT_TOF_3_R 0
#define FRONT_TOF_3_P (11.75 * PI) / 180.0
#define FRONT_TOF_3_H (13.5 * PI) / 180.0
#define FRONT_TOF_3_X 0
#define FRONT_TOF_3_Y 0
#define FRONT_TOF_3_Z 0

#define FRONT_TOF_4_R 0
#define FRONT_TOF_4_P (11.75 * PI) / 180.0
#define FRONT_TOF_4_H (40.5 * PI) / 180.0
#define FRONT_TOF_4_X 0
#define FRONT_TOF_4_Y 0
#define FRONT_TOF_4_Z 0

#define BACK_TOF_R 0
#define BACK_TOF_P (11.75 * PI) / 180.0
#define BACK_TOF_H 0
#define BACK_TOF_X 0
#define BACK_TOF_Y 0
#define BACK_TOF_Z 0

#define DEAD_ZONE_MM 300

#define TOF_MAX_M 1.5

//Pin Mapping
#define LEFT_PWM_PIN 3 //22
#define LEFT_DIR_PIN 2 //26
#define LEFT_RESET_PIN 0
#define LEFT_FF1_PIN 0
#define LEFT_FF2_PIN 0

#define TOP_LEFT_ENC_A_PIN 20 //16
#define TOP_LEFT_ENC_B_PIN 17

#define BOTTOM_LEFT_ENC_A_PIN 38 // 24
#define BOTTOM_LEFT_ENC_B_PIN 39 // 25

#define RIGHT_PWM_PIN 30 // 23
#define RIGHT_DIR_PIN 29 // 27
#define RIGHT_RESET_PIN 0
#define RIGHT_FF1_PIN 0
#define RIGHT_FF2_PIN 0

#define TOP_RIGHT_ENC_A_PIN 5 // 14
#define TOP_RIGHT_ENC_B_PIN 4 // 15

#define BOTTOM_RIGHT_ENC_A_PIN 27 // 34
#define BOTTOM_RIGHT_ENC_B_PIN 28 // 35

#define FRONT_LED_BLUE_PIN 23
#define FRONT_LED_RED_PIN 22
#define FRONT_LED_GREEN_PIN 21

#define BACK_LED_BLUE_PIN 35 
#define BACK_LED_RED_PIN 36
#define BACK_LED_GREEN_PIN 37

#define LEFT_BUTTON_PIN 11
#define SELECT_BUTTON_PIN 12
#define DOWN_BUTTON_PIN 24
#define RIGHT_BUTTON_PIN 26
#define UP_BUTTON_PIN 25

#define POWER_STATUS_LED_PIN 15
#define STATUS_STATUS_LED_PIN 14
#define BT_STATUS_LED_PIN 13

#define COMPASS_PIN 16

//#define POWER_STATUS_LED_PIN 11
//#define POWER_STATUS_LED_PIN 11
//#define POWER_STATUS_LED_PIN 11

//Update Rates
#define ENCODER_OPTIMIZE_INTERRUPTS
#define VELOCITY_UPDATE_TIME_uS 50000
#define TOF_POLL_PERIOD_mS 33
#define BT_SEND_INTERVAL_MS 1000

//PID Gains
#define VELOCITY_KP 0.05
#define VELOCITY_KI 0.05
#define VELOCITY_KI_MAX 1.0

//I2C Params
#define MUX_ADDR 0x70
#define TOF_ADDR 0x29

#define FRONT_TOF_1_PORT 2
#define FRONT_TOF_2_PORT 3
#define FRONT_TOF_3_PORT 4
#define FRONT_TOF_4_PORT 7
#define BACK_TOF_PORT 1

#define LCD_PORT 6

//Serial Params
//Edit listener ports if ports changed
#define BT_SERIAL_PORT Serial3
#define GPS_SERIAL_PORT Serial1
#define CAM_FRONT_SERIAL_PORT Serial5
#define CAM_RIGHT_SERIAL_PORT Serial2
#define CAM_BACK_SERIAL_PORT Serial6
#define CAM_LEFT_SERIAL_PORT Serial4
#define RPI_SERIAL Serial

//Color Names
#define WHITE {1.0, 1.0, 1.0}
#define RED {1.0, 0.0, 0.0}
#define GREEN {0.0, 1.0, 0.0}
#define BLUE {0.0, 0.0, 1.0}
#define ORANGE {1.0, 0.25, 0.0}

#endif