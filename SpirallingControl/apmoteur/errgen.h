#ifndef _ERRGEN_H
#define _ERRGEN_H
#include "data.h"

extern UNS16 errgen_state;
extern UNS8 errgen_laserState;

void errgen_set(UNS16 dat);
static void errgen_catch_err(UNS16 dat);
static char* errgen_get_title(UNS16 dat);
static char* errgen_get_content(UNS16 dat);

#include "errgen_laser.h"
#define LASER_ERROR(err)                     (0x80 | err)
#define ERR_LASER_REINIT                    0x0100
#define ERR_LASER_SERIAL_CONFIG             0x0101

#define ERR_SLAVE_CONFIG_ROT_REFPOS         0x0102
#define ERR_MASTER_CONFIG_PDOT2             0x0103
#define ERR_LASER_ASSERV_START              0x0104
#define ERR_LASER_ASSERV_STOP               0x0105
#define ERR_LASER_SIMU_START                0x0106
#define ERR_LASER_SIMU_STOP                 0x0107
#define ERR_ROT_CALC_ACCEL                  0x0108
#define ERR_ROT_WRITE_ACCEL                 0x0109
#define ERR_ROT_GET_ACCEL                   0x0110
#define ERR_ROT_GET_DECEL                   0x0111

#define ERR_DRIVER_UP                        0x01
#define ERR_DRIVER_LOAD                      0x02
#define ERR_DRIVER_OPEN                      0x03
#define ERR_FILE_CONFIG_GEN                  0x04
#define ERR_INIT_LOOP_RUN                    0x05
#define ERR_GUI_LOOP_RUN                     0x06
#define ERR_MASTER_STOP_SYNC                 0x07
#define ERR_MASTER_START_SYNC                0x08
#define ERR_MASTER_SET_PERIOD                0x09
#define ERR_MASTER_SET_HB_CONS               0x0A
#define ERR_MASTER_CONFIG_PDOR1              0x0B
#define ERR_MASTER_CONFIG_PDOR2              0x0C
#define ERR_MASTER_CONFIG_PDOR3              0x0D
#define ERR_MASTER_CONFIG_PDOT1              0x0E
#define ERR_MASTER_CONFIG                    0x0F
#define ERR_LSS_CONFIG                       0x10
#define ERR_SLAVE_CONFIG                     0x11
#define ERR_SLAVE_CONFIG_HB                  0x12
#define ERR_SLAVE_CONFIG_PDOT1               0x13
#define ERR_SLAVE_CONFIG_PDOT2               0x14
#define ERR_SLAVE_CONFIG_PDOT3               0x15
#define ERR_SLAVE_CONFIG_PDOR1               0x16
#define ERR_SLAVE_CONFIG_ACTIVE_PDO          0x17
#define ERR_SLAVE_CONFIG_CURRENT             0x18
#define ERR_SLAVE_CONFIG_PROFILE             0x19
#define ERR_SLAVE_CONFIG_MAX_VELOCITY        0x1A
#define ERR_SLAVE_CONFIG_ACCELERATION        0x1B
#define ERR_SLAVE_CONFIG_DECELERATION        0x1C
#define ERR_SLAVE_CONFIG_DECELERATION_QS     0x1D
#define ERR_SLAVE_CONFIG_TORQUE              0x1E
#define ERR_SLAVE_CONFIG_TORQUE_SLOPE        0x1F
#define ERR_SLAVE_CONFIG_TORQUE_MAX_VELOCITY 0x20
#define ERR_SLAVE_CONFIG_MOTOR_SON           0x21
#define ERR_SLAVE_SAVE_CONFIG                0x22
#define ERR_MOTOR_PAUSE                      0x23
#define ERR_MOTOR_RUN                        0x24
#define ERR_MOTOR_OFF                        0x25
#define ERR_LOAD_ACCELERATION                0x26
#define ERR_LOAD_DECELERATION                0x27
#define ERR_LOAD_DECELERATION_QS             0x28
#define ERR_LOAD_SLAVE_CONFIG                0x29
#define ERR_MOTOR_FORWARD                    0x2A
#define ERR_MOTOR_BACKWARD                   0x2B
#define ERR_ACCELERATION_SET                 0x2C
#define ERR_VELOCITY_SET                     0x2D
#define ERR_ACCELERATION_SAVE                0x2E
#define ERR_DECELERATION_SAVE                0x2F
#define ERR_DECELERATION_QS_SAVE             0x30
#define ERR_SAVE_FILE_GEN                    0x31
#define ERR_TORQUE_SET                       0x32
#define ERR_TORQUE_SAVE                      0x33
#define ERR_TORQUE_SLOPE_SET                 0x34
#define ERR_TORQUE_SLOPE_SAVE                0x35
#define ERR_LOAD_TORQUE                      0x36
#define ERR_LOAD_TORQUE_SLOPE                0x37


#endif // _ERRGEN
