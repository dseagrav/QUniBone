/*
  rk067.hpp: RK06/RK07 disk drive

  Copyright Bogodyne Metatechnics LLC
  Contributed under the BSD 2-clause license.
*/

#ifndef _RK067_HPP_
#define _RK067_HPP_

#include "storagedrive.hpp"

/* States of drive operation */
#define RK067_STOPPED 0
#define RK067_MOTOR_STARTING 1
#define RK067_MOTOR_STOPPING 2
#define RK067_BRUSH_CYCLE 3
#define RK067_HEAD_LOAD_IN 4
#define RK067_RTZ_COMMAND 5
#define RK067_HEAD_LOAD_OUT 6
#define RK067_HEAD_UNLOAD 7
#define RK067_RUNNING 8

/* Two kinds of messages are sent to and from the drive, Message A and Message B. */

// This is the Message A sent to the drive as the command identifier
typedef union {
    uint16_t word;
    uint8_t byte[2];
    struct {
	uint16_t DRIVE_ADDR:3;
	uint16_t SELECT_RELEASE_CMD:1;
	uint16_t SEEK_CMD:1;
	uint16_t RECALIBRATE_CMD:1;
	uint16_t START_SPINDLE_CMD:1;
	uint16_t _unused_7:1;
	uint16_t DRIVE_CLEAR_CMD:1;
	uint16_t FORMAT:1; // Set for 20-sector mode, reset for 22-sector mode.
	uint16_t UNLOAD_CMD:1;
	uint16_t PACK_ACKNOWLEDGE_CMD:1;
	uint16_t TRACK_ADDRESS:3;
	uint16_t PARITY:1;
    } __attribute__((packed));
} RK067_MSG_AH;

typedef union {
    uint16_t word;
    uint8_t byte[2];
    struct {
	uint16_t DRIVE_ADDR:3;
	uint16_t _unused_3_4:2;
	uint16_t DRIVE_AVAIL:1;
	uint16_t VOLUME_VALID:1;
	uint16_t DRIVE_READY:1;
	uint16_t DRIVE_TYPE:1;
	uint16_t FORMAT:1; // Set for 20-sector mode, reset for 22-sector mode.
	uint16_t OFFSET_ON:1;
	uint16_t WRITE_LOCK:1;
	uint16_t SPINDLE_ON:1;
	uint16_t POSIT_IN_PROGR:1;
	uint16_t DRIVE_STATUS_CHANGE:1;
	uint16_t PARITY:1;
    } __attribute__((packed));
} RK067_MSG_A0;

typedef union {
    uint16_t word;
    uint8_t byte[2];
    struct {
	uint16_t DRIVE_ADDR:3;
	uint16_t _unused_3:1;
	uint16_t SERVO_SIGNAL_PRESENT:1;
	uint16_t HEADS_HOME:1; // DCI HOME on prints, means retracted, *NOT* at track zero!
	uint16_t BRUSHES_HOME:1;
	uint16_t DOOR_LATCHED:1;
	uint16_t CARTRIDGE_PRESENT:1;
	uint16_t SPEED_OK:1;
	uint16_t FWD:1;
	uint16_t REV:1;
	uint16_t HEADS_LOADING:1;
	uint16_t RTZ:1;
	uint16_t UNLOADING_HEADS:1;
	uint16_t PARITY:1;
    } __attribute__((packed));
} RK067_MSG_A1;

typedef union {
    uint16_t word;
    uint8_t byte[2];
    struct {
	uint16_t DRIVE_ADDR:3;
	uint16_t _unused_3:1;
	uint16_t DIFFERENCE:10; // ZR6H tests this being 10 btis wide, not 9?
	uint16_t _unused_14:2;
	uint16_t PARITY:1;
    } __attribute__((packed));
} RK067_MSG_A2;

typedef union {
    uint16_t word;
    uint8_t byte[2];
    struct {
	uint16_t DRIVE_ADDR:3;
	uint16_t SERIAL_NUMBER:12; // BCD, 3 DIGITS
	uint16_t PARITY:1;
    } __attribute__((packed));
} RK067_MSG_A3;

typedef union {
    uint16_t word;
    uint8_t byte[2];
    struct {
	uint16_t MESSAGE_ID:2;
	uint16_t _unused_2_4:3;
	uint16_t INVALID_ADDRESS:1;
	uint16_t AC_LOW:1;
	uint16_t FAULT:1;
	uint16_t NONEXECUTABLE_FUNCTION:1;
	uint16_t C_D_PARITY_ERROR:1;
	uint16_t SEEK_INCOMPLETE:1;
	uint16_t WRITE_LOCK_ERROR:1;
	uint16_t SPEED_LOSS:1; // no, not that one
	uint16_t DRIVE_OFF_TRACK:1;
	uint16_t READ_WRITE_UNSAFE:1;
	uint16_t PARITY:1;
    } __attribute__((packed));
} RK067_MSG_B0;

typedef union {
    uint16_t word;
    uint8_t byte[2];
    struct {
	uint16_t MESSAGE_ID:2;
	uint16_t _unused_2_3:2;
	uint16_t SECTOR_ERROR:1;
	uint16_t WRITE_CURRENT_NO_GATE:1;
	uint16_t WRITE_GATE_NO_TX:1;
	uint16_t HEAD_FAULT:1;
	uint16_t MULTIPLE_HEAD_SELECT:1;
	uint16_t INDEX_ERROR:1;
	uint16_t TRIBIT_ERROR:1;
	uint16_t SERVO_SIGNAL_ERROR:1;
	uint16_t SEEK_NO_MOTION:1;
	uint16_t LIMIT_DETECT_ON_SEEK:1;
	uint16_t SERVO_UNSAFE:1;
	uint16_t PARITY:1;
    } __attribute__((packed));
} RK067_MSG_B1;

typedef union {
    uint16_t word;
    uint8_t byte[2];
    struct {
	uint16_t MESSAGE_ID:2;
	uint16_t _unused_2_3:2;
	uint16_t CYLINDER:10;
	uint16_t _unused_14:1;
	uint16_t PARITY:1;
    } __attribute__((packed));
} RK067_MSG_B2;

typedef union {
    uint16_t word;
    uint8_t byte[2];
    struct {
	uint16_t MESSAGE_ID:2;
	uint16_t _unused_2_3:2;
	uint16_t SECTOR_COUNT:5;
	uint16_t HEAD_ADDRESS:3;
	uint16_t _unused_12_14:3;
	uint16_t PARITY:1;
    } __attribute__((packed));
} RK067_MSG_B3;

class rk611_c;
class rk067_c: public storagedrive_c {
    friend class rk611_c;

private:
    rk611_c *host;
    int unit = 0;
    int state = RK067_STOPPED;
    bool spindle_stop = false; // Spindle is stopped by a spindle stop command.
    bool spindle_start = false;
    bool seek_pending = false;
    bool recal_pending = false;
    int spindle_speed = 0;  // In sector-pulse sized counts for convenience
    int brush_time = 0;     // Same idea for brush cycle
    int head_load_time = 0; // Same idea for head load time
    timeout_c seek_delay;
    int desired_cylinder = 0;
    int head = 0;
    int sector = 0;

    RK067_MSG_A0 Status_A0 = {0};
    RK067_MSG_A1 Status_A1 = {0};
    RK067_MSG_A2 Status_A2 = {0};
    RK067_MSG_A3 Status_A3 = {0};

    RK067_MSG_B0 Status_B0 = {0};
    RK067_MSG_B1 Status_B1 = {0};
    RK067_MSG_B2 Status_B2 = {0};
    RK067_MSG_B3 Status_B3 = {0};

public:
    rk067_c(rk611_c *_controller, int _unit);
    ~rk067_c();

    // Physical interlocks
    parameter_bool_c cover_open = parameter_bool_c(this,"cover_open","cover",false,"Cover open/closed"); // 同じ未来を見ていたい

    // Stuff on the panel
    parameter_bool_c run_btn = parameter_bool_c(this,"run_btn","run",false,"Status of the run/stop button");
    parameter_bool_c stop_lamp = parameter_bool_c(this,"stop_lamp","sl",true,"Status of the run/stop lamp");
    parameter_bool_c ready_lamp = parameter_bool_c(this,"ready_lamp","ready",true,"Status of the ready lamp");
    parameter_bool_c fault_lamp = parameter_bool_c(this,"fault_lamp","fault",true,"Status of the fault lamp");
    parameter_bool_c write_lock = parameter_bool_c(this,"write_lock","wl",false,"Status of the write lock button");
    parameter_bool_c port_a_btn = parameter_bool_c(this,"port_a_btn","pa",false,"Status of the Port A button");
    parameter_bool_c port_b_btn = parameter_bool_c(this,"port_b_btn","pb",false,"Status of the Port B button");
    parameter_bool_c port_a_lamp = parameter_bool_c(this,"port_a_lamp","la",true,"Status of the Port A lamp");
    parameter_bool_c port_b_lamp = parameter_bool_c(this,"port_b_lamp","lb",true,"Status of the Port B lamp");

    // Stuff inside the drive
    parameter_unsigned_c serial_number =
	parameter_unsigned_c(this,"serial_number","sn",false,"","%.3d","Three decimal digits",12,10);

    // Functions
    bool is_selected();
    void select();
    void deselect();
    bool handle_command(int fcn,uint16_t msg_a,uint16_t msg_b,int bad_parity);
    void set_type(enum drive_type_e _drivetype);
    bool on_param_changed(parameter_c *param) override;
    void on_power_changed(signal_edge_enum aclo_edge, signal_edge_enum dclo_edge) override;
    void on_init_changed(int init);
    void on_init_changed(void) override;
    void worker(unsigned instance) override;
};

#endif
