/*
  ts11_drive.hpp: TS04/TS05 tape drive

  Copyright Bogodyne Metatechnics LLC
  Contributed under the BSD 2-clause license.

  This is the drive attached to the TS11 or TSV11.
  If TS11, the drive is a TS04, which is specific to the TS11.
  If TSV05, the drive is a TS05, which is a rebranded Cipher F880.
*/

#ifndef _TS11_DRIVE_HPP_
#define _TS11_DRIVE_HPP_

#include "tapedrive.hpp"

class ts11_c;
class ts11_drive_c: public tapedrive_c {
    friend class ts11_c;

    // Internal state
    bool moving;
    bool rewinding;
    bool volume_check;
    timeout_c motion_delay; // Delay associated with tape motion
    double tape_position; // Inches
    double tape_total_length; // Inches also, updated from tape_length

    // Worker's state
    pthread_mutex_t worker_sync_mutex;
    pthread_cond_t worker_sync_cond = PTHREAD_COND_INITIALIZER;
    int worker_active;
    int worker_abort;
    int worker_operation;
    int worker_parameter;
    int worker_counter;
    int load_pending;
    int rewind_pending;

public:

    // Buttons on the TS04 (with analogous buttons on the TS05)
    parameter_bool_c ONL_btn = parameter_bool_c(this,"ONL_btn","ONL",false,"State of On-Line button");
    parameter_bool_c LOD_btn = parameter_bool_c(this,"LOD_btn","LOD",false,"State of Load button");
    // This determines if a mounted image is writable.
    parameter_bool_c write_ring = parameter_bool_c(this,"write_ring","ring",false,
						   "Whether or not a write ring is present on the supply reel");
    // This parameter determines the length (in feet) of the tape on the supply reel.
    parameter_unsigned_c tape_length = parameter_unsigned_c(this,"tape_length","len",false,"feet","%d",
							    "Length of the tape on the supply reel. Zero for infinite.",16,10);
    // This indicates the position into the tape in feet. The user could approximate this by looking at the tape.
    parameter_unsigned_c head_position = parameter_unsigned_c(this,"head_position","pos",true,"feet","%d",
							      "Approximate position of the tape head. BOT is at 15 feet.",16,10);
    // Lamps, only on the TS04.
    parameter_bool_c UOK = parameter_bool_c(this,"UOK_lamp","UOK",true,"State of the Microprocessor OK lamp");
    parameter_bool_c VCK = parameter_bool_c(this,"VCK_lamp","VCK",true,"State of the Volume Check lamp");
    parameter_bool_c DCK = parameter_bool_c(this,"DCK_lamp","DCK",true,"State of the Density Check lamp");
    parameter_bool_c WLK = parameter_bool_c(this,"WLK_lamp","WLK",true,"State of the Write Lock lamp");
    parameter_bool_c BOT = parameter_bool_c(this,"BOT_lamp","BOT",true,"State of the Beginning of Tape lamp");
    parameter_bool_c EOT = parameter_bool_c(this,"EOT_lamp","EOT",true,"State of the End of Tape lamp");

    // Functions
    ts11_drive_c(tapecontroller_c *controller);
    bool is_online();
    bool is_write_enabled();
    bool is_moving();
    bool is_at_eot();
    bool has_volume_check();
    void clear_volume_check();
    bool read_data_start();
    bool read_data_reverse_start();
    bool write_data_start(unsigned block_length);
    bool write_tape_mark_start();
    bool space_record_fwd();
    bool space_record_rev();
    bool space_file_fwd();
    bool space_file_rev();
    bool rewind_start();
    bool on_param_changed(parameter_c *param) override;
    void on_power_changed(signal_edge_enum aclo_edge,signal_edge_enum dclo_edge) override;
    void on_init_changed(void) override;
    void on_ssi();
    void worker(unsigned instance) override;
};

#endif
