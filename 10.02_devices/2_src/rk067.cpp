/*
  rk067.cpp: RK06/RK07 disk drive

  Copyright Bogodyne Metatechnics LLC
  Contributed under the BSD 2-clause license.
*/

#include "logger.hpp"
#include "qunibus.h"
#include "rk611.hpp"

/* *** DELAY TUNING CONSTANTS *** */
// Microseconds per sector rotating past the head. Nominally 1136
// Too short will wedge the BBB
#define RK067_SECTOR_TIME 1100
// Seek time minimum for RK06, in nanoseconds. Nominally 8000000
#define RK06_SEEK_MIN 8000000
// Per-track seek time increment for RK06, in nanoseconds. Nominally 182927
#define RK06_SEEK_PTRK 182927
// Same stuff for the RK07. Nominally 6500000 and 87223
#define RK07_SEEK_MIN 6500000
#define RK07_SEEK_PTRK 87223

rk067_c::rk067_c(rk611_c *_controller, int _unit): storagedrive_c(_controller){
    host = _controller;
    unit = _unit;
    state = RK067_STOPPED;
    name.value = "hk";
    name.value += std::to_string(unit);
    log_label = name.value;
    spindle_start = false;
    spindle_stop = false;
    seek_pending = false;
    recal_pending = false;
    stop_lamp.value = true;
    type_name.readonly = false;
    port_a_btn.value = true;
    cover_open.value = false;
    spindle_speed = 0;
    Status_A0.DRIVE_AVAIL = 1;
    Status_A1.BRUSHES_HOME = 1;
    Status_A1.HEADS_HOME = 1;
    Status_A1.SERVO_SIGNAL_PRESENT = 0;
    Status_A1.DOOR_LATCHED = 0;
    Status_A1.CARTRIDGE_PRESENT = 0;
    Status_A3.SERIAL_NUMBER = 0; // BCD
    Status_B3.HEAD_ADDRESS = 1; // Head 0
    Status_B0.MESSAGE_ID = 0;
    Status_B1.MESSAGE_ID = 1;
    Status_B2.MESSAGE_ID = 2;
    Status_B3.MESSAGE_ID = 3;
    set_type(drive_type_e::RK07);
}

rk067_c::~rk067_c(){

}

void rk067_c::set_type(enum drive_type_e _drivetype){
    drive_type = _drivetype;
    switch (drive_type) {

    case drive_type_e::RK06:
	geometry.cylinder_count = 411;
	type_name.value = "RK06";
	Status_A0.DRIVE_TYPE = 0;
	break;

    case drive_type_e::RK07:
	geometry.cylinder_count = 815;
	type_name.value = "RK07";
	Status_A0.DRIVE_TYPE = 1;
	break;

    default:
	FATAL("set_type() called with invalid type");
	break;
    }
    geometry.head_count = 3; // Fourth is the servo head
    geometry.sector_count = 22; // For 16-bit packs; We're not going to emulate 18-bit packs
    geometry.sector_size_bytes = 512; // 256 words
    capacity.value = geometry.get_raw_capacity();
    geometry.bad_sector_file_offset =
	(geometry.head_count*geometry.cylinder_count-1)*geometry.sector_count*geometry.sector_size_bytes;
}

bool rk067_c::on_param_changed(parameter_c *param){
    if(param == &enabled){
	if(enabled.new_value == true){
	    // Power on
	}else{
	    // Power off
	}
    }
    if(param == &type_name){
	if(!strcasecmp(type_name.new_value.c_str(),"RK06")){
	    set_type(drive_type_e::RK06);
	}else{
	    if(!strcasecmp(type_name.new_value.c_str(),"RK07")){
		set_type(drive_type_e::RK07);
	    }else{
		// Lose
		ERROR("Unit type must be RK06 or RK07");
		return(false);
	    }
	}
    }
    if(param == &cover_open){
	if(Status_A1.DOOR_LATCHED != 0){ return(false); }else{ return(true); }
    }
    if(param == &serial_number){
	// Within range?
	if(serial_number.new_value < 1000){
	    // Make BCD
	    uint16_t nval = ((serial_number.new_value/100)%10);
	    nval <<= 4; nval |= ((serial_number.new_value/10)%10);
	    nval <<= 4; nval |= (serial_number.new_value%10);
	    Status_A3.SERIAL_NUMBER = nval;
	    return(true);
	}else{
	    // No.
	    ERROR("Serial number must be between 000 and 999 inclusive");
	    return(false);
	}
    }
    if(image_is_param(param)){
	// Adding or removing image?
	if(image_recreate_on_param_change(param)){
	    Status_A1.CARTRIDGE_PRESENT = image_not_null();
	    return(true);
	}
    }

    return storagedrive_c::on_param_changed(param);
}

void rk067_c::on_power_changed(signal_edge_enum aclo_edge, signal_edge_enum dclo_edge){
    UNUSED(aclo_edge);
    UNUSED(dclo_edge);
}

void rk067_c::on_init_changed(int init){
    UNUSED(init); // DSC flop on I&T4 says no?
    Status_A0.DRIVE_STATUS_CHANGE = 0;
    Status_B0.C_D_PARITY_ERROR = 0;
    Status_B0.SEEK_INCOMPLETE = 0;
    Status_B0.INVALID_ADDRESS = 0;
    Status_B1.LIMIT_DETECT_ON_SEEK = 0;
    Status_B0.FAULT = 0;
    fault_lamp.value = false;
}

void rk067_c::on_init_changed(void){
    on_init_changed(0);
}

bool rk067_c::is_selected(){
    return(port_a_lamp.value);
}

void rk067_c::deselect(){
    // Deselected by controller
    port_a_lamp.value = false;
    INFO("Unit %d deselected",unit);
}

bool rk067_c::handle_command(int fcn,uint16_t msg_a,uint16_t msg_b,int bad_parity){
    RK067_MSG_AH MSG_A = {msg_a};
    if(bad_parity != 0){
	// Commands with bad parity are not always executed?
	Status_B0.C_D_PARITY_ERROR = 1;
	Status_B0.FAULT = 1;
	fault_lamp.value = true;
	switch(fcn){

	case RK067_SELECT_DRIVE: // Select proceeds as normal?
	case RK067_SEEK: // Handled below
	    break;

	default:
	    INFO("Unhandled command %d with bad parity",fcn);
	}
    }
    switch(fcn){

    case RK067_SELECT_DRIVE:
	// Select or deselect?
	if(MSG_A.SELECT_RELEASE_CMD == 0){
	    // Select
	    if(port_a_lamp.value != true){
		port_a_lamp.value = true;
		INFO("Unit %d selected",unit);
	    }
	}else{
	    // Deselect
	    if(port_a_lamp.value != false){
		port_a_lamp.value = false;
		INFO("Unit %d deselected",unit);
	    }
	}
	// If bad parity, raise attention
	if(bad_parity != 0){
	    Status_A0.DRIVE_STATUS_CHANGE = 1;
	    host->raise_attention(unit);
	}
	host->op_complete_strobe(true);
	if(port_a_lamp.value){ host->status_update(unit,false); }
	break;

    case RK067_DRIVE_CLEAR:
	// Clear errors and attention
	Status_A0.DRIVE_STATUS_CHANGE = 0;
	Status_B0.C_D_PARITY_ERROR = 0;
	Status_B0.SEEK_INCOMPLETE = 0;
	Status_B0.INVALID_ADDRESS = 0;
	Status_B1.LIMIT_DETECT_ON_SEEK = 0;
	Status_B0.FAULT = 0;
	fault_lamp.value = false;
	host->clear_attention(unit);
	host->op_complete_strobe(true);
	if(port_a_lamp.value){ host->status_update(unit,false); }
	break;

    case RK067_PACK_ACKNOWLEDGE:
	// Set Volume Valid, even if we are spun down, etc.
	Status_A0.VOLUME_VALID = 1;
	host->op_complete_strobe(true);
	if(port_a_lamp.value){ host->status_update(unit,false); }
	break;

    case RK067_UNLOAD:
	spindle_stop = true;
	Status_A0.DRIVE_READY = 0;
	ready_lamp.value = false;
	// ZR6H checks that this command returns immediately, and that SPINDLE ON is off.
	Status_A0.SPINDLE_ON = 0;
	Status_A0.DRIVE_STATUS_CHANGE = 1;
	Status_A1.SPEED_OK = 0;
	Status_A1.HEADS_HOME = 1;
	Status_A1.SERVO_SIGNAL_PRESENT = 0;
	host->op_complete_strobe(true);
	if(port_a_lamp.value){ host->status_update(unit,false); }
	host->raise_attention(unit);
	break;

    case RK067_START_SPINDLE:
	spindle_stop = false;
	spindle_start = true;
	Status_A0.SPINDLE_ON = 1;
	// Status_A1.HEADS_HOME = 1;
	host->op_complete_strobe(true);
	if(port_a_lamp.value){ host->status_update(unit,false); }
	break;

    case RK067_RECALIBRATE:
	// This should do RTZ and not a normal seek
	if(state == RK067_RUNNING){
	    head_load_time = 250;
	    Status_A1.RTZ = 1;
	    Status_A0.POSIT_IN_PROGR = 1;
	    Status_B3.HEAD_ADDRESS = 1;
	    state = RK067_RTZ_COMMAND;
	    recal_pending = true;
	}else{
	    FATAL("RECALIBRATE command in state %d?",state);
	}
	host->op_complete_strobe(true);
	if(port_a_lamp.value){ host->status_update(unit,false); }
	break;

    case RK067_SEEK:
	if(state == RK067_RUNNING){
	    RK067_MSG_B2 MSG_B = {msg_b};
	    // Pick up head select
	    switch(MSG_A.TRACK_ADDRESS){
	    case 0:
		Status_B3.HEAD_ADDRESS = 1; break;
	    case 1:
		Status_B3.HEAD_ADDRESS = 2; break;
	    case 2:
		Status_B3.HEAD_ADDRESS = 4; break;
	    case 3:
		// Produces error!
		Status_B3.HEAD_ADDRESS = 0;
		Status_B0.SEEK_INCOMPLETE = 1;
		Status_B0.INVALID_ADDRESS = 1;
		Status_B0.FAULT = 1;
		Status_A0.DRIVE_STATUS_CHANGE = 1;
		fault_lamp.value = true;
		host->raise_attention(unit);
		host->op_complete_strobe(true);
		if(port_a_lamp.value){ host->status_update(unit,false); }
		return(true);
	    }
	    if(MSG_B.CYLINDER > Status_B2.CYLINDER){
		Status_A2.DIFFERENCE = MSG_B.CYLINDER-Status_B2.CYLINDER;
		Status_A1.REV = 1;
	    }else{
		if(MSG_B.CYLINDER < Status_B2.CYLINDER){
		    Status_A2.DIFFERENCE = Status_B2.CYLINDER-MSG_B.CYLINDER;
		    Status_A1.FWD = 1;
		}else{
		    // Seek to myself is a no-op and leaves a difference of one,
		    // but only if not track zero?
		    if(MSG_B.CYLINDER == 0){
			Status_A2.DIFFERENCE = 0;
		    }else{
			Status_A2.DIFFERENCE = 1;
		    }
		}
	    }
	    Status_A0.POSIT_IN_PROGR = 1;
	    // If bad parity, clobber the seek and leave the rest alone
	    if(bad_parity != 0){
		Status_A1.REV = 0;
		Status_A1.FWD = 0;
		Status_A0.POSIT_IN_PROGR = 0;
		Status_B2.CYLINDER = MSG_B.CYLINDER; // We update this?
		Status_A0.DRIVE_STATUS_CHANGE = 1;
		host->raise_attention(unit);
		host->op_complete_strobe(true);
		if(port_a_lamp.value){ host->status_update(unit,false); }
		// INFO("Cancelled seek to cylinder %d due to bad parity",MSG_B.CYLINDER);
		return(true);
	    }
	    // Seek to self returns immediately
	    if(MSG_B.CYLINDER == Status_B2.CYLINDER){
		Status_A0.POSIT_IN_PROGR = 0;
		Status_A0.DRIVE_STATUS_CHANGE = 1;
		host->raise_attention(unit);
		host->op_complete_strobe(true);
		if(port_a_lamp.value){ host->status_update(unit,false); }
		return(true);
	    }
	    // Compute seek time?
	    int seek_time;
	    // Are we an RK06 or RK07?
	    if(Status_A0.DRIVE_TYPE == 0){
		// RK06
		seek_time = RK06_SEEK_MIN;
		seek_time += (Status_A2.DIFFERENCE*RK06_SEEK_PTRK);
	    }else{
		// RK07
		seek_time = RK07_SEEK_MIN;
		seek_time += (Status_A2.DIFFERENCE*RK07_SEEK_PTRK);
	    }
	    seek_delay.start_ns(seek_time);
	    desired_cylinder = MSG_B.CYLINDER;
	    seek_pending = true; // SET THIS LAST, THE THREAD WATCHES IT
	    // INFO("Seeking to cylinder %d will take %d nsec",MSG_B.CYLINDER,seek_time);
	}else{
	    FATAL("SEEK command in state %d?",state);
	}
	// Status_A0.DRIVE_STATUS_CHANGE = 1;
	// host->raise_attention(unit);
	host->op_complete_strobe(true);
	if(port_a_lamp.value){ host->status_update(unit,false); }
	break;

	/*
    case RK067_READ_DATA:
    case RK067_WRITE_DATA:
    case RK067_WRITE_CHECK:
	if(state == RK067_RUNNING){
	    // READ DATA and WRITE DATA are a two step sequence, the first part of which is a seek command.
	    RK067_MSG_B2 MSG_B = {msg_b};
	    // Pick up head select
	    switch(MSG_A.TRACK_ADDRESS){
	    case 0:
		Status_B3.HEAD_ADDRESS = 1; break;
	    case 1:
		Status_B3.HEAD_ADDRESS = 2; break;
	    case 2:
		Status_B3.HEAD_ADDRESS = 4; break;
	    case 3:
		// Produces error?
		FATAL("Head 3 selected in read data command?");
	    }
	    if(MSG_B.CYLINDER > Status_B2.CYLINDER){
		Status_A2.DIFFERENCE = MSG_B.CYLINDER-Status_B2.CYLINDER;
		Status_A1.REV = 1;
	    }else{
		if(MSG_B.CYLINDER < Status_B2.CYLINDER){
		    Status_A2.DIFFERENCE = Status_B2.CYLINDER-MSG_B.CYLINDER;
		    Status_A1.FWD = 1;
		}else{
		    // Seek to myself is a no-op and leaves a difference of one,
		    // but only if not track zero?
		    if(MSG_B.CYLINDER == 0){
			Status_A2.DIFFERENCE = 0;
		    }else{
			Status_A2.DIFFERENCE = 1;
		    }
		}
	    }
	    if(MSG_B.CYLINDER != Status_B2.CYLINDER){
		Status_A0.POSIT_IN_PROGR = 1;
		// If bad parity, clobber the seek and leave the rest alone
		if(bad_parity != 0){
		    FATAL("Need to handle READ DATA command with bad parity and seek");
		}
		// Compute seek time?
		int seek_time;
		// Are we an RK06 or RK07?
		if(Status_A0.DRIVE_TYPE == 0){
		    // RK06
		    seek_time = RK06_SEEK_MIN;
		    seek_time += (Status_A2.DIFFERENCE*RK06_SEEK_PTRK);
		}else{
		    // RK07
		    seek_time = RK07_SEEK_MIN;
		    seek_time += (Status_A2.DIFFERENCE*RK07_SEEK_PTRK);
		}
		seek_delay.start_ns(seek_time);
		desired_cylinder = MSG_B.CYLINDER;
		seek_pending = true; // SET THIS LAST, THE THREAD WATCHES IT
		// INFO("Seeking to cylinder %d will take %d nsec",MSG_B.CYLINDER,seek_time);
	    }
	}else{
	    FATAL("READ/WRITE DATA command in state %d?",state);
	}
	data_xfer_pending = true;
	if(port_a_lamp.value){ host->status_update(unit,false); }
	break;
	*/

    default:
	FATAL("Unimplemented command %d, message words %.6o %.6o",fcn,msg_a,msg_b);
	return(false);
    }
    return(true);
}

void rk067_c::worker(unsigned instance){
    UNUSED(instance); // Only one!
    /*
      The drive runs at 2400 rpm nominally, or 40 rps.
      At 22 sectors per revolution, we will see a sector pulse every 1136 microseconds.

      Data transfer rate to the controller is one word every 4.3 microseconds.

      Start and stop times are 30 seconds nominal, 60 maximum. Starting adds a brush cycle, which takes 11 seconds.

      Seek time differs: 8 - 75 ms for the RK06 and 6.5 - 71 ms for the RK07.
      That's 8 - 0.1824817518 ms per cylinder for the RK06 and 6.5 - 0.0871165644 ms per cylinder for the RK07.
      Taking the base and adding 182,927 or 87,223 nanoseconds respectively should give approximately correct results.
    */
    timeout_c delay;
    worker_init_realtime_priority(rt_device);
    while(!workers_terminate){
	switch(state){

	case RK067_STOPPED:
	    // Delay one sector worth of time
	    delay.wait_us(RK067_SECTOR_TIME);
	    // Is the run/stop button in?
	    if(run_btn.value){
		// Are we stopped because of an unload command?
		if(!spindle_stop){
		    // Is a pack mounted?
		    if(Status_A1.CARTRIDGE_PRESENT != 0){
			// Yes! Is the cover closed?
			if(cover_open.value == false){
			    // Open image
			    if(image_open(true)){
				// Make image parameters read-only
				image_params_readonly(true);
				// Is this a new image?
				if(image_size() == 0){
				    INFO("NEW IMAGE - Creating manufacturer data area");
				    uint8_t Buffer[512];
				    unsigned offset;
				    if(Status_A0.DRIVE_TYPE == 0){
					// RK06: Cylinder 410, head 2, sector 0
					offset = (33792*410)+(11264*2);
				    }else{
					// RK07: Cylinder 814, head 2, sector 0
					offset = (33792*814)+(11264*2);
				    }
				    // Fill out the sector:
				    // Word 0 and Word 1 are the serial number, which is octal.
				    // Word 2 is zeroes
				    // Word 3 is zeroes unless we are an alignment cartridge
				    // Words 4 through 255 are all ones to indicate an empty bad sector list
				    memset(Buffer,0xFF,512);
				    // Generate serial number
				    int seed = time(NULL);
				    srandom(seed);
				    uint32_t pack_sn = (uint32_t)random(); // We are not trying to be cryptographically secure
				    Buffer[0] = pack_sn&0xFF; pack_sn >>= 4;
				    Buffer[1] = pack_sn&0xFF; pack_sn >>= 4;
				    Buffer[2] = pack_sn&0xFF; pack_sn >>= 4;
				    Buffer[3] = pack_sn&0xFF;
				    Buffer[4] = 0;
				    Buffer[5] = 0;
				    Buffer[6] = 0;
				    Buffer[7] = 0;
				    // It gets written ten times?
				    int x = 0;
				    while(x < 10){
					image_write(Buffer,offset,512);
					offset += 512;
					x++;
				    }
				}
				// We are good to spin up!
				Status_A1.DOOR_LATCHED = 1;
				Status_A0.SPINDLE_ON = 1;
				stop_lamp.value = false;
				INFO("Spinning up, cover locked.");
				state = RK067_MOTOR_STARTING;
			    }else{
				// image_open() blew it!
				ERROR("image_open() failed, releasing the run button");
				run_btn.value = false;
			    }
			}
		    }
		}
	    }
	    break;

	case RK067_MOTOR_STARTING:
	    // Spin up takes 16 seconds approx
	    if(spindle_speed < 14085){
		delay.wait_us(RK067_SECTOR_TIME);
		spindle_speed++;
		if(spindle_stop || !run_btn.value){
		    INFO("Aborting start.");
		    state = RK067_MOTOR_STOPPING;
		}
		break;
	    }
	    // Brush cycle
	    Status_A1.BRUSHES_HOME = 0;
	    brush_time = 9683;
	    state = RK067_BRUSH_CYCLE;
	    INFO("At speed, brushie brushie");
	    break;

	case RK067_MOTOR_STOPPING:
	    // Spin down takes 28 seconds
	    if(spindle_speed < 24848){
		delay.wait_us(RK067_SECTOR_TIME);
		spindle_speed++;
		break;
	    }
	    // Done
	    spindle_speed = 0;
	    Status_A1.DOOR_LATCHED = 0;
	    Status_A0.SPINDLE_ON = 0;
	    stop_lamp.value = true;
	    // Close image if it wasn't invalidated by lossage
	    if(image_is_open()){ image_close(); }
	    // Make image parameters read-write
	    image_params_readonly(false);
	    // All done
	    state = RK067_STOPPED;
	    INFO("Stopped, cover unlocked.");
	    break;

	case RK067_BRUSH_CYCLE:
	    // This takes 11 seconds
	    if(brush_time > 0){
		delay.wait_us(RK067_SECTOR_TIME);
		brush_time--;
		break;
	    }
	    Status_A1.BRUSHES_HOME = 1;
	    head_load_time = 0;
	    Status_A1.HEADS_HOME = 0;
	    Status_A1.FWD = 1;
	    Status_A1.REV = 0;
	    state = RK067_HEAD_LOAD_IN;
	    INFO("Moving heads in toward spindle");
	    break;

	case RK067_HEAD_LOAD_IN:
	    // Moving from home position across outer limit to inner limit
	    // Takes 3.2 seconds
	    if(head_load_time < 2600){
		if(head_load_time == (167)){
		    // Crossing OUTER LIMIT
		    Status_A1.SPEED_OK = 1;
		    Status_A1.SERVO_SIGNAL_PRESENT = 1;
		    Status_A0.POSIT_IN_PROGR = 1;
		    Status_A1.RTZ = 1;
		}
		delay.wait_us(RK067_SECTOR_TIME);
		head_load_time++;
		break;
	    }
	    head_load_time = 0;
	    Status_A1.FWD = 0;
	    Status_A1.REV = 1;
	    state = RK067_HEAD_LOAD_OUT;
	    INFO("Moving heads out toward home");
	    break;

	case RK067_RTZ_COMMAND:
	    // From wherever we are to outer limit.
	    // Come here with PIP and RTZ set.
	    if(head_load_time > 0){
		delay.wait_us(RK067_SECTOR_TIME);
		head_load_time--;
		break;
	    }
	    Status_A1.FWD = 0;
	    Status_A1.REV = 1;
	    state = RK067_HEAD_LOAD_OUT;
	    break;

	case RK067_HEAD_LOAD_OUT:
	    // From inner limit back to outer limit
	    // TIming here checked against ZR6H
	    if(head_load_time < 2300){
		if(head_load_time == 1900){
		    // Hit outer limit, find track 0
		    Status_A1.REV = 0;
		    Status_A1.FWD = 1;
		}
		delay.wait_us(RK067_SECTOR_TIME);
		head_load_time++;
		break;
	    }
	    Status_A0.POSIT_IN_PROGR = 0;
	    Status_B2.CYLINDER = 0;
	    Status_A2.DIFFERENCE = 0;
	    Status_A1.FWD = 0;
	    Status_A1.REV = 0;
	    Status_A1.RTZ = 0;
	    Status_A0.DRIVE_READY = 1;
	    ready_lamp.value = true;
	    if(spindle_start == true || recal_pending == true){
		Status_A0.DRIVE_STATUS_CHANGE = 1;
		host->raise_attention(unit);
		spindle_start = false;
	    }
	    if(port_a_lamp.value){ host->status_update(unit,false); }
	    desired_cylinder = 0;
	    sector = 0;
	    state = RK067_RUNNING;
	    if(recal_pending == false){
		INFO("Drive ready");
	    }else{
		recal_pending = false;
	    }
	    break;

	case RK067_RUNNING:
	    // Delay one sector worth of time
	    delay.wait_us(RK067_SECTOR_TIME);
	    // Update sector position
	    sector++;
	    if(sector >= 22){ sector = 0; }
	    // Seeking?
	    if(seek_pending){
		// Check
		if(seek_delay.reached()){
		    Status_B2.CYLINDER = desired_cylinder;
		    Status_A2.DIFFERENCE = 0;
		    Status_A1.FWD = 0;
		    Status_A1.REV = 0;
		    Status_A0.POSIT_IN_PROGR = 0;
		    Status_A0.DRIVE_STATUS_CHANGE = 1;
		    if(port_a_lamp.value){ host->status_update(unit,false); }
		    host->raise_attention(unit);
		    seek_pending = false;
		    INFO("Seek complete, at cylinder %d",Status_B2.CYLINDER);
		}
	    }else{
		// If heads are settled, update sector count
		Status_B3.SECTOR_COUNT = sector;
	    }
	    // Stopping?
	    if(!run_btn.value || spindle_stop){
		Status_A0.SPINDLE_ON = 0; // Ensure down
		Status_A1.SPEED_OK = 0;
		Status_A1.HEADS_HOME = 1;
		Status_A1.SERVO_SIGNAL_PRESENT = 0;
		if(Status_B2.CYLINDER != 0){
		    // Seek back home
		    Status_B2.CYLINDER = 0;
		}
		Status_A2.DIFFERENCE = 0;
		INFO("Unloading heads");
		head_load_time = 0;
		state = RK067_HEAD_UNLOAD;
	    }
	    break;

	case RK067_HEAD_UNLOAD:
	    // Takes two seconds
	    if(head_load_time < 1700){
		delay.wait_us(RK067_SECTOR_TIME);
		head_load_time++;
		break;
	    }
	    if(run_btn.value && !spindle_stop){
		INFO("UNLOAD cancelled! Re-loading heads");
		Status_A0.SPINDLE_ON = 1; // Ensure up
		Status_A1.HEADS_HOME = 0;
		Status_A1.FWD = 1;
		Status_A1.REV = 0;
		head_load_time = 0;
		state = RK067_HEAD_LOAD_IN;
		break;
	    }else{
		INFO("Heads unloaded");
	    }
	    spindle_speed = 0;
	    state = RK067_MOTOR_STOPPING;
	    break;

	default:
	    FATAL("Unimplemented state %d",state);
	}
    }
}
