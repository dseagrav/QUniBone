/*
  ts11_drive.cpp: TS04/TS05 tape drive

  Copyright Bogodyne Metatechnics LLC
  Contributed under the BSD 2-clause license.

  This is the drive attached to the TS11 or TSV11.
  If TS11, the drive is a TS04, which is specific to the TS11.
  If TSV05, the drive is a TS05, which is a rebranded Cipher F880.
*/

#include "logger.hpp"
#include "timeout.hpp"
#include "ts11.hpp"
#include "ts11_drive.hpp"

// The Inter-Block Gap is 0.5 inches minimum. We'll use this
#define IBG_LENGTH 0.6
// A PE (1600 bpi) tape mark is 40 characters long and followed by the IBG, so we'll include the IBG in this.
#define TAPE_MARK_LENGTH 0.025
// One PE (1600 bpi) character on tape
#define TAPE_CHAR_LENGTH 0.000625

enum worker_op_enum {
    WKOP_READ = 0,
    WKOP_READ_REVERSE,
    WKOP_SPACE_RECORD,
    WKOP_SPACE_RECORD_REVERSE,
    WKOP_SPACE_FILE,
    WKOP_SPACE_FILE_REVERSE,
    WKOP_WRITE_DATA,
    WKOP_WRITE_MARK,
    WKOP_ERASE,
    WKOP_REWIND,
    WKOP_UNLOAD
};

ts11_drive_c::ts11_drive_c(tapecontroller_c *_controller): tapedrive_c(_controller){
    log_label = "TSDR"; // Will get clobbered on create
    ONL_btn.value = LOD_btn.value = write_ring.value = false;
    UOK.value = VCK.value = DCK.value = WLK.value = BOT.value = EOT.value = false;
    tape_length.value = 2400;
    head_position.value = 0;
    rewinding = false;
    tape_position = 0;
    tape_total_length = (12*tape_length.value);
    worker_active = 0;
    worker_abort = 0;
    worker_operation = 0;
    worker_parameter = 0;
    load_pending = 0;
    rewind_pending = 0;
    // We want our worker sync mutex to be recursive so that op_complete_strobe() can chain operations
    pthread_mutexattr_t ws_mutex_attributes;
    int rv = pthread_mutexattr_init(&ws_mutex_attributes);
    if(rv != 0){
	FATAL("Can't create attributes item for worker sync mutex.");
    }
    rv = pthread_mutexattr_settype(&ws_mutex_attributes,PTHREAD_MUTEX_RECURSIVE);
    if(rv != 0){
	FATAL("Can't modify attributes item for worker sync mutex.");
    }
    rv = pthread_mutex_init(&worker_sync_mutex,&ws_mutex_attributes);
    if(rv != 0){
	FATAL("Can't initialize worker sync mutex.");
    }
    pthread_mutexattr_destroy(&ws_mutex_attributes); // We don't really care if this loses.
}

bool ts11_drive_c::is_online(){
    // For the drive to be on-line, a tape must be mounted and the ONL button must be in.
    if(is_loaded() && ONL_btn.value == true){
	return(true);
    }
    return(false);
}

bool ts11_drive_c::is_write_enabled(){
    return(is_loaded() && ONL_btn.value == true && write_ring.value == true);
}

bool ts11_drive_c::has_volume_check(){
    return(VCK.value != false);
}

void ts11_drive_c::clear_volume_check(){
    VCK.value = false;
}

bool ts11_drive_c::is_moving(){
    return(moving);
}

bool ts11_drive_c::is_at_eot(){
    return(tape_total_length > 0 && tape_position >= tape_total_length);
}

bool ts11_drive_c::read_data_start(){
    // Is the worker busy?
    if(worker_active != 0){
	// Yes, we can't take commands now. (Maybe wait?)
	return(false);
    }
    pthread_mutex_lock(&worker_sync_mutex);
    pthread_cond_signal(&worker_sync_cond);
    worker_operation = WKOP_READ;
    worker_parameter = 0;
    worker_active = 1;
    pthread_mutex_unlock(&worker_sync_mutex);
    return(true);
}

bool ts11_drive_c::read_data_reverse_start(){
    // Is the worker busy?
    if(worker_active != 0){
	// Yes, we can't take commands now. (Maybe wait?)
	return(false);
    }
    pthread_mutex_lock(&worker_sync_mutex);
    pthread_cond_signal(&worker_sync_cond);
    worker_operation = WKOP_READ_REVERSE;
    worker_parameter = 0;
    worker_active = 1;
    pthread_mutex_unlock(&worker_sync_mutex);
    return(true);
}

bool ts11_drive_c::write_data_start(unsigned block_length){
    // Is the worker busy?
    if(worker_active != 0){
	// Yes, we can't take commands now. (Maybe wait?)
	return(false);
    }
    pthread_mutex_lock(&worker_sync_mutex);
    pthread_cond_signal(&worker_sync_cond);
    worker_operation = WKOP_WRITE_DATA;
    worker_parameter = block_length;
    worker_active = 1;
    pthread_mutex_unlock(&worker_sync_mutex);
    return(true);
}

bool ts11_drive_c::write_tape_mark_start(){
    // Is the worker busy?
    if(worker_active != 0){
	// Yes, we can't take commands now. (Maybe wait?)
	return(false);
    }
    pthread_mutex_lock(&worker_sync_mutex);
    pthread_cond_signal(&worker_sync_cond);
    worker_operation = WKOP_WRITE_MARK;
    worker_parameter = 0;
    worker_active = 1;
    pthread_mutex_unlock(&worker_sync_mutex);
    return(true);
}

bool ts11_drive_c::space_record_fwd(){
    if(worker_active != 0){
	return(false);
    }
    pthread_mutex_lock(&worker_sync_mutex);
    pthread_cond_signal(&worker_sync_cond);
    worker_operation = WKOP_SPACE_RECORD;
    worker_parameter = 0;
    worker_active = 1;
    pthread_mutex_unlock(&worker_sync_mutex);
    return(true);
}

bool ts11_drive_c::space_record_rev(){
    if(worker_active != 0){
	return(false);
    }
    pthread_mutex_lock(&worker_sync_mutex);
    pthread_cond_signal(&worker_sync_cond);
    worker_operation = WKOP_SPACE_RECORD_REVERSE;
    worker_parameter = 0;
    worker_active = 1;
    pthread_mutex_unlock(&worker_sync_mutex);
    return(true);
}

bool ts11_drive_c::space_file_fwd(){
    if(worker_active != 0){
	return(false);
    }
    pthread_mutex_lock(&worker_sync_mutex);
    pthread_cond_signal(&worker_sync_cond);
    worker_operation = WKOP_SPACE_FILE;
    worker_parameter = 0;
    worker_active = 1;
    pthread_mutex_unlock(&worker_sync_mutex);
    return(true);
}

bool ts11_drive_c::space_file_rev(){
    return(false);
}

bool ts11_drive_c::rewind_start(){
    // Is the worker busy?
    if(worker_active != 0){
	// Yes, we can't take commands now. (Maybe wait?)
	return(false);
    }
    // Are we online?
    if(ONL_btn.value != 0){
	// Are we loaded?
	if(is_loaded()){
	    // Are we at BOT?
	    if(!is_at_bot()){
		// Already rewinding?
		if(rewind_pending == 0){
		    // No, rewind.
		    pthread_mutex_lock(&worker_sync_mutex);
		    pthread_cond_signal(&worker_sync_cond);
		    worker_operation = WKOP_REWIND;
		    worker_parameter = 0;
		    worker_active = 1;
		    pthread_mutex_unlock(&worker_sync_mutex);
		    return(true);
		}
	    }
	}
    }
    return(false);
}

// Called when TS11 gets a SubSystem Initialize
void ts11_drive_c::on_ssi(){
    // Is a command running?
    if(worker_active != 0){
	// Abort it
	worker_abort = 1;
	INFO("Awaiting worker abort...");
	// Wait until we can get the mutex
	pthread_mutex_lock(&worker_sync_mutex);
	// We have it.
	INFO("Worker abort completed.");
	pthread_mutex_unlock(&worker_sync_mutex);
    }
    // Are we online?
    if(ONL_btn.value != 0){
	// Are we loaded?
	if(is_loaded()){
	    // Are we at BOT?
	    if(!is_at_bot()){
		// Already rewinding?
		if(rewind_pending == 0){
		    // No, rewind.
		    INFO("Rewinding the tape...");
		    pthread_mutex_lock(&worker_sync_mutex);
		    pthread_cond_signal(&worker_sync_cond);
		    rewind_pending = 1;
		    pthread_mutex_unlock(&worker_sync_mutex);
		    // Await completion of the rewind operation
		    while(rewind_pending != 0){
			timeout_c delay;
			delay.wait_ms(100);
		    }
		    LOD_btn.value = 0; // Just in case
		    INFO("Rewind completed.");
		}
	    }
	}
    }
    // Set UOK
    UOK.value = true;
    // Set VOLUME CHECK
    VCK.value = true;
    // INFO("Drive initialization completed");
}

bool ts11_drive_c::on_param_changed(parameter_c *param){
    if(param == &LOD_btn){
	if(LOD_btn.new_value != 0){
	    // Loading, rewinding, or unloading?
	    if(!is_loaded()){
		// Load.
		if(load_pending >= 0){
		    if(worker_active == 0 && rewind_pending == 0){
			INFO("Starting LOAD");
			pthread_mutex_lock(&worker_sync_mutex);
			pthread_cond_signal(&worker_sync_cond);
			load_pending = 1;
			pthread_mutex_unlock(&worker_sync_mutex);
		    }else{
			INFO("Queued LOAD");
			load_pending = 1;
		    }
		    return(true);
		}else{
		    return(false);
		}
	    }else{
		if(!is_at_bot() && !rewinding){
		    // Rewind
		    if(rewind_pending == 0){
			if(worker_active == 0 && load_pending == 0){
			    INFO("Starting REWIND");
			    pthread_mutex_lock(&worker_sync_mutex);
			    pthread_cond_signal(&worker_sync_cond);
			    rewind_pending = 1;
			    pthread_mutex_unlock(&worker_sync_mutex);
			}else{
			    INFO("Queued REWIND");
			    rewind_pending = 1;
			}
			return(true);
		    }else{
			return(false);
		    }
		}else{
		    // Unload
		    if(load_pending <= 0){
			if(worker_active == 0 && rewind_pending == 0){
			    INFO("Starting UNLOAD");
			    pthread_mutex_lock(&worker_sync_mutex);
			    pthread_cond_signal(&worker_sync_cond);
			    load_pending = -1;
			    pthread_mutex_unlock(&worker_sync_mutex);
			}else{
			    INFO("Queued UNLOAD");
			    load_pending = -1;
			}
			return(true);
		    }else{
			return(false);
		    }
		}
	    }
	}else{
	    // Cancel?
	}
	return(true);
    }
    if(param == &ONL_btn){
	ONL_btn.value = ONL_btn.new_value; // Do it immediately
	VCK.value = true; // Status change lights VCK
	controller->on_drive_status_changed(this); // Tell the controller
	return(true);
    }
    if(param == &tape_length){
	tape_total_length = (12*tape_length.new_value);
	return(true);
    }
    // Pass through
    return tapedrive_c::on_param_changed(param);
}

void ts11_drive_c::on_power_changed(signal_edge_enum aclo_edge, signal_edge_enum dclo_edge){
    UNUSED(aclo_edge);
    UNUSED(dclo_edge);
}

void ts11_drive_c::on_init_changed(void){

}

void ts11_drive_c::worker(unsigned instance){
    timeout_c delay;  // Used throughout
    UNUSED(instance); // Only one worker per TS11
    unsigned worker_index = 0;
    unsigned worker_bound = 0;
    assert(!pthread_mutex_lock(&worker_sync_mutex));
    worker_init_realtime_priority(rt_device);

    while(!workers_terminate){
	int wait_result = pthread_cond_wait(&worker_sync_cond,&worker_sync_mutex);
	if(wait_result != 0){
	    ERROR("ts11_drive_c::worker(): pthread_cond_wait returned %d: %s",wait_result,strerror(wait_result));
	    continue;
	}else{
	    // We have a task
	    if(load_pending != 0){
		// Load or unload
		if(load_pending > 0){
		    // Load
		    // A load operation should mount the tape image, then advance the tape to find the BOT marker.
		    // If the BOT marker is not seen by 25 feet, it rewinds to before the BOT marker and tries again.
		    // We'll assume the load operation takes ten seconds.
		    // The BOT mark is supposed to be at around 15 feet into the tape.
		    moving = true;
		    delay.wait_ms(10000);
		    moving = false;
		    if(do_load_op()){
			INFO("Tape load complete");
			tape_position = 180.0; // 15 feet in inches
			head_position.value = (tape_position/12);
		    }else{
			ERROR("Tape load failed");
		    }
		    LOD_btn.value = 0;
		    load_pending = 0;
		    controller->on_drive_status_changed(this);
		}else{
		    // Unload
		    moving = true;
		    delay.wait_ms(10000);
		    moving = false;
		    if(do_unload_op()){
			INFO("Tape unload complete");
			tape_position = 0;
			head_position.value = 0;
		    }else{
			ERROR("Tape unload failed");
		    }
		    LOD_btn.value = 0;
		    load_pending = 0;
		    controller->on_drive_status_changed(this);
		}
	    }else{
		if(rewind_pending != 0){
		    // Rewind to BOT.
		    // How long will it take? Rewind speed is 150 inches per second.
		    double rewind_time = (tape_position-180)/150;
		    INFO("Tape rewind will take %f seconds",rewind_time);
		    // Convert to nanoseconds and do the needful.
		    uint64_t wait_time = rewind_time*1000000000;
		    rewinding = true;
		    moving = true;
		    if(!do_rewind_op()){
			FATAL("do_rewind_op() failed");
		    }
		    delay.wait_ns(wait_time);
		    rewinding = false;
		    moving = false;
		    // Done.
		    INFO("Rewind completed");
		    LOD_btn.value = 0;
		    rewind_pending = 0;
		    tape_position = 180;
		    controller->on_drive_status_changed(this);
		}else{
		    // Do we have a command?
		    while(worker_active != 0){
			// INFO("WORKER ACTIVE, OP %d",worker_operation);
			// Do it
			switch(worker_operation){

			case WKOP_READ:
			    delay.wait_ms(8); // Start motion
			    moving = true;
			    if(!do_start_read_op()){
				switch(Op_Result.Code){

				case TPOP_TAPE_MARK:
				    tape_position += TAPE_MARK_LENGTH;
				    head_position.value = (tape_position/12);
				    worker_active = 0;
				    controller->op_complete_strobe(TPOP_TAPE_MARK,0);
				    moving = false;
				    break;

				case TPOP_IMAGE_EOF:
				    // Image EOF.
				    // The tape will move forward 25 feet and OPI will be raised.
				    tape_position += 300;
				    head_position.value = (tape_position/12);
				    worker_active = 0;
				    controller->op_complete_strobe(TPOP_IMAGE_EOF,0);
				    moving = false;
				    break;

				default:
				    // Something else
				    FATAL("do_start_read_op() failed, result code %d",Op_Result.Code);
				}
				worker_active = 0;
			    }else{
				worker_bound = Op_Result.Value;
				worker_index = 0;
				// INFO("Tape block has %d bytes",worker_bound);
				while(worker_index < worker_bound && worker_abort == 0){
				    // One frame happens every 13889 nanoseconds
				    uint8_t frame = 0;
				    delay.wait_ns(13889);
				    if(!do_read_data_op(&frame)){
					FATAL("do_read_data_op() failed");
					worker_active = 0;
				    }
				    tape_position += TAPE_CHAR_LENGTH;
				    if(!controller->read_data_strobe(&frame,false)){
					FATAL("read_data_strobe() blew it!");
				    }
				    worker_index++;
				}
				if(worker_abort == 0){
				    tape_position += IBG_LENGTH;
				    if(!do_end_read_op(worker_bound)){
					FATAL("do_end_read_op() failed");
					worker_active = 0;
				    }
				    controller->op_complete_strobe(TPOP_OK,0);
				}else{
				    INFO("Read operation aborted");
				}
				head_position.value = (tape_position/12);
				moving = false;
				worker_active = 0;
			    }
			    break;

			case WKOP_READ_REVERSE:
			    // Start read operaton
			    delay.wait_ms(8); // Start motion
			    moving = true;
			    if(!do_start_read_reverse_op()){
				switch(Op_Result.Code){

				case TPOP_TAPE_MARK:
				    // Saw a tape mark

				default:
				    // Something else
				    FATAL("do_start_read_reverse_op() failed, result code %d",Op_Result.Code);
				}
				worker_active = 0;
			    }else{
				worker_bound = Op_Result.Value;
				worker_index = 0;
				// INFO("Tape block has %d bytes",worker_bound);
				tape_position -= IBG_LENGTH; // Subtract previous block's IBG
				while(worker_index < worker_bound && worker_abort == 0){
				    // One frame happens every 13889 nanoseconds
				    uint8_t frame = 0;
				    delay.wait_ns(13889);
				    if(!do_read_reverse_data_op(&frame)){
					FATAL("do_read_reverse_data_op() failed");
					worker_active = 0;
				    }
				    tape_position -= TAPE_CHAR_LENGTH;
				    if(!controller->read_data_strobe(&frame,true)){
					FATAL("read_data_strobe() blew it!");
				    }
				    worker_index++;
				}
				if(worker_abort == 0){
				    if(!do_end_read_reverse_op(worker_bound)){
					FATAL("do_end_read_reverse_op() failed");
					worker_active = 0;
				    }
				    controller->op_complete_strobe(TPOP_OK,0);
				}else{
				    INFO("Read Reverse operation aborted");
				}
				head_position.value = (tape_position/12);
				moving = false;
				worker_active = 0;
			    }
			    break;

			case WKOP_WRITE_DATA: // Parameter is block size
			    // Start write operation.
			    delay.wait_ms(8); // Start motion
			    worker_bound = worker_parameter;
			    moving = true;
			    if(!do_start_data_block_op(worker_bound)){
				FATAL("do_start_data_block_op() failed");
				worker_active = 0;
			    }else{
				worker_index = 0;
				while(worker_index < worker_bound){
				    uint8_t frame;
				    if(worker_abort == 0){
					// One frame happens every 13889 nanoseconds
					delay.wait_ns(13889);
					if(!controller->write_data_strobe(&frame,false)){
					    FATAL("write_data_strobe() blew it!");
					}
				    }else{
					// Aborting the write. Don't delay, fill the rest of the block with zeroes
					frame = 0;
				    }
				    if(!do_write_data_op(frame)){
					FATAL("do_write_data_op() failed");
					worker_active = 0;
				    }
				    tape_position += TAPE_CHAR_LENGTH;
				    worker_index++;
				}
				if(!do_end_data_block_op(worker_bound)){
				    FATAL("do_end_data_block_op() failed");
				    worker_active = 0;
				}
				// Add IBG
				tape_position += IBG_LENGTH;
				// Did we abort?
				if(worker_abort == 0){
				    // No. Tell the controller
				    controller->op_complete_strobe(TPOP_OK,0);
				}else{
				    // Yes.
				    INFO("Write operation aborted");
				}
				head_position.value = (tape_position/12);
				moving = false;
				worker_active = 0;
			    }
			    break;

			case WKOP_WRITE_MARK:
			    // Start write operation.
			    delay.wait_ms(8); // Start motion
			    moving = true;
			    if(!do_write_tape_mark_op()){
				FATAL("do_write_tape_mark_op() failed");
				worker_active = 0;
			    }else{
				// Add IBG
				tape_position += IBG_LENGTH;
				controller->op_complete_strobe(TPOP_OK,0);
				head_position.value = (tape_position/12);
				moving = false;
				worker_active = 0;
			    }
			    break;

			case WKOP_SPACE_RECORD:
			    // Find block size.
			    delay.wait_ms(8); // Start motion
			    moving = true;
			    if(!do_start_read_op()){
				switch(Op_Result.Code){

				case TPOP_IMAGE_EOF:
				    tape_position += 300;
				    head_position.value = (tape_position/12);
				    worker_active = 0;
				    controller->op_complete_strobe(TPOP_IMAGE_EOF,0);
				    moving = false;
				    break;

				case TPOP_TAPE_MARK:
				    // Saw a tape mark
				    tape_position += TAPE_MARK_LENGTH;
				    head_position.value = (tape_position/12);
				    worker_active = 0;
				    controller->op_complete_strobe(TPOP_TAPE_MARK,0);
				    moving = false;
				    break;

				default:
				    // Something else
				    FATAL("do_start_read_op() failed, result code %d",Op_Result.Code);
				    worker_active = 0;
				}
			    }else{
				worker_bound = Op_Result.Value;
				worker_index = 0;
				// INFO("Tape block has %d bytes",worker_bound);
				while(worker_index < worker_bound && worker_abort == 0){
				    // Read forward, but don't send the byte to the host.
				    uint8_t frame = 0;
				    delay.wait_ns(13889);
				    if(!do_read_data_op(&frame)){
					FATAL("do_read_data_op() failed");
					worker_active = 0;
				    }
				    tape_position += TAPE_CHAR_LENGTH;
				    worker_index++;
				}
				worker_active = 0;
				if(worker_abort == 0){
				    tape_position += IBG_LENGTH;
				    if(!do_end_read_op(worker_bound)){
					FATAL("do_end_read_op() failed");
					worker_active = 0;
				    }
				    // If op_complete_strobe() returns false, another command follows this one.
				    controller->op_complete_strobe(TPOP_OK,0);
				}else{
				    INFO("Space Record Forward operation aborted");
				}
				head_position.value = (tape_position/12);
				moving = false;
			    }
			    break;

			case WKOP_SPACE_RECORD_REVERSE:
			    delay.wait_ms(8); // Start motion
			    moving = true;
			    if(!do_start_read_reverse_op()){
				switch(Op_Result.Code){

				case TPOP_TAPE_MARK:
				    // Saw a tape mark
				    tape_position -= TAPE_MARK_LENGTH;
				    head_position.value = (tape_position/12);
				    worker_active = 0;
				    controller->op_complete_strobe(TPOP_TAPE_MARK,0);
				    moving = false;
				    break;

				default:
				    // Something else
				    FATAL("do_start_read_reverse_op() failed, result code %d",Op_Result.Code);
				}
				worker_active = 0;
			    }else{
				worker_bound = Op_Result.Value;
				worker_index = 0;
				// INFO("Tape block has %d bytes",worker_bound);
				tape_position -= IBG_LENGTH; // Subtract previous block's IBG
				while(worker_index < worker_bound && worker_abort == 0){
				    // Read reverse, but don't send the byte to the host.
				    uint8_t frame = 0;
				    delay.wait_ns(13889);
				    if(!do_read_reverse_data_op(&frame)){
					FATAL("do_read_reverse_data_op() failed");
					worker_active = 0;
				    }
				    tape_position -= TAPE_CHAR_LENGTH;
				    worker_index++;
				}
				worker_active = 0;
				if(worker_abort == 0){
				    if(!do_end_read_reverse_op(worker_bound)){
					FATAL("do_end_read_reverse_op() failed");
				    }
				    controller->op_complete_strobe(TPOP_OK,0);
				}else{
				    INFO("Space Record Reverse operation aborted");
				}
				head_position.value = (tape_position/12);
				moving = false;
			    }
			    break;

			case WKOP_SPACE_FILE:
			    // Find block size.
			    delay.wait_ms(8); // Start motion
			    moving = true;
			    worker_counter = 0;
			    while(worker_active != 0){
				if(!do_start_read_op()){
				    switch(Op_Result.Code){

				    case TPOP_IMAGE_EOF:
					tape_position += 300;
					head_position.value = (tape_position/12);
					worker_active = 0;
					controller->op_complete_strobe(TPOP_IMAGE_EOF,worker_counter);
					moving = false;
					break;

				    case TPOP_TAPE_MARK:
					// Saw a tape mark
					INFO("Tape mark found at %d blocks",worker_counter);
					tape_position += TAPE_MARK_LENGTH;
					head_position.value = (tape_position/12);
					worker_active = 0;
					controller->op_complete_strobe(TPOP_OK,worker_counter);
					moving = false;
					break;

				    default:
					// Something else
					FATAL("do_start_read_op() failed, result code %d",Op_Result.Code);
					worker_active = 0;
					break;
				    }
				}else{
				    worker_counter++;
				    worker_bound = Op_Result.Value;
				    worker_index = 0;
				    // INFO("Tape block has %d bytes",worker_bound);
				    while(worker_index < worker_bound && worker_abort == 0){
					// Read forward, but don't send the byte to the host.
					uint8_t frame = 0;
					delay.wait_ns(13889);
					if(!do_read_data_op(&frame)){
					    FATAL("do_read_data_op() failed");
					    worker_active = 0;
					}
					tape_position += TAPE_CHAR_LENGTH;
					worker_index++;
				    }
				    if(worker_abort == 0){
					tape_position += IBG_LENGTH;
					if(!do_end_read_op(worker_bound)){
					    FATAL("do_end_read_op() failed");
					    worker_active = 0;
					}
					// Next record
				    }else{
					INFO("Space File Forward operation aborted");
				    }
				    head_position.value = (tape_position/12);
				}
			    }
			    break;

			case WKOP_REWIND:
			{
			    // How long will it take? Rewind speed is 150 inches per second.
			    double rewind_time = (tape_position-180)/150;
			    INFO("Tape is at %f inches, rewind will take %f seconds",tape_position,rewind_time);
			    // Convert to nanoseconds and do the needful.
			    uint64_t wait_time = rewind_time*1000000000;
			    rewinding = true;
			    moving = true;
			    if(!do_rewind_op()){
				FATAL("do_rewind_op() failed");
			    }
			    delay.wait_ns(wait_time);
			    rewinding = false;
			    // Done.
			    INFO("Rewind completed");
			    tape_position = 180;
			    LOD_btn.value = 0; // Just in case
			    rewind_pending = 0;
			    worker_active = 0;
			    controller->op_complete_strobe(TPOP_OK,0);
			    moving = false;
			}
			break;

			default:
			    FATAL("Unimplemented worker_operation %d",worker_operation);
			}
			// End of worker_active loop
		    }
		    // INFO("Worker is leaving");
		}
	    }
	}
	worker_abort = 0; // Clobber on recycle
    }
    assert(!pthread_mutex_unlock(&worker_sync_mutex));
}
