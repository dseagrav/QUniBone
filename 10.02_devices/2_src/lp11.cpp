/*
  lp11.cpp: LP11 controller + LP05 (Dataproducts 2230) printer.

  Copyright Bogodyne Metatechnics LLC
  Contributed under the BSD 2-clause license.

  This simulates the M7258 version, which replaced the M7930.
*/

#include <assert.h>
#include <unistd.h>

#include "logger.hpp"
#include "qunibus.h"
#include "qunibusadapter.hpp"
#include "lp11.hpp"

lp11_c::lp11_c() : qunibusdevice_c(){
    name.value = "LP11";
    type_name.value = "lp11_c";
    log_label = "lp";

    // Clobber state
    LPCS.word = 0;
    LPDB.word = 0;
    online.value = false;
    page_width.value = 132;
    page_length.value = 66;
    file_name.value = "printer.txt";
    file_append.value = true;
    fd = NULL;
    buffer_idx = 0;
    column_idx = 0;
    line_number = 0;

    // Base address, slot number, interrupt vector, BR level
    set_default_bus_params(0777514, 28, 0200, 4);

    // Unibus-visible registers
    register_count = 2;

    UBR[0] = &(this->registers[0]);
    strcpy(UBR[0]->name,"LPCS");
    UBR[0]->active_on_dati = false;
    UBR[0]->active_on_dato = true;
    UBR[0]->reset_value = 0;
    UBR[0]->writable_bits = 0100;

    UBR[1] = &(this->registers[1]);
    strcpy(UBR[1]->name,"LPDB");
    UBR[1]->active_on_dati = false;
    UBR[1]->active_on_dato = true;
    UBR[1]->reset_value = 0;
    UBR[1]->writable_bits = 0177;
}

lp11_c::~lp11_c(){
    if(fd != NULL){ fclose(fd); }
}

bool lp11_c::on_param_changed(parameter_c *param){
    if(param == &online){
	// Going online or offline?
	if(online.new_value == true){
	    // Going online, open file
	    if(fd == NULL){
		if(file_append.value){
		    fd = fopen(file_name.value.c_str(),"a");
		}else{
		    fd = fopen(file_name.value.c_str(),"w");
		}
		if(fd == NULL){
		    ERROR("Can't open %s: %s",file_name.value.c_str(),strerror(errno));
		}else{
		    file_name.readonly = true;
		}
	    }
	    LPCS.RDY = 1; LPCS.ERROR = 0;
	}else{
	    // Going offline
	    if(fd != NULL){
		fclose(fd);
		fd = NULL;
		file_name.readonly = false;
	    }
	    LPCS.ERROR = 1; LPCS.RDY = 0;
	}
	set_register_dati_value(UBR[0],LPCS.word,"update_LPCS");
	// Interrupt armed?
	if(enabled.value == true && LPCS.INTR_ENB != 0){
	    qunibusadapter->INTR(intr_request,NULL,0);
	}
	return(true);
    }
    if(param == &priority_slot){
	intr_request.set_priority_slot(priority_slot.new_value);
    }
    if(param == &intr_level){
	intr_request.set_level(intr_level.new_value);
    }
    if(param == &intr_vector){
	intr_request.set_vector(intr_vector.new_value);
    }
    return qunibusdevice_c::on_param_changed(param); // Pass through
}

void lp11_c::on_after_register_access(qunibusdevice_register_t *device_reg, uint8_t unibus_control, DATO_ACCESS access){
    UNUSED(unibus_control);
    uint16_t new_val = device_reg->active_dato_flipflops;
    switch(device_reg->index){

    case 0: // CSR
	switch(access){

	case DATO_WORD:
	case DATO_BYTEL:
	    // If INTR_ENB is being set and RDY or ERROR is up, interrupt.
	    if((new_val&0100) != 0){
		if(LPCS.INTR_ENB == 0 && (LPCS.RDY != 0 || LPCS.ERROR != 0)){
		    LPCS.INTR_ENB = 1;
		    qunibusadapter->INTR(intr_request,UBR[0],LPCS.word);
		}else{
		    LPCS.INTR_ENB = 1;
		    set_register_dati_value(UBR[0],LPCS.word,"update_LPCS");
		}
	    }else{
		LPCS.INTR_ENB = 0;
		qunibusadapter->cancel_INTR(intr_request);
		set_register_dati_value(UBR[0],LPCS.word,"update_LPCS");
	    }
	    break;

	case DATO_BYTEH:
	    // Ignore
	    break;
	}
	break;

    case 1: // DB
	// This is write-only.
	// If the printer's (20-character) buffer is not full, this should recycle READY in 1 µsec.
	// If the buffer is full, it stays down until the next character prints, which is nominally 34 milliseconds.
	qunibusadapter->cancel_INTR(intr_request);
	switch(access){

	case DATO_WORD:
	case DATO_BYTEL:
	    // Discard character if not ready or error
	    if(LPCS.RDY != 0 && LPCS.ERROR == 0){
		// Send it
		pthread_mutex_lock(&on_after_register_access_mutex);
		pthread_cond_signal(&on_after_register_access_cond);
		LPDB.word = new_val;
		LPCS.RDY = 0;
		set_register_dati_value(UBR[0],LPCS.word,"update_LPCS");
		pthread_mutex_unlock(&on_after_register_access_mutex);
	    }else{
		INFO("Character lost due to error or not ready");
	    }
	    break;

	case DATO_BYTEH:
	    // Ignore? Or emit a zero?
	    break;
	}
	set_register_dati_value(UBR[1],0,"update_LPDB");
	break;
    }
}

void lp11_c::on_power_changed(signal_edge_enum aclo_edge, signal_edge_enum dclo_edge){
    UNUSED(aclo_edge);
    UNUSED(dclo_edge);
    LPCS.word = 0;
    LPDB.word = 0;
    if(online.value == true){ LPCS.RDY = 1; }else{ LPCS.ERROR = 1; }
    set_register_dati_value(UBR[0],LPCS.word,"update_LPCS");
    qunibusadapter->cancel_INTR(intr_request);
}

void lp11_c::on_init_changed(void){
    LPCS.word = 0;
    LPDB.word = 0;
    if(online.value == true){ LPCS.RDY = 1; }else{ LPCS.ERROR = 1; }
    set_register_dati_value(UBR[0],LPCS.word,"update_LPCS");
    qunibusadapter->cancel_INTR(intr_request);
}

void lp11_c::worker(unsigned instance){
    UNUSED(instance);
    assert(!pthread_mutex_lock(&on_after_register_access_mutex));
    worker_init_realtime_priority(rt_device);
    while(!workers_terminate){
	int wait_result = pthread_cond_wait(&on_after_register_access_cond,&on_after_register_access_mutex);
	if(wait_result != 0){
	    ERROR("pthread_cond_wait returned %d: %s",wait_result,strerror(wait_result));
	    continue;
	}else{
	    // Character to send is in LPDB.
	    if(fd != NULL){
		// Store it
		buffer[buffer_idx] = LPDB.DATA;
		// Dispatch to handle it
		int send_buffer = 0;
		switch(LPDB.DATA){

		case 012: // Newline
		    // Pass through
		    send_buffer = 1;
		    // Inc line count
		    line_number++;
		    if(line_number >= page_length.value){
			// A new page just completed
			line_number = 0;
		    }
		    break;

		case 014: // Form Feed
		    // Pass through
		    send_buffer = 1;
		    // Clobber line count
		    line_number = 0;
		    break;

		case 015: // Carriage Return
		    column_idx = 0; // Reset column
		    send_buffer = 1;
		    break;

		default: // Anything printable
		    column_idx++; // Advance column
		    // At end of carriage?
		    if(column_idx == page_width.value){
			// Add CR and LF
			buffer_idx++;
			buffer[buffer_idx] = 015;
			buffer_idx++;
			buffer[buffer_idx] = 012;
			send_buffer = 1;
			// Inc line count
			line_number++;
			if(line_number >= page_length.value){
			    // A new page just completed
			    line_number = 0;
			}
		    }
		    break;
		}
		buffer_idx++; // Advance buffer
		// Send it?
		if(send_buffer != 0){
		    // LAUNCH THE DRIFT MISSILE
		    size_t rv = fwrite(&buffer,1,buffer_idx,fd);
		    if(rv != buffer_idx){
			ERROR("Can't write to printer.txt: %s",strerror(errno));
			fclose(fd);
			fd = NULL;
			file_name.readonly = false;
		    }else{
			fflush(fd);
			fsync(fileno(fd));
		    }
		    buffer_idx = 0;
		}
	    }
	    LPDB.word = 0;
	    LPCS.RDY = 1;
	    if(LPCS.INTR_ENB != 0){
		qunibusadapter->INTR(intr_request,NULL,0);
	    }
	}
    }
    assert(!pthread_mutex_unlock(&on_after_register_access_mutex));
}
