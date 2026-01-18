/*
  rk611.cpp: RK611 disk controller

  Copyright Bogodyne Metatechnics LLC
  Contributed under the BSD 2-clause license.
*/

#include "logger.hpp"
#include "qunibus.h"
#include "qunibusadapter.hpp"
#include "rk611.hpp"

rk611_c::rk611_c() : qunibusdevice_c(){
    name.value = "RK611";
    type_name.value = "rk611_c";
    log_label = "rk6";

    // Base address, slot number, interrupt vector, BR level
    set_default_bus_params(0777440, 11, 0210, 5);

    // Unibus-visible registers
    register_count = 16;

    RKCS1.word = 0;
    UBR[0] = &(this->registers[0]);
    strcpy(UBR[0]->name,"RKCS1");
    UBR[0]->active_on_dati = false;
    UBR[0]->active_on_dato = true;
    UBR[0]->reset_value = 0000200; // Set RDY
    UBR[0]->writable_bits = 0113777;

    RKWC.word = 0;
    UBR[1] = &(this->registers[1]);
    strcpy(UBR[1]->name,"RKWC");
    UBR[1]->active_on_dati = false;
    UBR[1]->active_on_dato = true;
    UBR[1]->reset_value = 0;
    UBR[1]->writable_bits = 0177777;

    RKBA.dword = 0;
    UBR[2] = &(this->registers[2]);
    strcpy(UBR[2]->name,"RKBA");
    UBR[2]->active_on_dati = false;
    UBR[2]->active_on_dato = true;
    UBR[2]->reset_value = 0;
    UBR[2]->writable_bits = 0177776;

    RKDA.word = 0;
    UBR[3] = &(this->registers[3]);
    strcpy(UBR[3]->name,"RKDA");
    UBR[3]->active_on_dati = false;
    UBR[3]->active_on_dato = true;
    UBR[3]->reset_value = 0;
    UBR[3]->writable_bits = 0003437;

    RKCS2.word = 0;
    UBR[4] = &(this->registers[4]);
    strcpy(UBR[4]->name,"RKCS2");
    UBR[4]->active_on_dati = false;
    UBR[4]->active_on_dato = true;
    UBR[4]->reset_value = 0000100; // Set IR
    UBR[4]->writable_bits = 0000077;

    RKDS.word = 0;
    UBR[5] = &(this->registers[5]);
    strcpy(UBR[5]->name,"RKDS");
    UBR[5]->active_on_dati = false;
    UBR[5]->active_on_dato = true;
    UBR[5]->reset_value = 0;
    UBR[5]->writable_bits = 0;

    RKER.word = 0;
    UBR[6] = &(this->registers[6]);
    strcpy(UBR[6]->name,"RKER");
    UBR[6]->active_on_dati = false;
    UBR[6]->active_on_dato = true;
    UBR[6]->reset_value = 0;
    UBR[6]->writable_bits = 0;

    RKAS.word = 0;
    UBR[7] = &(this->registers[7]);
    strcpy(UBR[7]->name,"RKAS");
    UBR[7]->active_on_dati = false;
    UBR[7]->active_on_dato = true;
    UBR[7]->reset_value = 0;
    UBR[7]->writable_bits = 0000377;

    RKDC.word = 0;
    UBR[8] = &(this->registers[8]);
    strcpy(UBR[8]->name,"RKDC");
    UBR[8]->active_on_dati = false;
    UBR[8]->active_on_dato = true;
    UBR[8]->reset_value = 0;
    UBR[8]->writable_bits = 0001777;

    UBR[9] = &(this->registers[9]);
    strcpy(UBR[9]->name,"UNUSED");
    UBR[9]->active_on_dati = false;
    UBR[9]->active_on_dato = false;
    UBR[9]->reset_value = 0;
    UBR[9]->writable_bits = 0;

    RKDB.word = 0;
    UBR[10] = &(this->registers[10]);
    strcpy(UBR[10]->name,"RKDB");
    UBR[10]->active_on_dati = true;
    UBR[10]->active_on_dato = true;
    UBR[10]->reset_value = 0;
    UBR[10]->writable_bits = 0177777;

    RKMR1.word = 0;
    UBR[11] = &(this->registers[11]);
    strcpy(UBR[11]->name,"RKMR1");
    UBR[11]->active_on_dati = false;
    UBR[11]->active_on_dato = true;
    UBR[11]->reset_value = 0022000; // Set MEWD, ECCW
    UBR[11]->writable_bits = 0001777;

    RKECPS.word = 0;
    UBR[12] = &(this->registers[12]);
    strcpy(UBR[12]->name,"RKECPS");
    UBR[12]->active_on_dati = false;
    UBR[12]->active_on_dato = true;
    UBR[12]->reset_value = 0004066; // This value because RKCS1.CFMT is zero
    UBR[12]->writable_bits = 0;

    RKECPT.word = 0;
    UBR[13] = &(this->registers[13]);
    strcpy(UBR[13]->name,"RKECPT");
    UBR[13]->active_on_dati = false;
    UBR[13]->active_on_dato = true;
    UBR[13]->reset_value = 0;
    UBR[13]->writable_bits = 0;

    RKMR2.word = 0;
    UBR[14] = &(this->registers[14]);
    strcpy(UBR[14]->name,"RKMR2");
    UBR[14]->active_on_dati = false;
    UBR[14]->active_on_dato = true;
    UBR[14]->reset_value = 0;
    UBR[14]->writable_bits = 0;

    RKMR3.word = 0;
    UBR[15] = &(this->registers[15]);
    strcpy(UBR[15]->name,"RKMR3");
    UBR[15]->active_on_dati = false;
    UBR[15]->active_on_dato = true;
    UBR[15]->reset_value = 0;
    UBR[15]->writable_bits = 0;

    silo_head = silo_tail = 0;
    memset(&SILO,0,sizeof(SILO));
}

rk611_c::~rk611_c(){

}

void rk611_c::controller_clear(int init){
    // Cancel any pending interrupt
    qunibusadapter->cancel_INTR(intr_request);
    RKCS1.word = 0000200; // Set RDY
    // RKWC does not get cleared by INIT, etc.
    set_register_dati_value(UBR[1],RKWC.word,"update_RKWC");
    RKBA.dword = 0;
    RKDA.word = 0;
    RKCS2.word = 0000100; // Set IR
    RKDS.word = 0;
    RKER.word = 0;
    RKAS.word = 0;
    RKDC.word = 0;
    RKDB.word = 0;
    RKMR1.word = 0022000; // Set MEWD, ECCW
    RKECPS.word = 0004066;
    RKECPT.word = 0;
    RKMR2.word = 0;
    RKMR3.word = 0;
    // Also clobber the silo
    silo_level = silo_head = silo_tail = 0;
    memset(&SILO,0,sizeof(SILO));
    if(init == 0){
	set_register_dati_value(UBR[0],RKCS1.word,"update_RKCS1");
	set_register_dati_value(UBR[2],RKBA.dword,"update_RKBA");
	set_register_dati_value(UBR[3],RKDA.word,"update_RKDA");
	set_register_dati_value(UBR[4],RKCS2.word,"update_RKCS2");
	set_register_dati_value(UBR[5],RKDS.word,"update_RKDS");
	set_register_dati_value(UBR[6],RKER.word,"update_RKER");
	set_register_dati_value(UBR[7],RKAS.word,"update_RKAS");
	set_register_dati_value(UBR[8],RKDC.word,"update_RKDC");
	set_register_dati_value(UBR[10],RKDB.word,"update_RKDB");
	set_register_dati_value(UBR[11],RKMR1.word,"update_RKMR1");
	set_register_dati_value(UBR[12],RKECPS.word,"update_RKECPS");
	set_register_dati_value(UBR[13],RKECPT.word,"update_RKECPT");
	set_register_dati_value(UBR[14],RKMR2.word,"update_RKMR2");
	set_register_dati_value(UBR[15],RKMR3.word,"update_RKMR3");
    }
}

void rk611_c::subsystem_clear(int init){
    // Do the drives too
    controller_clear(init);
}

bool rk611_c::silo_push(uint16_t *data){
    pthread_mutex_lock(&silo_mutex);
    // Going to SILO. Is it full?
    if(silo_level < 0102){
	// No.
	SILO[silo_head].word = *data;
	// INFO("SILO push: %.6o -> head %d, tail %d, level %.3o",*data,silo_head,silo_tail,silo_level);
	silo_head++;
	if(silo_head > 0101){ silo_head = 0; }
	silo_level++;
	// First write raises OR
	if(silo_level == 0001){
	    RKCS2.OR = 1;
	    set_register_dati_value(UBR[4],RKCS2.word,"update_RKCS2");
	}
	if(silo_level == 0102){
	    RKCS2.IR = 0;
	    set_register_dati_value(UBR[4],RKCS2.word,"update_RKCS2");
	}
	pthread_mutex_unlock(&silo_mutex);
	return(true);
    }else{
	// INFO("SILO push: %.6o discarded, head %d, tail %d, level %.3o, SILO over full",*data,silo_head,silo_tail,silo_level);
	RKCS1.CERR = 1;
	set_register_dati_value(UBR[0],RKCS1.word,"update_RKCS1");
	if(RKCS2.IR != 0){
	    RKCS2.IR = 0;
	    // INFO("CS2.IR cleared");
	}
	RKCS2.DLT = 1;
	set_register_dati_value(UBR[4],RKCS2.word,"update_RKCS2");
	if(RKCS1.IE != 0){
	    // DLT causes interrupt
	    qunibusadapter->INTR(intr_request,NULL,0);
	}
	pthread_mutex_unlock(&silo_mutex);
	return(false);
    }
}

// This returns what the NEXT word will be if there is a DATI from RKDB
bool rk611_c::silo_pop(uint16_t *data){
    pthread_mutex_lock(&silo_mutex);
    // Any data?
    if(RKCS2.OR != 0){
	// Is this the last word?
	if(silo_level > 0){
	    // No.
	    silo_tail++;
	    if(silo_tail > 0101){ silo_tail = 0; }
	    silo_level--;
	    *data = SILO[silo_tail].word; // Next word comes from SILO tail
	    // INFO("SILO pop: %.6o <- tail %d, head %d, level %.3o",*data,silo_tail,silo_head,silo_level);
	    if(RKCS2.IR == 0){
		RKCS2.IR = 1; // Input Ready (data can be popped)
		set_register_dati_value(UBR[4],RKCS2.word,"update_RKCS2");
		// INFO("CS2.IR set");
	    }
	    if(silo_level == 0){
		// That was the last word
		RKCS2.OR = 0;
		set_register_dati_value(UBR[4],RKCS2.word,"update_RKCS2");
	    }
	}else{
	    FATAL("OR was not cleared even though SILO is empty?");
	}
	pthread_mutex_unlock(&silo_mutex);
	return(true);
    }else{
	// SILO was empty and continues to be empty
	*data = 0;
	RKCS1.CERR = 1;
	set_register_dati_value(UBR[0],RKCS1.word,"update_RKCS1");
	RKMR1.RD_GATE = 1;
	set_register_dati_value(UBR[11],RKMR1.word,"update_RKMR1");
	RKCS2.DLT = 1;
	set_register_dati_value(UBR[4],RKCS2.word,"update_RKCS2");
	// INFO("SILO pop: Empty silo, tail %d, head %d, level %.3o",silo_tail,silo_head,silo_level);
	if(RKCS1.IE != 0){ qunibusadapter->INTR(intr_request,NULL,0); }
	pthread_mutex_unlock(&silo_mutex);
	return(false);
    }
}


bool rk611_c::on_param_changed(parameter_c *param){
    if(param == &priority_slot){
	dma_request.set_priority_slot(priority_slot.new_value);
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

void rk611_c::on_after_register_access(qunibusdevice_register_t *device_reg, uint8_t unibus_control, DATO_ACCESS access){
    RKDB_REG wrval = {device_reg->active_dato_flipflops};
    switch(device_reg->index){

    case 0: // RKCS1
	switch(access){
	case DATO_WORD:
	    // if CCLR is set, do it.
	    if((wrval.word&0100000) != 0){
		controller_clear(0);
		return;
	    }else{
		// If CFMT being changed, update RKECPS
		if((wrval.word&0010000) != 0 && RKCS1.CFMT == 0){
		    // Going to 20-sector (18-bit) mode
		    RKECPS.word = 0005066;
		    set_register_dati_value(UBR[12],RKECPS.word,"update_RKECPS");
		}else{
		    if((wrval.word&0010000) == 0 && RKCS1.CFMT != 0){
			// Going to 22-sector (16-bit) mode
			RKECPS.word = 0004066;
			set_register_dati_value(UBR[12],RKECPS.word,"update_RKECPS");
		    }
		}
		// If writing IE and RDY bits, generate interrupt.
		if((wrval.word&0300) == 0300){
		    qunibusadapter->INTR(intr_request,NULL,0);
		}else{
		    // Is IE going up?
		    if((wrval.word&0100) != 0 && RKCS1.IE == 0){
			// Yes. Check for things that will cause an interrupt
			if(RKCS2.DLT != 0){
			    qunibusadapter->INTR(intr_request,NULL,0);
			}
		    }
		}
		// If GO is being set, do it.
		if((wrval.word&01) != 0 && RKCS1.GO == 0){

		}
		RKCS1.word &= 0164200; // Mask off writables
		RKCS1.word |= (wrval.word&0013577); // Write back
	    }
	    break;
	case DATO_BYTEL:
	    // If writing IE and RDY bits, generate interrupt.
	    if((wrval.word&0300) == 0300){
		qunibusadapter->INTR(intr_request,NULL,0);
	    }else{
		// Is IE going up?
		if((wrval.word&0100) != 0 && RKCS1.IE == 0){
		    // Yes. Check for things that will cause an interrupt
		    if(RKCS2.DLT != 0){
			qunibusadapter->INTR(intr_request,NULL,0);
		    }
		}
	    }
	    // If GO is being set, do it.
	    if((wrval.word&01) != 0 && RKCS1.GO == 0){

	    }
	    RKCS1.byte[0] &= 0200;
	    RKCS1.byte[0] |= (wrval.byte[0]&0177);
	    break;
	case DATO_BYTEH:
	    // if CCLR is set, do it.
	    if((wrval.byte[1]&0200) != 0){
		controller_clear(0);
		return;
	    }else{
		// If CFMT being changed, update RKECPS
		if((wrval.word&0020) != 0 && RKCS1.CFMT == 0){
		    // Going to 20-sector (18-bit) mode
		    RKECPS.word = 0005066;
		    set_register_dati_value(UBR[12],RKECPS.word,"update_RKECPS");
		}else{
		    if((wrval.word&0020) == 0 && RKCS1.CFMT != 0){
			// Going to 22-sector (16-bit) mode
			RKECPS.word = 0004066;
			set_register_dati_value(UBR[12],RKECPS.word,"update_RKECPS");
		    }
		}
		RKCS1.byte[1] &= 0350;
		RKCS1.byte[1] |= (wrval.byte[1]&0027);
	    }
	    break;
	}
	RKBA.ADH = RKCS1.BA; // Update high BA bits
	set_register_dati_value(UBR[0],RKCS1.word,"update_RKCS1");
	break;

    case 1: // RKWC
	switch(access){
	case DATO_WORD:
	    RKWC.word = wrval.word; break;
	case DATO_BYTEL:
	    RKWC.byte[0] = wrval.byte[0]; break;
	case DATO_BYTEH:
	    RKWC.byte[1] = wrval.byte[1]; break;
	}
	set_register_dati_value(UBR[1],RKWC.word,"update_RKWC");
	break;

    case 2: // RKBA
	switch(access){
	case DATO_WORD:
	    RKBA.word[0] = wrval.word; break;
	case DATO_BYTEL:
	    RKBA.byte[0] = wrval.byte[0]; break;
	case DATO_BYTEH:
	    RKBA.byte[1] = wrval.byte[1]; break;
	}
	set_register_dati_value(UBR[2],RKBA.word[0],"update_RKBA");
	break;

    case 3: // RKDA
	switch(access){
	case DATO_WORD:
	    RKDA.word = wrval.word; break;
	case DATO_BYTEL:
	    RKDA.byte[0] = wrval.byte[0]; break;
	case DATO_BYTEH:
	    RKDA.byte[1] = wrval.byte[1]; break;
	}
	set_register_dati_value(UBR[3],RKDA.word,"update_RKDA");
	break;

    case 4: // RKCS2
	switch(access){
	case DATO_WORD:
	case DATO_BYTEL:
	    // SCLR?
	    if((wrval.byte[0]&040) != 0){
		// Yes
		subsystem_clear(0);
	    }else{
		RKCS2.byte[0] &= 0300;
		RKCS2.byte[0] |= (wrval.byte[0]&037);
	    }
	    break;
	case DATO_BYTEH:
	    // Nothing to do!
	    break;
	}
	set_register_dati_value(UBR[4],RKCS2.word,"update_RKCS2");
	break;

    case 5: // RKDS (is read only)
	set_register_dati_value(UBR[5],RKDS.word,"update_RKDS");
	break;

    case 6: // RKER (is read only)
	set_register_dati_value(UBR[6],RKER.word,"update_RKER");
	break;

    case 7: // RKAS
	switch(access){
	case DATO_WORD:
	case DATO_BYTEL:
	    RKAS.byte[0] = wrval.byte[0];
	    break;
	case DATO_BYTEH:
	    // Nothing to do!
	    break;
	}
	set_register_dati_value(UBR[7],RKAS.word,"update_RKAS");
	break;

    case 8: // RKDC
	switch(access){
	case DATO_WORD:
	    RKDC.word = wrval.word; break;
	case DATO_BYTEL:
	    RKDC.byte[0] = wrval.byte[0]; break;
	case DATO_BYTEH:
	    RKDC.byte[1] = wrval.byte[1]; break;
	}
	set_register_dati_value(UBR[8],RKDC.word,"update_RKDC");
	break;

    case 9: // UNUSED
	// Shouldn't get here!
	break;

    case 10: // RKDB
	if(unibus_control == QUNIBUS_CYCLE_DATO || unibus_control == QUNIBUS_CYCLE_DATOB){
	    // Fetch the data and so on
	    RKDB_REG Buffer = {RKDB.word};
	    switch(access){
	    case DATO_WORD:
		Buffer.word = wrval.word; break;
	    case DATO_BYTEL:
		FATAL("DATO_BYTEL to RKDB?");
		Buffer.byte[0] = wrval.byte[0]; break;
	    case DATO_BYTEH:
		FATAL("DATO_BYTEH to RKDB?");
		Buffer.byte[1] = wrval.byte[1]; break;
	    }
	    silo_push(&Buffer.word);
	    RKDB.word = SILO[silo_tail].word; // Keep lowest word in RKDB
	}else{
	    // The lowest word was just taken off the silo, get the next (if any)
	    silo_pop(&RKDB.word);
	}
	set_register_dati_value(UBR[10],RKDB.word,"update_RKDB");
	break;

    case 11: // RKMR1
	switch(access){
	case DATO_WORD:
	    RKMR1.word &= 0176000;
	    RKMR1.word |= (wrval.word&01777);
	    break;
	case DATO_BYTEL:
	    RKMR1.byte[0] = wrval.byte[0];
	    break;
	case DATO_BYTEH:
	    RKMR1.byte[1] &= 0374;
	    RKDC.byte[1] |= (wrval.byte[1]&0003);
	    break;
	}
	set_register_dati_value(UBR[11],RKMR1.word,"update_RKMR1");
	break;

    case 12: // RKECPS (is read only)
	set_register_dati_value(UBR[12],RKECPS.word,"update_RKECPS");
	break;

    case 13: // RKECPT (is read only)
	set_register_dati_value(UBR[13],RKECPT.word,"update_RKECPT");
	break;

    case 14: // RKMR2 (is read only)
	set_register_dati_value(UBR[14],RKMR2.word,"update_RKMR2");
	break;

    case 15: // RKMR3 (is read only)
	set_register_dati_value(UBR[15],RKMR3.word,"update_RKMR3");
	break;

    }
}

void rk611_c::on_power_changed(signal_edge_enum aclo_edge, signal_edge_enum dclo_edge){
    if(aclo_edge == SIGNAL_EDGE_FALLING || dclo_edge == SIGNAL_EDGE_FALLING){
	subsystem_clear(0);
    }
}

void rk611_c::on_init_changed(void){
    subsystem_clear(1);
}

void rk611_c::worker(unsigned instance){
    UNUSED(instance);
}
