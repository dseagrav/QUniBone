/*
  dz11.cpp: DZ11/DZV11 UNIBUS/QBUS controller

  Copyright Bogodyne Metatechnics LLC
  Contributed under the BSD 2-clause license.

*/

#include <assert.h>
#include "logger.hpp"
#include "qunibus.h"
#include "qunibusadapter.hpp"
#include "dz11.hpp"

dz11_c::dz11_c(int _unit) : qunibusdevice_c(){
    // Configuration
    unit = _unit;
#if defined(UNIBUS)
    name.value = "DZ";
    log_label = "dz";
#elif defined(QBUS)
    name.value = "DZV";
    log_label = "dzv";
#endif
    type_name.value = "dz11_c";
    // Append unit number
    name.value = name.value+std::to_string(unit);
    log_label = log_label+std::to_string(unit);

    // Clobber all state
    CSR.word = 0;
    RBUF.word = 0;
    TCR.word = 0;
    MSR.word = 0;
    TDR.word = 0;

    silo_level = silo_head = silo_tail = 0;
    silo_alarm_level = 16;
    memset(&SILO_Data,0,sizeof(SILO_Data));
    memset(&SILO_Line,0,sizeof(SILO_Line));
    memset(&SILO_Overrun,0,sizeof(SILO_Overrun));

    // Base address, slot number, interrupt vector, BR level
    int default_csr = 0760100+(010*unit);
    int default_slot = 21+(2*unit);
    int default_vector = 0300+(010*unit);
    set_default_bus_params(default_csr, default_slot, default_vector, 5);

    // Unibus-visible registers
    register_count = 4;

    // Set them up
    UBR[0] = &(this->registers[0]);
    strcpy(UBR[0]->name,"CSR");
    UBR[0]->active_on_dati = false;
    UBR[0]->active_on_dato = true;
    UBR[0]->reset_value = 0;
    UBR[0]->writable_bits = 0050170;

    UBR[1] = &(this->registers[1]);
    strcpy(UBR[1]->name,"RBUF/LPR");
    UBR[1]->active_on_dati = true;
    UBR[1]->active_on_dato = true;
    UBR[1]->reset_value = 0;
    UBR[1]->writable_bits = 0017777;

    UBR[2] = &(this->registers[2]);
    strcpy(UBR[2]->name,"TCR");
    UBR[2]->active_on_dati = false;
    UBR[2]->active_on_dato = true;
    UBR[2]->reset_value = 0;
    UBR[2]->writable_bits = 0177777;

    UBR[3] = &(this->registers[3]);
    strcpy(UBR[3]->name,"MSR/TDR");
    UBR[3]->active_on_dati = false;
    UBR[3]->active_on_dato = true;
    UBR[3]->reset_value = 0;
    UBR[3]->writable_bits = 0177777;

    // Create lines
    int x = 0;
    uint8_t lb = 1;
    while(x < DZ11_LINES){
	SLU[x] = new dz11_slu_c(this);
	SLU[x]->line = x;
	SLU[x]->line_bit = lb;
	SLU[x]->name.value = name.value+"L"+std::to_string(x);
	SLU[x]->log_label = log_label+"l"+std::to_string(x);
	SLU[x]->type_name.value = "dz11_slu_c";
	x++;
	lb <<= 1;
    }
}

dz11_c::~dz11_c(){

}

dz11_slu_c::dz11_slu_c(dz11_c *controller){
    set_workers_count(2); // We have two threads
    ctl = controller;
    netcon = NULL;
    line = 0;
    line_rate = 0;
    line_mask = 0;
    tx_ready = 1;
    rx_ready = 0;
    LPR.word = 0;
}

dz11_slu_c::~dz11_slu_c(){

}

// Find and fill TLINE. Line priority is in descending order.
void dz11_c::find_tline(int tln,int log){
    // MSE doesn't affect this.
    // If TRDY is up, see if we can skip the search
    if(CSR.TRDY != 0){
	// Presently selected line still active?
	if((TCR.byte[0]&SLU[CSR.TLINE]->line_bit) != 0 && SLU[CSR.TLINE]->tx_ready != 0){
	    // Yes, bail
	    return;
	}else{
	    // No! Clear TRDY and bail if no lines left enabled.
	    if(TCR.byte[0] == 0){
		      // INFO("FIND TLINE: ALL LINES DISABLED");
		CSR.TRDY = 0;
		set_register_dati_value(UBR[0],CSR.word,"update_CSR");
		// Knock down interrupt if it is set.
		qunibusadapter->cancel_INTR(tx_intr_request);
		return;
	    }
	}
    }
    if(TCR.byte[0] != 0){
	int trdy_raised = 0;
#if defined(UNIBUS)
	int line = 7; // Highest priority
	uint8_t line_bit = 0200;
#elif defined(QBUS)
	int line = 3; // Highest priority
	uint8_t line_bit = 010;
#endif
	// if(log != 0){ INFO("FINDING TLINE..."); }
	while(line >= 0){
	    if((TCR.byte[0]&line_bit) != 0 && SLU[line]->tx_ready != 0){
		// We have a winner!
		// INFO("FOUND LINE %d",line);
		if(CSR.TRDY == 0){
		    // INFO("TRDY RAISED");
		    trdy_raised = 1;
		    CSR.TRDY = 1;
		}
		CSR.TLINE = line;
		break;
	    }
	    line--;
	    line_bit >>= 1;
	}
	if(trdy_raised != 0 && CSR.TIE != 0){
	    qunibusadapter->INTR(tx_intr_request,UBR[0],CSR.word);
	    return;
	}else{
	    if(CSR.TRDY == 0){
		// Nothing was ready!
		qunibusadapter->cancel_INTR(tx_intr_request);
	    }
	}
	set_register_dati_value(UBR[0],CSR.word,"update_CSR");
    }
}

bool dz11_c::silo_push(uint8_t *data,int *line){
    pthread_mutex_lock(&silo_mutex);
    if(silo_level < 65){
	SILO_Data[silo_head] = *data;
	SILO_Line[silo_head] = *line;
	SILO_Overrun[silo_head] = 0;
	/*
	INFO("SILO push: %.3o, line %d -> %d, lv %d",
	     SILO_Data[silo_head],SILO_Line[silo_head],silo_head,silo_level);
	*/
	silo_head++;
	if(silo_head >= 65){ silo_head = 0; }
	silo_level++;
	int update_csr = 0;
	if(silo_level >= silo_alarm_level && CSR.SA == 0){
	    CSR.SA = 1;
	    update_csr = 1;
	}
	if(CSR.RDONE == 0){
	    CSR.RDONE = 1;
	    update_csr = 1;
	}
	// Update RBUF
	RBUF.DATA_VALID = 1;
	RBUF.RLINE = SILO_Line[silo_tail];
	RBUF.OVRN = SILO_Overrun[silo_tail];
	RBUF.byte[0] = SILO_Data[silo_tail];
	set_register_dati_value(UBR[1],RBUF.word,"update_RBUF");
	// Silo Alarm Enabled?
	if(CSR.SAE != 0){
	    // Yes, intr if SA is set
	    if(CSR.SA != 0){
		qunibusadapter->INTR(rx_intr_request,UBR[0],CSR.word);
		update_csr = 0; // Cancel this
	    }
	}else{
	    // RX interrupt enabled?
	    if(CSR.RIE != 0){
		qunibusadapter->INTR(rx_intr_request,UBR[0],CSR.word);
		update_csr = 0; // Cancel this
	    }
	}
	if(update_csr != 0){
	    set_register_dati_value(UBR[0],CSR.word,"update_CSR");
	}
	pthread_mutex_unlock(&silo_mutex);
	return(true);
    }else{
	// Shitter's full. Back up and clobber the last position
	silo_head--;
	if(silo_head < 0){ silo_head = 64; }
	SILO_Data[silo_head] = *data;
	SILO_Line[silo_head] = *line;
	SILO_Overrun[silo_head] = 1;
	/*
	INFO("SILO push: %.3o, line %d -> %d, lv %d (OVERRUN)",
	     SILO_Data[silo_head],SILO_Line[silo_head],silo_head,silo_level);
	*/
	silo_head++;
	if(silo_head >= 65){ silo_head = 0; }
	set_register_dati_value(UBR[1],RBUF.word,"update_RBUF");
	pthread_mutex_unlock(&silo_mutex);
	return(false);
    }
}

// This will pop the NEXT character on the assumption we just took the lowest character from RBUF
bool dz11_c::silo_pop(){
    pthread_mutex_lock(&silo_mutex);
    if(silo_level > 0){
	// Subtract the character we just took.
	silo_tail++;
	if(silo_tail >= 65){ silo_tail = 0; }
	silo_level--;
	// More?
	if(silo_level > 0){
	    // Yes
	    if(CSR.RDONE == 0){
		CSR.RDONE = 1;
		set_register_dati_value(UBR[0],CSR.word,"update_CSR");
	    }
	    if(CSR.SAE == 0 && CSR.RIE != 0){
		qunibusadapter->INTR(rx_intr_request,NULL,0);
	    }
	    // Update RBUF
	    RBUF.DATA_VALID = 1;
	    RBUF.RLINE = SILO_Line[silo_tail];
	    RBUF.OVRN = SILO_Overrun[silo_tail];
	    RBUF.byte[0] = SILO_Data[silo_tail];
	    /*
	    INFO("SILO pop: %.3o, line %d -> %d, lv %d",
		 SILO_Data[silo_tail],SILO_Line[silo_tail],silo_tail,silo_level);
	    */
	    set_register_dati_value(UBR[1],RBUF.word,"update_RBUF");
	    pthread_mutex_unlock(&silo_mutex);
	    return(true);
	}else{
	    // We just took the last word
	    // INFO("No more words in buffer");
	}
    }
    // Nothing is left in the silo.
    // INFO("Pop zeroes");
    RBUF.DATA_VALID = 0;
    RBUF.RLINE = 0;
    RBUF.byte[0] = 0;
    set_register_dati_value(UBR[1],RBUF.word,"update_RBUF");
    if(CSR.RDONE != 0){
	CSR.RDONE = 0;
	set_register_dati_value(UBR[0],CSR.word,"update_CSR");
    }
    pthread_mutex_unlock(&silo_mutex);
    return(false);
}

bool dz11_c::receive_data_on_line(int line,uint8_t data){
    // Scanner and receiver enabled?
    if(CSR.MSE != 0 && SLU[line]->LPR.RX_ON != 0){
	// Yes, put onto silo
	silo_push(&data,&line);
	return(true);
    }else{
	if(CSR.MSE == 0){
	    INFO("MSE IS OFF");
	}
	if(SLU[line]->LPR.RX_ON == 0){
	    INFO("LINE %d RX IS OFF",line);
	}
	return(false);
    }
}

void dz11_c::reset(bool hard){
    int x = 0;
    while(x < DZ11_LINES){
	SLU[x]->on_init_changed();
	x++;
    }
    // and those things are expected to stick!
    if(hard){
	INFO("Hard Reset");
	TCR.word = 0;
    }else{
	INFO("Soft Reset");
	TCR.byte[0] = 0;
    }
    // Anything that might have called find_tline() should be defanged by now.
    CSR.word = 0; // Clear TIE, MAINT, MSE, PIE, TLINE, SAE, SA, and TRDY.
    // Blow away the silo
    silo_level = silo_head = silo_tail = 0;
    silo_alarm_level = 16;
    memset(&SILO_Data,0,sizeof(SILO_Data));
    memset(&SILO_Line,0,sizeof(SILO_Line));
    memset(&SILO_Overrun,0,sizeof(SILO_Overrun));
    // Cancel any pending interrupts
    qunibusadapter->cancel_INTR(rx_intr_request);
    qunibusadapter->cancel_INTR(tx_intr_request);
    // MSR is not modified
    RBUF.word = 0;
    TDR.word = 0;
    set_register_dati_value(UBR[1],RBUF.word,"update_RBUF");
    set_register_dati_value(UBR[2],TCR.word,"update_TCR");
    set_register_dati_value(UBR[3],MSR.word,"update_MSR");
    set_register_dati_value(UBR[0],CSR.word,"update_CSR"); // Update this last to clear the reset flag
}

bool dz11_c::on_param_changed(parameter_c *param){
    if(param == &priority_slot){
	rx_intr_request.set_priority_slot(priority_slot.new_value);
	tx_intr_request.set_priority_slot(priority_slot.new_value+1);
    }
    if(param == &intr_level){
	rx_intr_request.set_level(intr_level.new_value);
	tx_intr_request.set_level(intr_level.new_value);
    }
    if(param == &intr_vector){
	rx_intr_request.set_vector(intr_vector.new_value);
	tx_intr_request.set_vector(intr_vector.new_value+4);
    }
    // If enabling or disabling, apply to all of our ports too.
    if(param == &enabled){
	int x = 0;
	while(x < DZ11_LINES){
	    SLU[x]->enabled.set(enabled.new_value);
	    x++;
	}
    }
    return qunibusdevice_c::on_param_changed(param); // Pass through
}

bool dz11_slu_c::on_param_changed(parameter_c *param){
    return(device_c::on_param_changed(param));
}

void dz11_slu_c::lpr_write(uint16_t val){
    LPR.word = val;
    // Recompute delay value. Start with microseconds per bit
    int bit_time;
    switch(LPR.FREQ){
    case 0: // 50
	bit_time = 20000;
	break;
    case 1: // 75
	bit_time = 13333;
	break;
    case 2: // 110
	bit_time = 9091;
	break;
    case 3: // 134.5
	bit_time = 7435;
	break;
    case 4: // 150
	bit_time = 6667;
	break;
    case 5: // 300
	bit_time = 3333;
	break;
    case 6: // 600
	bit_time = 1667;
	break;
    case 7: // 1200
	bit_time = 833;
	break;
    case 8: // 1800
	bit_time = 555; // NO JAM
	break;
    case 9: // 2000
	bit_time = 500;
	break;
    case 10: // 2400
	bit_time = 417;
	break;
    case 11: // 3600
	bit_time = 278;
	break;
    case 12: // 4800
	bit_time = 208;
	break;
    case 13: // 7200
	bit_time = 139;
	break;
    case 14: // 9600
	bit_time = 104;
	break;
    case 15: // Undocumented 19200
	bit_time = 52;
	break;
    }
    // Multiply by bit width
    line_rate = bit_time*(5+(LPR.CHAR_LENGTH));
    // Add a parity bit if one is present
    if(LPR.PARITY_ENABLE != 0){ line_rate += bit_time; }
    // Now add start and stop bits. Are we in 5-bit mode?
    if(LPR.CHAR_LENGTH == 0){
	line_rate += (bit_time*2); // One start bit, one stop bit
	if(LPR.STOP_CODE != 0){
	    line_rate += (bit_time/2); // Extra half stop bit
	}
    }else{
	if(LPR.STOP_CODE != 0){
	    line_rate += (bit_time*2); // One start bit, one stop bit
	}else{
	    line_rate += (bit_time*3); // One start bit, two stop bits
	}
    }
    // Generate line mask
    line_mask = ((1<<(5+LPR.CHAR_LENGTH))-1);
}

void dz11_slu_c::transmit(uint8_t data){
    if(tx_ready != 0){
	pthread_mutex_lock(&tx_worker_sync_mutex);
	pthread_cond_signal(&tx_worker_sync_cond);
	tx_ready = 0;
	transmit_data = (data&line_mask);
	// INFO("Transmit data handed to worker");
	pthread_mutex_unlock(&tx_worker_sync_mutex);
    }else{
	INFO("Dropped transmit character?");
    }
}

std::string dz11_slu_c::get_name(){
    return(name.value);
}

bool dz11_slu_c::recv_data_from_nc(uint8_t data){
    // Take the time even if the silo is busy etc
    timeout_c rx_timeout;
    rx_timeout.wait_us(line_rate);
    return(ctl->receive_data_on_line(line,(data&line_mask)));
}

bool dz11_slu_c::recv_break_from_nc(){
    // TBI
    return(true);
}

void dz11_c::on_after_register_access(qunibusdevice_register_t *device_reg, uint8_t unibus_control, DATO_ACCESS access){
    // What did we get?
    switch(device_reg->index){

    case 0: // CSR
	switch(access){

	case DATO_WORD:
	    // Mask off writables
	    // INFO("CSR WT: %.6o",UBR[0]->active_dato_flipflops);
	    CSR.word &= 0123600;
	    CSR.word |= (UBR[0]->active_dato_flipflops&0050170);
	    if(CSR.CLR != 0){
		// Clobber everything except the clear bit
		CSR.word = 0000020;
		// Punt worker to perform clear
		pthread_mutex_lock(&on_after_register_access_mutex);
		pthread_cond_signal(&on_after_register_access_cond);
		pthread_mutex_unlock(&on_after_register_access_mutex);
	    }
	    break;

	case DATO_BYTEL:
	    INFO("CSR L WT: %.3o",UBR[0]->active_dato_flipflops&0377);
	    CSR.byte[0] &= 0200;
	    CSR.byte[0] |= (UBR[0]->active_dato_flipflops&0170);
	    if(CSR.CLR != 0){
		// Clobber everything except the clear bit
		CSR.word = 0000020;
		// Punt worker to perform clear
		pthread_mutex_lock(&on_after_register_access_mutex);
		pthread_cond_signal(&on_after_register_access_cond);
		pthread_mutex_unlock(&on_after_register_access_mutex);
	    }
	    break;

	case DATO_BYTEH:
	    INFO("CSR H WT: %.3o",(UBR[0]->active_dato_flipflops>>8)&0377);
	    CSR.byte[1] &= 0247;
	    CSR.word |= (UBR[0]->active_dato_flipflops&0050000);
	    break;
	}
	if(CSR.CLR == 0){
	    // Generate interrupts if needed.
	    // NB: It looks like the Unibone won't actually allow the receive interrupt to override
	    // the transmit interrupt, so we have to work around it.
	    // INFO("INTCHK: TIE %d, RIE %d, TRDY %d, RDONE %d",CSR.TIE,CSR.RIE,CSR.TRDY,CSR.RDONE);
	    if(CSR.SAE != 0){
		if(CSR.SA != 0){
		    qunibusadapter->INTR(rx_intr_request,NULL,0);
		}
	    }else{
		if(CSR.RIE != 0 && CSR.RDONE != 0){
		    qunibusadapter->INTR(rx_intr_request,NULL,0);
		}else{
		    if(CSR.RIE == 0){
			qunibusadapter->cancel_INTR(rx_intr_request);
		    }
		}
	    }
	    if(CSR.TIE != 0 && CSR.TRDY != 0){
		qunibusadapter->INTR(tx_intr_request,NULL,0);
	    }else{
		// TIE going down?
		if(CSR.TIE == 0){
		    qunibusadapter->cancel_INTR(tx_intr_request);
		}
	    }
	}
	set_register_dati_value(UBR[0],CSR.word,"update_CSR");
	break;

    case 1: // RBUF/LPR
	if(unibus_control == QUNIBUS_CYCLE_DATO || unibus_control == QUNIBUS_CYCLE_DATOB){
	    // LPR doesn't support byte accesses
	    DZ11_LPR New_LPR = {UBR[1]->active_dato_flipflops};
	    switch(access){

	    case DATO_WORD:
		// INFO("LPR WT: %.6o",UBR[1]->active_dato_flipflops);
		SLU[New_LPR.LINE]->lpr_write(New_LPR.word);
		break;

	    case DATO_BYTEL:
		FATAL("Handle low byte access to LPR");
		break;

	    case DATO_BYTEH:
		FATAL("Handle high byte access to LPR");
		break;
	    }
	    set_register_dati_value(UBR[1],RBUF.word,"update_RBUF"); // Change back to RBUF
	    break;
	}else{
	    // RBUF was read, clear RDONE and SA if set and get the next state.
	    // Silo should have the next character within one microsecond.
	    if(CSR.SA != 0){
		CSR.SA = 0;
		set_register_dati_value(UBR[0],CSR.word,"update_CSR");
	    }
	    // Clear RX INT if it's set
	    qunibusadapter->cancel_INTR(rx_intr_request);
	    silo_pop(); // RDONE will be handled here and re-raised if necessary
	    silo_alarm_level = silo_level+16;
	    // set_register_dati_value(UBR[1],RBUF.word,"update_RBUF");
	}
	break;

    case 2: // TCR
    {
	DZ11_TCR New_TCR = {UBR[2]->active_dato_flipflops};
	switch(access){

	case DATO_WORD:
	    // INFO("TCR WT: %.6o",New_TCR.word);
	    break;

	case DATO_BYTEL:
	    // Bring high byte back in
	    New_TCR.byte[1] = TCR.byte[1];
	    INFO("TCR L WT: %.3o",New_TCR.byte[0]);
	    break;

	case DATO_BYTEH:
	    // Bring low byte back in
	    New_TCR.byte[0] = TCR.byte[0];
	    INFO("TCR H WT: %.3o",New_TCR.byte[1]);
	    break;
	}
	TCR.word = New_TCR.word;
	find_tline(0,0);
	set_register_dati_value(UBR[2],TCR.word,"update_TCR");
    }
    break;

    case 3: // TDR
	CSR.TRDY = 0; // This clears TRDY
	qunibusadapter->cancel_INTR(tx_intr_request);
	switch(access){

	case DATO_WORD:
	    // Transmit byte and/or send break
	    if((UBR[3]->active_dato_flipflops&0177400) != 0){
		INFO("TDR WT: %.6o (Line %d)",UBR[3]->active_dato_flipflops,CSR.TLINE);
	    }
	    SLU[CSR.TLINE]->transmit((UBR[3]->active_dato_flipflops&0377));
	    break;

	case DATO_BYTEL:
	    // Transmit byte
	    // INFO("TDR L WT: %.3o (Line %d)",(UBR[3]->active_dato_flipflops&0377),CSR.TLINE);
	    SLU[CSR.TLINE]->transmit((UBR[3]->active_dato_flipflops&0377));
	    break;

	case DATO_BYTEH:
	    // Send break
	    if((UBR[3]->active_dato_flipflops&0177400) != 0){
		INFO("TDR H WT: %.3o",(UBR[3]->active_dato_flipflops>>8)&0377);
	    }
	    break;
	}
	set_register_dati_value(UBR[3],MSR.word,"update_MSR"); // Change back to RBUF
	find_tline(0,0);
	break;

    }
}

std::string dz11_c::get_name(){
    return(name.value);
}

int dz11_c::find_free_line(){
    int x = 0;
    while(x < DZ11_LINES){
	if(SLU[x]->netcon == NULL){ return(x); }
	x++;
    }
    return(-1); // None found
}

netcom_line_c* dz11_c::get_slu(int ln){
    if(ln >= 0 && ln < DZ11_LINES){
	return(SLU[ln]);
    }
    return(NULL);
}

bool dz11_c::seize_line(int ln,netcon_c *conn){
    if(ln >= 0 && ln < DZ11_LINES && SLU[ln]->netcon == NULL){
	SLU[ln]->netcon = conn;
	// Activate modem control
	MSR.CO |= SLU[ln]->line_bit;
	return(true);
    }
    return(false);
}

void dz11_c::release_line(int ln){
    if(ln >= 0 && ln < DZ11_LINES && SLU[ln]->netcon != NULL){
	SLU[ln]->netcon = NULL;
	// Release modem control
	MSR.CO &= ~SLU[ln]->line_bit;
    }
}

void dz11_c::on_power_changed(signal_edge_enum aclo_edge, signal_edge_enum dclo_edge){
    if(aclo_edge == SIGNAL_EDGE_FALLING || dclo_edge == SIGNAL_EDGE_FALLING){
	reset(true);
    }
}

void dz11_slu_c::on_power_changed(signal_edge_enum aclo_edge, signal_edge_enum dclo_edge){
    if(aclo_edge == SIGNAL_EDGE_FALLING || dclo_edge == SIGNAL_EDGE_FALLING){
	LPR.word = 0;
	line_rate = 0;
	tx_ready = 1;
	rx_ready = 0;
    }
}

void dz11_c::on_init_changed(void){
    if(init_asserted){
	reset(true);
    }
}

void dz11_slu_c::on_init_changed(void){
    LPR.word = 0;
    line_rate = 0;
    tx_ready = 1;
    rx_ready = 0;
}

void dz11_c::worker(unsigned instance){
    // This one is just to manage the DZ11 interface itself. We don't want to do I/O here.
    UNUSED(instance);
    assert(!pthread_mutex_lock(&on_after_register_access_mutex));
    worker_init_realtime_priority(rt_device);
    while(!workers_terminate){
	int wait_result = pthread_cond_wait(&on_after_register_access_cond,&on_after_register_access_mutex);
	if(wait_result != 0){
	    ERROR("dz11_c::worker(): pthread_cond_wait returned %d: %s",wait_result,strerror(wait_result));
	    continue;
	}else{
	    if(CSR.CLR != 0){
		reset(false);
		INFO("Reset complete, TCR %.6o",TCR.word);
	    }else{
		// Do stuff
	    }
	}
    }
    assert(!pthread_mutex_unlock(&on_after_register_access_mutex)); // If we can't flush the register mutex, etc etc.
}

void dz11_slu_c::worker(unsigned instance){
    if(instance == 0){
	// This one is used for receiving data from the port
	timeout_c rx_timeout;
	// worker_init_realtime_priority(rt_device);
	while(!workers_terminate){
	    rx_timeout.wait_ms(100);
	}
    }else{
	// This one is used for transmitting data to the port
	transmit_worker();
    }
}

void dz11_slu_c::transmit_worker(void){
    // This one is used for transmitting data to the port
    timeout_c tx_timeout;
    assert(!pthread_mutex_lock(&tx_worker_sync_mutex));
    worker_init_realtime_priority(rt_device);
    while(!workers_terminate){
	int wait_result = pthread_cond_wait(&tx_worker_sync_cond,&tx_worker_sync_mutex);
	if(wait_result != 0){
	    ERROR("dz11_slu_c::transmit_worker(): pthread_cond_wait returned %d: %s",wait_result,strerror(wait_result));
	    continue;
	}else{
	    // We have a character to transmit. Are we in loopback?
	    int log_enable = 0;
	    if(ctl->CSR.MAINT == 0){
		// No. Are we connected to netcon?
		if(netcon != NULL){
		    log_enable = 1;
		    // Yes, send it
		    /*
		    if(transmit_data >= 040 && transmit_data < 0200){
			INFO("TX TO NET: %.3o: %c",transmit_data,transmit_data);
		    }else{
			INFO("TX TO NET: %.3o",transmit_data);
		    }
		    */
		    netcon->transmit_data(transmit_data);
		}
		tx_timeout.wait_us(line_rate);
	    }else{
		// Yes
		tx_timeout.wait_us(line_rate);
		if(!ctl->receive_data_on_line(line,transmit_data)){
		    // Character stays in receive buffer?
		    INFO("Unable to receive in loopback?");
		}
	    }
	    tx_ready = 1;
	    // INFO("Transmit done");
	    ctl->find_tline(line,log_enable);
	}
    }
    assert(!pthread_mutex_unlock(&tx_worker_sync_mutex));
}
