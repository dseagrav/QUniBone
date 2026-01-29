/*
  rk611.cpp: RK611 disk controller

  Copyright Bogodyne Metatechnics LLC
  Contributed under the BSD 2-clause license.
*/

#include "logger.hpp"
#include "qunibus.h"
#include "qunibusadapter.hpp"
#include "rk611.hpp"
#include "parity.hpp"

rk611_c::rk611_c() : storagecontroller_c(){
    name.value = "RK611";
    type_name.value = "rk611_c";
    log_label = "rk611";

    worker_active = 0;
    worker_abort = 0;
    worker_unit = 0;

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

    drive_selected = false;
    selected_drive = 0;

    // Create a drive
    drv[0] = new rk067_c(this,0);
    storagedrives.push_back(drv[0]);

}

rk611_c::~rk611_c(){
    int x = 0;
    while(x < 8){
	if(drv[x] != NULL){
	    delete drv[x];
	    drv[x] = NULL;
	}
	x++;
    }
}

void rk611_c::controller_clear(int init){
    // Cancel any pending interrupt
    qunibusadapter->cancel_INTR(intr_request);
    RKCS1.word &= 0040000; // Preserve DI
    RKCS1.word |= 0000200; // Set RDY
    // RKWC does not get cleared by INIT, etc.
    set_register_dati_value(UBR[1],RKWC.word,"update_RKWC");
    RKBA.dword = 0;
    RKDA.word = 0;
    RKCS2.word = 0000100; // Set IR
    RKDS.word = 0;
    RKER.word = 0;
    RKAS.word = 0;
    // Regenerate AS if not bus init
    if(init == false){
	int x = 0;
	uint8_t new_bit = 1;
	while(x < 8){
	    if(drv[x] != NULL && drv[x]->Status_A0.DRIVE_STATUS_CHANGE != 0){
		RKAS.ATN |= new_bit;
	    }
	    new_bit <<= 1;
	    x++;
	}
    }
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
    int x = 0;
    while(x < 8){
	if(drv[x] != NULL){ drv[x]->on_init_changed(init); }
	x++;
    }
    // If DI is set, clear it
    RKCS1.DI = 0;
    // Carry on
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

void rk611_c::handle_mr1_write(int hi, int lo, uint16_t nval){
    RKMR1_REG New_MR1 = {nval};
    // Is DMD up and staying up?
    if((hi == 0 || New_MR1.DMD != 0) && RKMR1.DMD != 0){
	// Look for MCLK transitions
	if(hi != 0){
	    if(New_MR1.MCLK != RKMR1.MCLK){
		if(New_MR1.MCLK == 0){
		    // MCLK UP
		    INFO("DMD: MCLK UP");
		}else{
		    // MCLK DN
		    INFO("DMD: MCLK DN");
		}
	    }
	}
    }
}

void rk611_c::handle_go_set(){
    // Our caller will execute set_register_dati_value for RKCS1, so we don't have to.
    // RDY gets cleared
    RKCS1.RDY = 0;
    // SVAL too?
    RKDS.SVAL = 0;
    set_register_dati_value(UBR[5],RKDS.word,"update_RKDS");
    // Build MESSAGE A, MESSAGE B, RKMR2, and RKMR3
    RKMR2.word = 0;
    RKMR3.word = 0;
    uint16_t Message_A = 0;
    uint16_t Message_B = 0;
    // INFO("GO set, unit %d, CS1 %.6o, Function %d",RKCS2.DS,RKCS1.word,RKCS1.F);
    // If CS1.F != Select Drive, clear MS
    if(RKCS1.F != RK067_SELECT_DRIVE){ RKMR1.MS = 0; }
    // Dispatch on function
    switch(RKCS1.F){

    case RK067_SELECT_DRIVE:
    {
	RK067_MSG_AH MSG_A = {0};
	// If we changed drives, deselect the old one
	if(drive_selected != false){
	    if(selected_drive != RKCS2.DS && drv[selected_drive]->is_selected()){
		drv[selected_drive]->deselect();
		// Ensure clobbered
		if(RKDS.SVAL != 0){
		    RKDS.SVAL = 0;
		    set_register_dati_value(UBR[5],RKDS.word,"update_RKDS");
		}
		drive_selected = false;
	    }
	}
	// Now set up
	MSG_A.DRIVE_ADDR = RKCS2.DS;
	MSG_A.FORMAT = RKCS1.CFMT;
	MSG_A.SELECT_RELEASE_CMD = RKCS2.RLS;
	// Generate track address
	switch(RKDA.TA){

	case 0:
	    MSG_A.TRACK_ADDRESS = 0; break;
	case 1:
	    MSG_A.TRACK_ADDRESS = 1; break;
	case 2:
	    MSG_A.TRACK_ADDRESS = 2; break;
	case 3:
	    MSG_A.TRACK_ADDRESS = 3; break;

	default:
	    FATAL("Unhandled RDKA.TA in select: %.2o",RKDA.TA);
	}
	if(RKMR1.PAT == 0){
	    MSG_A.PARITY = get_odd_parity_16b(MSG_A.word);
	}else{
	    MSG_A.PARITY = get_even_parity_16b(MSG_A.word);
	}
	RKMR2.DS = RKCS2.DS;
	RKMR2.RELEASE = RKCS2.RLS;
	RKMR2.FORMAT = RKCS1.CFMT;
	RKMR2.DRIVE_CLEAR = RKDS.DDT; // Drive type from DS?
	RKMR2.HEAD_SELECT = RKDA.TA;
	RKMR2.RESERVED = ((RKDA.word&0002000) != 0); // TA bit 3
	RKMR3.word = RKMR1.MS; // Copy message code
	Message_A = MSG_A.word;
	Message_B = 0;
    }
    break;

    case RK067_PACK_ACKNOWLEDGE:
    {
	RK067_MSG_AH MSG_A = {0};
	MSG_A.DRIVE_ADDR = RKCS2.DS;
	MSG_A.PACK_ACKNOWLEDGE_CMD = 1;
	MSG_A.FORMAT = RKCS1.CFMT;
	if(RKMR1.PAT == 0){
	    MSG_A.PARITY = get_odd_parity_16b(MSG_A.word);
	}else{
	    MSG_A.PARITY = get_even_parity_16b(MSG_A.word);
	}
	RKMR2.DS = RKCS2.DS;
	RKMR2.SET_VOLUME_VALID = 1;
	RKMR2.FORMAT = RKCS1.CFMT;
	// RKMR3 is zero
	Message_A = MSG_A.word;
	Message_B = 0;
    }
    break;

    case RK067_DRIVE_CLEAR:
    {
	RK067_MSG_AH MSG_A = {0};
	MSG_A.DRIVE_ADDR = RKCS2.DS;
	MSG_A.DRIVE_CLEAR_CMD = 1;
	MSG_A.FORMAT = RKCS1.CFMT;
	if(RKMR1.PAT == 0){
	    MSG_A.PARITY = get_odd_parity_16b(MSG_A.word);
	}else{
	    MSG_A.PARITY = get_even_parity_16b(MSG_A.word);
	}
	RKMR2.DS = RKCS2.DS;
	RKMR2.DRIVE_CLEAR = 1;
	RKMR2.FORMAT = RKCS1.CFMT;
	// RKMR3 is zero
	Message_A = MSG_A.word;
	Message_B = 0;
    }
    break;

    case RK067_UNLOAD:
    {
	RK067_MSG_AH MSG_A = {0};
	MSG_A.DRIVE_ADDR = RKCS2.DS;
	MSG_A.UNLOAD_CMD = 1;
	MSG_A.FORMAT = RKCS1.CFMT;
	if(RKMR1.PAT == 0){
	    MSG_A.PARITY = get_odd_parity_16b(MSG_A.word);
	}else{
	    MSG_A.PARITY = get_even_parity_16b(MSG_A.word);
	}
	RKMR2.DS = RKCS2.DS;
	RKMR2.SET_MEDIUM_OFFLINE = 1;
	RKMR2.FORMAT = RKCS1.CFMT;
	// RKMR3 is zero
	Message_A = MSG_A.word;
	Message_B = 0;
    }
    break;

    case RK067_START_SPINDLE:
    {
	RK067_MSG_AH MSG_A = {0};
	MSG_A.DRIVE_ADDR = RKCS2.DS;
	MSG_A.TRACK_ADDRESS = 1; // This is the command bit
	MSG_A.FORMAT = RKCS1.CFMT;
	if(RKMR1.PAT == 0){
	    MSG_A.PARITY = get_odd_parity_16b(MSG_A.word);
	}else{
	    MSG_A.PARITY = get_even_parity_16b(MSG_A.word);
	}
	RKMR2.DS = RKCS2.DS;
	RKMR2.START_SPINDLE_CMD = 1;
	RKMR2.FORMAT = RKCS1.CFMT;
	// RKMR3 is zero
	Message_A = MSG_A.word;
	Message_B = 0;
    }
    break;

    case RK067_RECALIBRATE:
    {
	RK067_MSG_AH MSG_A = {0};
	MSG_A.DRIVE_ADDR = RKCS2.DS;
	MSG_A.RECALIBRATE_CMD = 1;
	MSG_A.FORMAT = RKCS1.CFMT;
	if(RKMR1.PAT == 0){
	    MSG_A.PARITY = get_odd_parity_16b(MSG_A.word);
	}else{
	    MSG_A.PARITY = get_even_parity_16b(MSG_A.word);
	}
	RKMR2.DS = RKCS2.DS;
	RKMR2.RECAL_CMD = 1;
	RKMR2.FORMAT = RKCS1.CFMT;
	// RKMR3 is zero
	Message_A = MSG_A.word;
	Message_B = 0;
    }
    break;

    case RK067_OFFSET:
    {
	RK067_MSG_AH MSG_A = {0};
	MSG_A.DRIVE_ADDR = RKCS2.DS;
	// There is no command bit for the offset command
	MSG_A.FORMAT = RKCS1.CFMT;
	if(RKMR1.PAT == 0){
	    MSG_A.PARITY = get_odd_parity_16b(MSG_A.word);
	}else{
	    MSG_A.PARITY = get_even_parity_16b(MSG_A.word);
	}
	RKMR2.DS = RKCS2.DS;
	RKMR2.FORMAT = RKCS1.CFMT;
	RK067_MSG_B2 MSG_B = {0};
	if((RKAS.OF&0200) != 0){
	    // NEGATIVE
	    MSG_B.CYLINDER = ((~RKAS.OF)&077); // Bits are inverted
	    MSG_B.CYLINDER |= 0200; // Sign bit
	}else{
	    MSG_B.CYLINDER = ((~(RKAS.OF&077))&0177); // Bits are inverted
	    MSG_B.CYLINDER |= 0200; // Sign bit
	}
	MSG_B.word |= 0010000; // This offset bit is forced set
	RKMR3.word = MSG_B.word;
	if(RKMR1.PAT == 0){
	    MSG_B.PARITY = get_odd_parity_16b(MSG_B.word);
	}else{
	    MSG_B.PARITY = get_even_parity_16b(MSG_B.word);
	}
	Message_A = MSG_A.word;
	Message_B = MSG_B.word;
    }
    break;

    case RK067_SEEK:
    {
	RK067_MSG_AH MSG_A = {0};
	MSG_A.DRIVE_ADDR = RKCS2.DS;
	MSG_A.SEEK_CMD = 1;
	MSG_A.FORMAT = RKCS1.CFMT;
	switch(RKDA.TA){

	case 0:
	    MSG_A.TRACK_ADDRESS = 0; break;
	case 1:
	    MSG_A.TRACK_ADDRESS = 1; break;
	case 2:
	    MSG_A.TRACK_ADDRESS = 2; break;
	case 3:
	    MSG_A.TRACK_ADDRESS = 3; break;

	default:
	    FATAL("Unhandled RDKA.TA in seek: %.2o",RKDA.TA);
	}
	if(RKMR1.PAT == 0){
	    MSG_A.PARITY = get_odd_parity_16b(MSG_A.word);
	}else{
	    MSG_A.PARITY = get_even_parity_16b(MSG_A.word);
	}
	RKMR2.DS = RKCS2.DS;
	RKMR2.HEAD_SELECT = RKDA.TA;
	RKMR2.SEEK_CMD = 1;
	RKMR2.FORMAT = RKCS1.CFMT;
	RK067_MSG_B2 MSG_B = {0};
	if(RKCS1.CDT == 0){
	    MSG_B.CYLINDER = (RKDC.DC&0777); // MASK HIGH BIT FOR RK06
	}else{
	    MSG_B.CYLINDER = RKDC.DC;
	}
	RKMR3.word = MSG_B.word;
	if(RKMR1.PAT == 0){
	    MSG_B.PARITY = get_odd_parity_16b(MSG_B.word);
	}else{
	    MSG_B.PARITY = get_even_parity_16b(MSG_B.word);
	}
	Message_A = MSG_A.word;
	Message_B = MSG_B.word;
    }
    break;

    case RK067_READ_DATA:
    case RK067_WRITE_DATA:
    case RK067_WRITE_CHECK:
    {
	// These actually turns into two commands, the first of which is a seek.
	// The second doesn't carry any useful information, so we will send just the seek.
	RK067_MSG_AH MSG_A = {0};
	MSG_A.DRIVE_ADDR = RKCS2.DS;
	MSG_A.SEEK_CMD = 1;
	MSG_A.FORMAT = RKCS1.CFMT;
	switch(RKDA.TA){

	case 0:
	    MSG_A.TRACK_ADDRESS = 0; break;
	case 1:
	    MSG_A.TRACK_ADDRESS = 1; break;
	case 2:
	    MSG_A.TRACK_ADDRESS = 2; break;
	case 3:
	    MSG_A.TRACK_ADDRESS = 3; break;

	default:
	    FATAL("Unhandled RKDA.TA in read/write data command: %.2o",RKDA.TA);
	}
	if(RKMR1.PAT == 0){
	    MSG_A.PARITY = get_odd_parity_16b(MSG_A.word);
	}else{
	    MSG_A.PARITY = get_even_parity_16b(MSG_A.word);
	}
	RKMR2.DS = RKCS2.DS;
	RKMR2.HEAD_SELECT = RKDA.TA;
	RKMR2.SEEK_CMD = 1;
	RKMR2.FORMAT = RKCS1.CFMT;
	RK067_MSG_B2 MSG_B = {0};
	if(RKCS1.CDT == 0){
	    MSG_B.CYLINDER = (RKDC.DC&0777); // MASK HIGH BIT FOR RK06
	}else{
	    MSG_B.CYLINDER = RKDC.DC;
	}
	RKMR3.word = MSG_B.word;
	if(RKMR1.PAT == 0){
	    MSG_B.PARITY = get_odd_parity_16b(MSG_B.word);
	}else{
	    MSG_B.PARITY = get_even_parity_16b(MSG_B.word);
	}
	Message_A = MSG_A.word;
	Message_B = MSG_B.word;
    }
    break;

    default:
	FATAL("handle_go_set(): Unhandled function %d at MESSAGE A build time",RKCS1.F);
	break;
    }
    set_register_dati_value(UBR[14],RKMR2.word,"update_RKMR2");
    set_register_dati_value(UBR[15],RKMR3.word,"update_RKMR3");
    if(RKMR1.DMD != 0){
	// DIAGNOSTIC MODE STUFF HERE
	INFO("Diagnostic mode, returning");
	return;
    }
    // If the drive exists, we get Select Acknowledge about 300 nanoseconds after sending the command.
    RKCS1.CERR = 0;
    if(drv[RKCS2.DS] == NULL || !drv[RKCS2.DS]->enabled.value){
	// Light NED and bail
	RKCS2.NED = 1;
	RKER.DTYE = 0; // Since we can't see the type
	set_register_dati_value(UBR[4],RKCS2.word,"update_RKCS2");
	RKCS1.CERR = 1;
    }else{
	// Check type
	RKER.DTYE = (drv[RKCS2.DS]->Status_A0.DRIVE_TYPE != RKCS1.CDT);
	if(RKER.DTYE != 0){
	    // We get a status message from the drive if we do this. MSEL 0 means SVAL should go up
	    if(RKMR1.MS == 0){ RKDS.SVAL = 1; }
	    status_update(RKCS2.DS,true);
	    RKCS1.CERR = 1;
	}else{
	    RKER.DTYE = 0;
	    // Also check format?
	}
    }
    set_register_dati_value(UBR[6],RKER.word,"update_RKER");
    // Are we bailing?
    if(RKCS1.CERR != 0){
	RKCS1.GO = 0;
	RKCS1.RDY = 1;
	if(RKCS1.IE != 0){ qunibusadapter->INTR(intr_request,NULL,0); }
	// INFO("Bailed: RKCS1 %.6o RKCS2 %.6o RKER %.6o",RKCS1.word,RKCS2.word,RKER.word);
	return;
    }
    // If we are still here, we are good to go. Clean up any stray error bits
    RKCS2.NED = 0;
    set_register_dati_value(UBR[4],RKCS2.word,"update_RKCS2");
    // To have a hope of going fast enough, data transfers have to bypass the drive.
    if(RKCS1.F > RK067_SEEK){
	unsigned offset = (33792*RKDC.DC)+(11264*RKDA.TA)+(512*RKDA.SA);
	// Perform implied seek
	drv[RKCS2.DS]->Status_B2.CYLINDER = RKDC.DC;
	drv[RKCS2.DS]->Status_B3.SECTOR_COUNT = RKDA.SA;
	drv[RKCS2.DS]->Status_B3.HEAD_ADDRESS = (1<<RKDA.TA);
	// set_register_dati_value(UBR[8],RKDC.word,"update_RKDC");
	// set_register_dati_value(UBR[3],RKDA.word,"update_RKDA");
	// Punt the worker
	if(worker_active == 0){
	    pthread_mutex_lock(&on_after_register_access_mutex);
	    pthread_cond_signal(&on_after_register_access_cond);
	    worker_abort = 0;
	    worker_unit = RKCS2.DS;
	    worker_offset = offset;
	    worker_active = 1;
	    worker_function = RKCS1.F;
	    pthread_mutex_unlock(&on_after_register_access_mutex);
	}else{
	    FATAL("Worker is busy?");
	}
    }else{
	drv[RKCS2.DS]->handle_command(RKCS1.F,Message_A,Message_B,RKMR1.PAT);
    }
    // Then read back status
    if(RKCS1.F == RK067_SELECT_DRIVE){
	// Update selection
	drive_selected = true;
	selected_drive = RKCS2.DS;
	switch(RKMR1.MS){

	case 0:
	    RKMR2.word = drv[RKCS2.DS]->Status_A0.word;
	    RKMR3.word = drv[RKCS2.DS]->Status_B0.word;
	    if(RKCS2.RLS == 0){
		RKDS.SVAL = 1;
		set_register_dati_value(UBR[5],RKDS.word,"update_RKDS");
	    }
	    break;

	case 1:
	    RKMR2.word = drv[RKCS2.DS]->Status_A1.word;
	    RKMR3.word = drv[RKCS2.DS]->Status_B1.word;
	    break;

	case 2:
	    RKMR2.word = drv[RKCS2.DS]->Status_A2.word;
	    RKMR3.word = drv[RKCS2.DS]->Status_B2.word;
	    break;

	case 3:
	    RKMR2.word = drv[RKCS2.DS]->Status_A3.word;
	    RKMR3.word = drv[RKCS2.DS]->Status_B3.word;
	    break;

	default:
	    FATAL("Invalid message select %d?",RKMR1.MS);
	    break;
	}
    }else{
	RKMR2.word = drv[RKCS2.DS]->Status_A0.word;
	RKMR3.word = drv[RKCS2.DS]->Status_B0.word;
	if(RKCS2.RLS == 0){
	    RKDS.SVAL = 1;
	    set_register_dati_value(UBR[5],RKDS.word,"update_RKDS");
	}
    }
    if(RKMR1.PAT == 0){
	// Manual says odd, but ZR6H says even?
	if(get_even_parity_16b(RKMR2.word) != 0){ RKMR2.word |= 0100000; }
	if(get_even_parity_16b(RKMR3.word) != 0){ RKMR3.word |= 0100000; }
    }else{
	if(get_odd_parity_16b(RKMR2.word) != 0){ RKMR2.word |= 0100000; }
	if(get_odd_parity_16b(RKMR3.word) != 0){ RKMR3.word |= 0100000; }
	// Then throw errors
	RKMR3.word |= 01200; // FAULT, C-D PARITY ERROR
	RKCS1.DCT_PAR = 1;
	RKCS1.CERR = 1;
	RKCS1.GO = 0;
	RKCS1.RDY = 1;
	set_register_dati_value(UBR[14],RKMR2.word,"update_RKMR2");
	set_register_dati_value(UBR[15],RKMR3.word,"update_RKMR3");
	if(RKCS1.IE != 0){ qunibusadapter->INTR(intr_request,NULL,0); }
	// INFO("Bailed: RKCS1 %.6o RKCS2 %.6o RKER %.6o",RKCS1.word,RKCS2.word,RKER.word);
	return;
	// FATAL("Diagnostic parity reverse implementation needed?");
    }
    set_register_dati_value(UBR[14],RKMR2.word,"update_RKMR2");
    set_register_dati_value(UBR[15],RKMR3.word,"update_RKMR3");
    // INFO("RKMR2 %.6o RKMR3 %.6o",RKMR2.word,RKMR3.word);
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
		int go_set;
		if((wrval.word&01) != 0 && RKCS1.GO == 0){
		    go_set = 1;
		}else{
		    go_set = 0;
		}
		RKCS1.word &= 0164200; // Mask off writables
		RKCS1.word |= (wrval.word&0013577); // Write back
		if(go_set != 0){
		    handle_go_set();
		}
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
	    int go_set;
	    // If GO is being set, do it.
	    if((wrval.word&01) != 0 && RKCS1.GO == 0){
		go_set = 1;
	    }else{
		go_set = 0;
	    }
	    RKCS1.byte[0] &= 0200;
	    RKCS1.byte[0] |= (wrval.byte[0]&0177);
	    if(go_set != 0){
		    handle_go_set();
	    }
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
	RKDS.SVAL = 0; // Writing here clears SVAL
	set_register_dati_value(UBR[5],RKDS.word,"update_RKDS");
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
	    handle_mr1_write(1,1,wrval.word);
	    RKMR1.word &= 0176000;
	    RKMR1.word |= (wrval.word&01777);
	    break;
	case DATO_BYTEL:
	    handle_mr1_write(0,1,wrval.word);
	    RKMR1.byte[0] = wrval.byte[0];
	    break;
	case DATO_BYTEH:
	    handle_mr1_write(1,0,wrval.word);
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
	subsystem_clear(1);
    }
}

void rk611_c::on_init_changed(void){
    subsystem_clear(1);
}

void rk611_c::on_drive_status_changed(storagedrive_c *drive){

}

void rk611_c::status_update(int unit,bool forced){
    // Receive status
    if(drv[unit] != NULL && (drv[unit]->is_selected() || forced == true)){
	// Update RKER
	RKER.SKI = (drv[unit]->Status_B0.INVALID_ADDRESS != 0 ||
		    drv[unit]->Status_B1.SEEK_NO_MOTION != 0 ||
		    drv[unit]->Status_B1.LIMIT_DETECT_ON_SEEK != 0 ||
		    drv[unit]->Status_B1.SERVO_UNSAFE != 0);
	RKER.NXF = drv[unit]->Status_B0.NONEXECUTABLE_FUNCTION;
	RKER.DRPAR = drv[unit]->Status_B0.C_D_PARITY_ERROR;
	RKER.FMTE = (drv[unit]->Status_A0.FORMAT != RKCS1.CFMT);
	// RKER.DTYE is set by controller, not drive.
	RKER.ECH = 0; // Set by controller, not drive
	RKER.BSE = 0; // Same
	RKER.HRVC = 0; // Same
	RKER.COE = 0;
	RKER.IDAE = (drv[unit]->Status_B0.INVALID_ADDRESS != 0 ||
		     (drv[unit]->Status_A0.DRIVE_TYPE == 0 && RKDC.DC > 411));
	RKER.WLE = 0; // Handle later
	RKER.DTE = 0;
	RKER.OPI = 0; // Should equal SKI?
	RKER.UNS = (drv[unit]->Status_B1.SECTOR_ERROR != 0 ||
		    drv[unit]->Status_B1.WRITE_CURRENT_NO_GATE != 0 ||
		    drv[unit]->Status_B1.WRITE_GATE_NO_TX != 0 ||
		    drv[unit]->Status_B1.HEAD_FAULT != 0 ||
		    drv[unit]->Status_B1.MULTIPLE_HEAD_SELECT != 0 ||
		    drv[unit]->Status_B1.INDEX_ERROR != 0 ||
		    drv[unit]->Status_B1.TRIBIT_ERROR != 0 ||
		    drv[unit]->Status_B1.SERVO_SIGNAL_ERROR != 0);
	RKER.DCK = 0;
	set_register_dati_value(UBR[6],RKER.word,"update_RKER");
	// Update RKDS
	RKDS.DRA = drv[unit]->Status_A0.DRIVE_AVAIL;
	RKDS.OFST = drv[unit]->Status_A0.OFFSET_ON;
	RKDS.ACLO = 0;
	RKDS.SPLS = drv[unit]->Status_B0.SPEED_LOSS;
	RKDS.DROT = 0; // Later
	RKDS.VV = drv[unit]->Status_A0.VOLUME_VALID;
	RKDS.DRDY = drv[unit]->Status_A0.DRIVE_READY;
	RKDS.DDT = drv[unit]->Status_A0.DRIVE_TYPE;
	RKDS.WRL = drv[unit]->Status_A0.WRITE_LOCK;
	RKDS.PIP = drv[unit]->Status_A0.POSIT_IN_PROGR;
	RKDS.CDA = drv[unit]->Status_A0.DRIVE_STATUS_CHANGE;
	// RKDS.SVAL = 1;
	set_register_dati_value(UBR[5],RKDS.word,"update_RKDS");
	if(forced == false){
	    if(RKER.word != 0 || RKDS.SPLS != 0){
		RKCS1.CERR = 1;
		set_register_dati_value(UBR[0],RKCS1.word,"update_RKCS1");
		if(RKCS1.IE != 0){ qunibusadapter->INTR(intr_request,NULL,0); }
	    }
	    // INFO("Status update from %d: RKCS1 %.6o RKDS %.6o RKER %.6o",unit,RKCS1.word,RKDS.word,RKER.word);
	}
    }
}

void rk611_c::raise_attention(int unit){
    uint8_t new_bit = 1;
    new_bit <<= unit;
    RKAS.ATN |= new_bit;
    set_register_dati_value(UBR[7],RKAS.word,"update_RKAS");
    RKCS1.DI = 1;
    set_register_dati_value(UBR[0],RKCS1.word,"update_RKCS1");
    if(RKCS1.IE != 0){ qunibusadapter->INTR(intr_request,NULL,0); }
}

void rk611_c::clear_attention(int unit){
    uint8_t old_bit = 1;
    old_bit <<= unit;
    RKAS.ATN &= ~old_bit;
    set_register_dati_value(UBR[7],RKAS.word,"update_RKAS");
    if(RKAS.ATN == 0 && RKCS1.DI != 0){
	RKCS1.DI = 0;
	set_register_dati_value(UBR[0],RKCS1.word,"update_RKCS1");
    }
}

/*
bool rk611_c::sector_strobe(int unit,int sector){
    // Await our desired sector
    // if(sector == RKDA.SA){
	// Winner!
	unsigned offset = (33792*RKDC.DC)+(11264*RKDA.TA)+(512*RKDA.SA);
	// INFO("Hit desired disk address: C:H:S %d:%d:%d, image offset %d",RKDC.DC,RKDA.TA,RKDA.SA,offset);
	// Reading or writing?
	switch(RKCS1.F){

	case RK067_READ_DATA:
	case RK067_WRITE_CHECK:
	case RK067_WRITE_DATA:
	    // Punt the worker to perform the DMA operation. If the worker is busy, wait.
	    if(worker_active == 0){
		pthread_mutex_lock(&on_after_register_access_mutex);
		pthread_cond_signal(&on_after_register_access_cond);
		worker_abort = 0;
		worker_unit = unit;
		worker_offset = offset;
		worker_active = 1;
		worker_function = RKCS1.F;
		pthread_mutex_unlock(&on_after_register_access_mutex);
	    }else{
		ERROR("Worker is busy?");
		return(false);
	    }
	    return(true);
	    break;

	default:
	    FATAL("sector_strobe(): Unhandled function %d",RKCS1.F);
	    break;
	}
//    }
    return(false);
}
*/

void rk611_c::op_complete_strobe(bool winner){
    RKCS1.GO = 0;
    if(!winner){
	// Lose, lose!
	RKCS1.CERR = 1;
    }
    RKCS1.RDY = 1;
    set_register_dati_value(UBR[0],RKCS1.word,"update_RKCS1");
    if(RKCS1.IE != 0){ qunibusadapter->INTR(intr_request,NULL,0); }
}

// CALL FROM THE WORKER ONLY!
void rk611_c::advance_io_op(){
    // Advance sector, track, cylinder in that order as they overflow
    RKDA.SA++;
    if(RKDA.SA > 21){
	RKDA.SA = 0;
	RKDA.TA++;
	if(RKDA.TA > 2){
	    RKDA.TA = 0;
	    RKDC.DC++;
	    drv[worker_unit]->Status_B2.CYLINDER = RKDC.DC;
	    set_register_dati_value(UBR[8],RKDC.word,"update_RKDC");
	}
	drv[worker_unit]->Status_B3.HEAD_ADDRESS = (1<<RKDA.TA);
    }
    drv[RKCS2.DS]->Status_B3.SECTOR_COUNT = RKDA.SA;
    set_register_dati_value(UBR[3],RKDA.word,"update_RKDA");
    worker_offset += 512;
}

void rk611_c::worker(unsigned instance){
    UNUSED(instance);
    timeout_c pacing_delay;
    assert(!pthread_mutex_lock(&on_after_register_access_mutex));
    worker_init_realtime_priority(rt_device);
    while(!workers_terminate){
	int wait_result = pthread_cond_wait(&on_after_register_access_cond,&on_after_register_access_mutex);
	if(wait_result != 0){
	    ERROR("rk611_c::worker(): pthread_cond_wait returned %d: %s",wait_result,strerror(wait_result));
	    continue;
	}else{
	    while(worker_active != 0 && worker_abort == 0 && !workers_terminate){
		// What are we doing?
		int Buffer_Index = 0;
		switch(worker_function){

		case RK067_READ_DATA:
		{
		    // Fill sector from disk
		    drv[worker_unit]->image_read(Sector,worker_offset,512);
		    if(RKCS2.BAI == 0){
			// Write sector to memory
			int word_count = 0200000-RKWC.word;
			if(word_count > 256){ word_count = 256; }
			qunibusadapter->DMA(dma_request,true,QUNIBUS_CYCLE_DATO,RKBA.ADR,(uint16_t *)&Sector,word_count);
			if(!dma_request.success){
			    FATAL("DMA lossage");
			}
			// Increment address
			RKBA.ADR += (word_count*2);
			set_register_dati_value(UBR[2],RKBA.word[0],"update_RKBA");
			RKCS1.BA = RKBA.ADH;
			set_register_dati_value(UBR[0],RKCS1.word,"update_RKCS1");
			// Increment WC
			RKWC.word += word_count;
			set_register_dati_value(UBR[1],RKWC.word,"update_RKWC");
		    }else{
			FATAL("Need handling for BAI on read data");
		    }
		    advance_io_op();
		    if(RKWC.word == 0 || RKCS2.WCE != 0){
			// INFO("Read operation completed");
			op_complete_strobe(true);
			worker_active = 0;
		    }
		}
		break;

		case RK067_WRITE_CHECK:
		{
		    // Fill sector from disk
		    drv[worker_unit]->image_read(Sector,worker_offset,512);
		    // Fill buffer from memory
		    RKDB_REG Buffer[256];
		    if(RKCS2.BAI == 0){
			int word_count = 0200000-RKWC.word;
			if(word_count > 256){ word_count = 256; }
			// Get the data
			qunibusadapter->DMA(dma_request,true,QUNIBUS_CYCLE_DATI,RKBA.ADR,(uint16_t *)&Buffer,word_count);
			if(!dma_request.success){
			    FATAL("DMA lossage");
			}
		    }else{
			FATAL("Need handling for BAI on write check");
		    }
		    // Perform comparison and update registers
		    while(Buffer_Index < 512 && RKWC.word != 0 && RKCS2.WCE == 0){
			if(Sector[Buffer_Index] != Buffer[Buffer_Index/2].byte[0] ||
			   Sector[Buffer_Index+1] != Buffer[Buffer_Index/2].byte[1]){
			    uint16_t error_word = Sector[Buffer_Index+1];
			    error_word <<= 8;
			    error_word |= Sector[Buffer_Index];
			    INFO("Compared word at index %d to address %.6o, WC %.6o: Not Equal (%.6o vs %.6o)",
				 Buffer_Index,RKBA.ADR,RKWC.word,Buffer[Buffer_Index/2].word,error_word);
			    RKCS2.WCE = 1;
			    RKCS1.CERR = 1;
			    set_register_dati_value(UBR[4],RKCS2.word,"update_RKCS2");
			}
			if(RKCS2.BAI == 0){
			    // Increment address
			    RKBA.ADR += 2;
			    RKCS1.BA = RKBA.ADH;
			}
			RKWC.word++; // Increment WC
			Buffer_Index += 2;
		    }
		    // Update registers to reflect final results
		    set_register_dati_value(UBR[2],RKBA.word[0],"update_RKBA");
		    set_register_dati_value(UBR[0],RKCS1.word,"update_RKCS1");
		    set_register_dati_value(UBR[1],RKWC.word,"update_RKWC");
		    advance_io_op();
		    if(RKWC.word == 0 || RKCS2.WCE != 0){
			// INFO("Write Check operation completed");
			op_complete_strobe(true);
			worker_active = 0;
		    }
		}
		break;

		case RK067_WRITE_DATA:
		    // BAI?
		    if(RKCS2.BAI == 0){
			// No. Generate word count
			int word_count = 0200000-RKWC.word;
			if(word_count > 256){ word_count = 256; }
			// Get the data
			qunibusadapter->DMA(dma_request,true,QUNIBUS_CYCLE_DATI,RKBA.ADR,(uint16_t *)&Sector,word_count);
			if(!dma_request.success){
			    FATAL("DMA lossage");
			}
			// Increment address
			RKBA.ADR += (word_count*2);
			set_register_dati_value(UBR[2],RKBA.word[0],"update_RKBA");
			RKCS1.BA = RKBA.ADH;
			set_register_dati_value(UBR[0],RKCS1.word,"update_RKCS1");
			// Increment WC
			RKWC.word += word_count;
			set_register_dati_value(UBR[1],RKWC.word,"update_RKWC");
			// Did we fill the sector?
			if(word_count < 256){
			    // No, fill the balance
			    Buffer_Index = word_count*2;
			    while(Buffer_Index < 512){
				Sector[Buffer_Index] = 0;
				Buffer_Index++;
			    }
			}
		    }else{
			FATAL("Need handling for BAI on write data");
		    }
		    // Commit write
		    drv[worker_unit]->image_write(Sector,worker_offset,512);
		    advance_io_op();
		    if(RKWC.word == 0){
			// INFO("Write operation completed");
			op_complete_strobe(true);
			worker_active = 0;
		    }
		    break;

		default:
		    FATAL("worker(): Unhandled function %d",RKCS1.F);
		}
	    }
	    worker_abort = 0;
	}
    }
    assert(!pthread_mutex_unlock(&on_after_register_access_mutex)); // If we can't flush the register mutex, etc etc.
}
