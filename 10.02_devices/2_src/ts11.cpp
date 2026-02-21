/*
  ts11.cpp: TS11/TSV05 UNIBUS/QBUS controller

  Copyright Bogodyne Metatechnics LLC
  Contributed under the BSD 2-clause license.

  NB: storagedrive does not look ready for handling tape images yet, so tapedrive for now.

*/

#include <assert.h>

#include "logger.hpp"
#include "qunibus.h"
#include "ts11.hpp"
#include "ts11_drive.hpp"

// TSSR: Termination Classes
#define TSSR_TC_NORMAL 00
#define TSSR_TC_ATTENTION 01
#define TSSR_TC_STATUS_ALERT 02
#define TSSR_TC_FUNCTION_REJECT 03
#define TSSR_TC_RECOVERABLE_ERROR 04
#define TSSR_TC_RECOVERABLE_ERROR_NO_MOVEMENT 05
#define TSSR_TC_UNRECOVERABLE_ERROR 06
#define TSSR_TC_FATAL_ERROR 07

// TSSR: Fatal Error classes
#define TSSR_FC_UDIAG_ERROR 00
#define TSSR_FC_SEQ_CROM_PTY 01
#define TSSR_FC_UPX_CROM_PTY 02
#define TSSR_FC_POWER_FAILED 03

// Message Packet Header - Message Codes
#define MSG_PKT_CLASS_END 020
#define MSG_PKT_CLASS_FAIL 021
#define MSG_PKT_CLASS_ERROR 022
#define MSG_PKT_CLASS_ATTN 023

// Message Packet Header - Class Codes - Failure Messages
#define MSG_PKT_FAIL_CLASS_SBPE 00
#define MSG_PKT_FAIL_CLASS_OTHER 01
#define MSG_PKT_FAIL_CLASS_WLE_OR_NXF 02
#define MSG_PKT_FAIL_CLASS_MICRODIAG_FAIL 03

// Message Packet Header - Class Codes - Attention Messages
#define MSG_PKT_ATTN_CLASS_ON_OR_OFF_LINE 00
#define MSG_PKT_ATTN_CLASS_MICRODIAG_FAIL 01

ts11_c::ts11_c() : tapecontroller_c(){
    // Configuration
#if defined(UNIBUS)
    name.value = "TS11";
#elif defined(QBUS)
    name.value = "TSV05";
#endif
    type_name.value = "ts11_c";
    log_label = "ts";

    // Base address, slot number, interrupt vector, BR level
    set_default_bus_params(0772520, 29, 0224, 5);

    // Clobber private registers and state as needed.
    TSBA.dword = 0; // TSBA is not cleared by power up, should we make it contain trash?
    TSSR.word = 0; // Initial TSSR
    Command.Buffer_Address.dword = 0;
    // Command.Parameter_Word = 0; (Now part of above)
    Command_Step = 0;
    RBPCR = 0;
    XSTAT0.word = 0;
    XSTAT1.word = 0;
    XSTAT2.word = 0;
    XSTAT3.word = 0;
    memset(&Characteristics,0,sizeof(TS11_WCHR_DATA));
    memset(&Command,0,sizeof(TS11_COMMAND_PACKET));

    // Set up the worker's state
    pending_ssi = 0;
    worker_abort = 0;
    worker_active = 0;

    // Now define registers. There are two of them visible to the Unibus, the rest are private.
    register_count = 2;

    TSR0_reg = &(this->registers[0]);
    strcpy(TSR0_reg->name, "TSBA/TSDB");
    TSR0_reg->active_on_dati = false;
    TSR0_reg->active_on_dato = true; // Write means new command
    TSR0_reg->reset_value = 0;
    TSR0_reg->writable_bits = 0177777;

    TSSR_reg = &(this->registers[1]);
    strcpy(TSSR_reg->name, "TSSR");
    TSSR_reg->active_on_dati = false;
    TSSR_reg->active_on_dato = true; // A write here indicates a subsystem initialize
    TSSR_reg->reset_value = 0;
    TSSR_reg->writable_bits = 0;     // Read-Write, but not directly.

    // Create the attached transport. There can be only one.
    // and we assume it is either a TS04 (for unibus) or TS05 (for qbus)
    // TS04 is specific to the TS11, and TS05 is a rebadged Cipher F880.
    drivecount = 1;
    transport = new ts11_drive_c(this);
#if defined(UNIBUS)
    transport->name.value = "TS04";
#elif defined(QBUS)
    transport->name.value = "TS05";
#endif
    transport->type_name.value = transport->name.value;
    transport->log_label = "tsd";
    tapedrives.push_back(transport);
}

ts11_c::~ts11_c(){

}

void ts11_c::subsystem_init(){
    // Bail if already resetting
    if(pending_ssi != 0){ return; }
    // Cancel interrupt if any
    qunibusadapter->cancel_INTR(intr_request);
    // Is the worker idle?
    if(worker_active == 0){
	// No, start it
	// INFO("SSI started");
	pthread_mutex_lock(&on_after_register_access_mutex);
	pthread_cond_signal(&on_after_register_access_cond);
	pending_ssi = 2;
	pthread_mutex_unlock(&on_after_register_access_mutex);
    }else{
	// Yes, cancel whatever is going on.
	// INFO("SSI queued pending command cancellation");
	pending_ssi = 2;
	worker_abort = 1;
    }
}

bool ts11_c::on_param_changed(parameter_c *param){
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

void ts11_c::on_after_register_access(qunibusdevice_register_t *device_reg, uint8_t unibus_control, DATO_ACCESS access){
    UNUSED(unibus_control);
    // What did we get?
    switch(device_reg->index){

    case 0: // TSDB (On Write)
	// This resets the interrupt
	qunibusadapter->cancel_INTR(intr_request);
	// Is this DATO or DATOB?
	switch(access){

	case DATO_WORD:
	    // Is the worker busy? Either worker_active will be up or SSR will be down.
	    if(worker_active != 0 || TSSR.SSR == 0){
		// Yes, light the RMR bit and ignore this?
		TSSR.RMR = 1;
		set_register_dati_value(TSSR_reg,TSSR.word,"update_TSSR");
		ERROR("Command sent while we're busy? WA %d, SSR %d; RMR set; New TSSR %.6o",
		     worker_active,TSSR.SSR,TSSR.word);
	    }else{
		// Load TSBA from TSDB. We get 16 bits to put into an 18-bit register.
		TSBA.ADX = (TSR0_reg->active_dato_flipflops&03);
		TSBA.ADL = (TSR0_reg->active_dato_flipflops&0177774);
		// Write the low 16 bits of TSBA back into TSDB
		set_register_dati_value(TSR0_reg,TSBA.word[0],"update_TSR0");
		// Update TSSR: Clear SPE, UPE, RMR, NXM, and SSR
		// RSTS bootloader says we clear SC too?
		TSSR.SC = TSSR.SPE = TSSR.UPE = TSSR.RMR = TSSR.NXM = TSSR.SSR = 0;
		// Update the address extension bits.
		TSSR.ADX = TSBA.ADX;
		set_register_dati_value(TSSR_reg,TSSR.word,"update_TSSR");
		// INFO("Starting TSSR %.6o",TSSR.word);
		if(pending_ssi == 0){
		    // Sharif don't like it
		    pthread_mutex_lock(&on_after_register_access_mutex);
		    // Punt the worker thread
		    // INFO("New TSBA %.6o",TSBA.ADR);
		    pthread_cond_signal(&on_after_register_access_cond);
		    worker_abort = 0;
		    worker_active = 1;
		    pthread_mutex_unlock(&on_after_register_access_mutex);
		}else{
		    INFO("New TSBA %.6o, command queued due to SSI",TSBA.ADR);
		    worker_active = 1;
		}
		return;
	    }
	    break;

	case DATO_BYTEH:
	    // Internal Data Wrap Test:
	    INFO("Internal Data Wrap Test: %.6o",TSR0_reg->active_dato_flipflops);
	    {
		uint8_t wrap_byte = (TSR0_reg->active_dato_flipflops&0177400)>>8;
		TSBA.byte[0] = TSBA.byte[1] = wrap_byte; // Bits 0-7 and 8-15 get Unibus bits 8-15
		TSBA.ADX = (wrap_byte&03); // Bits 16-17 get Unibus bits 8 and 9
		INFO("Output value is        : %.6o",TSBA.ADR);
		// Write the low 16 bits of TSBA back into TSDB
		set_register_dati_value(TSR0_reg,TSBA.word[0],"update_TSR0");
		// Update TSSR
		TSSR.ADX = TSBA.ADX;
		// If SSR is clear, we set RMR, otherwise we clear SSR
		if(TSSR.SSR == 0){
		    TSSR.RMR = 1;
		}else{
		    TSSR.SSR = 0;
		}
		set_register_dati_value(TSSR_reg,TSSR.word,"update_TSSR");
		// Further status updates from the drive are inhibited until a subsystem initialize.
	    }
	    break;

	case DATO_BYTEL:
	    // External Data Wrap Test
	    INFO("External Data Wrap Test: %.6o",TSR0_reg->active_dato_flipflops);
	    TSBA.ADR = TSR0_reg->active_dato_flipflops; // Load the entire bus, high bits indeterminate
	    INFO("Output value is        : %.6o",TSBA.ADR);
	    // Write the low 16 bits of TSBA back into TSDB
	    set_register_dati_value(TSR0_reg,TSBA.word[0],"update_TSR0");
	    // Load TSSR from TSBA and rewrite the high address bits in TSSR
	    TSSR.word = TSBA.word[0];
	    TSSR.ADX = TSBA.ADX;
	    // This doesn't use SSR etc?
	    set_register_dati_value(TSSR_reg,TSSR.word,"update_TSSR");
	    break;

	default:
	    FATAL("Unhandled access %d for TSR0 in on_after_register_access",access);
	}
	break;

    case 1: // TSSR
	// Writing here causes a subsystem initialize. Reading here cancels the interrupt?
	if(pending_ssi == 0){
	    // INFO("TSSR written");
	    TSSR.word = 0;
	    set_register_dati_value(TSSR_reg,TSSR.word,"update_TSSR");
	    subsystem_init();
	}else{
	    INFO("TSSR write disregarded, SSI already in progress");
	}
	break;
    }
}

void ts11_c::on_power_changed(signal_edge_enum aclo_edge, signal_edge_enum dclo_edge){
    // Tell the drive if AC power is going away
    tapecontroller_c::on_power_changed(aclo_edge, dclo_edge);
    // Unclobber TSR0, but allow TSSR to get cleared until and do subsystem initialize completes.
    set_register_dati_value(TSR0_reg,TSBA.word[0],"update_TSR0");
    // TSSR = 0;
    // set_register_dati_value(TSSR_reg,TSSR,"update_TSSR");
    if(aclo_edge == SIGNAL_EDGE_FALLING){
	// INFO("AC LO cleared");
	subsystem_init();
    }
    // DC LO should block drive movement
    UNUSED(dclo_edge);
}

void ts11_c::on_init_changed(void){
    if(init_asserted){
	// XXDP tests that the TSSR_SSR bit will be cleared immediately after sending INIT.
	// We accomplished that by having the entire register clobbered. Reflect that clobberance here.
	TSSR.word = 0;
	set_register_dati_value(TSSR_reg,TSSR.word,"update_TSSR");
	// Then un-clobber TSR0.
	// set_register_dati_value(TSR0_reg,(uint16_t)(TSBA&0177777),"update_TSR0");
	// Completion of subsystem initialize will unclobber TSSR
    }else{
	// INFO("INIT cleared");
	// INIT dropped.
	// Ensure registers remain unclobbered
	// set_register_dati_value(TSR0_reg,(uint16_t)(TSBA&0177777),"update_TSR0");
	// set_register_dati_value(TSSR_reg,TSSR,"update_TSSR");
	// Do subsystem initialization
	// Destroy the command register too
	subsystem_init();
    }
}

void ts11_c::on_drive_status_changed(tapedrive_c *drive){
    // Update TSSR here if needed
    UNUSED(drive); // There's only one
    update_xstat();
    if(XSTAT0.ONL != 0){
	INFO("DSC: Transport is online");
	if(XSTAT0.WLK != 0){ INFO("DSC: Transport is write-locked"); }
	if(XSTAT0.VCK != 0){ INFO("DSC: Transport indicates volume check"); }
	if(XSTAT0.BOT != 0){ INFO("DSC: Transport is at beginning of tape"); }
	if(XSTAT0.EOT != 0){ INFO("DSC: Transport is at end of tape"); }
    }else{
	INFO("DSC: Transport is offline");
    }
}

// Update bits in XSTAT etc. from the drive.
void ts11_c::update_xstat(){
    XSTAT0.ONL = transport->is_online();
    XSTAT0.BOT = transport->is_at_bot();
    XSTAT0.EOT = transport->is_at_eot(); // PHYSICAL EOT!
    XSTAT0.WLK = (!transport->is_write_enabled());
    XSTAT0.PED = 1;
    XSTAT0.VCK = transport->has_volume_check();
    XSTAT0.MOT = transport->is_moving();
    TSSR.OFL = (XSTAT0.ONL == 0); // Get this too, I guess
}

// Update the message packet and write it out. Returns false if anything goes wrong.
// ONLY THE WORKER SHOULD CALL THIS!
bool ts11_c::write_message_packet(unsigned msg_class,unsigned class_code){
    if(worker_abort != 0){ return(true); }
    // Do we have a message buffer?
    if(TSSR.NBA != 0){
	FATAL("write_message_packet() called without a message buffer address?");
	return(false);
    }
    // Do we own the message buffer?
    TSBA.ADR = Characteristics.Message_Buffer_Address.ADR;
    TSSR.ADX = TSBA.ADX;
    set_register_dati_value(TSR0_reg,TSBA.word[0],"update_TSR0");
    set_register_dati_value(TSSR_reg,TSSR.word,"update_TSSR");
    TS11_MESSAGE_PACKET Message_Packet;
    memset(&Message_Packet,0,sizeof(TS11_MESSAGE_PACKET));

    if(worker_abort != 0){ return(true); }
    qunibusadapter->DMA(dma_request,true,QUNIBUS_CYCLE_DATI,TSBA.ADR,&Message_Packet.Header.word,1);
    if(!dma_request.success){
	FATAL("TS11 DMA lossage: Can't obtain message packet header");
	return(false);
    }
    // By giving us the command message, we were given the message buffer, so we don't have to check ownership.
    if(Characteristics.Message_Buffer_Extent < 016){
	FATAL("write_message_packet() called, but message packet extent is only %o bytes?",Characteristics.Message_Buffer_Extent);
	return(false);
    }else{
	// INFO("write_message_packet(): Message buffer is %o bytes long",Characteristics.Message_Buffer_Extent);
    }
    // Update any fields in the message packet that the worker doesn't update as part of command execution.
    update_xstat();
    // Build the message header
    Message_Packet.Header.Message_Class = msg_class;
    Message_Packet.Header.Format = 0;
    Message_Packet.Header.Class_Code = class_code;
    Message_Packet.Header.Reserved = 0;
    // Hand over the buffer. You would think this should be done last, but XXDP checks that the final bus address
    // matches the end of the message.
    Message_Packet.Header.ACK = 1;
    Message_Packet.Extent = 012; // Fixed size in bytes
    Message_Packet.RBPCR = RBPCR;
    Message_Packet.XSTAT0.word = XSTAT0.word;
    Message_Packet.XSTAT1.word = XSTAT1.word;
    Message_Packet.XSTAT2.word = XSTAT2.word;
    Message_Packet.XSTAT3.word = XSTAT3.word;
    if(worker_abort != 0){ return(true); }
    qunibusadapter->DMA(dma_request,true,QUNIBUS_CYCLE_DATO,TSBA.ADR,(uint16_t *)&Message_Packet,7);
    if(!dma_request.success){
	FATAL("TS11 DMA lossage: Can't write message packet");
	return(false);
    }
    /*
    INFO("MSG: %.6o %.6o %.6o %.6o %.6o %.6o %.6o",
	 Message_Packet.Header.word,Message_Packet.Extent,Message_Packet.RBPCR,Message_Packet.XSTAT0.word,
	 Message_Packet.XSTAT1.word,Message_Packet.XSTAT2.word,Message_Packet.XSTAT3.word);
    */
    TSBA.ADR += 14; TSSR.ADX = TSBA.ADX;
    set_register_dati_value(TSR0_reg,TSBA.word[0],"update_TSR0");
    set_register_dati_value(TSSR_reg,TSSR.word,"update_TSSR");
    // Final adddress is expected to be the full extent!
    // INFO("write_message_packet(): Final TSBA = %o",TSBA.ADR);
    return(true);
}

// These functions are called by the drive when it wants data.
// The real TS11 has a 64-byte silo to compensate for bus latency, but we will ignore that for now.

// TSBA should point at what we need to fetch or write.
// Return false if the unibus transaction fails in a way that will time-out the drive.
bool ts11_c::read_data_strobe(uint8_t *data,bool reverse){
    if(worker_abort != 0){ return(true); }
    // Reading data from the drive.
    // Past the end of the buffer?
    if(Data_Buffer_Index >= Command.Buffer_Extent){
	// Yes, discard
	Data_Buffer_Index++;
	return(true);
    }
    // We can use DATOB to store it.
    int byte = (TSBA.ADR&01);
    SILO.word = 0;
    // Swapping bytes?
    if(Command.Header.SWB != 0){
	// No, use DEC order
	byte ^= 1;
    }
    SILO.byte[byte] = *data;
    qunibusadapter->DMA(dma_request,true,QUNIBUS_CYCLE_DATOB,((TSBA.ADR&0777776)|byte),&SILO.word,1);
    if(!dma_request.success){
	FATAL("TS11 DMA lossage: Can't write data byte for read strobe");
	return(false);
    }
    // INFO("RDS: DMA to %.6o = %.6o, DBI %d, byte %d",((TSBA.ADR&0777776)|byte),SILO.word,Data_Buffer_Index,byte);
    if(!reverse){
	TSBA.ADR += 1;
    }else{
	TSBA.ADR -= 1;
    }
    TSSR.ADX = TSBA.ADX;
    set_register_dati_value(TSR0_reg,TSBA.word[0],"update_TSR0");
    set_register_dati_value(TSSR_reg,TSSR.word,"update_TSSR");
    Data_Buffer_Index++;
    return(true);
}

bool ts11_c::write_data_strobe(uint8_t *data,bool reverse){
    if(worker_abort != 0){ return(true); }
    // Writing data to the drive
    // Past the end of the buffer?
    if(Data_Buffer_Index >= Command.Buffer_Extent){
	// Yes, supply zeroes
	*data = 0;
	Data_Buffer_Index++;
	return(true);
    }
    // DATI must not assert A00.
    int byte = (TSBA.ADR&01);
    if(Data_Buffer_Index == 0 || byte == 0){
	qunibusadapter->DMA(dma_request,true,QUNIBUS_CYCLE_DATI,(TSBA.ADR&0777776),&SILO.word,1);
	if(!dma_request.success){
	    FATAL("TS11 DMA lossage: Can't obtain data word for write strobe");
	    *data = 0;
	    return(false);
	}
	// INFO("WDS: DMA from %.6o = %.6o, DBI %d, byte %d",(TSBA.ADR&0777776),SILO.word,Data_Buffer_Index,byte);
    }
    if(!reverse){
	TSBA.ADR += 1;
    }else{
	TSBA.ADR -= 1;
    }
    TSSR.ADX = TSBA.ADX;
    set_register_dati_value(TSR0_reg,TSBA.word[0],"update_TSR0");
    set_register_dati_value(TSSR_reg,TSSR.word,"update_TSSR");
    // Swapping bytes?
    if(Command.Header.SWB != 0){
	// Yes
	byte ^= 1;
    }
    *data = SILO.byte[byte];
    Data_Buffer_Index++;
    return(true);
}

// The operation completed.
// THE TSSR UPDATE MUST NOT HAPPEN BEFORE THE MESSAGE BUFFER IS WRITTEN!
// Badly written programs will poll the completion code and not the SSR bit?
bool ts11_c::op_complete_strobe(unsigned rcode,int32_t rvalue){
    if(worker_abort != 0){ return(true); }
    // Dispatch on rcode first.
    switch(rcode){

    case TPOP_TAPE_MARK:
	switch(Command.Header.Code){

	case 001: // READ
	    switch(Command.Header.Mode){

	    case 000: // READ NEXT
		XSTAT0.TMK = 1;
		XSTAT0.RLS = 1; // Because we didn't read anything! At least it makes sense to me...
		// INFO("TAPE MARK encountered");
		if(!write_message_packet(MSG_PKT_CLASS_END,0)){
		    FATAL("write_message_packet() blew it?");
		}
		TSSR.SC = 1;
		TSSR.TC = TSSR_TC_STATUS_ALERT;
		break;

	    default:
		FATAL("op_complete_strobe(): TPOP_TAPE_MARK: Unimplemented READ command mode %o",Command.Header.Mode);
	    }
	    break;

	case 010: // POSITION
	    switch(Command.Header.Mode){

	    case 000: // SPACE RECORDS FORWARD
	    case 001: // SPACE RECORDS REVERSE
		XSTAT0.TMK = 1;
		// Short?
		if(Data_Buffer_Index != 0){
		    // Yes
		    RBPCR = Data_Buffer_Index;
		    // INFO("TAPE MARK encountered, short by %d",RBPCR);
		    XSTAT0.RLS = 1;
		    if(!write_message_packet(MSG_PKT_CLASS_END,0)){
			FATAL("write_message_packet() blew it?");
		    }
		    TSSR.SC = 1;
		    TSSR.TC = TSSR_TC_STATUS_ALERT;
		}else{
		    // INFO("TAPE MARK encountered.");
		    RBPCR = 0;
		    if(!write_message_packet(MSG_PKT_CLASS_END,0)){
			FATAL("write_message_packet() blew it?");
		    }
		    TSSR.SC = 0;
		    TSSR.TC = TSSR_TC_NORMAL;
		}
		break;

	    default:
		FATAL("op_complete_strobe(): TPOP_TAPE_MARK: Unimplemented POSITION command mode %o",Command.Header.Mode);
	    }
	    break;

	default:
	    FATAL("op_complete_strobe(): TPOP_TAPE_MARK: Unimplemented command code %.2o",Command.Header.Code);
	}
	break;

    case TPOP_IMAGE_EOF:
	switch(Command.Header.Code){

	case 001: // READ
	    switch(Command.Header.Mode){

	    case 000: // READ NEXT
		ERROR("OPI raised");
		XSTAT3.OPI = 1;
		RBPCR = Data_Buffer_Index;
		if(!write_message_packet(MSG_PKT_CLASS_END,0)){
		    FATAL("write_message_packet() blew it?");
		}
		TSSR.SC = 1;
		TSSR.TC = TSSR_TC_UNRECOVERABLE_ERROR;
		break;

	    default:
		FATAL("op_complete_strobe(): TPOP_IMAGE_EOF: Unimplemented READ command mode %o",Command.Header.Mode);
	    }
	    break;

	case 010: // POSITION
	    switch(Command.Header.Mode){

	    case 000: // SPACE RECORDS FORWARD
		ERROR("OPI raised");
		XSTAT3.OPI = 1;
		RBPCR = Data_Buffer_Index;
		if(!write_message_packet(MSG_PKT_CLASS_END,0)){
		    FATAL("write_message_packet() blew it?");
		}
		TSSR.SC = 1;
		TSSR.TC = TSSR_TC_UNRECOVERABLE_ERROR;
		break;

	    default:
		FATAL("op_complete_strobe(): TPOP_IMAGE_EOF: Unimplemented POSITION command mode %o",Command.Header.Mode);
	    }
	    break;

	default:
	    FATAL("op_complete_strobe(): TPOP_IMAGE_EOF: Unimplemented command code %.2o",Command.Header.Code);
	}
	break;

    case TPOP_OK:
	// What command completed?
	switch(Command.Header.Code){

	case 001: // READ
	    switch(Command.Header.Mode){

	    case 000: // READ NEXT
	    case 001: // READ PREVIOUS
		// These are single-step operations.
		// Check that we got the correct byte count
		if(Data_Buffer_Index == Command.Buffer_Extent){
		    if(!write_message_packet(MSG_PKT_CLASS_END,0)){
			FATAL("write_message_packet() blew it?");
		    }
		    TSSR.SC = 0;
		    TSSR.TC = TSSR_TC_NORMAL;
		}else{
		    if(Data_Buffer_Index < Command.Buffer_Extent){
			// Buffer was short
			XSTAT0.RLS = 1;
			RBPCR = Command.Buffer_Extent-Data_Buffer_Index;
			// INFO("RECORD LENGTH SHORT: %d vs %d",Data_Buffer_Index,Command.Buffer_Extent);
		    }else{
			// Buffer was long
			XSTAT0.RLL = 1;
			RBPCR = Data_Buffer_Index-Command.Buffer_Extent;
			// INFO("RECORD LENGTH LONG: %d vs %d",Data_Buffer_Index,Command.Buffer_Extent);
		    }
		    if(!write_message_packet(MSG_PKT_CLASS_END,0)){
			FATAL("write_message_packet() blew it?");
		    }
		    TSSR.SC = 1;
		    TSSR.TC = TSSR_TC_STATUS_ALERT;
		}
		break;

	    default:
		FATAL("op_complete_strobe(): TPOP_OK: Unimplemented READ command mode %o",Command.Header.Mode);
	    }
	    break;

	case 005: // WRITE
	    switch(Command.Header.Mode){

	    case 000: // WRITE DATA
		if(!write_message_packet(MSG_PKT_CLASS_END,0)){
		    FATAL("write_message_packet() blew it?");
		}
		TSSR.SC = 0;
		TSSR.TC = TSSR_TC_NORMAL;
		break;

	    default:
		FATAL("op_complete_strobe(): TPOP_OK: Unimplemented WRITE command mode %o",Command.Header.Mode);
	    }
	    break;

	case 010: // POSITION
	    switch(Command.Header.Mode){

	    case 000: // SPACE RECORDS FORWARD
		// Are we done?
		if(Data_Buffer_Index != 0){
		    // No.
		    // INFO("Space not done");
		    Data_Buffer_Index--;
		    if(!transport->space_record_fwd()){
			FATAL("Failed to start space record forward on transport");
		    }
		    return(false);
		}
		INFO("SRF completed");
		// Otherwise yes!
		if(!write_message_packet(MSG_PKT_CLASS_END,0)){
		    FATAL("write_message_packet() blew it?");
		}
		TSSR.SC = 0;
		TSSR.TC = TSSR_TC_NORMAL;
		break;

	    case 001: // SPACE RECORDS REVERSE
		// Are we done?
		if(Data_Buffer_Index != 0){
		    // No.
		    // INFO("Space not done");
		    Data_Buffer_Index--;
		    if(!transport->space_record_rev()){
			FATAL("Failed to start space record forward on transport");
		    }
		    return(false);
		}
		// Otherwise yes!
		if(!write_message_packet(MSG_PKT_CLASS_END,0)){
		    FATAL("write_message_packet() blew it?");
		}
		TSSR.SC = 0;
		TSSR.TC = TSSR_TC_NORMAL;
		break;

	    case 002: // SKIP TAPE MARKS FORWARD
		// We hit a tape mark.
		switch(Command_Step){

		case 0: // Main state
		    // Are we done?
		    if(Data_Buffer_Index != 0){
			FATAL("Successive file skip not implemented");
		    }else{
			// XSTAT0.TMK = 1;
			RBPCR = 0;
			if(!write_message_packet(MSG_PKT_CLASS_END,0)){
			    FATAL("write_message_packet() blew it?");
			}
			TSSR.SC = 0;
			TSSR.TC = TSSR_TC_NORMAL;
		    }
		    break;
		}
		break;

	    case 004: // REWIND
		if(!write_message_packet(MSG_PKT_CLASS_END,0)){
		    FATAL("write_message_packet() blew it?");
		}
		TSSR.SC = 0;
		TSSR.TC = TSSR_TC_NORMAL;
		break;

	    default:
		FATAL("op_complete_strobe(): TPOP_OK: Unimplemented POSITION command mode %o",Command.Header.Mode);
	    }
	    break;

	case 011: // FORMAT
	    switch(Command.Header.Mode){

	    case 000: // WRITE TAPE MARK
		XSTAT0.TMK = 1;
		if(!write_message_packet(MSG_PKT_CLASS_END,0)){
		    FATAL("write_message_packet() blew it?");
		}
		TSSR.SC = 0;
		TSSR.TC = TSSR_TC_NORMAL;
		break;

	    default:
		FATAL("op_complete_strobe(): TPOP_OK: Unimplemented FORMAT command mode %o",Command.Header.Mode);
	    }
	    break;

	default:
	    FATAL("op_complete_strobe(): TPOP_OK: Unimplemented command code %.2o",Command.Header.Code);
	}
	break;

    default:
	FATAL("op_complete_strobe(): unhandled rcode %d, value %d",rcode,rvalue);
    }
    if(worker_abort != 0){ return(true); }
    // Light SSR and commit TSSR
    TSSR.SSR = 1;
    set_register_dati_value(TSSR_reg,TSSR.word,"update_TSSR");
    // INFO("New TSSR %o",TSSR.word);
    // Interrupt enabled?
    if(Command.Header.Format == 4){
	qunibusadapter->INTR(intr_request,NULL,0);
	// INFO("Raising interrupt");
    }
    return(true);
}

// The oft-oppressed worker
void ts11_c::worker(unsigned instance){
    timeout_c delay;  // Used throughout
    UNUSED(instance); // Only one worker per TS11
    assert(!pthread_mutex_lock(&on_after_register_access_mutex)); // If the register mutex is locked, eat shit and die

    // Become less-realtime than the unibus thread, but still more than the common rabble
    worker_init_realtime_priority(rt_device);

    // And now, the run loop
    while(!workers_terminate){
	int wait_result = pthread_cond_wait(&on_after_register_access_cond,&on_after_register_access_mutex); // wktk
	if(wait_result != 0){
	    ERROR("ts11_c::worker(): pthread_cond_wait returned %d: %s",wait_result,strerror(wait_result)); // ktmn
	    continue;
	}else{
	    // ktkr
	    if(!init_asserted){
		// OK, what happened?
		while(!workers_terminate && (worker_active != 0 || pending_ssi != 0)){
		    if(pending_ssi != 0){
			// Subsystem Initialize requested.
			// XXDP ZTSI will fail the board if we come back too quickly.
			// On an 11/34A, a delay of 400 milliseconds failed but 450 passed.
			// I added 50 milliseconds extra for safety's sake.
			// delay.wait_ms(500);
			// First, abort any ongoing operation and clear all drive state.
			transport->on_ssi();
			// Clobber state
			Command_Buffer_Address.dword = 0;
			RBPCR = 0;
			XSTAT0.word = 0;
			XSTAT1.word = 0;
			XSTAT2.word = 0;
			XSTAT3.word = 0;
			memset(&Characteristics,0,sizeof(TS11_WCHR_DATA));
			// Then clobber TSSR
			TSSR.SPE = TSSR.UPE = TSSR.RMR = TSSR.NXM = TSSR.SSR = 0;
			TSSR.ADX = TSBA.ADX;
			// Set NBA since we don't have a buffer address anymore. Set SSR too since we are almost done.
			TSSR.NBA = TSSR.SSR = 1;
			// Update drive status
			update_xstat();
			// Finish up. Clear the command if this is a hard reset.
			if(pending_ssi == 2){
			    memset(&Command,0,sizeof(TS11_COMMAND_PACKET));
			}
			pending_ssi = 0;
			worker_abort = 0;
			if(worker_active == 0){
			    set_register_dati_value(TSSR_reg,TSSR.word,"update_TSSR");
			    if(Command.Header.Format == 4){
				qunibusadapter->INTR(intr_request,NULL,0);
				// INFO("Raising interrupt");
			    }else{
				// INFO("Can't interrupt, not enabled (CMT %o)",Command.Header.Format);
			    }
			}else{
			    set_register_dati_value(TSSR_reg,TSSR.word,"update_TSSR");
			    INFO("Can't interrupt, worker busy");
			}
			// INFO("SSI finished, new TSSR %o",TSSR.word);
		    }
		    if(worker_active != 0){
			// We were given a bus address that should point to a command packet.
			// Stash the address.
			Command_Buffer_Address.ADR = TSBA.ADR;
			// Fetch the critter.
			qunibusadapter->DMA(dma_request,true,QUNIBUS_CYCLE_DATI,TSBA.ADR,(uint16_t *)&Command,4);
			// Did it win?
			if(!dma_request.success){
			    // No, lose
			    FATAL("TS11 DMA lossage: Can't obtain command packet");
			    worker_active = 0;
			    break;
			}
			if(worker_abort != 0){ worker_active = 0; continue; }
			// INFO("CP: DMA from %.6o",TSBA.ADR);
			TSBA.ADR += 8; TSSR.ADX = TSBA.ADX;
			set_register_dati_value(TSR0_reg,TSBA.word[0],"update_TSR0");
			set_register_dati_value(TSSR_reg,TSSR.word,"update_TSSR");
			/*
			INFO("Command code %o fmt %o mode %o",
			     Command.Header.Code,Command.Header.Format,Command.Header.Mode);
			*/
			/*
			INFO("CP: CMD: Code %o",Command.Header.Code);
			INFO("CP: CMD: Format %o",Command.Header.Format);
			INFO("CP: CMD: Mode %o",Command.Header.Mode);
			INFO("CP: CMD: SWB %o",Command.Header.SWB);
			INFO("CP: CMD: OPP %o",Command.Header.OPP);
			INFO("CP: CMD: CVC %o",Command.Header.CVC);
			INFO("CP: CMD: ACK %o",Command.Header.ACK);
			*/
			// Is ACK set?
			if(Command.Header.ACK == 0){
			    // No, we don't own this buffer.
			    // The host must have lost the message buffer.
			    FATAL("Host gave us a command packet we don't own? Disregarding.");
			    // Light SSR.
			    TSSR.SSR = 1;
			    worker_active = 0;
			    set_register_dati_value(TSSR_reg,TSSR.word,"update_TSSR");
			    // INFO("New TSSR %o",TSSR.word);
			    // Interrupt enabled?
			    if(Command.Header.Format == 4){
				// Yes, generate one. We don't need to mess with any registers.
				qunibusadapter->INTR(intr_request,NULL,0);
				// INFO("Raising interrupt");
			    }
			    break;
			}else{
			    // Clobber the step counter
			    Command_Step = 0;
			    // Clobber all of the error bits in XSTATx
			    RBPCR = 0;
			    XSTAT0.word &= 0000337;
			    XSTAT0.IE = ((Command.Header.Format&4) != 0); // Stash the IE bit
			    XSTAT1.word = 0;
			    XSTAT2.word = 0;
			    XSTAT3.word &= 0177402;
			    // CVC bit set?
			    if(Command.Header.CVC != 0 && transport->has_volume_check()){
				// Clear Volume Check
				transport->clear_volume_check();
				// INFO("VOLUME CHECK cleared");
			    }
			    // If 4-word command, clean up excess address bits
			    if((Command.Header.Code&030) == 0){
				Command.Buffer_Address.dword = Command.Buffer_Address.ADR;
			    }
			    // Obtain remaining fields of the command packet.
			    // The real TS11 always reads all four words!
			    /*
			    if((Command.Header.Code&030) == 0){
				// 4-word message.
				// INFO("Message is a 4-word command");
				TSBA.ADR += 2; TSSR.ADX = TSBA.ADX;
				set_register_dati_value(TSR0_reg,TSBA.word[0],"update_TSR0");
				set_register_dati_value(TSSR_reg,TSSR.word,"update_TSSR");
				qunibusadapter->DMA(dma_request,true,QUNIBUS_CYCLE_DATI,
						    TSBA.ADR,&Data_Buffer_Address.word[0],1);
				if(!dma_request.success){
				    FATAL("TS11 DMA lossage: Can't obtain command packet word 1");
				    worker_active = 0;
				    break;
				}
				INFO("CP: DMA from %.6o = %.6o",TSBA.ADR,Data_Buffer_Address.word[0]);
				TSBA.ADR += 2; TSSR.ADX = TSBA.ADX;
				set_register_dati_value(TSR0_reg,TSBA.word[0],"update_TSR0");
				set_register_dati_value(TSSR_reg,TSSR.word,"update_TSSR");
				qunibusadapter->DMA(dma_request,true,QUNIBUS_CYCLE_DATI,
						    TSBA.ADR,&Data_Buffer_Address.word[1],1);
				if(!dma_request.success){
				    FATAL("TS11 DMA lossage: Can't obtain command packet word 2");
				    worker_active = 0;
				    break;
				}
				INFO("CP: DMA from %.6o = %.6o",TSBA.ADR,Data_Buffer_Address.word[1]);
				Data_Buffer_Address.dword = Data_Buffer_Address.ADR;
				// INFO("CP: Data Buffer Address = %o",Data_Buffer_Address.dword);
				// Then get extent
				TSBA.ADR += 2; TSSR.ADX = TSBA.ADX;
				set_register_dati_value(TSR0_reg,TSBA.word[0],"update_TSR0");
				set_register_dati_value(TSSR_reg,TSSR.word,"update_TSSR");
				qunibusadapter->DMA(dma_request,true,QUNIBUS_CYCLE_DATI,TSBA.ADR,&Command.Buffer_Extent,1);
				if(!dma_request.success){
				    FATAL("TS11 DMA lossage: Can't obtain command packet word 3");
				    worker_active = 0;
				    break;
				}
				INFO("CP: DMA from %.6o = %.6o",TSBA.ADR,Command.Buffer_Extent);
			    }else{
				// 2-word message
				// INFO("Message is a 2-word command");
				TSBA.ADR += 2; TSSR.ADX = TSBA.ADX;
				set_register_dati_value(TSR0_reg,TSBA.word[0],"update_TSR0");
				set_register_dati_value(TSSR_reg,TSSR.word,"update_TSSR");
				qunibusadapter->DMA(dma_request,true,QUNIBUS_CYCLE_DATI,TSBA.ADR,&Command_Parameter_Word,1);
				if(!dma_request.success){
				    FATAL("TS11 DMA lossage: Can't obtain command packet word 1");
				    worker_active = 0;
				    break;
				}
				INFO("CMSG: DMA from %.6o = %.6o",TSBA.ADR,Command_Parameter_Word);
				// Read the other two words.
				uint16_t scratch;
				TSBA.ADR += 2; TSSR.ADX = TSBA.ADX;
				set_register_dati_value(TSR0_reg,TSBA.word[0],"update_TSR0");
				set_register_dati_value(TSSR_reg,TSSR.word,"update_TSSR");
				qunibusadapter->DMA(dma_request,true,QUNIBUS_CYCLE_DATI,TSBA.ADR,&scratch,1);
				if(!dma_request.success){
				    FATAL("TS11 DMA lossage: Can't obtain command packet word 2");
				    worker_active = 0;
				    break;
				}
				INFO("CP: DMA from %.6o = %.6o",TSBA.ADR,scratch);
				TSBA.ADR += 2; TSSR.ADX = TSBA.ADX;
				set_register_dati_value(TSR0_reg,TSBA.word[0],"update_TSR0");
				set_register_dati_value(TSSR_reg,TSSR.word,"update_TSSR");
				qunibusadapter->DMA(dma_request,true,QUNIBUS_CYCLE_DATI,TSBA.ADR,&scratch,1);
				if(!dma_request.success){
				    FATAL("TS11 DMA lossage: Can't obtain command packet word 3");
				    worker_active = 0;
				    break;
				}
				INFO("CP: DMA from %.6o = %.6o",TSBA.ADR,scratch);
			    }
			    */
			    if(worker_abort != 0){ worker_active = 0; continue; }
			    // If NBA is set and command is not WRITE CHARACTERISTICS, error.
			    if(TSSR.NBA != 0 && Command.Header.Code != 004){
				INFO("Command rejected, needs message buffer address");
				TSSR.TC = TSSR_TC_FUNCTION_REJECT;
				TSSR.SC = 1;
				TSSR.SSR = 1;
				worker_active = 0;
				set_register_dati_value(TSSR_reg,TSSR.word,"update_TSSR");
				if(Command.Header.Format == 4){
				    qunibusadapter->INTR(intr_request,NULL,0);
				    // INFO("Raising interrupt");
				}
			    }else{
				if(worker_abort != 0){ worker_active = 0; continue; }
				switch(Command.Header.Code){

				case 001: // READ
				    switch(Command.Header.Mode){

				    case 000: // READ NEXT
					// This does a READ FORWARD command
					// INFO("READ NEXT command");
					// Is the drive loaded and on-line?
					if(transport->is_online()){
					    // Volume check?
					    if(transport->has_volume_check()){
						XSTAT0.NEF = 1;
						TSSR.TC = TSSR_TC_FUNCTION_REJECT;
						TSSR.SC = 1;
						INFO("Tape motion command rejected due to volume check");
					    }else{
						Data_Buffer_Index = 0; // Ensure reset
						// Set starting address
						TSBA.ADR = Command.Buffer_Address.ADR;
						TSSR.ADX = TSBA.ADX;
						set_register_dati_value(TSR0_reg,TSBA.word[0],"update_TSR0");
						set_register_dati_value(TSSR_reg,TSSR.word,"update_TSSR");
						// INFO("New TSSR %o",TSSR.word);
						// Start a block.
						// INFO("Starting block of %d frames",Command.Buffer_Extent);
						if(!transport->read_data_start()){
						    FATAL("Failed to start block read on transport");
						}
						// When the drive is ready for data, it will issue a read data strobe.
						worker_active = 0;
						break;
					    }
					}else{
					    // Drive is offline, termination class S.
					    TSSR.TC = TSSR_TC_FUNCTION_REJECT;
					    TSSR.SC = 1;
					}
					set_register_dati_value(TSSR_reg,TSSR.word,"update_TSSR");
					// Write the message buffer
					if(!write_message_packet(MSG_PKT_CLASS_END,0)){
					    FATAL("write_message_packet() blew it?");
					}
					// Light SSR.
					TSSR.SSR = 1;
					worker_active = 0;
					set_register_dati_value(TSSR_reg,TSSR.word,"update_TSSR");
					// INFO("New TSSR %o",TSSR.word);
					// Interrupt enabled?
					if(Command.Header.Format == 4){
					    qunibusadapter->INTR(intr_request,NULL,0);
					    // INFO("Raising interrupt");
					}
					break;

				    case 001: // READ PREVIOUS
					// This does a READ REVERSE command
					INFO("READ PREVIOUS command");
					// Is the drive loaded and on-line?
					if(transport->is_online()){
					    // Is the drive at BOT?
					    if(!transport->is_at_bot()){
						// Volume check?
						if(transport->has_volume_check()){
						    XSTAT0.NEF = 1;
						    TSSR.TC = TSSR_TC_FUNCTION_REJECT;
						    TSSR.SC = 1;
						    INFO("Tape motion command rejected due to volume check");
						}else{
						    // Drive is not at BOT
						    Data_Buffer_Index = 0; // Ensure reset
						    // Set starting address. We were given the START of the buffer,
						    // but we need to write in reverse order!
						    TSBA.dword = Command.Buffer_Address.ADR;
						    TSBA.dword += Command.Buffer_Extent;
						    TSBA.dword -= 1;
						    TSBA.dword = TSBA.ADR; // Discard any extraneous high bits
						    TSSR.ADX = TSBA.ADX;
						    set_register_dati_value(TSR0_reg,TSBA.word[0],"update_TSR0");
						    set_register_dati_value(TSSR_reg,TSSR.word,"update_TSSR");
						    // Start a block.
						    XSTAT3.REV = 1; // Reverse direction
						    INFO("Starting block of %d frames",Command.Buffer_Extent);
						    if(!transport->read_data_reverse_start()){
							FATAL("Failed to start block read on transport");
						    }
						    // When the drive is ready for data, it will issue a read data strobe.
						    worker_active = 0;
						    break;
						}
					    }else{
						// Drive is at BOT, termination class S.
						TSSR.TC = TSSR_TC_FUNCTION_REJECT;
						TSSR.SC = 1;
					    }
					}else{
					    // Drive is offline, termination class S.
					    TSSR.TC = TSSR_TC_FUNCTION_REJECT;
					    TSSR.SC = 1;
					}
					set_register_dati_value(TSSR_reg,TSSR.word,"update_TSSR");
					// Write the message buffer
					if(!write_message_packet(MSG_PKT_CLASS_END,0)){
					    FATAL("write_message_packet() blew it?");
					}
					// Light SSR.
					TSSR.SSR = 1;
					worker_active = 0;
					set_register_dati_value(TSSR_reg,TSSR.word,"update_TSSR");
					// INFO("New TSSR %o",TSSR.word);
					// Interrupt enabled?
					if(Command.Header.Format == 4){
					    qunibusadapter->INTR(intr_request,NULL,0);
					    // INFO("Raising interrupt");
					}
					break;

				    case 002: // REREAD PREVIOUS
					// This does a SPACE RECORD REVERSE followed by a READ FORWARD
					FATAL("Unimplemented REREAD PREVIOUS command");
					break;

				    case 003: // REREAD NEXT
					// This does a SPACE RECORD FORWARD followed by a READ REVERSE
					FATAL("Unimplemented REREAD NEXT command");
					break;


				    default:
					FATAL("Unimplemented READ command mode %o",Command.Header.Mode);
				    }
				    break;

				case 004: // WRITE CHARACTERISTICS
				    if(Command.Header.Mode != 0){
					FATAL("Unimplemented WRITE CHARACTERISTICS command mode %o",Command.Header.Mode);
				    }else{
					// INFO("WRITE CHARACTERISTICS command, data @ %o",Command.Buffer_Address.ADR);
					if(Command.Buffer_Extent < 010){
					    FATAL("WRITE CHARACTERISTICS with data extent of %o bytes?",Command.Buffer_Extent);
					}
					TSBA.ADR = Command.Buffer_Address.ADR;
					TSSR.ADX = TSBA.ADX;
					set_register_dati_value(TSR0_reg,TSBA.word[0],"update_TSR0");
					set_register_dati_value(TSSR_reg,TSSR.word,"update_TSSR");
					// Obtain messge buffer
					qunibusadapter->DMA(dma_request,true,QUNIBUS_CYCLE_DATI,
							    TSBA.ADR,(uint16_t *)&Characteristics,4);
					if(!dma_request.success){
					    FATAL("TS11 DMA lossage: Can't obtain WRITE CHARACTERISTICS buffer word 0");
					    worker_active = 0;
					    break;
					}
					TSBA.ADR += 8; TSSR.ADX = TSBA.ADX;
					set_register_dati_value(TSR0_reg,TSBA.word[0],"update_TSR0");
					set_register_dati_value(TSSR_reg,TSSR.word,"update_TSSR");
					/*
					INFO("WCD: DMA from %.6o = %.6o",TSBA.ADR,Characteristics.word[0]);
					TSBA.ADR += 2; TSSR.ADX = TSBA.ADX;
					set_register_dati_value(TSR0_reg,TSBA.word[0],"update_TSR0");
					set_register_dati_value(TSSR_reg,TSSR.word,"update_TSSR");
					qunibusadapter->DMA(dma_request,true,QUNIBUS_CYCLE_DATI,
							    TSBA.ADR,&Characteristics.word[1],1);
					if(!dma_request.success){
					    FATAL("TS11 DMA lossage: Can't obtain WRITE CHARACTERISTICS buffer word 1");
					    worker_active = 0;
					    break;
					}
					INFO("WCD: DMA from %.6o = %.6o",TSBA.ADR,Characteristics.word[1]);
					TSBA.ADR += 2; TSSR.ADX = TSBA.ADX;
					set_register_dati_value(TSR0_reg,TSBA.word[0],"update_TSR0");
					set_register_dati_value(TSSR_reg,TSSR.word,"update_TSSR");
					qunibusadapter->DMA(dma_request,true,
							    QUNIBUS_CYCLE_DATI,TSBA.ADR,&Characteristics.word[2],1);
					if(!dma_request.success){
					    FATAL("TS11 DMA lossage: Can't obtain WRITE CHARACTERISTICS buffer word 2");
					    worker_active = 0;
					    break;
					}
					INFO("WCD: DMA from %.6o = %.6o",TSBA.ADR,Characteristics.word[2]);
					TSBA.ADR += 2; TSSR.ADX = TSBA.ADX;
					set_register_dati_value(TSR0_reg,TSBA.word[0],"update_TSR0");
					set_register_dati_value(TSSR_reg,TSSR.word,"update_TSSR");
					qunibusadapter->DMA(dma_request,true,QUNIBUS_CYCLE_DATI,
							    TSBA.ADR,&Characteristics.word[3],1);
					if(!dma_request.success){
					    FATAL("TS11 DMA lossage: Can't obtain WRITE CHARACTERISTICS buffer word 0");
					    worker_active = 0;
					    break;
					}
					INFO("WCD: DMA from %.6o = %.6o",TSBA.ADR,Characteristics.word[3]);
					*/
					// Clean up extra bits in the buffer address if there are any
					Characteristics.Message_Buffer_Address.dword = Characteristics.Message_Buffer_Address.ADR;
					// Log this
					/*
					INFO("Message Buffer Address: %o",Characteristics.Message_Buffer_Address.ADR);
					INFO("Message Buffer Extent: %o",Characteristics.Message_Buffer_Extent);
					INFO("Characteristics Mode Byte: %o",Characteristics.Mode.byte[0]);
					*/
					// We have a buffer address, so clear NBA
					TSSR.NBA = 0;
					// We should put a microcode revision level in XSTAT2,
					// but I don't know what to put there.
					// Indicate winning result
					TSSR.TC = TSSR_TC_NORMAL;
					TSSR.SC = 0;
					set_register_dati_value(TSSR_reg,TSSR.word,"update_TSSR");
					RBPCR = 0; // Ensure clobbered?
					// Write the message buffer
					if(!write_message_packet(MSG_PKT_CLASS_END,0)){
					    FATAL("write_message_packet() blew it?");
					}
					// Light SSR.
					TSSR.SSR = 1;
					worker_active = 0;
					set_register_dati_value(TSSR_reg,TSSR.word,"update_TSSR");
					// INFO("New TSSR %o",TSSR.word);
					// Interrupt enabled?
					if(Command.Header.Format == 4){
					    qunibusadapter->INTR(intr_request,NULL,0);
					    // INFO("Raising interrupt");
					}
				    }
				    break;

				case 005: // WRITE
				    switch(Command.Header.Mode){

				    case 000: // WRITE DATA
					INFO("WRITE DATA command");
					// Is the drive loaded and on-line?
					if(transport->is_online()){
					    // Is the drive write-locked?
					    if(transport->is_write_enabled()){
						// Drive is not write-locked.
						// Volume check?
						if(transport->has_volume_check()){
						    XSTAT0.NEF = 1;
						    TSSR.TC = TSSR_TC_FUNCTION_REJECT;
						    TSSR.SC = 1;
						    INFO("Tape motion command rejected due to volume check");
						}else{
						    Data_Buffer_Index = 0; // Ensure reset
						    // Set starting address
						    TSBA.ADR = Command.Buffer_Address.ADR;
						    TSSR.ADX = TSBA.ADX;
						    set_register_dati_value(TSR0_reg,TSBA.word[0],"update_TSR0");
						    set_register_dati_value(TSSR_reg,TSSR.word,"update_TSSR");
						    // Start a block.
						    INFO("Starting block of %d frames @ %.6o",
							 Command.Buffer_Extent,TSBA.ADR);
						    if(!transport->write_data_start(Command.Buffer_Extent)){
							FATAL("Failed to start block write on transport");
						    }
						    // When the drive is ready for data, it will issue a write data strobe.
						    worker_active = 0;
						    break;
						}
					    }else{
						// Drive is write-locked, termination class S.
						TSSR.TC = TSSR_TC_FUNCTION_REJECT;
						TSSR.SC = 1;
					    }
					}else{
					    // Drive is offline, termination class S.
					    TSSR.TC = TSSR_TC_FUNCTION_REJECT;
					    TSSR.SC = 1;
					}
					set_register_dati_value(TSSR_reg,TSSR.word,"update_TSSR");
					// Write the message buffer
					if(!write_message_packet(MSG_PKT_CLASS_END,0)){
					    FATAL("write_message_packet() blew it?");
					}
					// Light SSR.
					TSSR.SSR = 1;
					worker_active = 0;
					set_register_dati_value(TSSR_reg,TSSR.word,"update_TSSR");
					// INFO("New TSSR %o",TSSR.word);
					// Interrupt enabled?
					if(Command.Header.Format == 4){
					    qunibusadapter->INTR(intr_request,NULL,0);
					    // INFO("Raising interrupt");
					}
					break;

				    case 002: // WRITE DATA RETRY
					FATAL("Unimplemented WRITE DATA RETRY command");
					break;

				    default:
					FATAL("Unimplemented WRITE command mode %o",Command.Header.Mode);
				    }
				    break;

				case 006: // WRITE SUBSYSTEM MEMORY
				    if(Command.Header.Mode != 0){
					FATAL("Unimplemented WRITE SUBSYSTEM MEMORY command mode %o",Command.Header.Mode);
				    }else{
					// The data is a diagnostic block.
					// It gets pushed onto the internal processor stack, one byte at a time,
					// and then executed as code.
					// Simulating this is going to be an effen hassle...
					uint16_t stack_memory[16];
					unsigned index = 0;
					INFO("WRITE SUBSYSTEM MEMORY command: stand by for Shenanigans!");
					TSBA.ADR = Command.Buffer_Address.ADR;
					while(index < 16){
					    qunibusadapter->DMA(dma_request,true,QUNIBUS_CYCLE_DATI,
								TSBA.ADR,&stack_memory[index],1);
					    if(!dma_request.success){
						FATAL("TS11 DMA lossage: Can't obtain diagnostic block word %d",index);
						worker_active = 0;
						break;
					    }
					    INFO("DIABUF: DMA from %.6o = %.6o",TSBA.ADR,stack_memory[index]);
					    TSBA.ADR += 2; TSSR.ADX = TSBA.ADX;
					    set_register_dati_value(TSR0_reg,TSBA.word[0],"update_TSR0");
					    set_register_dati_value(TSSR_reg,TSSR.word,"update_TSSR");
					    index++;
					}
					// Now determine which program it is?

					// When done, indicate winning result
					TSSR.TC = TSSR_TC_NORMAL;
					TSSR.SC = 0;
					set_register_dati_value(TSSR_reg,TSSR.word,"update_TSSR");
					// Write the message buffer
					if(!write_message_packet(MSG_PKT_CLASS_END,0)){
					    FATAL("write_message_packet() blew it?");
					}
					// Light SSR.
					TSSR.SSR = 1;
					worker_active = 0;
					set_register_dati_value(TSSR_reg,TSSR.word,"update_TSSR");
					// INFO("New TSSR %.6o",TSSR.word);
					// Interrupt enabled?
					if(Command.Header.Format == 4){
					    qunibusadapter->INTR(intr_request,NULL,0);
					    // INFO("Raising interrupt");
					}
				    }
				    break;

				case 010: // POSITION
				    switch(Command.Header.Mode){

				    case 000: // SPACE RECORDS FORWARD
					// Parameter is record count.
					// INFO("SPACE RECORDS FORWARD command, %d records",Command.Parameter_Word);
					// Is the drive loaded and on-line?
					if(transport->is_online()){
					    // Volume check?
					    if(transport->has_volume_check()){
						XSTAT0.NEF = 1;
						TSSR.SC = 1;
						TSSR.TC = TSSR_TC_FUNCTION_REJECT;
						INFO("Tape motion command rejected due to volume check");
					    }else{
						Data_Buffer_Index = Command.Parameter_Word; // Take record count
						Data_Buffer_Index--; // Decrement
						if(!transport->space_record_fwd()){
						    FATAL("Failed to start space record forward on transport");
						}
						worker_active = 0;
						break;
					    }
					}else{
					    // Drive is offline, termination class S.
					    TSSR.SC = 1;
					    TSSR.TC = TSSR_TC_FUNCTION_REJECT;
					}
					set_register_dati_value(TSSR_reg,TSSR.word,"update_TSSR");
					// Write the message buffer
					if(!write_message_packet(MSG_PKT_CLASS_END,0)){
					    FATAL("write_message_packet() blew it?");
					}
					// Light SSR.
					TSSR.SSR = 1;
					worker_active = 0;
					set_register_dati_value(TSSR_reg,TSSR.word,"update_TSSR");
					// INFO("New TSSR %.6o",TSSR.word);
					// Interrupt enabled?
					if(Command.Header.Format == 4){
					    qunibusadapter->INTR(intr_request,NULL,0);
					    // INFO("Raising interrupt");
					}
					break;

				    case 001: // SPACE RECORDS REVERSE
					// Parameter is record count.
					// INFO("SPACE RECORDS REVERSE command, %d records",Command.Parameter_Word);
					// Is the drive loaded and on-line?
					if(transport->is_online()){
					    // Volume check?
					    if(transport->has_volume_check()){
						XSTAT0.NEF = 1;
						TSSR.TC = TSSR_TC_FUNCTION_REJECT;
						TSSR.SC = 1;
						INFO("Tape motion command rejected due to volume check");
					    }else{
						Data_Buffer_Index = Command.Parameter_Word;
						Data_Buffer_Index--;
						if(!transport->space_record_rev()){
						    FATAL("Failed to start space record reverse on transport");
						}
						worker_active = 0;
						break;
					    }
					}else{
					    // Drive is offline, termination class S.
					    TSSR.TC = TSSR_TC_FUNCTION_REJECT;
					    TSSR.SC = 1;
					}
					set_register_dati_value(TSSR_reg,TSSR.word,"update_TSSR");
					// Write the message buffer
					if(!write_message_packet(MSG_PKT_CLASS_END,0)){
					    FATAL("write_message_packet() blew it?");
					}
					// Light SSR.
					TSSR.SSR = 1;
					worker_active = 0;
					set_register_dati_value(TSSR_reg,TSSR.word,"update_TSSR");
					// INFO("New TSSR %.6o",TSSR.word);
					// Interrupt enabled?
					if(Command.Header.Format == 4){
					    qunibusadapter->INTR(intr_request,NULL,0);
					    // INFO("Raising interrupt");
					}
					break;

				    case 002: // SKIP TAPE MARKS FORWARD
					// Parameter is mark count.
					INFO("SKIP TAPE MARKS FORWARD command, %d marks",Command.Parameter_Word);
					// Is the drive loaded and on-line?
					if(transport->is_online()){
					    // Volume check?
					    if(transport->has_volume_check()){
						XSTAT0.NEF = 1;
						TSSR.SC = 1;
						TSSR.TC = TSSR_TC_FUNCTION_REJECT;
						INFO("Tape motion command rejected due to volume check");
					    }else{
						Data_Buffer_Index = Command.Parameter_Word; // Take mark count
						Data_Buffer_Index--; // Decrement
						if(!transport->space_file_fwd()){
						    FATAL("Failed to start space file forward on transport");
						}
						worker_active = 0;
						break;
					    }
					}else{
					    // Drive is offline, termination class S.
					    TSSR.SC = 1;
					    TSSR.TC = TSSR_TC_FUNCTION_REJECT;
					}
					set_register_dati_value(TSSR_reg,TSSR.word,"update_TSSR");
					// Write the message buffer
					if(!write_message_packet(MSG_PKT_CLASS_END,0)){
					    FATAL("write_message_packet() blew it?");
					}
					// Light SSR.
					TSSR.SSR = 1;
					worker_active = 0;
					set_register_dati_value(TSSR_reg,TSSR.word,"update_TSSR");
					// INFO("New TSSR %.6o",TSSR.word);
					// Interrupt enabled?
					if(Command.Header.Format == 4){
					    qunibusadapter->INTR(intr_request,NULL,0);
					    // INFO("Raising interrupt");
					}
					break;

				    case 003: // SKIP TAPE MARKS REVERSE
					FATAL("Unimplemented SKIP TAPE MARKS REVERSE command");
					break;

				    case 004: // REWIND
					INFO("REWIND command");
					// Already at BOT?
					if(!transport->is_at_bot()){
					    // No, go kick the drive
					    // Is this supposed to set XSTAT3.REV?
					    if(!transport->rewind_start()){
						FATAL("Failed to start rewind on transport");
					    }
					    // Wait for it to complete
					    worker_active = 0;
					}else{
					    // Yes, return immediately
					    TSSR.TC = TSSR_TC_NORMAL;
					    TSSR.SC = 0;
					    set_register_dati_value(TSSR_reg,TSSR.word,"update_TSSR");
					    if(!write_message_packet(MSG_PKT_CLASS_END,0)){
						FATAL("write_message_packet() blew it?");
					    }
					    TSSR.SSR = 1;
					    worker_active = 0;
					    set_register_dati_value(TSSR_reg,TSSR.word,"update_TSSR");
					    // INFO("New TSSR %.6o",TSSR.word);
					    if(Command.Header.Format == 4){
						qunibusadapter->INTR(intr_request,NULL,0);
						// INFO("Raising interrupt");
					    }
					}
					break;

				    default:
					FATAL("Unimplemented POSITION command mode %o",Command.Header.Mode);
				    }
				    break;

				case 011: // FORMAT
				    switch(Command.Header.Mode){

				    case 000: // WRITE TAPE MARK
					INFO("WRITE TAPE MARK command");
					// Is the drive loaded and on-line?
					if(transport->is_online()){
					    // Is the drive write-locked?
					    if(transport->is_write_enabled()){
						// No. Volume check?
						if(transport->has_volume_check()){
						    XSTAT0.NEF = 1;
						    TSSR.TC = TSSR_TC_FUNCTION_REJECT;
						    TSSR.SC = 1;
						    INFO("Tape motion command rejected due to volume check");
						}else{
						    // Good to go! Write a tape mark
						    if(!transport->write_tape_mark_start()){
							FATAL("Failed to write tape mark on transport");
						    }
						    worker_active = 0;
						    break;
						}
					    }else{
						// Drive is write-locked, termination class S.
						TSSR.TC = TSSR_TC_FUNCTION_REJECT;
						TSSR.SC = 1;
					    }
					}else{
					    // Drive is offline, termination class S.
					    TSSR.TC = TSSR_TC_FUNCTION_REJECT;
					    TSSR.SC = 1;
					}
					set_register_dati_value(TSSR_reg,TSSR.word,"update_TSSR");
					// Write the message buffer
					if(!write_message_packet(MSG_PKT_CLASS_END,0)){
					    FATAL("write_message_packet() blew it?");
					}
					// Light SSR.
					TSSR.SSR = 1;
					worker_active = 0;
					set_register_dati_value(TSSR_reg,TSSR.word,"update_TSSR");
					// INFO("New TSSR %.6o",TSSR.word);
					// Interrupt enabled?
					if(Command.Header.Format == 4){
					    qunibusadapter->INTR(intr_request,NULL,0);
					    // INFO("Raising interrupt");
					}
					break;

				    case 001: // ERASE
					// The erase command blanks 3 inches of tape.

				    case 002: // WRITE TAPE MARK ENTRY

				    default:
					FATAL("Unimplemented FORMAT command mode %o",Command.Header.Mode);
				    }
				    break;

				case 012: // CONTROL
				    switch(Command.Header.Mode){

				    case 000: // MESSAGE BUFFER RELEASE
				    case 001: // REWIND AND UNLOAD
				    case 002: // CLEAN

				    default:
					FATAL("Unimplemented CONTROL command mode %o",Command.Header.Mode);
				    }
				    break;

				case 013: // INITIALIZE
				    if(Command.Header.Mode != 0){
					FATAL("Unimplemented INITIALIZE command mode %o",Command.Header.Mode);
				    }else{
					// INFO("INITIALIZE command");
					// If there is no error code, immediately return.
					// Otherwise, do the same as writing TSSR
					if(TSSR.SC != 0 || (TSSR.TC != TSSR_TC_NORMAL && TSSR.TC != TSSR_TC_ATTENTION)){
					    // Clobbering needed
					    /*
					    INFO("SSI required, TSSR is %.6o, SC %o, TC %o",
						 TSSR.word,TSSR.SC,TSSR.TC);
					    */
					    TSSR.word = 0;
					    set_register_dati_value(TSSR_reg,TSSR.word,"update_TSSR");
					    pending_ssi = 1;
					    worker_active = 0;
					    continue; // Loop!
					}else{
					    // No clobbering needed
					    TSSR.TC = TSSR_TC_NORMAL;
					    TSSR.SC = 0;
					    set_register_dati_value(TSSR_reg,TSSR.word,"update_TSSR");
					    if(!write_message_packet(MSG_PKT_CLASS_END,0)){
						FATAL("write_message_packet() blew it?");
					    }
					    TSSR.SSR = 1;
					    worker_active = 0;
					    set_register_dati_value(TSSR_reg,TSSR.word,"update_TSSR");
					    // INFO("New TSSR %.6o",TSSR.word);
					    if(Command.Header.Format == 4){
						qunibusadapter->INTR(intr_request,NULL,0);
						// INFO("Raising interrupt");
					    }
					}
				    }
				    break;

				case 017: // GET STATUS IMMEDIATE
				    if(Command.Header.Mode != 0){
					FATAL("Unimplemented GET STATUS IMMEDIATE command mode %o",Command.Header.Mode);
				    }else{
					// Update and output the message buffer
					// INFO("GET STATUS IMMEDIATE command");
					// We should put a residual capstan tick count in XSTAT2.
					// When done, indicate winning result
					TSSR.TC = TSSR_TC_NORMAL;
					TSSR.SC = 0;
					set_register_dati_value(TSSR_reg,TSSR.word,"update_TSSR");
					if(!write_message_packet(MSG_PKT_CLASS_END,0)){
					    FATAL("write_message_packet() blew it?");
					}
					TSSR.SSR = 1;
					worker_active = 0;
					set_register_dati_value(TSSR_reg,TSSR.word,"update_TSSR");
					// INFO("New TSSR %.6o",TSSR.word);
					if(Command.Header.Format == 4){
					    qunibusadapter->INTR(intr_request,NULL,0);
					    // INFO("Raising interrupt");
					}
				    }
				    break;

				default:
				    FATAL("Unimplemented command code %.2o",Command.Header.Code);
				}
			    }
			    if(worker_active != 0){
				FATAL("Command execution fell off end?");
			    }
			}
		    }
		    // End of worker busy loop
		}
	    }
	}
    }
    assert(!pthread_mutex_unlock(&on_after_register_access_mutex)); // If we can't flush the register mutex, etc etc.
}
