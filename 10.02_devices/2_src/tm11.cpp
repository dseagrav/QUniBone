/*
  tm11.cpp: TM11-B DECmagtape controller

  Copyright Bogodyne Metatechnics LLC
  Contributed under the BSD 2-clause license.
*/

#include "logger.hpp"
#include "qunibus.h"
#include "tm11.hpp"

tm11_c::tm11_c() : tapecontroller_c(){
    name.value = "TM11";
    type_name.value = "tm11_c";
    log_label = "tm";

    // Base address, slot number, interrupt vector, BR level
    set_default_bus_params(0772520, 29, 0224, 5);

    MTS.word = 0;
    MTC.word = 0;
    MTBRC.word = 0;
    MTCMA.dword = 0;
    MTD.word = 0;
    MTRD.word = 0;

    register_count = 6;

    UBR[0] = &(this->registers[0]);
    strcpy(UBR[0]->name,"MTS");
    UBR[0]->active_on_dati = false;
    UBR[0]->active_on_dato = true;
    UBR[0]->reset_value = 0;
    UBR[0]->writable_bits = 0; // Read Only

    UBR[1] = &(this->registers[1]);
    strcpy(UBR[1]->name,"MTC");
    UBR[1]->active_on_dati = false;
    UBR[1]->active_on_dato = true;
    UBR[1]->reset_value = 0;
    UBR[1]->writable_bits = 0;

    UBR[2] = &(this->registers[2]);
    strcpy(UBR[2]->name,"MTBRC");
    UBR[2]->active_on_dati = false;
    UBR[2]->active_on_dato = true;
    UBR[2]->reset_value = 0;
    UBR[2]->writable_bits = 0;

    UBR[3] = &(this->registers[3]);
    strcpy(UBR[3]->name,"MTCMA");
    UBR[3]->active_on_dati = false;
    UBR[3]->active_on_dato = true;
    UBR[3]->reset_value = 0;
    UBR[3]->writable_bits = 0;

    UBR[4] = &(this->registers[4]);
    strcpy(UBR[4]->name,"MTD");
    UBR[4]->active_on_dati = false;
    UBR[4]->active_on_dato = true;
    UBR[4]->reset_value = 0;
    UBR[4]->writable_bits = 0;

    UBR[5] = &(this->registers[5]);
    strcpy(UBR[5]->name,"MTRD");
    UBR[5]->active_on_dati = false;
    UBR[5]->active_on_dato = true;
    UBR[5]->reset_value = 0;
    UBR[5]->writable_bits = 0; // Read Only

}

tm11_c::~tm11_c(){

}

bool tm11_c::on_param_changed(parameter_c *param){
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

void tm11_c::on_after_register_access(qunibusdevice_register_t *device_reg, uint8_t unibus_control, DATO_ACCESS access){
    switch(device_reg->index){
    case 0: // MTS
	set_register_dati_value(UBR[0],MTS.word,"update_MTS");
	break;

    case 1: // MTC
	break;

    case 2: // MTBRC
	break;

    case 3: // MTCMA
	break;

    case 4: // MTD
	break;

    case 5: // MTRD
	set_register_dati_value(UBR[5],MTS.word,"update_MTRD");
	break;
    }
}

void tm11_c::on_power_changed(signal_edge_enum aclo_edge, signal_edge_enum dclo_edge){

}

void tm11_c::on_init_changed(void){

}

void tm11_c::on_drive_status_changed(tapedrive_c *drive){

}

bool tm11_c::read_data_strobe(uint8_t *data,bool reverse){

}

bool tm11_c::write_data_strobe(uint8_t *data,bool reverse){

}

bool tm11_c::op_complete_strobe(unsigned rcode,int32_t rvalue){

}

void tm11_c::worker(unsigned instance){

}
