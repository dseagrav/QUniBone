/*
  tapecontoller.cpp: A qunibus device with one or more tapedrives attached.

  Copyright Bogodyne Metatechnics LLC
  Contributed under the BSD 2-clause license.

  OK, so storagedrive looks not ready for tape drive support right now, so I'm creating this as a stopgap solution
  to enable work on the TS11 to proceed. I don't know how much time I'll have to work on this.

  Someone else can either merge this into storagedrive later or decide it's a better solution, whatever works.
*/

#include "tapecontroller.hpp"

tapecontroller_c::tapecontroller_c(): qunibusdevice_c(){
    this->drivecount = 0;
}

tapecontroller_c::~tapecontroller_c(){

}

// Return false to cause a failure if not ready to be attached to the bus
bool tapecontroller_c::on_before_install(void){
    return true;
}

// Uninstall, uninstall
void tapecontroller_c::on_after_uninstall(void){
    // この星の無数の塵のひとつだと...
    unsigned i=0;
    while(i < drivecount){
	tapedrives[i]->enabled.set(false);
	i++;
    }
}

bool tapecontroller_c::on_param_changed(parameter_c *param){
    return qunibusdevice_c::on_param_changed(param); // Pass thru
}

void tapecontroller_c::on_power_changed(signal_edge_enum aclo_edge, signal_edge_enum dclo_edge){
    std::vector<tapedrive_c*>::iterator it = tapedrives.begin();
    while(it != tapedrives.end()){
	(*it)->on_power_changed(aclo_edge, dclo_edge);
	it++;
    }
}

void tapecontroller_c::on_init_changed(){
    std::vector<tapedrive_c*>::iterator it = tapedrives.begin();
    while(it != tapedrives.end()){
	(*it)->init_asserted = init_asserted;
	(*it)->on_init_changed();
	it++;
    }
}
