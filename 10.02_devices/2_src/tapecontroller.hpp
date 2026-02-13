/*
  tapecontoller.hpp: A qunibus device with one or more tapedrives attached.

  Copyright Bogodyne Metatechnics LLC
  Contributed under the BSD 2-clause license.

  OK, so storagedrive looks not ready for tape drive support right now, so I'm creating this as a stopgap solution
  to enable work on the TS11 to proceed. I don't know how much time I'll have to work on this.

  Someone else can either merge this into storagedrive later or decide it's a better solution, whatever works.

*/

#ifndef _TAPECONTROLLER_HPP_
#define _TAPECONTROLLER_HPP_

#include <vector>

#include "qunibusdevice.hpp"
#include "tapedrive.hpp"

class tapecontroller_c: public qunibusdevice_c {
public:
    unsigned drivecount;
    std::vector<tapedrive_c *> tapedrives;

    tapecontroller_c(void);
    virtual ~tapecontroller_c();

    virtual bool on_before_install(void) override;
    virtual void on_after_uninstall(void) override;

    virtual bool read_data_strobe(uint8_t *data,bool reverse) = 0;  // Reading data from the drive
    virtual bool write_data_strobe(uint8_t *data,bool reverse) = 0; // Writing data to the drive
    virtual bool op_complete_strobe(unsigned rcode,int32_t rvalue) = 0;                          // Drive operation completed

    virtual bool on_param_changed(parameter_c *param) override;
    virtual void on_power_changed(signal_edge_enum aclo_edge, signal_edge_enum dclo_edge) override;
    virtual void on_init_changed() override;
    virtual void on_drive_status_changed(tapedrive_c *drive) = 0;
};

#endif

