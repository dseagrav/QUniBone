/*
  lp11.hpp: LP11 controller + LP05 (Dataproducts 2230) printer.

  Copyright Bogodyne Metatechnics LLC
  Contributed under the BSD 2-clause license.
*/

#ifndef _LP11_HPP_
#define _LP11_HPP_

#include "qunibusdevice.hpp"

typedef union {
    uint16_t word;
    uint8_t byte[2];
    struct {
	uint16_t _unused_0_5:6;
	uint16_t INTR_ENB:1;
	uint16_t RDY:1;
	uint16_t _unused_8_14:7;
	uint16_t ERROR:1;
    } __attribute__((packed));
} LP11_CSR;

typedef union {
    uint16_t word;
    uint8_t byte[2];
    struct {
	uint16_t DATA:7;
	// Remainder unused
    } __attribute__((packed));
} LP11_DB;

class lp11_c: public qunibusdevice_c {
private:

    qunibusdevice_register_t *UBR[2];
    intr_request_c intr_request = intr_request_c(this);
    uint8_t buffer[256]; // Just in case
    unsigned buffer_idx;
    unsigned column_idx;
    unsigned line_number;
    FILE *fd;

    LP11_CSR LPCS;
    LP11_DB LPDB;

public:
    lp11_c();
    ~lp11_c();

    parameter_bool_c online = parameter_bool_c(this,"online","onl",false,"State of On-Line button");
    parameter_unsigned_c page_width = parameter_unsigned_c(this,"carriage_width","width",false,"characters","%d",
							   "Width of the printer carriage, generally 80 or 132",8,10);
    parameter_unsigned_c page_length = parameter_unsigned_c(this,"page_length","length",false,"lines","%d",
							    "Length of the forms loaded in the printer, generally 66",8,10);
    parameter_string_c file_name = parameter_string_c(this,"file_name","file",false,"Path to printer output file/device.");
    parameter_bool_c file_append = parameter_bool_c(this,"file_append","append",false,"Append or overwrite file on open");

    bool on_param_changed(parameter_c *param) override;
    void on_after_register_access(qunibusdevice_register_t *device_reg, uint8_t unibus_control, DATO_ACCESS access)
	override;
    void on_power_changed(signal_edge_enum aclo_edge, signal_edge_enum dclo_edge) override;
    void on_init_changed(void) override;
    void worker(unsigned instance) override;
};

#endif
