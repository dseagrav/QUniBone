/*
  tm11.hpp: TM11-B DECmagtape controller

  Copyright Bogodyne Metatechnics LLC
  Contributed under the BSD 2-clause license.
*/

#ifndef _TM11_HPP_
#define _TM11_HPP_

#include "qunibusadapter.hpp"
#include "tapecontroller.hpp"

// MTS
typedef union rTM11_MTS {
    uint16_t word;
    uint8_t byte[2];
    struct {
	uint16_t TUR:1;
	uint16_t RWS:1;
	uint16_t WRL:1;
	uint16_t SDWN:1;
	uint16_t SEVEN_CH:1;
	uint16_t BOT:1;
	uint16_t SELR:1;
	uint16_t NXM:1;
	uint16_t BTE:1;
	uint16_t RLE:1;
	uint16_t EOT:1;
	uint16_t BGL:1;
	uint16_t PAE:1;
	uint16_t CRE:1;
	uint16_t FMK:1; // This is really called EOF, but that is a macro defined somewhere
	uint16_t ILC:1;
    } __attribute__((packed));
} TM11_MTS;

typedef union rTM11_MTC {
    uint16_t word;
    uint8_t byte[2];
    struct {
	uint16_t GO:1;
	uint16_t FCTN:3;
	uint16_t AH:2;
	uint16_t INT_ENB:1;
	uint16_t CU_RDY:1;
	uint16_t SEL:3;
	uint16_t PEVN:1;
	uint16_t PWR_CLR:1;
	uint16_t DEN:2;
	uint16_t ERR:1;
    } __attribute__((packed));
} TM11_MTC;

typedef union rTM11_MTBRC {
    uint16_t word;
    uint8_t byte[2];
} TM11_MTBRC;

typedef union rTM11_MTCMA {
    uint32_t dword;
    uint16_t word[2];
    uint8_t byte[4];
    struct {
	uint32_t ADL:16;
	uint32_t ADX:2;
    } __attribute__((packed));
    struct {
	uint32_t ADR:18;
    } __attribute__((packed));
} TM11_MTCMA;

typedef union rTM11_MTD {
    uint16_t word;
    uint8_t byte[2];
    struct {
	uint16_t DATA:8;
	uint16_t PARITY:1;
    } __attribute__((packed));
} TM11_MTD;

typedef union rTM11_MTRD {
    uint16_t word;
    uint8_t byte[2];
    struct {
	uint16_t DATA:8;
	uint16_t PARITY:1;
	uint16_t _unused_9_11:3;
	uint16_t GAP_SHUTDOWN:1;
	uint16_t BTE_GEN:1;
	uint16_t CHAR_SEL:1;
	uint16_t TIMER:1;
    } __attribute__((packed));
} TM11_MTRD;

class tm11_c: public tapecontroller_c {
private:

    TM11_MTS MTS;
    TM11_MTC MTC;
    TM11_MTBRC MTBRC;
    TM11_MTCMA MTCMA;
    TM11_MTD MTD;
    TM11_MTRD MTRD;

    qunibusdevice_register_t *UBR[6];
    dma_request_c dma_request = dma_request_c(this);
    intr_request_c intr_request = intr_request_c(this);

public:
    tm11_c();
    ~tm11_c();

    bool on_param_changed(parameter_c *param) override;
    void on_after_register_access(qunibusdevice_register_t *device_reg, uint8_t unibus_control, DATO_ACCESS access)
	override;
    void on_power_changed(signal_edge_enum aclo_edge, signal_edge_enum dclo_edge) override;
    void on_init_changed(void) override;
    void on_drive_status_changed(tapedrive_c *drive);
    bool read_data_strobe(uint8_t *data,bool reverse) override;
    bool write_data_strobe(uint8_t *data,bool reverse) override;
    bool op_complete_strobe(unsigned rcode,int32_t rvalue) override;
    void worker(unsigned instance) override;
};

#endif
