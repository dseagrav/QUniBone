/*
  rk611.hpp: RK611 disk controller

  Copyright Bogodyne Metatechnics LLC
  Contributed under the BSD 2-clause license.
*/

#ifndef _RK611_HPP_
#define _RK611_HPP_

#include "qunibusdevice.hpp"
#include "rk067.hpp"

typedef union {
    uint16_t word;
    uint8_t byte[2];
    struct {
	uint16_t GO:1;
	uint16_t F:4;
	uint16_t _unused_5:1;
	uint16_t IE:1;
	uint16_t RDY:1;
	uint16_t BA:2;
	uint16_t CDT:1;
	uint16_t CT0:1;
	uint16_t CFMT:1;
	uint16_t DCT_PAR:1;
	uint16_t DI:1;
	uint16_t CERR:1; // OCLR when written
    } __attribute__((packed));
} RKCS1_REG;

typedef union {
    uint16_t word;
    uint8_t byte[2];
} RKWC_REG;

typedef union {
    uint32_t dword;
    uint16_t word[2];
    uint8_t byte[4];
    struct {
	uint32_t ADL:16;
	uint32_t ADH:2;
    } __attribute__((packed));
    struct {
	uint32_t ADR:18;
    } __attribute__((packed));
} RKBA_REG;

typedef union {
    uint16_t word;
    uint8_t byte[2];
    struct {
	uint16_t SA:5;
	uint16_t _unused_5_7:3;
	uint16_t TA:3;
    } __attribute__((packed));
} RKDA_REG;

typedef union {
    uint16_t word;
    uint8_t byte[2];
    struct {
	uint16_t DS:3;
	uint16_t RLS:1;
	uint16_t BAI:1;
	uint16_t SCLR:1;
	uint16_t IR:1;
	uint16_t OR:1;
	uint16_t UFE:1;
	uint16_t MDS:1;
	uint16_t PGE:1;
	uint16_t NEM:1;
	uint16_t NED:1;
	uint16_t UPE:1;
	uint16_t WCE:1;
	uint16_t DLT:1;
    } __attribute__((packed));
} RKCS2_REG;

typedef union {
    uint16_t word;
    uint8_t byte[2];
    struct {
	uint16_t DRA:1;
	uint16_t _unused_1:1;
	uint16_t OFST:1;
	uint16_t ACLO:1;
	uint16_t SPLS:1;
	uint16_t DROT:1;
	uint16_t VV:1;
	uint16_t DRDY:1;
	uint16_t DDT:1;
	uint16_t _unused_9_10:2;
	uint16_t WRL:1;
	uint16_t _unused_12:2;
	uint16_t PIP:1;
	uint16_t SDA:1;
	uint16_t SVAL:1;
    } __attribute__((packed));
} RKDS_REG;

typedef union {
    uint16_t word;
    uint8_t byte[2];
    struct {
	uint16_t ILF:1;
	uint16_t SKI:1;
	uint16_t NXF:1;
	uint16_t DRPAR:1;
	uint16_t FMTE:1;
	uint16_t DTYE:1;
	uint16_t ECH:1;
	uint16_t BSE:1;
	uint16_t HVRC:1;
	uint16_t COE:1;
	uint16_t IDAE:1;
	uint16_t WLE:1;
	uint16_t DTE:1;
	uint16_t OPI:1;
	uint16_t UNS:1;
	uint16_t DCK:1;
    } __attribute__((packed));
} RKER_REG;

typedef union {
    uint16_t word;
    uint8_t byte[2];
    struct {
	uint16_t OF:8;
	uint16_t ATN:8;
    } __attribute__((packed));
} RKAS_REG;

typedef union {
    uint16_t word;
    uint8_t byte[2];
    struct {
	uint16_t DC:10;
    } __attribute__((packed));
} RKDC_REG;

typedef union {
    uint16_t word;
    uint8_t byte[2];
} RKDB_REG;

typedef union {
    uint16_t word;
    uint8_t byte[2];
    struct {
	uint16_t MS:4;
	uint16_t PAT:1;
	uint16_t DMD:1;
	uint16_t MSP:1;
	uint16_t MIND:1;
	uint16_t MCLK:1;
	uint16_t MERD:1;
	uint16_t MEWD:1;
	uint16_t PCA:1;
	uint16_t PCD:1;
	uint16_t ECCW:1;
	uint16_t WRT_GATE:1;
	uint16_t RD_GATE:1;
    } __attribute__((packed));
} RKMR1_REG;

typedef union {
    uint16_t word;
    uint8_t byte[2];
    struct {
	uint16_t EPS:13;
    } __attribute__((packed));
} RKECPS_REG;

typedef union {
    uint16_t word;
    uint8_t byte[2];
    struct {
	uint16_t EPT:11;
    } __attribute__((packed));
} RKECPT_REG;

typedef union {
    uint16_t word;
    uint8_t byte[2];
} RKMR2_REG;

typedef union {
    uint16_t word;
    uint8_t byte[2];
} RKMR3_REG;

class rk611_c: public qunibusdevice_c {
private:

    qunibusdevice_register_t *UBR[16];
    dma_request_c dma_request = dma_request_c(this);
    intr_request_c intr_request = intr_request_c(this);

    RKCS1_REG RKCS1;
    RKWC_REG RKWC;
    RKBA_REG RKBA;
    RKDA_REG RKDA;
    RKCS2_REG RKCS2;
    RKDS_REG RKDS;
    RKER_REG RKER;
    RKAS_REG RKAS;
    RKDC_REG RKDC;
    RKDB_REG RKDB;
    RKMR1_REG RKMR1;
    RKECPS_REG RKECPS;
    RKECPT_REG RKECPT;
    RKMR2_REG RKMR2;
    RKMR3_REG RKMR3;

    int silo_head;
    int silo_tail;
    int silo_level;
    pthread_mutex_t silo_mutex = PTHREAD_MUTEX_INITIALIZER;
    RKDB_REG SILO[66]; // The Silo plus its input and output buffers

    void controller_clear(int init);
    void subsystem_clear(int init);
    bool silo_push(uint16_t *data);
    bool silo_pop(uint16_t *data);

public:
    rk611_c();
    ~rk611_c();

    bool on_param_changed(parameter_c *param) override;
    void on_after_register_access(qunibusdevice_register_t *device_reg, uint8_t unibus_control, DATO_ACCESS access)
	override;
    void on_power_changed(signal_edge_enum aclo_edge, signal_edge_enum dclo_edge) override;
    void on_init_changed(void) override;
    void worker(unsigned instance) override;
};

#endif
