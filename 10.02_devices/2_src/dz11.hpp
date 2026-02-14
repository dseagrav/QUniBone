/*
  dz11.hpp: DZ11/DZV11 UNIBUS/QBUS controller

  Copyright Bogodyne Metatechnics LLC
  Contributed under the BSD 2-clause license.

*/

#ifndef _DZ11_HPP_
#define _DZ11_HPP_

#include "qunibusdevice.hpp"
#include "netcom.hpp"

// DZ11 has 8 lines, DZV11 has 4.
#if defined(UNIBUS)
#define DZ11_LINES 8
#elif defined(QBUS)
#define DZ11_LINES 4
#endif

typedef union rDZ11_CSR {
    uint16_t word;
    uint8_t byte[2];
    struct {
	uint16_t _unused_0_2:3;
	uint16_t MAINT:1;       // Maintenance Mode
	uint16_t CLR:1;         // Clear
	uint16_t MSE:1;         // Master Scan Enable
	uint16_t RIE:1;         // Receiver Interrupt Enable
	uint16_t RDONE:1;       // Receiver Done
	uint16_t TLINE:3;       // Transmit Line
	uint16_t _unused_11:1;
	uint16_t SAE:1;         // Silo Alarm Enable
	uint16_t SA:1;          // Silo Alarm
	uint16_t TIE:1;         // Transmitter Interrupt Enable
	uint16_t TRDY:1;        // Transmitter Ready
    } __attribute__((packed));
} DZ11_CSR;

typedef union rDZ11_RBUF {
    uint16_t word;
    uint8_t byte[2];
    struct {
	uint16_t DATA:8;
	uint16_t RLINE:3;
	uint16_t _unused_11:1;
	uint16_t PAR_ERR:1;
	uint16_t FRAME_ERR:1;
	uint16_t OVRN:1;
	uint16_t DATA_VALID:1;
    } __attribute__((packed));
} DZ11_RBUF;

typedef union rDZ11_LPR {
    uint16_t word;
    uint8_t byte[2];
    struct {
	uint16_t LINE:3;
	uint16_t CHAR_LENGTH:2;
	uint16_t STOP_CODE:1;
	uint16_t PARITY_ENABLE:1;
	uint16_t ODD_PARITY:1;
	uint16_t FREQ:4;
	uint16_t RX_ON:1;
	// Remainder unused
    } __attribute__((packed));
} DZ11_LPR;

typedef union rDZ11_TCR {
    uint16_t word;
    uint8_t byte[2];
    struct {
	uint16_t LINE_ENABLE:8;
	uint16_t DTR:8;
    } __attribute__((packed));
} DZ11_TCR;

typedef union rDZ11_MSR {
    uint16_t word;
    uint8_t byte[2];
    struct {
	uint16_t RI:8;
	uint16_t CO:8;
    } __attribute__((packed));
} DZ11_MSR;

typedef union rDZ11_TDR {
    uint16_t word;
    uint8_t byte[2];
    struct {
	uint16_t TBUF:8;
	uint16_t BRK:8;
    } __attribute__((packed));
} DZ11_TDR;

/* A single line of a DZ11/DZV11 */

class dz11_c;
class dz11_slu_c: public device_c, public netcom_line_c {
    friend class dz11_c;

private:
    dz11_c *ctl;
    int line;
    netcon_c *netcon;
    uint8_t line_bit; // Used in DZ registers
    int line_rate; // Delay time between characters in microseconds
    uint8_t line_mask; // Bit mask for character width
    int tx_ready;
    int rx_ready;
    DZ11_LPR LPR;
    pthread_mutex_t tx_worker_sync_mutex = PTHREAD_MUTEX_INITIALIZER;
    pthread_cond_t tx_worker_sync_cond = PTHREAD_COND_INITIALIZER;
    uint8_t transmit_data;
    uint8_t receive_data;

public:
    dz11_slu_c(dz11_c *controller);
    ~dz11_slu_c();

    bool on_param_changed(parameter_c *param) override;
    void on_power_changed(signal_edge_enum aclo_edge,signal_edge_enum dclo_edge) override;
    void on_init_changed(void) override;
    void lpr_write(uint16_t val);
    void transmit(uint8_t data);
    std::string get_name() override;
    bool recv_data_from_nc(uint8_t data) override;
    bool recv_break_from_nc() override;
    void worker(unsigned instance) override;
    void transmit_worker(void);
};

/* The DZ11/DZV11 itself */

class dz11_c: public qunibusdevice_c, public netcom_mux_c {
    friend class dz11_slu_c;

private:

    qunibusdevice_register_t *UBR[4];
    intr_request_c rx_intr_request = intr_request_c(this);
    intr_request_c tx_intr_request = intr_request_c(this);
    int unit;

    DZ11_CSR CSR;   // DR0 R/W
    DZ11_RBUF RBUF; // DR2 R
    DZ11_TCR TCR;   // DR4 R/W
    DZ11_MSR MSR;   // DR6 R
    DZ11_TDR TDR;   // DR6 W

    int silo_head = 0;
    int silo_tail = 0;
    int silo_level = 0;
    int silo_alarm_level = 16;
    pthread_mutex_t silo_mutex = PTHREAD_MUTEX_INITIALIZER;
    // SILO is 64 words + buffer
    uint8_t SILO_Data[65];
    int SILO_Overrun[65];
    int SILO_Line[65];

    dz11_slu_c *SLU[DZ11_LINES];

    bool silo_push(uint8_t *data,int *line);
    bool silo_pop();

public:
    dz11_c(int _unit);
    ~dz11_c();

    void reset(bool hard);
    void find_tline(int tln,int log);
    bool receive_data_on_line(int line,uint8_t data);
    bool on_param_changed(parameter_c *param) override;
    void on_after_register_access(qunibusdevice_register_t *device_reg, uint8_t unibus_control, DATO_ACCESS access)
	override;
    void on_power_changed(signal_edge_enum aclo_edge, signal_edge_enum dclo_edge) override;
    void on_init_changed(void) override;
    std::string get_name() override;
    int find_free_line() override;
    netcom_line_c *get_slu(int ln) override;
    bool seize_line(int ln,netcon_c *conn) override;
    void release_line(int ln) override;
    void worker(unsigned instance) override;
};

#endif
