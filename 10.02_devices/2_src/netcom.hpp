/*
  netcom.hpp: Network communications for serial multiplexers.

  Copyright Bogodyne Metatechnics LLC
  Contributed under the BSD 2-clause license.
*/

#ifndef _NETCOM_HPP_
#define _NETCOM_HPP_

#include <netinet/in.h>
#include <sys/socket.h>

/****************************
 * TELNET PROTOCOL MESSAGES *
 ****************************/

// Interpret As Command
#define TN_IAC 0xFF
// Stop Doing / Don't
#define TN_DO_NOT 0xFE
// Start Doing / Do
#define TN_DO 0xFD
// I Refuse To
#define TN_WILL_NOT 0xFC
// I Will
#define TN_WILL 0xFB
// Subnegotiation of option follows
#define TN_SUBNEGOTIATE 0xFA
// You may transmit
#define TN_GO_AHEAD 0xF9
// As named
#define TN_ERASE_LINE 0xF8
#define TN_ERASE_CHARACTER 0xF7
#define TN_ARE_YOU_THERE 0xF6
#define TN_ABORT_OUTPUT 0xF5
#define TN_INTERRUPT_PROCESS 0xF4
#define TN_BREAK 0xF3
// Data Mark is the data portion of a a Synch, and the corresponding TCP header should have URGENT set.
#define TN_DATA_MARK 0xF2
// Do nothing
#define TN_NOP 0xF1
// End of subnegotiation
#define TN_END_SUBNEGOTIATION 0xF0

/***********************
 * TELNET OPTION CODES *
 ***********************/

#define TN_OPT_LINE_MODE 0x22
#define TN_OPT_TIMING_MARK 0x06
#define TN_OPT_SUPPRESS_GO_AHEAD 0x03
#define TN_OPT_ECHO 0x01

/**************************
 * NETCOM PROTOCOL STATES *
 **************************/

#define NCPS_INIT -1
#define NCPS_MAIN 0
#define NCPS_IAC 1
#define NCPS_IAC_WILL 2
#define NCPS_IAC_WILL_NOT 3
#define NCPS_IAC_DO 4
#define NCPS_IAC_DO_NOT 5

/***************************
 * NETCOM INTERFACE TO MUX *
 ***************************/

class netcon_c;
class netcom_line_c;
class netcom_mux_c {
public:
    virtual int find_free_line() = 0;
    virtual netcom_line_c *get_slu(int ln) = 0;
    virtual bool seize_line(int ln,netcon_c *conn) = 0;
    virtual void release_line(int ln) = 0;
};

/****************************
 * NETCOM INTERFACE TO LINE *
 ****************************/

class netcom_line_c {
public:
    virtual bool recv_data_from_nc(uint8_t data) = 0;
};

/*********************
 * NETCOM CONNECTION *
 *********************/

class netcom_c;
class netcon_c: public device_c {
    friend class netcom_c;
private:
    netcom_c *ctl;
    netcom_mux_c *mux;
    netcom_line_c *slu;
    int fd;
    int line;
    struct sockaddr_in remote_addr;
    int protocol_state;

public:
    netcon_c(netcom_c *_ctl,int _fd,struct sockaddr_in *_sa,netcom_mux_c *_mux,netcom_line_c *_slu,int _line);
    ~netcon_c();

    bool transmit_data(uint8_t data);
    bool on_param_changed(parameter_c *param) override;
    void on_power_changed(signal_edge_enum aclo_edge,signal_edge_enum dclo_edge) override;
    void on_init_changed(void) override;
    void on_worker_terminated(unsigned instance) override;
    void worker(unsigned instance) override;
};

/*****************
 * NETCOM SERVER *
 *****************/

class netcom_c: public device_c {
    friend class netcon_c;
private:
    int idx;
    int socket_fd;
    std::vector<netcom_mux_c *>muxes;
    std::vector<netcon_c *> netcons;

    bool enable_socket();
    bool disable_socket();

public:
    netcom_c(int _idx);
    ~netcom_c();

    parameter_bool_c raw_mode = parameter_bool_c(this,"raw_mode","raw",false,"Disable telnet option negotiation");
    parameter_unsigned_c port_number = parameter_unsigned_c(this,"port_number","port",false,"","%d","TCP port number",16,10);

    bool attach_mux(netcom_mux_c *mux);
    bool detach_mux(netcom_mux_c *mux);
    void on_netcon_closed(netcon_c *conn);
    bool on_param_changed(parameter_c *param) override;
    void on_power_changed(signal_edge_enum aclo_edge,signal_edge_enum dclo_edge) override;
    void on_init_changed(void) override;
    void worker(unsigned instance) override;
};

#endif
