/*
  ts11.hpp: TS11/TSV05 UNIBUS/QBUS controller

  Copyright Bogodyne Metatechnics LLC
  Contributed under the BSD 2-clause license.

*/

#ifndef _TS11_HPP_
#define _TS11_HPP_

#include "qunibusadapter.hpp"
#include "tapecontroller.hpp"
#include "ts11_drive.hpp"

// TSBA (but also used for constructing other addresses)
typedef union rTS11_TSBA {
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
} TS11_TSBA;

// A silo word
typedef union rTS11_SILO_WORD {
    uint16_t word;
    uint8_t byte[2];
} TS11_SILO_WORD;

// Message Packet Header Word
typedef union rTS11_MSG_PKT_HDR {
    uint16_t word;
    uint8_t byte[2];
    struct {
	uint16_t Message_Class:5;
	uint16_t Format:3;
	uint16_t Class_Code:4;
	uint16_t Reserved:3;
	uint16_t ACK:1;
    } __attribute__((packed));
} TS11_MSG_PKT_HDR;

// TSSR
typedef union rTS11_TSSR {
    uint16_t word;
    uint8_t byte[2];
    struct {
	uint16_t _unused_0:1;
	uint16_t TC:3;  // Termination Class
	uint16_t FC:2;  // Fatal Termination Class
	uint16_t OFL:1; // Transport Off-Line
	uint16_t SSR:1; // SubSystem Ready
	uint16_t ADX:2; // Address eXtension
	uint16_t NBA:1; // Need message Buffer Address
	uint16_t NXM:1; // NXM error
	uint16_t RMR:1; // Register Modification Refused
	uint16_t SPE:1; // Serial bus Parity Error
	uint16_t UPE:1; // Unibus Parity Error
	uint16_t SC:1;  // Special Condition
    } __attribute__((packed));
} TS11_TSSR;

// XSTAT0
typedef union rTS11_XSTAT0 {
    uint16_t word;
    uint8_t byte[2];
    struct {
	uint16_t EOT:1; // End Of Tape
	uint16_t BOT:1; // Beginning Of Tape
	uint16_t WLK:1; // Write Locked
	uint16_t PED:1; // Phase-Encoded Drive
	uint16_t VCK:1; // Volume Check
	uint16_t IE:1;  // Interrupt Enable
	uint16_t ONL:1; // Transport On-Line
	uint16_t MOT:1; // Capstan is moving
	uint16_t ILA:1; // Illegal Address
	uint16_t ILC:1; // Illegal Command
	uint16_t NEF:1; // Non-Executable Function
	uint16_t WLE:1; // Write Lock Error
	uint16_t RLL:1; // Record Length Long
	uint16_t LET:1; // Logical End of Tape
	uint16_t RLS:1; // Record Length Short
	uint16_t TMK:1; // Tape Mark Detected
    } __attribute__((packed));
} TS11_XSTAT0;

// XSTAT1
typedef union rTS11_XSTAT1 {
    uint16_t word;
    uint8_t byte[2];
    struct {
	uint16_t MTE:1; // Multi-Track Error
	uint16_t UNC:1; // Uncorrectable Error
	uint16_t POL:1; // Postamble Long
	uint16_t POS:1; // Postamble Short
	uint16_t IED:1; // Invalid End of Data
	uint16_t IPO:1; // Invalid Postable
	uint16_t SYN:1; // Synchronization Failure
	uint16_t IPR:1; // Invalid Preamble
	uint16_t _unused_8:1;
	uint16_t SCK:1; // Speed Check (Aspen Two Zero, I show...)
	uint16_t DBF:1; // Deskew Buffer Fail
	uint16_t TIG:1; // Trash in the Gap (Someone failed to mind the gap!)
	uint16_t CRS:1; // Crease Detected
	uint16_t COR:1; // Correctable Data
	uint16_t _unused_14:1;
	uint16_t DLT:1; // Data Late
    } __attribute__((packed));
} TS11_XSTAT1;

// XSTAT2
typedef union rTS11_XSTAT2 {
    uint16_t word;
    uint8_t byte[2];
    struct {
	uint16_t DT:8;  // Dead Track 0-7
	uint16_t DTP:1; // Dead Track Parity
	uint16_t _unused_9:1;
	uint16_t WCF:1; // Write Card Failure
	uint16_t _unused_11:1;
	uint16_t CAF:1; // Capstan Acceleration Failure
	uint16_t BPE:1; // Serial Bus Parity Error at drive
	uint16_t SIP:1; // Silo Parity Error
	uint16_t OPM:1; // Operation in progress (tape moved)
    } __attribute__((packed));
} TS11_XSTAT2;

// XSTAT3
typedef union rTS11_XSTAT3 {
    uint16_t word;
    uint8_t byte[2];
    struct {
	uint16_t RIB:1; // Reversed Into BOT
	uint16_t LXS:1; // (Tape Tension) Limit Exceeded Statically
	uint16_t NOI:1; // (COME ON, FEEL THE) Noise Record
	uint16_t DCK:1; // Density Check
	uint16_t CRF:1; // Capstan Response Fail
	uint16_t REV:1; // Reverse
	uint16_t OPI:1; // Operation Incomplete (based on tape movement)
	uint16_t LMX:1; // (Tape Tension) Limit Exceeded
	uint16_t Microdiagnostic_Error_Code:8; // Only one, 377 means that the capstan was commanded to stop and didn't.
    } __attribute__((packed));
} TS11_XSTAT3;

// The entire message packet
typedef struct rTS11_MESSAGE_PACKET {
    TS11_MSG_PKT_HDR Header;
    uint16_t Extent;
    uint16_t RBPCR;
    TS11_XSTAT0 XSTAT0;
    TS11_XSTAT0 XSTAT1;
    TS11_XSTAT0 XSTAT2;
    TS11_XSTAT0 XSTAT3;
} __attribute__((packed)) TS11_MESSAGE_PACKET;

// Command Packet Header Word
typedef union rTS11_CMD_PKT_HDR {
    uint16_t word;
    uint8_t byte[2];
    struct {
	uint16_t Code:5;
	uint16_t Format:3;
	uint16_t Mode:4;
	uint16_t SWB:1;
	uint16_t OPP:1;
	uint16_t CVC:1;
	uint16_t ACK:1;
    } __attribute__((packed));
} TS11_CMD_PKT_HDR;

// An entire command packet
typedef struct rTS11_COMMAND_PACKET {
    TS11_CMD_PKT_HDR Header;
    union {
	TS11_TSBA Buffer_Address;
	uint16_t Parameter_Word;
    } __attribute__((packed));
    uint16_t Buffer_Extent;
} __attribute__((packed)) TS11_COMMAND_PACKET;

// The "WRITE CHARACTERISTICS" command's parameter block
typedef union rTS11_WCHR_MODE_WORD {
    uint16_t word;
    uint8_t byte[2];
    struct {
	uint16_t _unused_0_3:4;
	uint16_t ERI:1; // Enable Message Buffer Release Interrupts
	uint16_t EAI:1; // Enable Attention Interrupts
	uint16_t ENB:1; // LET modifier for EAI
	uint16_t ESS:1; // Enable Skip Tape Marks Stop
	uint16_t _unused_8_15:8;
    } __attribute__((packed));
} TS11_WCHR_MODE_WORD;

typedef union rTS11_WCHR_DATA {
    uint32_t dword[2];
    uint16_t word[4];
    uint8_t byte[8];
    struct {
	TS11_TSBA Message_Buffer_Address;
	uint16_t Message_Buffer_Extent;
	TS11_WCHR_MODE_WORD Mode;
    } __attribute__((packed));
} TS11_WCHR_DATA;

// Here's the controller class
class ts11_c: public tapecontroller_c {
private:

    // The tape drive
    ts11_drive_c *transport;

    // Unibus Registers
    qunibusdevice_register_t *TSR0_reg; // Base Address/Data Buffer
    qunibusdevice_register_t *TSSR_reg; // Status Register

    // NPR and BR plumbing
    dma_request_c dma_request = dma_request_c(this);
    intr_request_c intr_request = intr_request_c(this);

    // Private Registers
    TS11_TSBA TSBA; // The real base address register. Not cleared by INIT, power up, or subsystem initialize command.
    TS11_TSSR TSSR; // The real TSSR, since INIT should clobber it only after following the subsystem initialize process.
    TS11_XSTAT0 XSTAT0; // Extended Status, used when writing the message buffer
    TS11_XSTAT1 XSTAT1;
    TS11_XSTAT2 XSTAT2;
    TS11_XSTAT3 XSTAT3;
    TS11_COMMAND_PACKET Command;      // The currently executing command
    // TS11_CMD_PKT_HDR Command;         // The currently executing command
    int Command_Step;                 // Progress indicator for multi-step commands.
    // uint16_t Command_Parameter_Word;  // The command parameter word, used in 2-word commands.
    TS11_TSBA Command_Buffer_Address; // Loaded by command issuance, used to clear the Acknowledge bit.
    // TS11_TSBA Data_Buffer_Address;    // Loaded by command issuance, used to hold address for data/parameters
    uint16_t Data_Buffer_Index;       // Used when doing block transfers
    // uint16_t Data_Buffer_Extent;      // Loaded by command issuance, used to hold the length of the above buffer
    TS11_SILO_WORD SILO;              // The data silo. On the real thing, it's 32 words.
    TS11_WCHR_DATA Characteristics;   // Characteristics, written by the WRITE CHARACTERISTICS command.

    // The status message
    uint16_t RBPCR;

    // The worker's state
    int worker_active;
    int worker_abort;
    int pending_ssi; // Set to 2 for hard resets (clobber command) or 1 for initialize command (perserve command)

    // Internal-use functions
    void update_xstat();
    bool write_message_packet(unsigned msg_class,unsigned class_code);

public:
    ts11_c();
    ~ts11_c();

    void subsystem_init();
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
