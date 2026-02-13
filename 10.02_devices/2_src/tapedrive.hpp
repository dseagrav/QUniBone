/*
  tapedrive.hpp: A tape drive, with an image file or possibly a physical tape drive as storage medium.

  Copyright Bogodyne Metatechnics LLC
  Contributed under the BSD 2-clause license.

  OK, so storagedrive looks not ready for tape drive support right now, so I'm creating this as a stopgap solution
  to enable work on the TS11 to proceed. I don't know how much time I'll have to work on this.

  Someone else can either merge this into storagedrive later or decide it's a better solution, whatever works.
*/

#ifndef _TAPEDRIVE_HPP_
#define _TAPEDRIVE_HPP_

#include "utils.hpp"
#include "storageimage.hpp"
#include "device.hpp"
#include "parameter.hpp"

// Supported tape image formats
enum tape_image_format_enum {
    TFMT_NONE = 0,     // No image mounted, or maybe physical tape drive?
    TFMT_TPC = 1,      // .TPC image format
    TFMT_SIMH_STD = 2, // SIMH Standard image format
    TFMT_SIMH_XTD = 3, // SIMH Extended image format
    TFMT_E11 = 4       // E11 image format
};

// Result codes from drive operation functions
enum tape_op_result_enum {
    TPOP_OK = 0,            // Nothing that we are aware has gone wrong yet.
    TPOP_GENERAL_FAULT = 1, // Generic lossage
    TPOP_BOT = 2,           // Drive encountered BOT marker
    TPOP_PHYSICAL_EOT = 3,  // Drive encountered physical EOT marker
    TPOP_LOGICAL_EOT = 4,   // Drive encountered logical EOT marker
    TPOP_IMAGE_EOF = 5,     // Image file EOF, drive should turn this into physical or logical EOT.
    TPOP_TAPE_MARK = 6,     // Drive encountered tape marker
    TPOP_IO_ERROR = 7,      // Drive encountered an OS-signalled IO error condition
    TPOP_NO_IMAGE = 8,      // No image file was given or required image parameters are empty
    TPOP_BAD_IMAGE = 9      // Processing of tape image failed due to corrupt image file or incorrect format
};

// Result structure
typedef struct rTape_Op_Result {
    tape_op_result_enum Code;
    int32_t Value;
} Tape_Op_Result;

// And now the actual class declaration
class tapecontroller_c;
class tapedrive_c: public device_c {
private:

    // The underlying image file, if there is one. If storageimage_binfile_c ever accumulates something
    // specific to disks and/or simh, this will have to change too...
    storageimage_rawfile_c *image = nullptr;
    tape_image_format_enum image_format = TFMT_NONE;

    // Private functions
    bool autodetect_image_format();
    bool verify_image_format(tape_image_format_enum check_format);

public:
    tapecontroller_c *controller; // link to parent
    // Result of the last tape operation
    Tape_Op_Result Op_Result;
    // User controls
    parameter_string_c image_filepath = parameter_string_c(this,"image","img",false,"Path to tape image file. Empty for none.");
    parameter_string_c desired_image_format = parameter_string_c(this,"format","fmt",false,
							      "Tape image file format. Empty for none/autodetect.");
    // Shared host dir probably can't be supported because flat files do not have blocking factor.
    // Worry about it later.
    virtual bool on_param_changed(parameter_c *param) override;

    // Functions
    tapedrive_c(tapecontroller_c *controller);
    virtual ~tapedrive_c();
    void image_delete();

    // Status information
    bool is_loaded();
    bool is_at_bot();
    // bool is_at_eot();

    // Tape operations
    bool do_load_op();
    bool do_unload_op();
    bool do_rewind_op();
    bool do_start_read_op();
    bool do_read_data_op(uint8_t *data);
    bool do_end_read_op(unsigned block_size);
    bool do_start_read_reverse_op();
    bool do_read_reverse_data_op(uint8_t *data);
    bool do_end_read_reverse_op(unsigned block_size);
    bool do_start_data_block_op(unsigned block_size);
    bool do_write_data_op(uint8_t data);
    bool do_end_data_block_op(unsigned block_size);
    bool do_write_tape_mark_op();
};

#endif
