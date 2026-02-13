/*
  tapedrive.cpp: A tape drive, with an image file or possibly a physical tape drive as storage medium.

  Copyright Bogodyne Metatechnics LLC
  Contributed under the BSD 2-clause license.

  OK, so storagedrive looks not ready for tape drive support right now, so I'm creating this as a stopgap solution
  to enable work on the TS11 to proceed. I don't know how much time I'll have to work on this.

  Someone else can either merge this into storagedrive later or decide it's a better solution, whatever works.

*/

#include "logger.hpp"
#include "tapecontroller.hpp"
#include "tapedrive.hpp"

// SIMH block header
typedef union rSIMH_BLOCK_HEADER {
    uint32_t dword;
    uint16_t word[2];
    uint8_t byte[4];
    struct {
	uint32_t Value:27;
	uint32_t Class:4;
    } __attribute__((packed));
} SIMH_BLOCK_HEADER;

tapedrive_c::tapedrive_c(tapecontroller_c *_controller): device_c(){
    // Note the controller
    controller = _controller;

    // Clobber as required
    image = nullptr;
    image_format = TFMT_NONE;
    desired_image_format.value = "";
    Op_Result.Value = 0;
    Op_Result.Code = TPOP_OK;
}

tapedrive_c::~tapedrive_c(){
    image_delete();
}

void tapedrive_c::image_delete(){
    if(image == nullptr){ return; } // Bail if no image
    if(image->is_open()){ image->close(); }
    storageimage_base_c *tmpimage = image; // Fetch into temporary class decl
    image = nullptr; // nuke it, dot it
    delete tmpimage; // help the whelp groups
}

bool tapedrive_c::is_loaded(void){
    if(image != nullptr && image->is_open()){
	return(true);
    }
    return(false);
}

bool tapedrive_c::is_at_bot(){
    if(image != nullptr && image->is_open() && image->getpos() == 0){
	return(true);
    }
    return(false);
}

// Called to automatically detect the format of the image file given.
// Returns false if unsuccessful, otherwise sets image_format and returns true
bool tapedrive_c::autodetect_image_format(){
    // Image file shorter than 8 bytes?
    uint64_t image_size = image->size();
    if(image_size < 8){
	ERROR("Image file is too short for autodetection.");
	return(false);
    }
    // Read 32 bits from the file
    uint32_t record_size;
    if(image->read((uint8_t *)&record_size,4) != 4){
	ERROR("Failed to read 4 bytes from the image file, format detection failed.");
	return(false);
    }
    /* Finish this later
    // Is the result larger than the file size?
    if(record_size > image_size){
	// Yes, this is either a TPC file, or a SIMH-Extended file.
	// Check SIMH-Extended class value
	switch((image_size&0xF0000000)){

	case 0x00000000: // Data Record
	case 0x80000000: // Bad Data Record
	    if((image_size&0x0FFFFFFF) > image_size){
		// Truncated image or TPC image
	    }else{
		// Possible SIMH-Extended file
	    }
	    break;

	default:
	    // Other classes are implausibe
	}
    }else{
	// No. All options still on the table.
    }
    */
    ERROR("Image format autodetection incomplete.");
    return(false);
}

// Verify that the image file given conforms to the format selected in image_format.
bool tapedrive_c::verify_image_format(tape_image_format_enum check_format){
    uint64_t image_size = image->size();
    image->setpos(0); // Seek to start
    switch(check_format){

    case TFMT_SIMH_STD:
    {
	uint64_t present_position = 0;
	SIMH_BLOCK_HEADER block_header;
	bool done = false;
	while(!done){
	    if(image->read((uint8_t *)&block_header.dword,4) != 4){
		ERROR("Failed to read 4 bytes from the image file at position %d, verification failed",
		      present_position);
		return(false);
	    }
	    present_position += 4;
	    switch(block_header.Class){

	    case 0x0: // Tape Mark or Data Record
		if(block_header.dword == 0){
		    // Tape Mark. If we are at the end of the file, we win, otherwise carry on.
		    if(present_position == image_size){
			// Winner!
			return(true);
		    }
		}else{
		    // Data Record
		    if(present_position+block_header.Value > image_size){
			ERROR("Block of %d bytes at position %d exceeds image file size; Image truncated or not SIMH-Standard",
			      block_header.Value,present_position);
			return(false);
		    }else{
			// Seek past the block
			image->setpos(present_position+block_header.Value);
			present_position += block_header.Value;
			// Read the tail and check it
			uint32_t block_trailer = 0;
			if(image->read((uint8_t *)&block_trailer,4) != 4){
			    ERROR("Failed to read 4 bytes from the image file at position %d, verification failed",
				  present_position);
			    return(false);
			}
			if(block_trailer != block_header.dword){
			    ERROR("Block trailer %.8X does not match block header %.8X, image corrupted or not SIMH-Standard",
				  block_trailer,block_header.dword);
			}
			present_position += 4;
			// At end of file?
			if(present_position == image_size){
			    // Winner!
			    return(true);
			}
		    }
		}
		break;

	    case 0x8: // Bad Data Record
	    case 0xF: // Reserved


	    default:
		ERROR("Invalid block header class %X seen at position %d, not SIMH-Standard format",
		      block_header.Class,present_position);
		return(false);
	    }
	}
    }
    break;

    default:
	ERROR("Check format %d not implented");
	return(false);
    }

    ERROR("Image format verification incomplete.");
    return(false);
}

bool tapedrive_c::on_param_changed(parameter_c *param){
    // Is the image being changed?
    if(param == &image_filepath){
	// Yes. Are we clobbering it?
	if(image != nullptr){
	    // Yes, dismount the old image.
	    image_delete();
	}
	// If image format is autodetect, we can't allow creation.
	image = new storageimage_rawfile_c(image_filepath.new_value);
	if(image == nullptr){
	    ERROR("Unable to create image file handle");
	    return(false);
	}
	// The image is instantiated.
	// Leave it here until a load operation is performed.
	return(true);
    }
    // Changing format?
    if(param == &desired_image_format){
	// Image mounted?
	if(image == nullptr){
	    // No. Valid type?
	    if(strcasecmp("",desired_image_format.new_value.c_str()) == 0 ||
	       strcasecmp("auto",desired_image_format.new_value.c_str()) == 0){
		// Nothing/Auto
		desired_image_format.new_value = "";
		image_format = TFMT_NONE;
		return(true);
	    }
	    if(strcasecmp("tpc",desired_image_format.new_value.c_str()) == 0){
		image_format = TFMT_TPC;
		desired_image_format.new_value = "TPC";
		return(true);
	    }
	    if(strcasecmp("simh",desired_image_format.new_value.c_str()) == 0){
		image_format = TFMT_SIMH_STD;
		desired_image_format.new_value = "SIMH";
		return(true);
	    }
	    if(strcasecmp("simh-extended",desired_image_format.new_value.c_str()) == 0){
		image_format = TFMT_SIMH_XTD;
		desired_image_format.new_value = "SIMH-EXTENDED";
		return(true);
	    }
	    if(strcasecmp("e11",desired_image_format.new_value.c_str()) == 0){
		image_format = TFMT_E11;
		desired_image_format.new_value = "E11";
		return(true);
	    }
	    ERROR("Invalid image format. Must be empty or one of auto,tpc,simh,simh-extended,or e11.");
	    return(false);
	}else{
	    ERROR("Can't change image formats while an image is mounted.");
	    return(false);
	}
    }
    // Fall through
    return(device_c::on_param_changed(param));
}

// Tape operations
bool tapedrive_c::do_load_op(){
    if(image == nullptr){
	Op_Result.Code = TPOP_NO_IMAGE;
	Op_Result.Value = 0;
	return(false);
    }
    // Creating a new image requires that the format be specified.
    if(image_format != TFMT_NONE){
	if(!image->open(nullptr,true)){
	    // Additional complaint
	    ERROR("Unable to open or create the specified file.");
	    Op_Result.Code = TPOP_IO_ERROR;
	    Op_Result.Value = 0;
	    return(false);
	}
    }else{
	if(!image->open(nullptr,false)){
	    ERROR("Unable to open the specified file.");
	    ERROR("NB: Tape image format must be specified to create new images.");
	    Op_Result.Code = TPOP_IO_ERROR;
	    Op_Result.Value = 0;
	    return(false);
	}
    }
    // Now handle format detection and/or verification.
    // Later.
    /*
    if(image->size() != 0){
	if(image_format == TFMT_NONE){
	    if(!autodetect_image_format()){
		image_delete();
		return(false);
	    }
	}else{
	    if(!verify_image_format()){
		image_delete();
		return(false);
	    }
	}
    }
    */
    // If we are still here, we won.
    // Go to BOT because it won't by default
    image->setpos(0);
    Op_Result.Code = TPOP_OK;
    Op_Result.Value = 0;
    return(true);
}

bool tapedrive_c::do_unload_op(){
    if(image == nullptr || !image->is_open()){
	Op_Result.Code = TPOP_NO_IMAGE;
	Op_Result.Value = 0;
	return(false);
    }
    image_delete();
    Op_Result.Code = TPOP_OK;
    Op_Result.Value = 0;
    return(true);
}

bool tapedrive_c::do_rewind_op(){
    if(image == nullptr || !image->is_open()){
	Op_Result.Code = TPOP_NO_IMAGE;
	Op_Result.Value = 0;
	return(false);
    }
    image->setpos(0);
    Op_Result.Code = TPOP_OK;
    Op_Result.Value = 0;
    return(true);
}

bool tapedrive_c::do_start_read_op(){
    if(image == nullptr || !image->is_open()){
	ERROR("do_start_read_op(): No image mounted");
	Op_Result.Code = TPOP_NO_IMAGE;
	Op_Result.Value = 0;
	return(false);
    }
    switch(image_format){

    case TFMT_SIMH_STD:
    case TFMT_SIMH_XTD:
    {
	SIMH_BLOCK_HEADER block_header;
	unsigned rv = image->read((uint8_t *)&block_header,4);
	if(rv != 4){
	    if(rv == 0 && image->is_eof()){
		Op_Result.Code = TPOP_IMAGE_EOF;
		Op_Result.Value = 0;
		return(false);
	    }else{
		ERROR("do_start_read_op(): image->read() returned %d",rv);
		Op_Result.Code = TPOP_IO_ERROR;
		Op_Result.Value = 0;
		return(false);
	    }
	}
	/*
	printf("do_start_read_op(): Got record header class %X, value %d\n",
	       block_header.Class,block_header.Value);
	*/
	switch(block_header.Class){

	case 0:
	    if(block_header.Value != 0){
		// Valid data block
		Op_Result.Code = TPOP_OK;
		Op_Result.Value = block_header.Value;
		return(true);
	    }else{
		// Tape Mark
		Op_Result.Code = TPOP_TAPE_MARK;
		Op_Result.Value = 0;
		return(false);
	    }
	    break;

	default:
	    FATAL("do_start_read_op(): Need handling for SIMH block class %X",block_header.Class);
	    Op_Result.Code = TPOP_GENERAL_FAULT;
	    Op_Result.Value = 0;
	}
    }
    break;

    default:
	FATAL("do_start_data_block_op: Unimplemented image format %d",image_format);
	Op_Result.Code = TPOP_GENERAL_FAULT;
	Op_Result.Value = 0;
    }
    return(false);
}

bool tapedrive_c::do_read_data_op(uint8_t *data){
    if(image == nullptr || !image->is_open()){
	ERROR("do_read_data_op(): No image mounted");
	Op_Result.Code = TPOP_NO_IMAGE;
	Op_Result.Value = 0;
	return(false);
    }
    switch(image_format){

    case TFMT_SIMH_STD:
    case TFMT_SIMH_XTD:
    {
	unsigned rv = image->read(data,1);
	if(rv != 1){
	    ERROR("do_read_data_op(): image->read() returned %d",rv);
	    Op_Result.Code = TPOP_IO_ERROR;
	    Op_Result.Value = 0;
	    return(false);
	}else{
	    Op_Result.Code = TPOP_OK;
	    Op_Result.Value = 0;
	    return(true);
	}
    }
    break;

    default:
	FATAL("do_read_data_op: Unimplemented image format %d",image_format);
	Op_Result.Code = TPOP_GENERAL_FAULT;
	Op_Result.Value = 0;
    }
    return(false);
}

bool tapedrive_c::do_end_read_op(unsigned block_size){
    if(image == nullptr || !image->is_open()){
	ERROR("do_end_read_op(): No image mounted");
	Op_Result.Code = TPOP_NO_IMAGE;
	Op_Result.Value = 0;
	return(false);
    }
    switch(image_format){

    case TFMT_SIMH_STD:
    case TFMT_SIMH_XTD:
    {
	SIMH_BLOCK_HEADER block_trailer;
	unsigned rv = image->read((uint8_t *)&block_trailer,4);
	if(rv != 4){
	    ERROR("do_end_read_op(): image->read() returned %d",rv);
	    Op_Result.Code = TPOP_IO_ERROR;
	    Op_Result.Value = 0;
	    return(false);
	}
	/*
	printf("do_end_read_op(): Got record trailer class %X, value %d\n",
	       block_trailer.Class,block_trailer.Value);
	*/
	switch(block_trailer.Class){

	case 0:
	    if(block_trailer.Value != 0){
		// Valid data block
		if(block_trailer.Value != block_size){
		    FATAL("Block size miscompare, got %d, expected %d",block_trailer.Value,block_size);
		    Op_Result.Code = TPOP_BAD_IMAGE;
		    Op_Result.Value = block_trailer.Value;
		    return(false);
		}else{
		    Op_Result.Code = TPOP_OK;
		    Op_Result.Value = 0;
		    return(true);
		}
	    }else{
		// Tape Mark. Shouldn't be here?
		Op_Result.Code = TPOP_TAPE_MARK;
		Op_Result.Value = 0;
		return(false);
	    }
	    break;

	default:
	    FATAL("do_end_read_op(): Need handling for SIMH block class %X",block_trailer.Class);
	    Op_Result.Code = TPOP_GENERAL_FAULT;
	    Op_Result.Value = 0;
	}
    }
    break;

    default:
	FATAL("do_end_read_op: Unimplemented image format %d",image_format);
	Op_Result.Code = TPOP_GENERAL_FAULT;
	Op_Result.Value = 0;
    }
    return(false);
}

bool tapedrive_c::do_start_read_reverse_op(){
    if(image == nullptr || !image->is_open()){
	ERROR("do_start_read_reverse_op(): No image mounted");
	Op_Result.Code = TPOP_NO_IMAGE;
	Op_Result.Value = 0;
	return(false);
    }
    uint64_t start_pos = image->getpos();
    // At BOT?
    if(start_pos == 0){
	ERROR("do_start_read_reverse_op(): At BOT");
	Op_Result.Code = TPOP_BOT;
	Op_Result.Value = 0;
	return(false);
    }
    switch(image_format){

    case TFMT_SIMH_STD:
    case TFMT_SIMH_XTD:
    {
	SIMH_BLOCK_HEADER block_trailer;
	// Obtain previous block trailer
	image->setpos(start_pos-4);
	unsigned rv = image->read((uint8_t *)&block_trailer,4);
	if(rv != 4){
	    ERROR("do_start_read_reverse_op(): image->read() returned %d",rv);
	    Op_Result.Code = TPOP_IO_ERROR;
	    Op_Result.Value = 0;
	    return(false);
	}
	/*
	printf("do_start_read_reverse_op(): Got record trailer class %X, value %d\n",
	       block_trailer.Class,block_trailer.Value);
	*/
	image->setpos(start_pos-4);
	switch(block_trailer.Class){

	case 0:
	    if(block_trailer.Value != 0){
		// Valid data block
		Op_Result.Code = TPOP_OK;
		Op_Result.Value = block_trailer.Value;
		return(true);
	    }else{
		// Tape Mark
		Op_Result.Code = TPOP_TAPE_MARK;
		Op_Result.Value = 0;
		return(false);
	    }
	    break;

	default:
	    FATAL("do_start_read_reverse_op(): Need handling for SIMH block class %X",block_trailer.Class);
	    Op_Result.Code = TPOP_GENERAL_FAULT;
	    Op_Result.Value = 0;
	}
    }
    break;

    default:
	FATAL("do_start_data_block_op: Unimplemented image format %d",image_format);
	Op_Result.Code = TPOP_GENERAL_FAULT;
	Op_Result.Value = 0;
    }
    return(false);
}

bool tapedrive_c::do_read_reverse_data_op(uint8_t *data){
    if(image == nullptr || !image->is_open()){
	ERROR("do_read_reverse_data_op(): No image mounted");
	Op_Result.Code = TPOP_NO_IMAGE;
	Op_Result.Value = 0;
	return(false);
    }
    switch(image_format){

    case TFMT_SIMH_STD:
    case TFMT_SIMH_XTD:
    {
	uint64_t start_pos = image->getpos();
	if(start_pos == 0){
	    ERROR("do_read_reverse_data_op(): At BOT");
	    Op_Result.Code = TPOP_BOT;
	    Op_Result.Value = 0;
	    return(false);
	}
	image->setpos(start_pos-1);
	unsigned rv = image->read(data,1);
	image->setpos(start_pos-1);
	if(rv != 1){
	    ERROR("do_read_reverse_data_op(): image->read() returned %d",rv);
	    Op_Result.Code = TPOP_IO_ERROR;
	    Op_Result.Value = 0;
	    return(false);
	}else{
	    Op_Result.Code = TPOP_OK;
	    Op_Result.Value = 0;
	    return(true);
	}
    }
    break;

    default:
	FATAL("do_read_reverse_data_op: Unimplemented image format %d",image_format);
	Op_Result.Code = TPOP_GENERAL_FAULT;
	Op_Result.Value = 0;
    }
    return(false);
}

bool tapedrive_c::do_end_read_reverse_op(unsigned block_size){
    if(image == nullptr || !image->is_open()){
	ERROR("do_end_read_reverse_op(): No image mounted");
	Op_Result.Code = TPOP_NO_IMAGE;
	Op_Result.Value = 0;
	return(false);
    }
    uint64_t start_pos = image->getpos();
    if(start_pos == 0){
	ERROR("do_end_read_reverse_op(): At BOT");
	Op_Result.Code = TPOP_BOT;
	Op_Result.Value = 0;
	return(false);
    }
    switch(image_format){

    case TFMT_SIMH_STD:
    case TFMT_SIMH_XTD:
    {
	SIMH_BLOCK_HEADER block_header;
	image->setpos(start_pos-4);
	unsigned rv = image->read((uint8_t *)&block_header,4);
	if(rv != 4){
	    ERROR("do_end_read_reverse_op(): image->read() returned %d",rv);
	    Op_Result.Code = TPOP_IO_ERROR;
	    Op_Result.Value = 0;
	    return(false);
	}
	/*
	printf("do_end_read_reverse_op(): Got record header class %X, value %d\n",
	       block_header.Class,block_header.Value);
	*/
	image->setpos(start_pos-4);
	switch(block_header.Class){

	case 0:
	    if(block_header.Value != 0){
		// Valid data block
		if(block_header.Value != block_size){
		    FATAL("Block size miscompare, got %d, expected %d",block_header.Value,block_size);
		    Op_Result.Code = TPOP_BAD_IMAGE;
		    Op_Result.Value = block_header.Value;
		    return(false);
		}else{
		    Op_Result.Code = TPOP_OK;
		    Op_Result.Value = 0;
		    return(true);
		}
	    }else{
		// Tape Mark. Shouldn't be here?
		Op_Result.Code = TPOP_TAPE_MARK;
		Op_Result.Value = 0;
		return(false);
	    }
	    break;

	default:
	    FATAL("do_end_read_reverse_op(): Need handling for SIMH block class %X",block_header.Class);
	    Op_Result.Code = TPOP_GENERAL_FAULT;
	    Op_Result.Value = 0;
	}
    }
    break;

    default:
	FATAL("do_end_read_reverse_op: Unimplemented image format %d",image_format);
	Op_Result.Code = TPOP_GENERAL_FAULT;
	Op_Result.Value = 0;
    }
    return(false);
}

bool tapedrive_c::do_start_data_block_op(unsigned block_size){
    if(image == nullptr || !image->is_open()){
	ERROR("do_start_data_block_op(): No image mounted");
	Op_Result.Code = TPOP_NO_IMAGE;
	Op_Result.Value = 0;
	return(false);
    }
    switch(image_format){

    case TFMT_SIMH_STD:
    case TFMT_SIMH_XTD:
    {
	SIMH_BLOCK_HEADER block_header;
	block_header.Class = 0;
	block_header.Value = block_size;
	unsigned rv = image->write((uint8_t *)&block_header,4);
	if(rv != 4){
	    ERROR("do_start_data_block_op(): image->write() returned %d",rv);
	    Op_Result.Code = TPOP_IO_ERROR;
	    Op_Result.Value = 0;
	    return(false);
	}else{
	    Op_Result.Code = TPOP_OK;
	    Op_Result.Value = 0;
	    return(true);
	}
    }
    break;

    default:
	FATAL("do_start_data_block_op: Unimplemented image format %d",image_format);
	Op_Result.Code = TPOP_GENERAL_FAULT;
	Op_Result.Value = 0;
    }
    return(false);
}

bool tapedrive_c::do_write_data_op(uint8_t data){
    if(image == nullptr || !image->is_open()){
	ERROR("do_write_data_op(): No image mounted");
	Op_Result.Code = TPOP_NO_IMAGE;
	Op_Result.Value = 0;
	return(false);
    }
    switch(image_format){

    case TFMT_SIMH_STD:
    case TFMT_SIMH_XTD:
    {
	unsigned rv = image->write(&data,1);
	if(rv != 1){
	    ERROR("do_write_data_op(): image->write() returned %d",rv);
	    Op_Result.Code = TPOP_IO_ERROR;
	    Op_Result.Value = 0;
	    return(false);
	}else{
	    Op_Result.Code = TPOP_OK;
	    Op_Result.Value = 0;
	    return(true);
	}
    }
    break;

    default:
	FATAL("do_write_data_op: Unimplemented image format %d",image_format);
	Op_Result.Code = TPOP_GENERAL_FAULT;
	Op_Result.Value = 0;
    }
    return(false);
}

bool tapedrive_c::do_end_data_block_op(unsigned block_size){
    if(image == nullptr || !image->is_open()){
	ERROR("do_end_data_block_op(): No image mounted");
	Op_Result.Code = TPOP_NO_IMAGE;
	Op_Result.Value = 0;
	return(false);
    }
    switch(image_format){

    case TFMT_SIMH_STD:
    case TFMT_SIMH_XTD:
    {
	SIMH_BLOCK_HEADER block_trailer;
	block_trailer.Class = 0;
	block_trailer.Value = block_size;
	unsigned rv = image->write((uint8_t *)&block_trailer,4);
	if(rv != 4){
	    ERROR("do_end_data_block_op(): image->write() returned %d",rv);
	    Op_Result.Code = TPOP_IO_ERROR;
	    Op_Result.Value = 0;
	    return(false);
	}else{
	    Op_Result.Code = TPOP_OK;
	    Op_Result.Value = 0;
	    return(true);
	}
    }
    break;

    default:
	FATAL("do_end_data_block_op: Unimplemented image format %d",image_format);
	Op_Result.Code = TPOP_GENERAL_FAULT;
	Op_Result.Value = 0;
    }
    return(false);
}

bool tapedrive_c::do_write_tape_mark_op(){
   if(image == nullptr || !image->is_open()){
	ERROR("do_write_tape_mark_op(): No image mounted");
	Op_Result.Code = TPOP_NO_IMAGE;
	Op_Result.Value = 0;
	return(false);
    }
    switch(image_format){

    case TFMT_SIMH_STD:
    case TFMT_SIMH_XTD:
    {
	SIMH_BLOCK_HEADER block_header;
	block_header.Class = 0;
	block_header.Value = 0;
	unsigned rv = image->write((uint8_t *)&block_header,4);
	if(rv != 4){
	    ERROR("do_write_tape_mark_op(): image->write() returned %d",rv);
	    Op_Result.Code = TPOP_IO_ERROR;
	    Op_Result.Value = 0;
	    return(false);
	}else{
	    Op_Result.Code = TPOP_OK;
	    Op_Result.Value = 0;
	    return(true);
	}
    }
    break;

    default:
	FATAL("do_write_tape_mark_op: Unimplemented image format %d",image_format);
	Op_Result.Code = TPOP_GENERAL_FAULT;
	Op_Result.Value = 0;
    }
    return(false);

}
