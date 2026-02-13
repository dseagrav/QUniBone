/*
  netcom.cpp: Network communications for serial multiplexers.

  Copyright Bogodyne Metatechnics LLC
  Contributed under the BSD 2-clause license.
*/

#include <assert.h>
#include <unistd.h>

#include "timeout.hpp"
#include "logger.hpp"
#include "device.hpp"
#include "netcom.hpp"

// Telnet default negotiations
#define NCTN_GREETING_LENGTH 18
static const char netcon_tn_greeting[NCTN_GREETING_LENGTH] = {
    TN_IAC,TN_WILL,TN_OPT_SUPPRESS_GO_AHEAD,
    TN_IAC,TN_DO,TN_OPT_SUPPRESS_GO_AHEAD,
    TN_IAC,TN_WILL,TN_OPT_ECHO,
    TN_IAC,TN_DO_NOT,TN_OPT_ECHO,
    TN_IAC,TN_WILL_NOT,TN_OPT_LINE_MODE,
    TN_IAC,TN_DO_NOT,TN_OPT_LINE_MODE};

// Telnet timing mark
static const char netcon_tn_timing_mark[3] = {TN_IAC,TN_WILL,TN_OPT_TIMING_MARK};

netcon_c::netcon_c(netcom_c *_ctl,int _fd,struct sockaddr_in *_sa,netcom_mux_c *_mux,netcom_line_c *_slu,int _line){
    ctl = _ctl;
    fd = _fd;
    mux = _mux;
    slu = _slu;
    line = _line;
    protocol_state = -1; // Initialize
    memcpy(&remote_addr,_sa,sizeof(remote_addr));
    name.value = "NC"+std::to_string(ctl->idx)+"C"+std::to_string(fd);
    log_label = "nc"+std::to_string(ctl->idx)+"c"+std::to_string(fd);
    type_name.value = "netcon_c";
}

netcon_c::~netcon_c(){

}

bool netcon_c::on_param_changed(parameter_c *param){
    if(param == &enabled){
	if(!enabled.new_value){
	    // Being disabled. If the connection is open, close it.
	    if(fd > 0){
		close(fd);
		fd = 0;
		ctl->on_netcon_closed(this);
	    }
	}
    }
    return(device_c::on_param_changed(param));
}

void netcon_c::on_power_changed(signal_edge_enum aclo_edge, signal_edge_enum dclo_edge){
    UNUSED(aclo_edge);
    UNUSED(dclo_edge);
}

void netcon_c::on_init_changed(void){

}

void netcon_c::on_worker_terminated(unsigned instance){
    UNUSED(instance);
    if(enabled.value){
	enabled.set(false);
    }
    delete(this);
}

bool netcon_c::transmit_data(uint8_t data){
    if(fd <= 0){ return(false); }
    ssize_t res;
    if(!ctl->raw_mode.value && data == 0xFF){
	// Prefix with IAC
	res = write(fd,&data,1);
	if(res != 1){
	    ERROR("write(): %s",strerror(errno));
	    close(fd);
	    fd = 0;
	    ctl->on_netcon_closed(this);
	}
    }
    res = write(fd,&data,1);
    if(res != 1){
	ERROR("write(): %s",strerror(errno));
	close(fd);
	fd = 0;
	ctl->on_netcon_closed(this);
    }
    return(true);
}

void netcon_c::worker(unsigned instance){
    UNUSED(instance);
    uint8_t incoming_byte;
    ssize_t res;
    // Only one. Listen on the connection and handle the input.
    worker_init_realtime_priority(none_rt); // Does not need to be realtime
    while(!workers_terminate && fd > 0){
	timeout_c retry_timeout;
	switch(protocol_state){

	case NCPS_INIT:
	    if(!ctl->raw_mode.value){
		res = write(fd,&netcon_tn_greeting,NCTN_GREETING_LENGTH);
		if(res != NCTN_GREETING_LENGTH){
		    ERROR("write(): %s",strerror(errno));
		    close(fd);
		    fd = 0;
		    ctl->on_netcon_closed(this);
		}
	    }
	    protocol_state = NCPS_MAIN;
	    break;

	case NCPS_MAIN:
	    res = read(fd,(char *)&incoming_byte,1);
	    if(res == 1){
		if(!ctl->raw_mode.value && incoming_byte == TN_IAC){
		    protocol_state = NCPS_IAC;
		}else{
		    // Send character to line. Characters not accepted in 1 second are abandoned.
		    retry_timeout.start_ms(1000);
		    while(1){
			bool win = slu->recv_data_from_nc(incoming_byte);
			if(win || retry_timeout.reached()){ break; }
		    }
		}
	    }else{
		if(res != 0){
		    // Error happened?
		    ERROR("read(): %s",strerror(errno));
		}
		close(fd);
		fd = 0;
		ctl->on_netcon_closed(this);
	    }
	    break;

	case NCPS_IAC:
	    res = read(fd,(char *)&incoming_byte,1);
	    if(res == 1){
		switch(incoming_byte){

		case TN_IAC:
		    // Escaped IAC
		    retry_timeout.start_ms(1000);
		    while(1){
			bool win = slu->recv_data_from_nc(incoming_byte);
			if(win || retry_timeout.reached()){ break; }
		    }
		    break;

		case TN_WILL:
		    protocol_state = NCPS_IAC_WILL;
		    break;

		case TN_WILL_NOT:
		    protocol_state = NCPS_IAC_WILL_NOT;
		    break;

		case TN_DO:
		    protocol_state = NCPS_IAC_DO;
		    break;

		case TN_DO_NOT:
		    protocol_state = NCPS_IAC_DO_NOT;
		    break;

		case TN_INTERRUPT_PROCESS:
		    // Client will send this sometimes if ^C is typed?
		    protocol_state = NCPS_MAIN;
		    break;

		case TN_BREAK:
		    // Tell the MUX about this
		    protocol_state = NCPS_MAIN;
		    break;

		case TN_ARE_YOU_THERE:
		    // Yes, we are. Send back a bell.
		    incoming_byte = 007;
		    res = write(fd,&incoming_byte,1);
		    if(res != 1){
			ERROR("write(): %s",strerror(errno));
			close(fd);
			fd = 0;
			ctl->on_netcon_closed(this);
		    }
		    protocol_state = NCPS_MAIN;
		    break;

		case TN_ABORT_OUTPUT:
		    // Tell the MUX about this?
		    protocol_state = NCPS_MAIN;
		    break;

		case TN_ERASE_CHARACTER:
		    // Tell the MUX about this?
		    protocol_state = NCPS_MAIN;
		    break;

		case TN_ERASE_LINE:
		    // Tell the MUX about this?
		    protocol_state = NCPS_MAIN;
		    break;

		default:
		    ERROR("Unhandled telnet option 0x%.2X (%.3o) after IAC",incoming_byte,incoming_byte);
		    close(fd);
		    fd = 0;
		    ctl->on_netcon_closed(this);
		}
	    }else{
		if(res != 0){
		    // Error happened?
		    ERROR("read(): %s",strerror(errno));
		}
		close(fd);
		fd = 0;
		ctl->on_netcon_closed(this);
	    }
	    break;

	case NCPS_IAC_WILL:
	    res = read(fd,(char *)&incoming_byte,1);
	    if(res == 1){
		switch(incoming_byte){

		case TN_OPT_SUPPRESS_GO_AHEAD:
		    // Good, we don't want it.
		    protocol_state = NCPS_MAIN;
		    break;

		default:
		    ERROR("Unhandled telnet option 0x%.2X (%.3o) after WILL",incoming_byte,incoming_byte);
		    close(fd);
		    fd = 0;
		    ctl->on_netcon_closed(this);
		}
	    }else{
		if(res != 0){
		    // Error happened?
		    ERROR("read(): %s",strerror(errno));
		}
		close(fd);
		fd = 0;
		ctl->on_netcon_closed(this);
	    }
	    break;

	case NCPS_IAC_WILL_NOT:
	    res = read(fd,(char *)&incoming_byte,1);
	    if(res == 1){
		switch(incoming_byte){

		default:
		    ERROR("Unhandled telnet option 0x%.2X (%.3o) after WILL NOT",incoming_byte,incoming_byte);
		    close(fd);
		    fd = 0;
		    ctl->on_netcon_closed(this);
		}
	    }else{
		if(res != 0){
		    // Error happened?
		    ERROR("read(): %s",strerror(errno));
		}
		close(fd);
		fd = 0;
		ctl->on_netcon_closed(this);
	    }
	    break;

	case NCPS_IAC_DO:
	    res = read(fd,(char *)&incoming_byte,1);
	    if(res == 1){
		switch(incoming_byte){

		case TN_OPT_ECHO:
		case TN_OPT_SUPPRESS_GO_AHEAD:
		    // Good, we want to.
		    protocol_state = NCPS_MAIN;
		    break;

		case TN_OPT_TIMING_MARK:
		    // Send IAC WILL TIMING-MARK, see https://www.rfc-editor.org/rfc/rfc860.html
		    res = write(fd,&netcon_tn_timing_mark,3);
		    if(res != NCTN_GREETING_LENGTH){
			ERROR("write(): %s",strerror(errno));
			close(fd);
			fd = 0;
			ctl->on_netcon_closed(this);
		    }
		    break;

		default:
		    ERROR("Unhandled telnet option 0x%.2X (%.3o) after DO",incoming_byte,incoming_byte);
		    close(fd);
		    fd = 0;
		    ctl->on_netcon_closed(this);
		}
	    }else{
		if(res != 0){
		    // Error happened?
		    ERROR("read(): %s",strerror(errno));
		}
		close(fd);
		fd = 0;
		ctl->on_netcon_closed(this);
	    }
	    break;

	case NCPS_IAC_DO_NOT:
	    res = read(fd,(char *)&incoming_byte,1);
	    if(res == 1){
		switch(incoming_byte){

		default:
		    ERROR("Unhandled telnet option 0x%.2X (%.3o) after DO NOT",incoming_byte,incoming_byte);
		    close(fd);
		    fd = 0;
		    ctl->on_netcon_closed(this);
		}
	    }else{
		if(res != 0){
		    // Error happened?
		    ERROR("read(): %s",strerror(errno));
		}
		close(fd);
		fd = 0;
		ctl->on_netcon_closed(this);
	    }
	    break;

	default:
	    FATAL("Unhandled protocol state %d",protocol_state);
	    protocol_state = -1;
	    break;
	}
    }
}

netcom_c::netcom_c(int _idx){
    idx = _idx;
    name.value = "NC";
    log_label = "nc";
    name.value = name.value+std::to_string(idx);
    log_label = log_label+std::to_string(idx);
    type_name.value = "netcom_c";
    raw_mode.value = false;
    port_number.value = 0;
    socket_fd = 0;
}

netcom_c::~netcom_c(){

}

bool netcom_c::on_param_changed(parameter_c *param){
    if(param == &enabled){
	if(enabled.new_value){
	    // Enabling
	    if(!enable_socket()){ return(false); }
	}else{
	    // Disabling
	    disable_socket();
	}
    }
    return(device_c::on_param_changed(param));
}

bool netcom_c::enable_socket(){
    // Port supplied?
    if(port_number.value == 0){
	ERROR("Port number must be non-zero");
	return(false);
    }
    socket_fd = socket(AF_INET,SOCK_STREAM,0);
    if(socket_fd < 0){
	ERROR("socket(): %s",strerror(errno));
	socket_fd = 0;
	return(false);
    }
    // Become reusable
    int flags = 1;
    if(setsockopt(socket_fd,SOL_SOCKET,SO_REUSEADDR,&flags,sizeof(flags)) < 0){
	ERROR("setsockopt(): %s",strerror(errno));
	close(socket_fd);
	socket_fd = 0;
	return(false);
    }
    // Initialize sockaddr_in for socket
    struct sockaddr_in socket_addr;
    memset((char *)&socket_addr,0,sizeof(socket_addr));
    socket_addr.sin_family = AF_INET;
    socket_addr.sin_addr.s_addr = INADDR_ANY;
    socket_addr.sin_port = htons(port_number.value);
    // Bind socket
    if(bind(socket_fd,(struct sockaddr *)&socket_addr,sizeof(socket_addr)) < 0){
	ERROR("bind(): %s",strerror(errno));
	close(socket_fd);
	socket_fd = 0;
	return(false);
    }
    // Start listening
    listen(socket_fd,4);
    INFO("Listening on port %d",port_number.value);
    // Make port read-only
    port_number.readonly = true;
    return(true);
}

bool netcom_c::disable_socket(){
    if(socket_fd > 0){
	close(socket_fd);
	socket_fd = 0;
    }
    port_number.readonly = false;
    return(true);
}


void netcom_c::on_power_changed(signal_edge_enum aclo_edge, signal_edge_enum dclo_edge){
    UNUSED(aclo_edge);
    UNUSED(dclo_edge);
}

void netcom_c::on_init_changed(void){

}

void netcom_c::on_netcon_closed(netcon_c *conn){
    // Remove this connection from netcons, it's going to delete itself.
    // Also disconnect it from the MUX if it's connected to one.
    if(conn->mux != NULL){
	conn->mux->release_line(conn->line);
    }
    netcons.erase(find(netcons.begin(),netcons.end(),conn));
    INFO("Connection closed, %d remain",netcons.size());
}

bool netcom_c::attach_mux(netcom_mux_c *mux){
    if(find(muxes.begin(),muxes.end(),mux) != muxes.end()){
	return(false); // Already here!
    }
    muxes.push_back(mux);
    return(true);
}

bool netcom_c::detach_mux(netcom_mux_c *mux){
    if(find(muxes.begin(),muxes.end(),mux) == muxes.end()){
	return(false); // Not here!
    }
    muxes.erase(find(muxes.begin(),muxes.end(),mux));
    return(true);
}

void netcom_c::worker(unsigned instance){
    UNUSED(instance);
    // Only one. Listen for new connections and dispatch them.
    worker_init_realtime_priority(none_rt); // Does not need to be realtime
    while(!workers_terminate){
	if(socket_fd > 0){
	    struct sockaddr_in client_addr;
	    socklen_t socklen = sizeof(client_addr);
	    int tmp_fd = accept(socket_fd,(struct sockaddr *)&client_addr,&socklen);
	    if(tmp_fd < 0){
		ERROR("accept(): %s",strerror(errno));
		close(socket_fd);
		socket_fd = 0;
	    }else{
		// We have a new connection. Any free lines on any mux?
		std::vector<netcom_mux_c*>::iterator itr = muxes.begin();
		int line = -1;
		while(itr != muxes.end()){
		    line = (*itr)->find_free_line();
		    if(line >= 0){
			INFO("Found free line @%d",line);
			break;
		    }
		    ++itr;
		}
		if(line < 0){
		    // Bail
		    const char emsg[32] = "No free lines.\r\n";
		    write(tmp_fd,&emsg,strlen(emsg)+1); // If it fails, we don't care
		    close(tmp_fd);
		}else{
		    // Assign it a handler and punt it.
		    netcon_c *newcon = new netcon_c(this,tmp_fd,&client_addr,(*itr),(*itr)->get_slu(line),line);
		    (*itr)->seize_line(line,newcon);
		    netcons.push_back(newcon);
		    INFO("Connection open, %d active",netcons.size());
		    newcon->enabled.set(true);
		}
	    }
	}else{
	    // Delay
	    timeout_c pause;
	    pause.wait_ms(100);
	}
    }
}

