/******************************************************************
 * @file cmd.cpp
 * Copyright 2019 Prachandabhanu, Vishnuprasad

 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License. 
 *************************************************************** */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <iostream>

#include "wsg_50_tcp_driver/common.h"
#include "wsg_50_tcp_driver/msg.h"
#include "wsg_50_tcp_driver/cmd.h"


/** 
 * This constructor initialize TCP socket communication
 * @param *addr String containing IP address
 * @param port Port number (remote)
*/

iwtros::cmd::cmd(const char *addr, unsigned short port){
    int res;
    tcp_params_t params;
    if(!addr) std::cout << "No address is given" << '\n';
    // if(connected) stop the constructor initialization
    params.addr = str_to_ipaddr(addr);
    params.port = port;

    _msg = new msg(&params);
    if(_msg->result)
        connected = true;
    else
        connected = false;
}

iwtros::cmd::~cmd(){
    delete this->_msg;
}

/**
 * Disconnect
 */
bool iwtros::cmd::disconnected(void){
    status_t status;
    int res;
    unsigned char *resp;
    unsigned int resp_len;
    
    printf("Closing the connection\n");
    
    res = this->submit(0x07, NULL, 0, false, &resp, &resp_len);
    if ( res != 2 ) printf( "Disconnect announcement failed: Response payload length doesn't match (is %d, expected 2)\n", res );
	else{
		// Check response status
		status = this->get_response_status( resp );
		if ( status != E_SUCCESS ) printf( "Command ANNOUNCE DISCONNECT not successful: %s\n", status_to_str( status ) );
	}
    if ( res > 0 ) delete resp;
    _msg->close_tcp();
}

/** Send command and wait for answer
 *
 * @param id		Command ID
 * @param len		Payload length
 * @param *payload	Payload data
 * @param pending	Flag indicating whether CMD_PENDING
 * 					is allowed return status
 * @return Number of bytes received. -1 on error.
 */
int iwtros::cmd::submit(unsigned char id, unsigned char *payload, unsigned int len, 
                        bool pending, unsigned char **response, unsigned int *response_len){

    int res;
    status_t status;

    msg_t msg_sr ={msg_sr.id = id,
                    msg_sr.len = len,
                    msg_sr.data = payload};
    if(!connected){
        fprintf(stderr, "Interface is not connected\n");
        return -1;
    }

    //Send command
    res = _msg->send(&msg_sr);
    if ( res < 0 ){
		fprintf( stderr, "Message send failed\n" );
		return -1;
	}

    // Reuse message struct to receive response
	memset( &msg_sr, 0, sizeof( msg_sr ) );
    //Receive response. Repeat if pending
    do{
        //Free response
        _msg->_delete(&msg_sr);
        //Receive response
        res = _msg->receive(&msg_sr);
        if(res < 0){
            fprintf(stderr, "Message receive failed\n");
            return -1;
        }

        if(msg_sr.id != id){
            fprintf( stderr, "Response ID (%2x) does not match submitted command ID (%2x)\n", msg_sr.id, id );
			return -1;
        }
        
        if(pending){
            if(msg_sr.len < 2){
                fprintf( stderr, "No status code received\n" );
				return -1;
            }
            status = (status_t) make_short(msg_sr.data[0], msg_sr.data[1]);
        }
    }while (pending && status == E_CMD_PENDING);

    *response_len = msg_sr.len;
    if(msg_sr.len > 0) *response = msg_sr.data;
    else *response = 0;
    
    return (int)msg_sr.len;
}

/** Get connection state
 * @return true if the command interface is connected, else false
 */
bool iwtros::cmd::is_connected( void ){
    return connected;
}

/** Get status code from response data
 * @return Status code
 */
status_t iwtros::cmd::get_response_status( unsigned char *response ){
	status_t status;
	assert( response != NULL );
	status = (status_t) make_short( response[0], response[1] );
	return status;
}