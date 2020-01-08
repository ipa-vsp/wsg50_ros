/******************************************************************
 * @file function.h
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
#include <cmath>
#include <string>
#include <iostream>

#include "wsg_50_tcp_driver/common.h"
#include "wsg_50_tcp_driver/cmd.h"
#include "wsg_50_tcp_driver/msg.h"
#include "wsg_50_tcp_driver/function.h"


/** 
 * Initialisation
*/
iwtros::function::function(const char *addr, unsigned short port) : cmd(addr, port) {}
iwtros::function::~function() {}

/** 
 * Support function
*/
float iwtros::function::convert(unsigned char *b){
    float temp;
    unsigned int src = 0;
    src = b[3] * 16777216 + b[2] * 65536 + b[1] * 256 + b[0];
    memcpy(&temp, &src, sizeof(temp));          // This looks like wrong
    return temp;
}

/** 
 * Homming
*/
int iwtros::function::homing(void){
    status_t status;
    int res;
	unsigned char payload[1];
	unsigned char *resp;
	unsigned int resp_len;

    // Set flags: Homing direction (0: default, 1: widthitive movement, 2: negative movement).
	payload[0] = 0x00;

    // Submit command and wait for response. Push result to stack.
    res = this->submit(0x20, payload, 1, true, &resp, &resp_len);
    if ( res != 2 ){
		dbgPrint( "Response payload length doesn't match (is %d, expected 2)\n", res );
		if ( res > 0 ) delete resp;
		return 0;
	}

    //Check response status
    status = this->get_response_status(resp);
    delete resp;
    if(status != E_SUCCESS){
        dbgPrint( "Command HOMING not successful: %s\n", status_to_str( status ) );
		return -1;
    }
    return 0;
}

/** Move
 * @brief Send move command (0x21) to gripper
 * @param ignore_response Do not read back response from gripper. (Must be read elsewhere, for auto update.)
*/
bool iwtros::function::move( float width, float speed, bool stop_on_block, bool ignore_response){
    status_t status;
	int res;
	unsigned char payload[9];
	unsigned char *resp;
	unsigned int resp_len;

    // Set flags: Absolute movement (bit 0 is 0), stop on block (bit 1 is 1).
	payload[0] = 0x00;
    if(stop_on_block) payload[0] |= 0x02;

    //Copy target width and speed
    memcpy(&payload[1], &width, sizeof(float));
    memcpy(&payload[5], &speed, sizeof(float));

    if(!ignore_response){
        res = this->submit(0x21, payload, 9, true, &resp, &resp_len);
        if(res != 2){
            dbgPrint( "Response payload length doesn't match (is %d, expected 2)\n", res );
            if ( res > 0 ) delete resp;
            return true;            //Chage the return to 0
        }

        //Check response status
        status = this->get_response_status(resp);
        delete resp;
        if(status != E_SUCCESS){
            dbgPrint( "Command MOVE not successful: %s\n", status_to_str( status ) );
            return false;
        }
    }else{
        //Submit the command, do not wait for the response
        msg_t msg_send;
        msg_send.id = 0x21; msg_send.len = 9; msg_send.data = &payload[0];
        res = this->_msg->send(&msg_send);
        if (res <= 0) {
            dbgPrint("Failed to send command MOVE\n");
            return false;
        }
    }
    return true;
}

/** 
 * Stop
*/
int iwtros::function::stop( bool ignore_response ){
    status_t status;
	int res;
	unsigned char payload[1];
	unsigned char *resp;
	unsigned int resp_len;

    if(!ignore_response){
        res = this->submit(0x22, payload, 0, true, &resp, &resp_len);
        if(res != 2){
            dbgPrint( "Response payload length doesn't match (is %d, expected 2)\n", res );
            if ( res > 0 ) delete resp;
            return 0;          
        }

        //Check response status
        status = this->get_response_status(resp);
        delete resp;
        if(status != E_SUCCESS){
            dbgPrint( "Command STOP not successful: %s\n", status_to_str( status ) );
            return -1;
        }
    }else{
        //Submit the command, do not wait for the response
        msg_t msg_send;
        msg_send.id = 0x22; msg_send.len = 0; msg_send.data = &payload[0];
        res = this->_msg->send(&msg_send);
        if (res <= 0) {
            dbgPrint("Failed to send command STOP\n");
            return -1;
        }
    }
    return 0;
}

/** 
 * Acknowledge Fault
*/

int iwtros::function::ack_fault(void){
    status_t status;
	int res;
	unsigned char payload[3];
	unsigned char *resp;
	unsigned int resp_len;

    payload[0] = 0x61;                  //Refere datasheet
    payload[1] = 0x63;
    payload[2] = 0x6B;

    //Submit the command and wait for the response
    res = this->submit(0x24, payload, 3, true, &resp, &resp_len);
    if(res != 2){
        dbgPrint( "Response payload length doesn't match (is %d, expected 2)\n", res );
        if ( res > 0 ) delete resp;
        return 0;          
    }

    //Check response status
    status = this->get_response_status(resp);
    delete resp;
    if(status != E_SUCCESS){
        dbgPrint( "Command ACK not successful: %s\n", status_to_str( status ) );
        return -1;
    }
    return 0;
}

/** Grasp
 * @param objWidth Object width
 * @param speed grasp speed
 * @return true if successful and false if fail
*/
bool iwtros::function::grasp (float objWidth, float speed){
    status_t status;
	int res;
	unsigned char payload[8];
	unsigned char *resp;
	unsigned int resp_len;

    // Copy part width and speed
	memcpy( &payload[0], &objWidth, sizeof( float ) );
	memcpy( &payload[4], &speed, sizeof( float ) );

    // Submit command and wait for response. Push result to stack.
    res = this->submit(0x25, payload, 8, true, &resp, &resp_len);
    if(res != 2){
        dbgPrint( "Response payload length doesn't match (is %d, expected 2)\n", res );
        if ( res > 0 ) delete resp;
        return true;            //Chage the return to 0
    }

    //Check response status
    status = this->get_response_status(resp);
    delete resp;
    if(status != E_SUCCESS){
        dbgPrint( "Command GRASP not successful: %s\n", status_to_str( status ) );
        return false;
    }
    return true;
}

/** Release
 * @param Width Fingure with 
 * @param speed release speed
 * @return 0 if successful and -1 if fail
*/
int iwtros::function::release(float width, float speed){
    status_t status;
	int res;
	unsigned char payload[8];
	unsigned char *resp;
	unsigned int resp_len;

    // Copy part width and speed
	memcpy( &payload[0], &width, sizeof( float ) );
	memcpy( &payload[4], &speed, sizeof( float ) );

    // Submit command and wait for response. Push result to stack.
    res = this->submit(0x26, payload, 8, true, &resp, &resp_len);
    if(res != 2){
        dbgPrint( "Response payload length doesn't match (is %d, expected 2)\n", res );
        if ( res > 0 ) delete resp;
        return 0;            //Chage the return to 0
    }

    //Check response status
    status = this->get_response_status(resp);
    delete resp;
    if(status != E_SUCCESS){
        dbgPrint( "Command RELEASE not successful: %s\n", status_to_str( status ) );
        return -1;
    }
    return 0;
}

/** Command and measure
 * @param cmd_type:	0 - read only; 1 - position control; 2 - speed control
 */
int iwtros::function::script_measure_move (unsigned char cmd_type, float cmd_width, float cmd_speed, gripper_response & info){
    status_t status;
    int res;
    const unsigned char CMD_CUSTOM = 0xB0;
    unsigned char payload[9];
	unsigned char *resp;
	unsigned int resp_len;

    // Custom payload format:
	// 0:	Unused
	// 1:	float, target width, used for 0xB1 command
	// 5:	float, target speed, used for 0xB1 and 0xB2 command
	payload[0] = 0x00;
	memcpy(&payload[1], &cmd_width, sizeof(float));
	memcpy(&payload[5], &cmd_speed, sizeof(float));

    //Submit command and process result
    res = this->submit(CMD_CUSTOM + cmd_type, payload, 9, true, &resp, &resp_len);
    try{
        if(res < 2) throw std::string("Invalid Respose");
        status = this->get_response_status(resp);

        if(status == E_CMD_UNKNOWN) throw std::string("Command unknown - make sure script is running");
        if (status != E_SUCCESS) throw std::string("Command failed");
        if (res != 23) throw std::string("Response payload incorrect (" + std::to_string(res) + ")");

        std::cout << "Extracting the data from response....." << '\n';
        int off=2;
        unsigned char resp_state[6] = {0,0,0,0,0,0};
        resp_state[2] = resp[2];
        info.state = resp[2];                                       off += 1;
        info.state_text = std::string(getStateValue(resp_state));
        info.position = convert(&resp[off]);                        off += 4;
        info.speed = convert(&resp[off]);                           off += 4;
        info.f_motor = convert(&resp[off]);                         off += 4;
        info.f_finger0 = convert(&resp[off]);                       off += 4;
        info.f_finger1 = convert(&resp[off]);                       off += 4; 

        info.isMoving = (info.state & 0x02) != 0;

        if (0)
			printf("Received: %02X, %6.2f,%6.2f,%6.2f,%6.2f,%6.2f\n  %s\n",
				info.state, info.position, info.speed, info.f_motor, info.f_finger0, info.f_finger1,
				info.state_text.c_str());
    }catch(std::string msg){
        msg = "measure_move: " + msg + "\n"; 
        dbgPrint ("%s", msg.c_str());
		if (res > 0) delete(resp);
		return 0;
    }

    delete resp;
    return 1;
}


/** Set Acceleration 
 * @param acc acceleration value
*/
int iwtros::function::setAcceleration(float acc){
    status_t status;
	int res;
	unsigned char payload[4];
	unsigned char *resp;
	unsigned int resp_len;

    memcpy(&payload[0], &acc, sizeof(float));
    // Submit command and wait for response. Push result to stack.
    res = this->submit(0x30, payload, 4, true, &resp, &resp_len);
    if(res != 2){
        dbgPrint( "Response payload length doesn't match (is %d, expected 2)\n", res );
        if ( res > 0 ) delete resp;
        return 0;            //Chage the return to 0
    }

    //Check response status
    status = this->get_response_status(resp);
    delete resp;
    if(status != E_SUCCESS){
        dbgPrint( "Command SET ACCELERATION not successful: %s\n", status_to_str( status ) );
        return -1;
    }
    return 0;
}

/** Set Force 
 * @param force force value
*/
int iwtros::function::setGraspingForceLimit(float force){
    status_t status;
	int res;
	unsigned char payload[4];
	unsigned char *resp;
	unsigned int resp_len;

    memcpy(&payload[0], &force, sizeof(float));
    // Submit command and wait for response. Push result to stack.
    res = this->submit(0x32, payload, 4, true, &resp, &resp_len);
    if(res != 2){
        dbgPrint( "Response payload length doesn't match (is %d, expected 2)\n", res );
        if ( res > 0 ) delete resp;
        return 0;            //Chage the return to 0
    }

    //Check response status
    status = this->get_response_status(resp);
    delete resp;
    if(status != E_SUCCESS){
        dbgPrint( "Command SET FORCE not successful: %s\n", status_to_str( status ) );
        return -1;
    }
    return 0;
}

/** Do Tare
 * @param NULL
*/
int iwtros::function::doTare(void){
    status_t status;
    int res;
    unsigned char payload[3];
    unsigned char *resp;
    unsigned int resp_len;

    ;
    // Submit command and wait for response. Push result to stack.
    res = this->submit(0x38, payload, 0, true, &resp, &resp_len);
    if(res != 2){
        dbgPrint( "Response payload length doesn't match (is %d, expected 2)\n", res );
        if ( res > 0 ) delete resp;
        return 0;            //Chage the return to 0
    }

    //Check response status
    status = this->get_response_status(resp);
    delete resp;
    if(status != E_SUCCESS){
        dbgPrint( "Command TARE not successful: %s\n", status_to_str( status ) );
        return -1;
    }
    return 0;
}

/**
 * Get System State
*/
const char * iwtros::function::systemState(void){
    status_t status;
	int res;
	unsigned char payload[3];
	unsigned char *resp;
	unsigned int resp_len;

	// Don't use automatic update, so the payload bytes are 0.
	memset( payload, 0, 3 );

    // Submit command and wait for response. Expecting exactly 4 bytes response payload.
    res = this->submit(0x40, payload, 3, false, &resp, &resp_len);
    if(res != 6){
        dbgPrint( "Response payload length doesn't match (is %d, expected 6)\n", res );
		if ( res > 0 ) delete resp;
		return 0;
    }

    // Check response status    -----> Incomplete
	status = this->get_response_status( resp );
    return getStateValue(resp);
    if ( status != E_SUCCESS ){
		dbgPrint( "Command GET SYSTEM STATE not successful: %s\n", status_to_str( status ) );
		delete resp;
		return 0;
	}
	delete resp;
    return 0;    
}

/**
 * Get Grasping State 
 * @brief re-examin the return value
*/
int iwtros::function::graspingState(void){
    status_t status;
	int res;
	unsigned char payload[3];
	unsigned char *resp;
	unsigned int resp_len;

    memset(payload, 0, 3);
    // Submit command and wait for response. Push result to stack.
    res = this->submit(0x41, payload, 4, false, &resp, &resp_len);
    if(res != 3){
        dbgPrint( "Response payload length doesn't match (is %d, expected 2)\n", res );
        if ( res > 0 ) delete resp;
        return 0;            
    }

    //Check response status
    status = this->get_response_status(resp);
    if(status != E_SUCCESS){
        dbgPrint( "Command GET GRASPING STATE not successful: %s\n", status_to_str( status ) );
        delete resp;
        return 0;
    }
    delete resp;
    dbgPrint("GRASPING STATUS: %s\n", status_to_str (status) );
    return (int)resp[2];
}

/**
 * Get Opening Speed Force
*/
float iwtros::function::getOpeningSpeedForce(unsigned char cmd, int auto_update){
    status_t status;
    int res;
    unsigned char payload[3];
    unsigned char *resp;
    unsigned int resp_len;
    std::string names[] = { "opening", "speed", "force", "???" };

    // Payload = 0, except for auto update
    memset(payload, 0, 3);
    if (auto_update > 0) {
        payload[0] = 0x01;
        payload[1] = (auto_update & 0xff);
        payload[2] = ((auto_update & 0xff00) >> 8);
    }

    // Submit command and wait for response. Expecting exactly 4 bytes response payload.
    res = this->submit(cmd, payload, 3, false, &resp, &resp_len ); // 0x43
    if (res != 6) {
        dbgPrint( "Response payload length doesn't match (is %d, expected 3)\n", res );
        if ( res > 0 ) delete( resp );
        return 0;
    }

    // Check response status
    status = this->get_response_status( resp );
    if ( status != E_SUCCESS )	{
        const char *info = names[3].c_str();
        if (cmd >= 0x43 && cmd <= 0x45)
            info = names[cmd-0x43].c_str();
        dbgPrint( "Command 0xfalse%02X get %s not successful: %s\n", cmd, info, status_to_str( status ) );
        delete( resp );
        return 0;
    }

    float r = convert(&resp[2]);
    delete( resp );
    return r;
}

/** Get Opening 
 * @brief Read measured opening (width/position) from gripper (0x43).
 * @param auto_update Request periodic updates (unit: ms) from the gripper; responses need to be read out elsewhere.
 */
float iwtros::function::getOpening(int auto_update) {
    return getOpeningSpeedForce(0x43, auto_update);
}

/** Get Speed 
 * @brief Read measured speed from gripper (0x44).
 * @param auto_update Request periodic updates (unit: ms) from the gripper; responses need to be read out elsewhere.
 */
float iwtros::function::getSpeed(int auto_update) {
    return getOpeningSpeedForce(0x44, auto_update);
}

/** Get Froce 
 * @brief brief Read measured force from gripper (0x45).
 * @param auto_update Request periodic updates (unit: ms) from the gripper; responses need to be read out elsewhere.
 */
float iwtros::function::getForce(int auto_update){
    return getOpeningSpeedForce(0x45, auto_update);
}


int iwtros::function::getAcceleration( void )
{
	status_t status;
	int res;
	unsigned char payload[6];
	unsigned char *resp;
	unsigned int resp_len;
	unsigned char vResult[4];

	// Don't use automatic update, so the payload bytes are 0.
	memset( payload, 0, 1 );

	// Submit command and wait for response. Expecting exactly 4 bytes response payload.
	res = this->submit( 0x31, payload, 0, false, &resp, &resp_len );
	if ( res != 6 ){
		dbgPrint( "Response payload length doesn't match (is %d, expected 3)\n", res );
		if ( res > 0 ) free( resp );
		return 0;
	}

	// Check response status
	status = this->get_response_status( resp );
	if ( status != E_SUCCESS ){
		dbgPrint( "Command GET ACCELERATION not successful: %s\n", status_to_str( status ) );
		delete( resp );
		return 0;
	}

	vResult[0] = resp[2];
	vResult[1] = resp[3];
	vResult[2] = resp[4];
	vResult[3] = resp[5];

	delete( resp );

	return convert(vResult);

	//return (int) resp[2];
}

int iwtros::function::getGraspingForceLimit( void )
{
	status_t status;
	int res;
	unsigned char payload[6];
	unsigned char *resp;
	unsigned int resp_len;
	unsigned char vResult[4];

	// Don't use automatic update, so the payload bytes are 0.
	memset( payload, 0, 1 );

	// Submit command and wait for response. Expecting exactly 4 bytes response payload.
	res = this->submit( 0x33, payload, 0, false, &resp, &resp_len );
	if ( res != 6 ){
		dbgPrint( "Response payload length doesn't match (is %d, expected 3)\n", res );
		if ( res > 0 ) delete( resp );
		return 0;
	}

	// Check response status
	status = this->get_response_status( resp );
	if ( status != E_SUCCESS ){
		dbgPrint( "Command GET GRASPING FORCE not successful: %s\n", status_to_str( status ) );
		delete( resp );
		return 0;
	}

	vResult[0] = resp[2];
	vResult[1] = resp[3];
	vResult[2] = resp[4];
	vResult[3] = resp[5];

	delete( resp );

	return convert(vResult);

	//return (int) resp[2];
}