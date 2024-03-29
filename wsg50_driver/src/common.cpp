/******************************************************************
 * @file common.cpp
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
 ****************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "wsg50_driver/common.h"


/**
 * @brief Convert IP address string to IP address type
 * @param *str		String containing IP address
 * @return IP address type
 */
ip_addr_t str_to_ipaddr( const char *str ){
	unsigned int i, res;
	unsigned int buf[4];
	ip_addr_t addr = 0;
	res = sscanf( str, "%d.%d.%d.%d", &buf[3], &buf[2], &buf[1], &buf[0] );
	if ( res != 4 ) return( 0 );
	for ( i = 0; i < 4; i++ )
	{
		if ( buf[i] > 255 ) return( 0 );
		addr <<= 8;
		addr |= (int) buf[i];
	}
	return( addr );
}


/**
 * @brief Convert status code to string
 * @param status	Status code
 * @return Status string
 */
const char * status_to_str( status_t status )
{
	switch( status )
	{
		case E_SUCCESS:					return( "No error" );
		case E_NOT_AVAILABLE:			return( "Service or data is not available" );
		case E_NO_SENSOR:				return( "No sensor connected" );
		case E_NOT_INITIALIZED:			return( "The device is not initialized" );
		case E_ALREADY_RUNNING:			return( "Service is already running" );
		case E_FEATURE_NOT_SUPPORTED:	return( "The requested feature is not supported" );
		case E_INCONSISTENT_DATA:		return( "One or more dependent parameters mismatch" );
		case E_TIMEOUT:					return( "Timeout error" );
		case E_READ_ERROR:				return( "Error while reading from a device" );
		case E_WRITE_ERROR:				return( "Error while writing to a device" );
		case E_INSUFFICIENT_RESOURCES:	return( "No memory available" );
		case E_CHECKSUM_ERROR:			return( "Checksum error" );
		case E_NO_PARAM_EXPECTED:		return( "No parameters expected" );
		case E_NOT_ENOUGH_PARAMS:		return( "Not enough parameters" );
		case E_CMD_UNKNOWN:				return( "Unknown command" );
		case E_CMD_FORMAT_ERROR:		return( "Command format error" );
		case E_ACCESS_DENIED:			return( "Access denied" );
		case E_ALREADY_OPEN:			return( "Interface already open" );
		case E_CMD_FAILED:				return( "Command failed" );
		case E_CMD_ABORTED:				return( "Command aborted" );
		case E_INVALID_HANDLE:			return( "Invalid handle" );
		case E_NOT_FOUND:				return( "Device not found" );
		case E_NOT_OPEN:				return( "Device not open" );
		case E_IO_ERROR:				return( "General I/O-Error" );
		case E_INVALID_PARAMETER:		return( "Invalid parameter" );
		case E_INDEX_OUT_OF_BOUNDS:		return( "Index out of bounds" );
		case E_CMD_PENDING:				return( "Command is pending..." );
		case E_OVERRUN:					return( "Data overrun" );
		case E_RANGE_ERROR:				return( "Value out of range" );
		case E_AXIS_BLOCKED:			return( "Axis is blocked" );
		case E_FILE_EXISTS:				return( "File already exists" );
		default:						return( "Internal error. Unknown error code." );
	}
}


/**
 * Quit program for the given reason
 * @param *reason	String telling why we're quitting
 */
void quit( const char *reason )
{
	if ( reason ) fprintf( stderr, "%s\n", reason );
	exit(1);
}


const char * getStateValue( unsigned char *b ){

	/*
	unsigned char aux[4];

	aux[0] = b[0];
	aux[1] = b[1];
	aux[2] = b[2];
	aux[3] = b[3];

	dbgPrint("Dins de getStateValues.\n");
	dbgPrint("b[2] = 0x%x\n", b[2]);
	dbgPrint("b[3] = 0x%x\n", b[3]);
	dbgPrint("b[4] = 0x%x\n", b[4]);
	dbgPrint("b[5] = 0x%x\n", b[5]);
	*/

	char resp[1024] = "| ";

	if (b[2] & 0x1){	// D0 ==> LSB
		//dbgPrint("Fingers Referenced.\n");
		char aux0[21] = "Fingers Referenced |";
		strcat(resp, aux0);
	}
	if (b[2] & 0x2){  // D1
		//dbgPrint("The Fingers are currently moving.\n");
		char aux1[36]=" The Fingers are currently moving |";
		strcat(resp, aux1);
	}
	if (b[2] & 0x4){  // D2
		//dbgPrint("Axis is blocked in negative moving direction.\n");
		char aux2[48] =" Axis is blocked in negative moving direction |";
		strcat(resp, aux2);
	}
     	if (b[2] & 0x8){  // D3
		//dbgPrint("Axis is blocked in positive moving direction.\n");
		char aux3[48] =" Axis is blocked in positive moving direction |";
		strcat(resp, aux3);
	}
	if (b[2] & 0x10){ // D4
		//dbgPrint("Negative direction soft limit reached.\n");
		char aux4[42] = " Negative direction soft limit reached |";
		strcat(resp, aux4);
	}
	if (b[2] & 0x20){ // D5
		//dbgPrint("Positive direction soft limit reached.\n");
		char aux5[42] = " Positive direction soft limit reached |";
		strcat(resp, aux5);
	}
	if (b[2] & 0x40){ // D6
		//dbgPrint("Axis Stopped.\n");
		char aux6[18] = " Axis Stopped |";
		strcat(resp, aux6);
	}
	if (b[2] & 0x80){ // D7
		//dbgPrint("Target Pos reached.\n");
		char aux7[22] = " Target Pos reached |";
		strcat(resp, aux7);
	}

	if (b[3] & 0x1){ // D8
		//dbgPrint("Overdrive Mode.\n");
		char aux8[18] = " Overdrive Mode |";
		strcat(resp, aux8);
	}
	if (b[3] & 0x10){ // D12
		//dbgPrint("Fast Stop.\n");
		char aux12[13] = " Fast Stop |";
		strcat(resp, aux12);
	}
	if (b[3] & 0x20){ // D13
		//dbgPrint("Temperature Warning.\n");
		char aux13[23] = " Temperature Warning |";
		strcat(resp,aux13);
	}
	if (b[3] & 0x40){ // D14
		//dbgPrint("Temperature Error.\n");
		char aux14[21]= " Temperature Error |";
		strcat(resp, aux14);
	}
	if (b[3] & 0x80){ // D15
		//dbgPrint("Power Error.\n");
		char aux15[15]= " Power Error |";
		strcat(resp, aux15);
	}

	if (b[4] & 0x1){  // D16
		//dbgPrint("Engine Current Error.\n");
		char aux16[24]= " Engine Current Error |";
		strcat(resp, aux16);
	}
	if (b[4] & 0x2){  // D17
		//dbgPrint("Finger Fault.\n");
		char aux17[16] = " Finger Fault |";
		strcat(resp, aux17);
	}
	if (b[4] & 0x4){  // D18
		//dbgPrint("Command Error.\n");
		char aux18[17] = " Command Error |";
		strcat(resp, aux18);
	}
     	if (b[4] & 0x8){  // D19
		//dbgPrint("A script is currently running.\n");
		char aux19[33] = " A script is currently running |";
		strcat(resp, aux19);
	}
	if (b[4] & 0x10){ // D20
		//dbgPrint("Script Error.\n");
		char aux20[16] = " Script Error |";
		strcat(resp, aux20);
	}

	// [D21 - D31] RESERVED

	// D31 ==> MSB

	//dbgPrint("%s\n", resp);
	return resp;
}
