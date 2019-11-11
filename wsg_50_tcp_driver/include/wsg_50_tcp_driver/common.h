/******************************************************************
 * @file common.h
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
#ifndef _COMMON_H_
#define _COMMON_H_


// Macros ------------------------------------>

#define dbgPrint( fmt, ... )		fprintf( stderr, "%s()\t" fmt, __FUNCTION__ , ##__VA_ARGS__ )
#define dbgPrintPlain( fmt, ... )	fprintf( stderr, fmt, ##__VA_ARGS__ )
#define dbgPrintF( fmt, ... )		fprintf( stderr, "%s()\t" fmt, __FUNCTION__ , ##__VA_ARGS__ )
#define dbgPrintPlainF( fmt, ... )	fprintf( stderr, fmt, ##__VA_ARGS__ )

#define ASSERT( cond )				assert( cond )

//! Macro for detecting errors and exiting using the error code:
#define EXIT_ON_ERROR( error_code, msg ) \
	do { \
		if ( error_code != E_SUCCESS ) \
		{ \
			fprintf( stderr, "%s: ERROR %s: %s\n", __FUNCTION__, msg, error_to_str( error_code )); \
			return( error_code ); \
		} \
	} while( 0 )

#define STATUS_DESCRIPTORS {	  \
    "E_SUCCESS", 	              \
    "E_NOT_AVAILABLE",            \
    "E_NO_SENSOR",                \
    "E_NOT_INITIALIZED",          \
    "E_ALREADY_RUNNING",          \
    "E_FEATURE_NOT_SUPPORTED",    \
    "E_INCONSISTENT_DATA",        \
    "E_TIMEOUT",                  \
    "E_READ_ERROR",               \
    "E_WRITE_ERROR",              \
    "E_INSUFFICIENT_RESOURCES",   \
    "E_CHECKSUM_ERROR",           \
    "E_NO_PARAM_EXPECTED",        \
    "E_NOT_ENOUGH_PARAMS",        \
    "E_CMD_UNKNOWN",              \
    "E_CMD_FORMAT_ERROR",         \
    "E_ACCESS_DENIED",            \
    "E_ALREADY_OPEN",             \
    "E_CMD_FAILED",               \
    "E_CMD_ABORTED",              \
    "E_INVALID_HANDLE",           \
    "E_NOT_FOUND",        		  \
    "E_NOT_OPEN",        		  \
    "E_IO_ERROR",                 \
    "E_INVALID_PARAMETER",        \
    "E_INDEX_OUT_OF_BOUNDS",      \
    "E_CMD_PENDING",              \
    "E_OVERRUN",				  \
    "E_RANGE_ERROR",			  \
    "E_AXIS_BLOCKED",			  \
    "E_FILE_EXISTS",			  \
    NULL	       				  \
}


// Typedefs, enums, structs ---------------------------------------------->

typedef enum {
    E_SUCCESS = 0,              // No error
    E_NOT_AVAILABLE,            // Device, service or data is not available
    E_NO_SENSOR,                // No sensor connected
    E_NOT_INITIALIZED,          // The device is not initialized
    E_ALREADY_RUNNING,          // Service is already running
    E_FEATURE_NOT_SUPPORTED,    // The asked feature is not supported
    E_INCONSISTENT_DATA,        // One or more dependent parameters mismatch
    E_TIMEOUT,                  // Timeout error
    E_READ_ERROR,               // Error while reading from a device
    E_WRITE_ERROR,              // Error while writing to a device
    E_INSUFFICIENT_RESOURCES,   // No memory available
    E_CHECKSUM_ERROR,           // Checksum error
    E_NO_PARAM_EXPECTED,        // No parameters expected
    E_NOT_ENOUGH_PARAMS,        // Not enough parameters
    E_CMD_UNKNOWN,              // Unknown command
    E_CMD_FORMAT_ERROR,         // Command format error
    E_ACCESS_DENIED,            // Access denied
    E_ALREADY_OPEN,             // The interface is already open
    E_CMD_FAILED,               // Command failed
    E_CMD_ABORTED,              // Command aborted
    E_INVALID_HANDLE,           // invalid handle
    E_NOT_FOUND,        	    // device not found
    E_NOT_OPEN,        		    // device not open
    E_IO_ERROR,                 // I/O error
    E_INVALID_PARAMETER,        // invalid parameter
    E_INDEX_OUT_OF_BOUNDS,      // index out of bounds
    E_CMD_PENDING,              // Command was received correctly, but the execution needs more time. If the command was completely processed, another status message is returned indicating the command's result
    E_OVERRUN,					// Data overrun
    E_RANGE_ERROR,				// Range error
    E_AXIS_BLOCKED,				// Axis is blocked
    E_FILE_EXISTS				// File already exists
} status_t;

typedef unsigned int ip_addr_t;

// Devices
typedef enum
{
	UNKNOWN = 0,
	DSACON32,
	WSG50,
	WSG32,
	KMS40
} device_t;

ip_addr_t str_to_ipaddr(const char *str);
const char * status_to_str(status_t status);
void quit(const char * reason);
const char * getStateValue(unsigned char * b);

#endif // !COMMON_H_
