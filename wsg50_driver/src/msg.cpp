/******************************************************************
 * @file msg.h
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
#include <iostream>

#include "wsg50_driver/common.h"
#include "wsg50_driver/checksum.h"
#include "wsg50_driver/msg.h"

iwtros::msg::msg(const void *param): tcp(param){
    if(this->result){
        std::cout << "Open message communition with TCP socket SUCCESSFUL" << '\n';
    }else{
        std::cerr << "Failed to create message communication bridge with TCP socket" << '\n';
    }

}

iwtros::msg::~msg(void){
    //Will the descruct the tcp??
}

/** Send command
 * @param id    Command ID
 * @param len   Payload length
 * @param *payload  Payload data
 * @return 0 on success, else -1
*/
int iwtros::msg::send(msg_t *msg){
    unsigned char header[MSG_PREAMBLE_LEN + 3];
    unsigned short crc;
    int i, res;

    // Preamble
    for(i=0; i< MSG_PREAMBLE_LEN; i++) header[i] = MSG_PREAMBLE_BYTE;
    header[MSG_PREAMBLE_LEN] = msg->id;
    header[MSG_PREAMBLE_LEN + 1] = lo(msg->len);
    header[MSG_PREAMBLE_LEN + 2] = hi(msg->len);

    //Checksum
    crc = checksum_crc16(header, 6);
    crc = checksum_update_crc16(msg->data, msg->len, crc);

    //Write
    unsigned char *buf = (unsigned char *) new char( 6 + msg->len + 2 ); // 6+2 fixes (PREAMBLE, ID, PAYLOAD / ... / CRC)
    memcpy(buf, header, 6);
    memcpy(buf + 6, msg->data, msg->len);
    memcpy(buf + 6 + msg->len, (unsigned char *)&crc, 2);

    res = this->write(buf, 6 + msg->len + 2);
    if(res < 6 + (int)msg->len+2){
        this->close_tcp();
        quit("Failed to submit message checksum");
        return -1;
    }

    delete[] buf; // free dymanically allocated memory
    return msg->len + 8;
}

/** Receive answer
 * @param **response		Data buffer
 * @param len				Expected size of message
 * @return Overall number of bytes received, including header and checksum. -1 on error.
 */
int iwtros::msg::receive(msg_t * msg){
    int res;
	unsigned char header[3];			// 1 byte command, 2 bytes payload length
	unsigned short checksum = 0x50f5;	// Checksum over preamble (0xaa 0xaa 0xaa)
	unsigned int sync;

    //Syncing - necessary for compatiblility
    sync = 0;
    while ( sync != MSG_PREAMBLE_LEN){
        res = this->read(header, 1);
        if(header[0] == MSG_PREAMBLE_BYTE) sync++;
        else if((res == -1) || (res == 0)){
            printf("Failed to read data \n");
            return -1;
        }else {
            sync = 0;
        }
    }

    //Read headers;
    res = this->read(header, 3);
    if(res < 3 ){
        fprintf(stderr, "Failed to receive data (%d bytes to read)\n", res);
        return -1;
    }

    //calculate checksun over header
    checksum = checksum_update_crc16(header, 3, checksum);
    //get message id
    msg->id = header[0];
    //get payload size of received massage
    msg->len = make_short(header[1], header[2]);
    // Allocate space for paylaod abd checksum
    msg->data = (unsigned char *) new char(msg->len + 2u);
    //Read the payload and checksum
    res = this->read (msg->data, msg->len + 2);
    if(res < (int)msg->len + 2){
        fprintf(stderr, "Not enough data (%d, expected %d)\n", res, msg->len + 2);
        return -1;
    }

    //checksum
    checksum = checksum_update_crc16(msg->data, msg->len + 2, checksum);
    if(checksum != 0){
        fprintf (stderr, "Checksum err\n");
        return -1;
    }
    return msg->len + 8; // ToDo: Why 8? need to check this on the datasheet
}


/** Free message struct
 * @param *msg Pointer to the message struct
 */
void iwtros::msg::_delete(msg_t *msg){
    if (msg->data) delete msg->data;
    memset(msg, 0, sizeof(msg));
}
