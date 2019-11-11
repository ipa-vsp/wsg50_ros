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

#include "wsg_50_tcp_driver/common.h"
#include "wsg_50_tcp_driver/checksum.h"
#include "wsg_50_tcp_driver/msg.h"

iwtros::msg::msg(const void *param){
    interface_t = new iwtros::tcp::tcp(param);
    if(interface_t->result){
        std::cout << "Open message communition with TCP socket SUCCESSFULL" << '\n';
    }else{
        std::cerr << "Failed to create message communication brigde with TCP socket" << '\n';
    }
    
}

iwtros::msg::~msg(void){
    interface_t->~tcp();
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

    res = interface_t->write(buf, 6 + msg->len + 2);
    if(res < 6 + (int)msg->len+2){
        interface_t->~tcp();
        quit("Failed to submit message checksum");
        return -1;
    }

    delete buf; // free dymanically allocated memory
    return msg->len + 8;
}

int iwtros::msg::receive(msg_t * msg){
    int res;
	unsigned char header[3];			// 1 byte command, 2 bytes payload length
	unsigned short checksum = 0x50f5;	// Checksum over preamble (0xaa 0xaa 0xaa)
	unsigned int sync;

    //Syncing - neccessary for compatiblility
    sync = 0;
    while ( sync != MSG_PREAMBLE_LEN){
        res = interface_t->read(header, 1);
        if(header[0] == MSG_PREAMBLE_BYTE) sync++;
    }

    //Read headers;
    res = interface_t->read(header, 3);
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
    res = interface_t->read (msg->data, msg->len + 2);
    if(res < (int)msg->len + 2){
        fprintf(stderr, "Not enough data (%d, expected %d", res, msg->len + 2);
        return -1;
    }

    //checksum
    checksum = checksum_update_crc16(msg->data, msg->len + 2, checksum);
    if(checksum != 0){
        fprintf (stderr, "Checksum err\n");
        return -1;
    }
    return msg->len + 8; // Why 8? need to check this on the datasheet
}