/******************************************************************
 * @file tcp.cpp
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

//Includes
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>

/** @ToDo */
//#include "wsg_50/interface.h"
#include "wsg_50_tcp_driver/tcp.h"

//Macro
#define TCP_RCV_TIMEOUT_SEC     600

iwtros::tcp::tcp(const void *params){
    int res;
    _tcp = (tcp_params_t *) params;
    conn.server = _tcp->addr;
    
    try{
        conn.sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
    }
    catch(const std::exception& e){
        std::cerr << e.what() << '\n';
    }

    memset((char *) &conn.si_server, 0, sizeof(conn.si_server));
    conn.si_server.sin_family = AF_INET;
    conn.si_server.sin_port = htons(_tcp->port);
    conn.si_server.sin_addr.s_addr = _tcp->addr;

    unsigned int val = 1024; 
    setsockopt(conn.sock, SOL_SOCKET, SO_RCVBUF, (void *) &val, (socklen_t)sizeof(val));
    struct timeval timeout = { timeout.tv_sec = TCP_RCV_TIMEOUT_SEC, timeout.tv_usec = 0 };
    setsockopt(conn.sock, SOL_SOCKET, SO_RCVTIMEO, (void *) &timeout, (socklen_t) sizeof(timeout));    /// Used "timeout" instead of "struct timeout" inside sizeof

    try{
        connect(conn.sock, (struct sockaddr *) &conn.si_server, sizeof(conn.si_server));
        result = true;
    }catch(const std::exception& e){
        std::cerr << e.what() << " = Open TCP socket is FAILED" << '\n';
        result = false; 
    }
}

iwtros::tcp::~tcp(){ 
    this->close_tcp();
}

void iwtros::tcp::close_tcp(){
    close (this->conn.sock);
    conn.sock = 0;
}

/** Read character from TCP socket 
 * @return Charactor read
*/
int iwtros::tcp::read (unsigned char *buf, unsigned int len){
    int res;
    if (conn.sock <= 0 || buf == NULL ) return -1;
    if (len == 0 ) return 0;

    //Read desired number fo bytes
    res = recv(conn.sock, buf, len, 0);
    if(res < 0){
        close(conn.sock);
        quit("Failed to read data from TCP socket \n");
    }
    return res;
}

/** Write to TCP Scocket
 * @param *buf Pointer to buffer that holds data to be sent
 * @param len Number of bytes to send
 * @return 0 if successful, -1 on failure.
*/
int iwtros::tcp::write (unsigned char *buf, unsigned int len){
    int res;
    if ( conn.sock <= 0 ) return( -1 );
    res = send(conn.sock, buf, len, 0);
    if(res >= 0) return (res);
    else{
        fprintf(stderr, "Failed to send data using TCP socket\n");
        return -1;
    }
}
