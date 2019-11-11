/******************************************************************
 * @file tcp.h
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

#ifndef TCP_H_
#define TCP_H_

#ifdef WIN32
    #include "winsock.h"
    #include "winsock2.h"
#else // WIN32
    #include <unistd.h>
    #include <sys/types.h>
    #include <sys/socket.h>
    #include <sys/ioctl.h>
    #include <sys/select.h>
    #include <arpa/inet.h>
    #include <netinet/in.h>
#endif

#include "common.h"

namespace iwtros{
    typedef struct{
        ip_addr_t addr;
        unsigned short port;
    }tcp_params_t;

    typedef struct{
        int sock;
        struct sockaddr_in si_server;
        ip_addr_t server;
    }tcp_conn_t;

    class tcp{
    private:
        static tcp_conn_t conn;
        tcp_params_t * _tcp;
    public:
        tcp(const void *params);
        int read (unsigned char *buf, unsigned int len);
        int write (unsigned char *buf, unsigned int len);
        ~tcp();
    }; // class tcp
} // namespace iwtros

#endif // !TCP_H_h
