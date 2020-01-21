/******************************************************************
 * @file cmd.h
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
#ifndef CMD_H_
#define CMD_H_

#include "common.h"
#include "wsg_50_tcp_driver/msg.h"

namespace iwtros{
    class cmd{
        public:
            cmd(const char *addr, unsigned short port);
            ~cmd();  
            bool disconnected(void);
            bool is_connected( void );
            status_t get_response_status(unsigned char * response);
            int submit(unsigned char id, unsigned char *payload, unsigned int len, bool pending, unsigned char **response, unsigned int *response_len);
            bool connected = false;
            msg * _msg;
    };
}

#endif // !CMD_H_
