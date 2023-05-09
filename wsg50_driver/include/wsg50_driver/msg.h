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
#ifndef MSG_H_
#define MSG_H_

#include "common.h"
#include "wsg50_driver/tcp.h"

// Macros
#define MSG_PREAMBLE_BYTE 0xaa
#define MSG_PREAMBLE_LEN 3

#define make_short(lowbyte, highbyte) ((unsigned short)lowbyte | ((unsigned short)highbyte << 8))
#define make_signed_short(lowbyte, highbyte) ((signed short)((unsigned short)lowbyte | ((unsigned short)highbyte << 8)))
#define make_int(lowbyte, mid1, mid2, highbyte)                                                                        \
    ((unsigned int)lowbyte | ((unsigned int)mid1 << 8) | ((unsigned int)mid2 << 16) | ((unsigned int)highbyte << 24))
#define make_float(result, byteptr) memcpy(&result, byteptr, sizeof(float))

// Byte access
#define hi(x) (unsigned char)(((x) >> 8) & 0xff) // Returns the upper byte of the passed short
#define lo(x) (unsigned char)((x)&0xff)          // Returns the lower byte of the passed short

typedef struct
{
    unsigned char id;
    unsigned int len;
    unsigned char *data;
} msg_t;

namespace iwtros
{
class msg : public tcp
{
  public:
    msg(const void *param);
    ~msg();
    int send(msg_t *msg);
    int receive(msg_t *msg);
    void _delete(msg_t *msg);
};

} // namespace iwtros

#endif // !MSG_H_
