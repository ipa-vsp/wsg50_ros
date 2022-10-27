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

#include "wsg50_driver/cmd.h"
#include <string>

namespace iwtros
{
typedef struct
{
    unsigned int state;
    bool isMoving;
    float position, speed;
    float f_motor, f_finger0, f_finger1;
    std::string state_text;
} gripper_response;

class function : public cmd
{
  protected:
    float getOpeningSpeedForce(unsigned char cmd, int auto_update);

  public:
    function(const char *addr, unsigned short port);
    ~function();

    // Gripper Controlls
    float convert(unsigned char *b);
    int homing(void);
    bool move(float width, float speed, bool stop_on_block, bool ignore_response = false);
    int stop(bool ignore_response = false);
    bool grasp(float objWidth, float speed);
    int release(float width, float speed);
    int ack_fault(void);

    int setAcceleration(float acc);
    int setGraspingForceLimit(float force);
    int doTare(void);

    const char *systemState(void);
    int graspingState(void);
    float getOpening(int auto_update = 0);
    float getForce(int auto_update = 0);
    float getSpeed(int auto_update = 0);
    int getAcceleration(void);
    int getGraspingForceLimit(void);

    int script_measure_move(unsigned char cmd_type, float cmd_width, float cmd_speed, gripper_response &info);
};
} // namespace iwtros