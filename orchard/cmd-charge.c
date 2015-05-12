/*
    ChibiOS/RT - Copyright (C) 2006-2013 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include "ch.h"
#include "hal.h"
#include "shell.h"
#include "chprintf.h"

#include "orchard.h"
#include "orchard-shell.h"

#include "charger.h"

static int boostmode = 0;

static int should_stop(void) {
  uint8_t bfr[1];
  return chnReadTimeout(serialDriver, bfr, sizeof(bfr), 1);
}

void cmd_shipmode(BaseSequentialStream *chp, int argc, char *argv[])
{

  (void)argv;
  (void)argc;
  chprintf(chp, "Battery will disconnect and system will power off\r\n");
  chprintf(chp, "You must disconnect/reconnect power via microUSB to reconnect battery\r\n");

  chargerShipMode();
}

orchard_command("shipmode", cmd_shipmode);

void cmd_boost(BaseSequentialStream *chp, int argc, char *argv[])
{

  (void)argv;
  (void)argc;
  if( boostmode == 0 ) {
    chprintf(chp, "Turning boost mode on\r\n");
    chargerBoostMode(1);
    boostmode = 1;
  } else {
    chprintf(chp, "Turning boost mode off\r\n");
    chargerBoostMode(0);
    boostmode = 0;
  }
}

orchard_command("boost", cmd_boost);


void cmd_chgstat(BaseSequentialStream *chp, int argc, char *argv[])
{

  (void)argv;
  (void)argc;
  chargerStates cstate;

  cstate = chargerCurrentState();
  
  switch(cstate) {
  case CHG_CHARGE:
    chprintf(chp, "Charger is charging (power attached, charging battery)\r\n");
    break;

  case CHG_IDLE:
    chprintf(chp, "Charger is idling (not charging, LEDs on only if power attached)\r\n");
    break;

  case CHG_BOOST:
    chprintf(chp, "Charger is boosting (power not attached, driving LEDs)\r\n");
    break;

  default:
    chprintf(chp, "Charger is in an unknown state (program error)\r\n");
  }
}

orchard_command("chgstat", cmd_chgstat);
