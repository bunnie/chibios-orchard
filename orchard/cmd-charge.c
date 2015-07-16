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
#include "gasgauge.h"

#include <string.h>
#include <stdlib.h>

void cmd_shipmode(BaseSequentialStream *chp, int argc, char *argv[])
{

  (void)argv;
  (void)argc;

  /// NOTE TO SELF: implement flush of volatile genome state (such as playtime and favorites)
  /// to FLASH before going into shipmode
  
  chprintf(chp, "Battery will disconnect and system will power off\r\n");
  chprintf(chp, "You must disconnect/reconnect power via microUSB to reconnect battery\r\n");
  
  // ggSetHibernate(); // optional, seems to do nothing
  chargerShipMode();
}

orchard_command("shipmode", cmd_shipmode);

void cmd_boost(BaseSequentialStream *chp, int argc, char *argv[])
{

  (void)argv;
  (void)argc;

  if( argc != 1 ) {
    chprintf(chp, "boost [on|off]\r\n" );
    return;
  }
  chprintf( chp, "wtf: %s\n\r", argv[0] );
  if( strncmp( argv[0], "on", 2 ) == 0 ) {
    chprintf(chp, "Turning boost intent on\r\n");
    chargerBoostIntent(1);
  } else {
    chprintf(chp, "Turning boost intent off\r\n");
    chargerBoostIntent(0);
  }
}

orchard_command("boost", cmd_boost);


void cmd_charge(BaseSequentialStream *chp, int argc, char *argv[])
{

  if( argc != 1 ) {
    chprintf(chp, "charge [on|off]\r\n" );
    return;
  }
  if( strncmp( argv[0], "on", 2 ) == 0 ) {
    chprintf(chp, "Turning charge intent on and setting charger parameter\r\n");
    chargerSetTargetCurrent(1500);
    chargerSetTargetVoltage(4160);
    chargerChargeIntent(1);
  } else {
    chprintf(chp, "Turning charge intent off\r\n");
    chargerChargeIntent(0);
  }
}

orchard_command("charge", cmd_charge);

void cmd_chgstat(BaseSequentialStream *chp, int argc, char *argv[])
{

  (void)argv;
  (void)argc;
  chargerStat cstat;
  chargerFault cfault;

  cstat = chargerGetStat();
  if( cstat == STAT_FAULT ) {
    cfault = chargerFaultCode();
    switch(cfault) {
    case ENORMAL:
      chprintf(chp, "Charger is normal, but fault code reported. Strange.\r\n");
      break;
      
    case EBOOSTOVP:
      chprintf(chp, "Boost circuit in over voltage protect\r\n");
      break;
      
    case ELOWV:
      chprintf(chp, "Input voltage is too low or boost overcurrent\r\n");
      break;
      
    case ETHERMAL:
      chprintf(chp, "Thermal shutdown\r\n" );
      break;

    case ETIMER:
      chprintf(chp, "Safety or watchdog timer fault\r\n" );
      break;
      
    case EBATTOVP:
      chprintf(chp, "Batterey OVP\r\n" );
      break;
      
    case ENOBATT:
      chprintf(chp, "No battery connected\r\n" );
      break;
      
    default:
      chprintf(chp, "Charger is in an unknown state (program error)\r\n");
    }
  } else {
    switch( cstat ) {
    case STAT_RDY:
      chprintf(chp, "Charger is idle\r\n" );
      break;

    case STAT_CHARGING:
      chprintf(chp, "Charger is charging\r\n" );
      break;

    case STAT_DONE:
      chprintf(chp, "Charger is in charge mode, but finished charging\r\n" );
      break;

    case STAT_FAULT:
      chprintf(chp, "Charge has a fault\r\n" );
      break;

    default:
      chprintf(chp, "Charge is ina n unknown state (program error)\r\n" );
    }
  }

  chprintf(chp, "Port current capability reported at %dmA\r\n", chargerGetHostCurrent() );
  chprintf(chp, "Battery target voltage is %dmV\r\n", chargerGetTargetVoltage() );
  chprintf(chp, "Battery target current is %dmA\r\n", chargerGetTargetCurrent() );
  chprintf(chp, "Battery termination current is %dmA\r\n", chargerGetTerminationCurrent() );
  chprintf(chp, "Boost current limit is %dmA\r\n", chargerGetBoostLimit() );
}

orchard_command("chgstat", cmd_chgstat);

void cmd_chgcap(BaseSequentialStream *chp, int argc, char *argv[]) {
  uint16_t capacity;
  uint16_t energy;
  uint16_t termV;
  uint16_t taper;

  if( argc != 4 ) {
    chprintf(chp, "chgcap [capacity] [energy] [termV] [taper] where capacity is in mAh, energy is in mWh, termination voltage in mV, taper rate in 0.1Hr\r\n" );
    return;
  }

  capacity = strtoul(argv[0], NULL, 0);
  energy = strtoul(argv[1], NULL, 0);
  termV = strtoul(argv[2], NULL, 0);
  taper = strtoul(argv[3], NULL, 0);
  
  if( (capacity < 300) || (capacity > 6000) ) {
    chprintf(chp, "Capacity value is suspicious, aborting.\n\r" );
    return;
  }

  if( (energy < 1110) || (energy > 22200) ) {
    chprintf(chp, "Energy value is suspicious, aborting.\n\r" );
    return;
  }
  
  setDesignCapacity( capacity, energy, termV, taper );
}

orchard_command("chgcap", cmd_chgcap);

void cmd_ggstat(BaseSequentialStream *chp, int argc, char *argv[]) {
  (void) argc;
  (void) argv;

  chprintf(chp, "Full charge capacity: %dmA\n\r", ggFullChargeCap() );
  chprintf(chp, "Full available capacity: %dmA\n\r", ggFullAvailableCap() );
  chprintf(chp, "Nominal available capacity: %dmA\n\r", ggNomAvailableCap() );
  chprintf(chp, "Gasgauge control status code: %x\n\r", ggCtlStat() );
  chprintf(chp, "Gasgauge flags: %x\n\r", ggFlags() );
  chprintf(chp, "Remaining capacity unfiltered: %dmA\n\r", ggRemainingUnfiltered() );
  chprintf(chp, "Remaining capacity filtered: %dmA\n\r", ggCapFiltered() );
  chprintf(chp, "Full charge capacity unfiltered: %dmA\n\r", ggFullChargeCapUnfiltered() );
  chprintf(chp, "Full charge capacity filtered: %dmA\n\r", ggFullChargeCapFiltered() );
  chprintf(chp, "State of charge unfiltered: %d%%\n\r", ggStateofChargeUnfiltered() );
  // chprintf(chp, "Design capacity: %dmA\n\r", getDesignCapacity() );
}
orchard_command("ggstat", cmd_ggstat);
