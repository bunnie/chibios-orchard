
#include "ch.h"
#include "hal.h"
#include "i2c.h"

#include "charger.h"
#include "orchard.h"
#include "string.h"

#include "orchard-test.h"
#include "test-audit.h"
#include "analog.h"    // for test
#include "gasgauge.h"  // for test

static I2CDriver *driver;
static chargerIntent chgIntent = CHG_IDLE;
static chargerIntent shipIntent = CHG_IDLE; // one-way flag for shipmode

static void charger_set(uint8_t reg, uint8_t val) {

  uint8_t tx[2] = {reg, val};

  i2cAcquireBus(driver);
  i2cMasterTransmitTimeout(driver, chargerAddr,
                           tx, sizeof(tx),
                           NULL, 0,
                           TIME_INFINITE);
  i2cReleaseBus(driver);
}

static void charger_get(uint8_t adr, uint8_t *data) {
  uint8_t tx[1];
  uint8_t rx[1];

  tx[0] = adr;

  i2cAcquireBus(driver);
  i2cMasterTransmitTimeout(driver, chargerAddr,
                           tx, sizeof(tx),
                           rx, sizeof(rx),
                           TIME_INFINITE);
  i2cReleaseBus(driver);

  *data = rx[0];
}


static void do_charger_watchdog(void) {
  if( shipIntent == CHG_SHIPMODE ) {
    charger_set(0x00, 0x00); // turn off boost if it's turned on
    // give it a few ms to discharge caps and prevent bounceback    
    chThdSleepMilliseconds(100);
    charger_set(0x00, 0x08); // command for ship mode
  } else {
    switch( chgIntent ) {
    case CHG_CHARGE:
      charger_set(0x00, 0x80); // command for charge mode
      break;
      
    case CHG_BOOST:
      charger_set(0x00, 0xC0); // command for boost mode
      break;
      
    case CHG_IDLE:
    default:
      break;
      // do nothing
    }
  }
}

static THD_WORKING_AREA(waChargerWatchdogThread, 128);
static THD_FUNCTION(charger_watchdog_thread, arg) {
  (void)arg;

  chRegSetThreadName("Charger watchdog thread");
  while (1) {
    chThdSleepMilliseconds(1000);
    do_charger_watchdog();
  }

  return;
}

void chargerStop(void) {
  chargerBoostIntent(0);
  charger_set(CHG_REG_CTL, 0xF); // set Hi-Z mode for charger
}

void chargerStart(I2CDriver *i2cp) {

  driver = i2cp;

  // 0x6 0xb0    -- 6 hour fast charger time limit, 1A ILIM, no TS, DPM 4.2V
  // 0x4 0x19    -- charge current at 300mA, term sense at 50mA
  // 0x1 0x2c    -- 500mA charging, enable stat and charge term
  // 0x2 0x64    -- set 4.0V as charging target
  charger_set(CHG_REG_SAFETY, 0xb0);
  charger_set(CHG_REG_CURRENT, 0x19);
  charger_set(CHG_REG_CTL, 0x2c);
  charger_set(CHG_REG_BATTV, 0x64);

  // ... and let's overwrite those with the proper callbacks
  chargerSetTargetVoltage(4200);

  chThdCreateStatic(waChargerWatchdogThread, sizeof(waChargerWatchdogThread),
                    LOWPRIO + 1, charger_watchdog_thread, NULL);
}

// this function sets the charger into "ship mode", e.g. power fully
// disconnected from the system, allowing safe long-term storage of the
// battery while shipping or in storage. The only way out of this is to plug
// power into the microUSB port, which re-engages power to the whole system.
msg_t chargerShipMode(void) {
  chprintf(stream, "Setting shipmode, prepare for shutdown!.\n\r" );
  shipIntent = CHG_SHIPMODE;  // this is a one-way door
  
  return MSG_OK;
}

// this function sets boost intent based on the enable argument
// enable = 1 turn on boost intent
// enable = 0 turn off boost intent
msg_t chargerBoostIntent(uint8_t enable) {
  if( enable ) {
    chgIntent = CHG_BOOST;
  } else {
    chgIntent = CHG_IDLE;
    charger_set(0x00, 0x00); // force to idle mode quickly
  }
  
  return MSG_OK;
}

// this function sets charge intent 
// enable = 1 turn on charge intent
// enable = 0 turn off charge intent
msg_t chargerChargeIntent(uint8_t enable) {
  // charger_set(0x00, 0x08); // command for ship mode
  if( enable ) {
    chgIntent = CHG_CHARGE;
  } else {
    chgIntent = CHG_IDLE;
    charger_set(0x00, 0x00); // force to idle mode quickly
  }
  
  return MSG_OK;
}

chargerIntent chargerCurrentIntent(void) {
  return chgIntent;
}

chargerFault chargerFaultCode(void) {
  uint8_t data;
  charger_get(CHG_REG_STATUS, &data);

  return (chargerFault) (data & 0x7);
}

chargerStat chargerGetStat(void) {
  uint8_t data;
  charger_get(CHG_REG_STATUS, &data);

  return (chargerStat) ((data >> 4) & 0x3);
}

void chargerForceDetect(void) {
  uint8_t data;
  charger_get(CHG_REG_DPM, &data);
  data |= 10;
  charger_set(CHG_REG_DPM, data);
}

void chargerForce500(void) {
  uint8_t data;

  charger_get(CHG_REG_CTL, &data);
  data &= 0x0F;
  data |= 0x20;
  charger_set(CHG_REG_CTL, data);
}

void chargerForce900(void) {
  uint8_t data;

  charger_get(CHG_REG_CTL, &data);
  data &= 0x0F;
  data |= 0x30;
  charger_set(CHG_REG_CTL, data);
}

void chargerForce1500(void) {
  uint8_t data;

  charger_get(CHG_REG_CTL, &data);
  data &= 0x0F;
  data |= 0x40;
  charger_set(CHG_REG_CTL, data);
}

// returns host port current capability in mA
uint16_t chargerGetHostCurrent(void) {
  uint8_t data;
  charger_get(CHG_REG_CTL, &data);

  data = (data >> 4) & 0x7;
  switch(data) {
  case 0:
    return 100;
    break;
  case 1:
    return 150;
    break;
  case 2:
    return 500;
    break;
  case 3:
    return 900;
    break;
  case 4:
    return 1500;
    break;
  case 5:
    return 1950;
    break;
  case 6:
    return 2500;
    break;
  case 7:
    return 2000;
    break;
  default:
    return 0;
  }
}

// returns current battery target voltage in mV
uint16_t chargerGetTargetVoltage(void) {
  uint8_t data;
  charger_get(CHG_REG_BATTV, &data);

  return ((data >> 2) & 0x3F) * 20 + 3500;
}


// returns current battery target current in mA
uint16_t chargerGetTargetCurrent(void) {
  uint8_t data;
  charger_get(CHG_REG_CURRENT, &data);

  return ((data >> 3) & 0x1F) * 100 + 500;
}

// returns battery termination current threshold in mA
uint16_t chargerGetTerminationCurrent(void) {
  uint8_t data;
  uint16_t retval;
  charger_get(CHG_REG_CTL, &data);
  if( !(data & 0x4) ) {
    // charge termination is disabled
    return 0;
  }
  
  charger_get(CHG_REG_CURRENT, &data);

  retval = (data & 0x7) * 50 + 50;
  retval = retval > 300 ? 300 : retval;

  return retval;
}

// returns boost current limit
uint16_t chargerGetBoostLimit(void) {
  uint8_t data;
  charger_get(CHG_REG_SAFETY, &data);

  return (data & 0x10) ? 1000: 500;
}

void chargerSetTargetVoltage(uint16_t voltage) {
  uint16_t code;
  uint8_t data;
  
  if( voltage > 4160 )
    voltage = 4160;

  code = (voltage - 3500) / 20;
  if( code > 0x21 ) // this safety check caps voltage to 4.16V
    code = 0x21;
  
  charger_get(CHG_REG_BATTV, &data);
  data &= 0x3;
  data |= code << 2;
  charger_set(CHG_REG_BATTV, data);
}

void chargerSetTargetCurrent(uint16_t current) {
  uint16_t code;
  uint8_t data;
  
  if( current > 3000 )
    current = 3000;

  code = (current - 500) / 100;
  if( code > 0x14 ) // this safety check caps current to 2000mA
    code = 0x14;
  
  charger_get(CHG_REG_CURRENT, &data);
  data &= 0x7;
  data |= code << 3;
  charger_set(CHG_REG_CURRENT, data);
}

#define TEST_CHG_TIMEOUT 10000
#define TEST_CHG_THRESH 100  // threshold, in mA, for a test to pass charging
#define TEST_CHG_DTHRESH -20 // threshold, in mA, for proper discharging
OrchardTestResult test_charger(const char *my_name, OrchardTestType test_type) {
  (void) my_name;
  uint8_t ret;
  usbStat usbStatus;
  uint32_t starttime;
  char chgprompt[16];
  int16_t current;
  uint8_t waspluggedin = 1;
  
  switch(test_type) {
  case orchardTestPoweron:
  case orchardTestTrivial:
    charger_get(CHG_REG_ID, &ret);
    if( ret != 0x46 ) {
      return orchardResultFail;
    } else {
      return orchardResultPass;
    }
    break;
  case orchardTestInteractive:
    usbStatus = analogReadUsbStatus();
    if( usbStatus == usbStatNC ) {
      waspluggedin = 0;
      orchardTestPrompt("plug in USB", "to proceed", 0);
      starttime = chVTGetSystemTime();
      while( (analogReadUsbStatus() == usbStatNC) ) {
	if( chVTGetSystemTime() - starttime > TEST_CHG_TIMEOUT ) {
	  orchardTestPrompt("charger not found", "FAIL", 0);
	  return orchardResultFail;
	}
	chThdYield();
	chThdSleepMilliseconds(100);
      }
    }
    
    // we should now be plugged in
    orchardTestPrompt("detecting", "charger...", 0);
    chThdSleepMilliseconds(GG_UPDATE_INTERVAL_MS); // wait for gg to update
    usbStatus = analogReadUsbStatus();
    if( (usbStatus == usbStat500) || (usbStatus == usbStat1500) ) {
      current = ggAvgCurrent();
      if( current < TEST_CHG_THRESH ) {
	orchardTestPrompt("charge test fail", "charge current low", 0);
      }
      chsnprintf(chgprompt, sizeof(chgprompt), "%d mAh", current );
      if( usbStatus == usbStat500 ) 
	orchardTestPrompt("found 500mA chg", chgprompt, 0);
      else
	orchardTestPrompt("found 1.5A chg", chgprompt, 0);
    } else {
      orchardTestPrompt("charge test fail", "charger ID fail", 0);
    }
      
    chThdSleepMilliseconds(GG_UPDATE_INTERVAL_MS);
    
    orchardTestPrompt("unplug USB", "to proceed", 0);
    starttime = chVTGetSystemTime();
    while( (analogReadUsbStatus() != usbStatNC) ) {
      if( chVTGetSystemTime() - starttime > TEST_CHG_TIMEOUT ) {
	orchardTestPrompt("unplug detect", "failure!", 0);
	  return orchardResultFail;
      }
      chThdYield();
      chThdSleepMilliseconds(100);
    }

    orchardTestPrompt("detecting", "discharge...", 0);
    chThdSleepMilliseconds(GG_UPDATE_INTERVAL_MS);

    current = ggAvgCurrent();
    if( current > TEST_CHG_DTHRESH ) { // positive, or a less negative number than required
      chsnprintf(chgprompt, sizeof(chgprompt), "%d mAh", current );
      orchardTestPrompt("discharge fail", chgprompt, 0);
      return orchardResultFail;
    }
    if( waspluggedin ) {
      // let's plug back in if that was the case before...
      orchardTestPrompt("plug in USB", "to proceed", 0);
      starttime = chVTGetSystemTime();
      while( (analogReadUsbStatus() == usbStatNC) ) {
	if( chVTGetSystemTime() - starttime > TEST_CHG_TIMEOUT ) {
	  orchardTestPrompt("charger not found", "FAIL", 0);
	  return orchardResultFail;
	}
	chThdYield();
	chThdSleepMilliseconds(100);
      }
    }
    orchardTestPrompt("charger test", "PASSED", 0);
    return orchardResultPass;
    break;
    
  default:
    return orchardResultNoTest;
  }
  
  return orchardResultNoTest;
}
orchard_test("charger", test_charger);
