
#include "ch.h"
#include "hal.h"
#include "i2c.h"

#include "orchard.h"
#include "gasgauge.h"

#include "chprintf.h"
#include "userconfig.h"

#include "orchard-test.h"
#include "test-audit.h"

#include "orchard-shell.h"

void cmd_ggdump(BaseSequentialStream *chp, int argc, char *argv[]);

static I2CDriver *driver;

static void gg_set(uint8_t cmdcode, int16_t val) {

  uint8_t tx[3] = {cmdcode, (uint8_t) val & 0xFF, (uint8_t) (val >> 8) & 0xFF};

  i2cMasterTransmitTimeout(driver, ggAddr,
                           tx, sizeof(tx),
                           NULL, 0,
                           TIME_INFINITE);
}

static void gg_set_byte(uint8_t cmdcode, uint8_t val) {

  uint8_t tx[2] = {cmdcode, val};

  i2cMasterTransmitTimeout(driver, ggAddr,
                           tx, sizeof(tx),
                           NULL, 0,
                           TIME_INFINITE);
}

static void gg_get(uint8_t cmdcode, int16_t *data) {
  uint8_t tx[1];
  uint8_t rx[2];

  tx[0] = cmdcode;

  i2cMasterTransmitTimeout(driver, ggAddr,
                           tx, sizeof(tx),
                           rx, sizeof(rx),
                           TIME_INFINITE);

  *data = rx[0] | (rx[1] << 8);
}

static void gg_get_byte(uint8_t cmdcode, uint8_t *data) {
  uint8_t tx[1];
  uint8_t rx[1];

  tx[0] = cmdcode;

  i2cMasterTransmitTimeout(driver, ggAddr,
                           tx, sizeof(tx),
                           rx, sizeof(rx),
                           TIME_INFINITE);

  *data = rx[0];
}

void ggStart(I2CDriver *i2cp) {

  driver = i2cp;

  // clear hibernate state, if it was set
  i2cAcquireBus(driver);
  gg_set(GG_CMD_CNTL, GG_CODE_CLR_HIB);
  i2cReleaseBus(driver);
}

void ggSetHibernate(void) {
  i2cAcquireBus(driver);
  gg_set(GG_CMD_CNTL, GG_CODE_SET_HIB);
  i2cReleaseBus(driver);
}

int16_t ggAvgCurrent(void) {
  int16_t data;
  
  i2cAcquireBus(driver);
  gg_get( GG_CMD_AVGCUR, &data );
  i2cReleaseBus(driver);

  return data;
}

int16_t ggAvgPower(void) {
  int16_t data;
  
  i2cAcquireBus(driver);
  gg_get( GG_CMD_AVGPWR, &data );
  i2cReleaseBus(driver);

  return data;
}

int16_t ggRemainingCapacity(void) {
  int16_t data;
  
  i2cAcquireBus(driver);
  gg_get( GG_CMD_RM, &data );
  i2cReleaseBus(driver);

  return data;
}

int16_t ggStateofCharge(void) {
  int16_t soc;
  int16_t soc_unfilt;
  
  i2cAcquireBus(driver);
  gg_get( GG_CMD_SOC, &soc );
  gg_get( GG_CMD_SOC_UN, &soc_unfilt );
  i2cReleaseBus(driver);

  // compute overrides to give some indication of SoC before filtering
  // kicks in
#if 0
  if( soc == 0 ) {
    if( soc_unfilt > 0 )
      return soc_unfilt;
  }
  if( soc == 100 ) {
    if( soc_unfilt < 100 )
      return soc_unfilt;
  }
#endif
  
  return soc;
}

int16_t ggStateofChargeUnpatch(void) {
  int16_t data;
  
  i2cAcquireBus(driver);
  gg_get( GG_CMD_SOC, &data );
  i2cReleaseBus(driver);

  return data;
}

int16_t ggVoltage(void) {
  int16_t data;
  
  i2cAcquireBus(driver);
  gg_get( GG_CMD_VOLT, &data );
  i2cReleaseBus(driver);

  return data;
}

int16_t ggFullChargeCap(void) {
  int16_t data;
  
  i2cAcquireBus(driver);
  gg_get( GG_CMD_FCC, &data );
  i2cReleaseBus(driver);

  return data;
}

int16_t ggFullAvailableCap(void) {
  int16_t data;
  
  i2cAcquireBus(driver);
  gg_get( GG_CMD_FULL_CAP, &data );
  i2cReleaseBus(driver);

  return data;
}

int16_t ggNomAvailableCap(void) {
  int16_t data;
  
  i2cAcquireBus(driver);
  gg_get( GG_CMD_NOM_CAP, &data );
  i2cReleaseBus(driver);

  return data;
}

int16_t ggRemainingUnfiltered(void) {
  int16_t data;
  
  i2cAcquireBus(driver);
  gg_get( GG_CMD_RM_UN, &data );
  i2cReleaseBus(driver);

  return data;
}

int16_t ggCapFiltered(void) {
  int16_t data;
  
  i2cAcquireBus(driver);
  gg_get( GG_CMD_CAP, &data );
  i2cReleaseBus(driver);

  return data;
}

int16_t ggFullChargeCapUnfiltered(void) {
  int16_t data;
  
  i2cAcquireBus(driver);
  gg_get( GG_CMD_FCC_UN, &data );
  i2cReleaseBus(driver);

  return data;
}

int16_t ggFullChargeCapFiltered(void) {
  int16_t data;
  
  i2cAcquireBus(driver);
  gg_get( GG_CMD_FCC_F, &data );
  i2cReleaseBus(driver);

  return data;
}

int16_t ggStateofChargeUnfiltered(void) {
  int16_t data;
  
  i2cAcquireBus(driver);
  gg_get( GG_CMD_SOC_UN, &data );
  i2cReleaseBus(driver);

  return data;
}

uint16_t ggCtlStat(void) {
  int16_t data;
  
  i2cAcquireBus(driver);
  gg_get( GG_CMD_CNTL, &data );
  i2cReleaseBus(driver);

  return (uint16_t) data;
}

uint16_t ggFlags(void) {
  int16_t flags;
  
  i2cAcquireBus(driver);
  gg_get(GG_CMD_FLAG, &flags);
  i2cReleaseBus(driver);

  return (uint16_t) flags;
}

void compute_checksum(uint8_t *blockdata) {
  uint8_t i;
  uint8_t sum = 0;

  for( i = 0; i < 32; i++ ) {
    sum += blockdata[i];
  }

  blockdata[32] = 255 - sum;
}

uint16_t getDesignCapacity(void) {
  int16_t flags;
  uint16_t designCapacity;
  uint8_t  blockdata[33];
  uint8_t  i;

  i2cAcquireBus(driver);
  
  gg_set(GG_CMD_CNTL, GG_CODE_DEVTYPE);
  gg_get(GG_CMD_CNTL, &flags);
  chprintf( stream, "Device type: %04x\n\r", flags );

  // unseal the device by writing the unseal command twice
  gg_set(GG_CMD_CNTL, GG_CODE_UNSEAL);
  gg_set(GG_CMD_CNTL, GG_CODE_UNSEAL);

  // confirm cfgupdate mode by polling flags until bit 4 is set; takes up to 1 second
  do {
    gg_get(GG_CMD_FLAG, &flags);
  } while( !(flags & 0x10 ) );

  gg_set(GG_CMD_CNTL, GG_CODE_CTLSTAT);
  gg_get(GG_CMD_CNTL, &flags);
  chprintf( stream, "control status: %04x\n\r", flags );

  gg_set_byte(GG_EXT_BLKDATACTL, 0x00);    // enable block data memory control
  gg_set_byte(GG_EXT_BLKDATACLS, 0x52);    // set data class to 0x52 - state subclass

  gg_set_byte(GG_EXT_BLKDATAOFF, 0x00);    // set the block data offset
  chThdSleepMilliseconds(1); // give time for the block to load 
  // data offset is computed by (parameter * 32)

  for( i = 0; i < 33; i++ ) {
    gg_get_byte( GG_EXT_BLKDATABSE + i, &(blockdata[i]) );
  }

  designCapacity = blockdata[11] | (blockdata[10] << 8);

  // seal up the gas gauge
  gg_set( GG_CMD_CNTL, GG_CODE_SEAL ); 

  gg_set(GG_CMD_CNTL, GG_CODE_CTLSTAT);
  gg_get(GG_CMD_CNTL, &flags);
  chprintf( stream, "control status: %04x\n\r", flags );

  i2cReleaseBus(driver);

  return designCapacity;
  
}

// this function is to be used only once
// returns the previously recorded design capacity
uint16_t setDesignCapacity(uint16_t mAh, uint16_t mWh, uint16_t termV, uint16_t taper) {
  int16_t flags;
  uint16_t designCapacity;
  uint8_t  blockdata[33];
  uint8_t  i;

  i2cAcquireBus(driver);
  
  gg_set(GG_CMD_CNTL, GG_CODE_DEVTYPE);
  gg_get(GG_CMD_CNTL, &flags);
  chprintf( stream, "Device type: %04x\n\r", flags );

  // unseal the device by writing the unseal command twice
  gg_set(GG_CMD_CNTL, GG_CODE_UNSEAL);
  gg_set(GG_CMD_CNTL, GG_CODE_UNSEAL);

  // set configuration update command
  gg_set(GG_CMD_CNTL, GG_CODE_CFGUPDATE);
  
  // confirm cfgupdate mode by polling flags until bit 4 is set; takes up to 1 second
  do {
    gg_get(GG_CMD_FLAG, &flags);
  } while( !(flags & 0x10 ) );

  gg_set(GG_CMD_CNTL, GG_CODE_CTLSTAT);
  gg_get(GG_CMD_CNTL, &flags);
  chprintf( stream, "control status: %04x\n\r", flags );

  gg_set_byte(GG_EXT_BLKDATACTL, 0x00);    // enable block data memory control
  gg_set_byte(GG_EXT_BLKDATACLS, 0x52);    // set data class to 0x52 - state subclass

  gg_set_byte(GG_EXT_BLKDATAOFF, 0x00);    // set the block data offset
  chThdSleepMilliseconds(1); // give time for the block to load 
  // data offset is computed by (parameter * 32)

  for( i = 0; i < 33; i++ ) {
    gg_get_byte( GG_EXT_BLKDATABSE + i, &(blockdata[i]) );
  }

  designCapacity = blockdata[11] | (blockdata[10] << 8);
  chprintf( stream, "Debug: current design capacity: %dmAh checksum: %02x\n\r", designCapacity, blockdata[32] );
  designCapacity = blockdata[13] | (blockdata[12] << 8);
  chprintf( stream, "Debug: current design energy: %dmWh\n\r", designCapacity );
  designCapacity = blockdata[17] | (blockdata[16] << 8);
  chprintf( stream, "Debug: current term voltage: %dmV\n\r", designCapacity );
  designCapacity = blockdata[28] | (blockdata[27] << 8);
  chprintf( stream, "Debug: Taper rate: %d 0.1Hr\n\r", designCapacity );

  blockdata[11] = mAh & 0xFF;
  blockdata[10] = (mAh >> 8) & 0xFF;
  blockdata[13] = mWh & 0xFF;
  blockdata[12] = (mWh >> 8) & 0xFF;
  blockdata[17] = termV & 0xFF;
  blockdata[16] = (termV >> 8) & 0xFF;
  blockdata[28] = taper & 0xFF;
  blockdata[27] = (taper >> 8) & 0xFF;

  compute_checksum(blockdata);
  
  // commit the new data and checksum
  for( i = 0; i < 33; i++ ) {
    gg_set_byte( GG_EXT_BLKDATABSE + i, blockdata[i] );
  }
  chprintf( stream, "Debug: new design capacity: %dmAh checksum: %02x\n\r", mAh, blockdata[32]);

  gg_set( GG_CMD_CNTL, GG_CODE_RESET );
  //gg_set( GG_CMD_CNTL, GG_CODE_EXIT_CFGUPDATE );

  // confirm we're out of update mode, may take up to 1 second
  do {
    gg_get(GG_CMD_FLAG, &flags);
  } while( (flags & 0x10 ) );

  // seal up the gas gauge
  gg_set( GG_CMD_CNTL, GG_CODE_SEAL ); 

  gg_set(GG_CMD_CNTL, GG_CODE_CTLSTAT);
  gg_get(GG_CMD_CNTL, &flags);
  chprintf( stream, "control status: %04x\n\r", flags );

  i2cReleaseBus(driver);

  return designCapacity;
  
}

void dumpSubClass(uint8_t subclass, uint8_t offset, uint8_t *blockdata) {
  int i;
  int16_t stat;

  // check that we're unsealed -- it's the caller's resposibility
  i2cAcquireBus(driver);
  gg_set(GG_CMD_CNTL, GG_CODE_CTLSTAT);
  gg_get(GG_CMD_CNTL, &stat);
  if( (stat & 0x2000) ) {
    chprintf( stream, "gasgauge is sealed, can't proceed.\n\r" );
  }
  i2cReleaseBus(driver);
    
  i2cAcquireBus(driver);
  gg_set_byte(GG_EXT_BLKDATACTL, 0x00);    // enable block data memory control
  gg_set_byte(GG_EXT_BLKDATACLS, subclass);  

  gg_set_byte(GG_EXT_BLKDATAOFF, offset);    // set the block data offset
  chThdSleepMilliseconds(3); // give time for the block to load 

  for( i = 0; i < 33; i++ ) {
    gg_get_byte( GG_EXT_BLKDATABSE + i, &(blockdata[i]) );
  }
  i2cReleaseBus(driver);
  
  return;
}

#define GG_BM_CAPACITY 4000   // 4000 mAh
#define GG_BM_ENERGY 14800    // 4000 mAh * 3.7V = 14800 mWh
#define GG_BM_TERMV  3000     // 3.00V stop voltage -- manual says "abs min"
#define GG_BM_TAPER  105      // set for a shy less than 400mA charge term
// per quickstart taper rate = design capacity / (0.1 * taper current)
// taper current = 300mA * 15%. so, 4000 / (0.1 * 345) = 114
// set for 400mA as taper rate. Counting on power supply 

#define GG_BM_QMAX_MAX 25000  // roughly 50% over nominal max charge
#define GG_BM_QMAX_MIN 1700   // roughly 10% of original capacity -- we're def. not doing well
#define GG_BM_QMAX_NOM 16384  // default qmax

void ggCheckUpdate(uint8_t forceUpdate) {
  int16_t flags;
  uint16_t designCapacity, designEnergy, termV, taper;
  int16_t qmax;
  uint8_t  blockdata[33];
  uint8_t  i;
  uint32_t starttime, curtime;
  uint8_t timeout = 0;
  char uistr[32];
  int16_t patchval;

  i2cAcquireBus(driver);
  // unseal the device by writing the unseal command twice
  gg_set(GG_CMD_CNTL, GG_CODE_UNSEAL);
  gg_set(GG_CMD_CNTL, GG_CODE_UNSEAL);

  gg_set_byte(GG_EXT_BLKDATACTL, 0x00);    // enable block data memory control
  gg_set_byte(GG_EXT_BLKDATACLS, 0x52);    // set data class to 0x52 - state subclass

  gg_set_byte(GG_EXT_BLKDATAOFF, 0x00);    // set the block data offset
  chThdSleepMilliseconds(1); // give time for the block to load 
  // data offset is computed by (parameter * 32)

  for( i = 0; i < 33; i++ ) {
    gg_get_byte( GG_EXT_BLKDATABSE + i, &(blockdata[i]) );
  }
  i2cReleaseBus(driver);

  designCapacity = blockdata[11] | (blockdata[10] << 8);
  chprintf( stream, "ggUpdate: current design capacity: %dmAh checksum: %02x\n\r", designCapacity, blockdata[32] );
  designEnergy = blockdata[13] | (blockdata[12] << 8);
  chprintf( stream, "ggUpdate: current design energy: %dmWh\n\r", designEnergy );
  
  termV = blockdata[17] | (blockdata[16] << 8);
  chprintf( stream, "ggUpdate: current term voltage: %dmV\n\r", termV );
  taper = blockdata[28] | (blockdata[27] << 8);
  chprintf( stream, "ggUpdate: Taper rate: %d 0.1Hr\n\r", taper );

  qmax = blockdata[1] | (blockdata[0] << 8);
  chprintf( stream, "ggUpdate: Qmax: %d/16384 of design cap\n\r", qmax );

  // don't consider qmax is resetting params -- I don't trust the docs on default qmax value...
  if( (designCapacity != GG_BM_CAPACITY) || (designEnergy != GG_BM_ENERGY) ||
      (termV != GG_BM_TERMV) || (taper != GG_BM_TAPER) || forceUpdate) {
    // enter update mode
    // set configuration update command
    i2cAcquireBus(driver);
    gg_set(GG_CMD_CNTL, GG_CODE_CFGUPDATE);
    i2cReleaseBus(driver);
  
    timeout = 0;
    // confirm cfgupdate mode by polling flags until bit 4 is set; takes up to 1 second
    do {
      starttime = chVTGetSystemTime();
      do {
	i2cAcquireBus(driver);
	gg_get(GG_CMD_FLAG, &flags);
	i2cReleaseBus(driver);
	curtime = chVTGetSystemTime();
      } while( !(flags & 0x10) && (curtime < (starttime + 1300)) );
      if( curtime >= (starttime + 1300)) {
	timeout++;
	chprintf(stream, "ggCheckUpdate timeout going to update mode: %d tries\n\r", timeout);
	// try to unseal aagain
	//i2cAcquireBus(driver);
	//gg_set( GG_CMD_CNTL, GG_CODE_RESET ); 
	//i2cReleaseBus(driver);

	// this basically happens only if you reset the device twice rapidly in sequence.
	// you can only unseal after 4 seconds from sealing
	chsnprintf(uistr, sizeof(uistr), "attempt %d of 3", timeout);
	orchardTestPrompt("gasgauge timeout...", uistr, -5);
      
	i2cAcquireBus(driver);
	// unseal the device by writing the unseal command twice
	gg_set(GG_CMD_CNTL, GG_CODE_UNSEAL);
	gg_set(GG_CMD_CNTL, GG_CODE_UNSEAL);
	
	// set configuration update command
	gg_set(GG_CMD_CNTL, GG_CODE_CFGUPDATE);
	i2cReleaseBus(driver);
      }
    } while( !(flags & 0x10) && (timeout < 3) );
    if( timeout >= 3 ) {
      chprintf( stream, "ggCheckUpdate ERROR: Couldn't unseal gasgauge and enter update mode. Aborting.\n\r" );
      return;
    }
    
    /////////////// fix the capacity and energy settings...
    dumpSubClass(82, 0, blockdata);  // subclass 82
    
    designCapacity = GG_BM_CAPACITY;
    designEnergy = GG_BM_ENERGY;
    termV = GG_BM_TERMV;
    taper = GG_BM_TAPER;
    qmax = GG_BM_QMAX_NOM; // restore default qmax
    
    blockdata[11] = designCapacity & 0xFF;
    blockdata[10] = (designCapacity >> 8) & 0xFF;
    blockdata[13] = designEnergy & 0xFF;
    blockdata[12] = (designEnergy >> 8) & 0xFF;

    blockdata[17] = termV & 0xFF;
    blockdata[16] = (termV >> 8) & 0xFF;
    blockdata[28] = taper & 0xFF;
    blockdata[27] = (taper >> 8) & 0xFF;

    blockdata[1] = qmax & 0xFF;  // reset capacity tracking to nominal
    blockdata[0] = (qmax >> 8) & 0xFF;
    
    compute_checksum(blockdata);

    i2cAcquireBus(driver);
    // commit the new data and checksum
    for( i = 0; i < 33; i++ ) {
      gg_set_byte( GG_EXT_BLKDATABSE + i, blockdata[i] );
    }
    i2cReleaseBus(driver);
    chprintf( stream, "ggUpdate: updated design capacity: %dmAh energy: %dmWh checksum: %02x\n\r", designCapacity, designEnergy, blockdata[32]);

    ////////// also restore CAL data, because it's probably off if capacity was set wrong...
    dumpSubClass(105, 0, blockdata); // cc cal

    patchval = 0; // restore default cc offset
    blockdata[1] = patchval & 0xFF;
    blockdata[0] = (patchval >> 8) & 0xFF;
    patchval = 2982; // restore default cal temp
    blockdata[3] = patchval & 0xFF;
    blockdata[2] = (patchval >> 8) & 0xFF;
    
    compute_checksum(blockdata);
    
    i2cAcquireBus(driver);
    // commit the new data and checksum
    for( i = 0; i < 33; i++ ) {
      gg_set_byte( GG_EXT_BLKDATABSE + i, blockdata[i] );
    }
    i2cReleaseBus(driver);
    chprintf( stream, "ggUpdate: also restored cal data to default\n\r");

    ///////////// also update/fix QMAX change limits and simulation parameters
    dumpSubClass(80, 1, blockdata); // data class 80 - IT Cfg

    blockdata[45-32] = 100; // allow a larger qmax change
    blockdata[46-32] = 50; // allow a larger %age of design capacity change in qmax
    blockdata[47-32] = 150; // seems our batteries have larger capacities than printed on the case

    patchval = 200; // use a 200mA discharge rate for simulation
    blockdata[53-32] = patchval & 0xFF;
    blockdata[52-32] = (patchval >> 8) & 0xFF;

    patchval = 740; // use a 740 mWh discharge power for simulation 
    blockdata[55-32] = patchval & 0xFF;
    blockdata[54-32] = (patchval >> 8) & 0xFF;
    
    compute_checksum(blockdata);
    
    i2cAcquireBus(driver);
    // commit the new data and checksum
    for( i = 0; i < 33; i++ ) {
      gg_set_byte( GG_EXT_BLKDATABSE + i, blockdata[i] );
    }
    i2cReleaseBus(driver);
    chprintf( stream, "ggUpdate: also updated delta QMAX and sim parameters\n\r");
    
    ////////////// also pre-load avg current run/power #s and delta voltage
    dumpSubClass(82, 1, blockdata); // data class 80 - IT Cfg, offset 1

    patchval = 2; // use a 2mV deltaV
    blockdata[40-32] = patchval & 0xFF;
    blockdata[39-32] = (patchval >> 8) & 0xFF;

    // -125mAh average discharge = 4000/(-125 * 0.1) = -320
    patchval = -320;
    blockdata[36-32] = patchval & 0xFF;
    blockdata[35-32] = (patchval >> 8) & 0xFF;

    // power is ratiometric as well, so same patchval as I
    patchval = -320;
    blockdata[38-32] = patchval & 0xFF;
    blockdata[37-32] = (patchval >> 8) & 0xFF;
    
    compute_checksum(blockdata);
    
    i2cAcquireBus(driver);
    // commit the new data and checksum
    for( i = 0; i < 33; i++ ) {
      gg_set_byte( GG_EXT_BLKDATABSE + i, blockdata[i] );
    }
    i2cReleaseBus(driver);
    chprintf( stream, "ggUpdate: also updated avg I/P/deltaV params\n\r");

    ////////////// also set current thresholds
    dumpSubClass(81, 0, blockdata); // data class 81 - current thresholds

    // discharge threshold = 100 mA
    // 4000 / (100 * 0.1) = 400
    patchval = 400;
    blockdata[1] = patchval & 0xFF;
    blockdata[0] = (patchval >> 8) & 0xFF;
    
    // charge threshold = 300 mA
    // 4000 / (300 * 0.1) = 133
    patchval = 133;
    blockdata[3] = patchval & 0xFF;
    blockdata[2] = (patchval >> 8) & 0xFF;

    // quit current = 50 mA
    // 4000 / (50 * 0.1) = 800
    patchval = 800;
    blockdata[5] = patchval & 0xFF;
    blockdata[4] = (patchval >> 8) & 0xFF;

    compute_checksum(blockdata);
    
    i2cAcquireBus(driver);
    // commit the new data and checksum
    for( i = 0; i < 33; i++ ) {
      gg_set_byte( GG_EXT_BLKDATABSE + i, blockdata[i] );
    }
    i2cReleaseBus(driver);
    chprintf( stream, "ggUpdate: also updated current thresholds\n\r");
    
    
} else {
    chprintf( stream, "ggUpdate: Seems we are already updated, skipping update & sealing...\n\r" );
    i2cAcquireBus(driver);
    // seal up the gas gauge
    gg_set( GG_CMD_CNTL, GG_CODE_SEAL ); 
    i2cReleaseBus(driver);

    cmd_ggdump(stream, 0, NULL);  // call this to get out of "board cal mode"
    return;
  }
  
  // if we got here, our configuration was updated
  i2cAcquireBus(driver);
  gg_set( GG_CMD_CNTL, GG_CODE_RESET );
  //gg_set( GG_CMD_CNTL, GG_CODE_EXIT_CFGUPDATE );
  // this forces an OCV measurement and resimulation to calibrate the gasgauge
  i2cReleaseBus(driver);

  timeout = 0;
  // confirm we're out of update mode, may take up to 1 second
  do {
    starttime = chVTGetSystemTime();
    do {
      i2cAcquireBus(driver);
      gg_get(GG_CMD_FLAG, &flags);
      i2cReleaseBus(driver);
      curtime = chVTGetSystemTime();
    } while( (flags & 0x10) && (curtime < (starttime + 1500)) );
    if( curtime >= (starttime + 1500)) {
      timeout++;
      
      //i2cAcquireBus(driver);  // can't do this because it'll reset the config data too
      //gg_set( GG_CMD_CNTL, GG_CODE_RESET );
      //i2cReleaseBus(driver);
    }
  } while( (flags & 0x10) && (timeout < 1) );
  if( timeout >= 4 ) {
    chprintf( stream, "ggCheckUpdate ERROR: Couldn't reset gasgauge. Sealing anyways.\n\r" );
  }
    
  i2cAcquireBus(driver);
  // seal up the gas gauge
  gg_set( GG_CMD_CNTL, GG_CODE_SEAL ); 
  i2cReleaseBus(driver);

  chprintf( stream, "ggUpdate: update complete...\n\r" );
}

void printBlock(uint8_t *block) {
  int i;
  for( i = 0; i < 32; i += 2 ) {
    if( (i % 8) == 0 )
      chprintf( stream, "\n\r%02x: ", i );
    chprintf( stream, "%6d ", (int16_t) (block[i] << 8) | block[i+1] );
  }
  chprintf( stream, "%02x\n\r", block[32] ); // checksum
}

void printBlockOdd(uint8_t *block) {
  int i;
  for( i = 1; i < 32; i += 2 ) {
    if( (i % 8) == 1 )
      chprintf( stream, "\n\r%02x: ", i );
    chprintf( stream, "%6d ", (int16_t) (block[i] << 8) | block[i+1] );
  }
  chprintf( stream, "%02x\n\r", block[32] ); // checksum
}

void printBlock8(uint8_t *block) {
  int i;
  for( i = 0; i < 32; i += 1 ) {
    if( (i % 8) == 0 )
      chprintf( stream, "\n\r%02x: ", i );
    chprintf( stream, "%3d ", (uint8_t) block[i] );
  }
  chprintf( stream, "%02x\n\r", block[32] ); // checksum
}
 
void cmd_ggdump(BaseSequentialStream *chp, int argc, char *argv[]) {
  (void) chp;
  (void) argc;
  (void) argv;
  uint8_t  i;
  uint8_t  blockdata[33];
  
  i2cAcquireBus(driver);
  // unseal the device by writing the unseal command twice
  gg_set(GG_CMD_CNTL, GG_CODE_UNSEAL);
  gg_set(GG_CMD_CNTL, GG_CODE_UNSEAL);
  i2cReleaseBus(driver);

  dumpSubClass(64, 0, blockdata);
  chprintf(stream, "OpConfig: %x\n\r", blockdata[0] << 8 | blockdata[1] );
  chprintf(stream, "OpConfigB: %x\n\r", blockdata[2] << 8 | blockdata[3] );

  dumpSubClass(36, 0, blockdata);
  chprintf(stream, "Charge termination" );
  printBlock8(blockdata);
  
#if 1   // these match across gauge
  chprintf(stream, "\n\rIT Cfg:" );
  for( i = 0; i < 3; i++ ) {
    dumpSubClass(80, i, blockdata);
    if( i != 1 )
      printBlock(blockdata);
    if( i == 1 ) {
      chprintf(stream, "\n\rIT Cfg block 1 (offset 32) as bytes:" );
      printBlock8(blockdata);
    }
  }
#endif

  chprintf(stream, "\n\rCurrent thresholds:" );
  dumpSubClass(81, 0, blockdata);
  printBlock(blockdata);

  chprintf(stream, "\n\rState:" );
  for( i = 0; i < 2; i++ ) {
    dumpSubClass(82, i, blockdata);
    if( i == 0 )
      printBlock(blockdata);
    if( i == 1 )
      printBlockOdd(blockdata);
      
  }

  chprintf(stream, "\n\rR_a RAM:" );
  dumpSubClass(89, 0, blockdata);
  printBlock(blockdata);

  chprintf(stream, "\n\rCal data:" );
  dumpSubClass(104, 0, blockdata);
  printBlock(blockdata);

  chprintf(stream, "\n\rCC cal:" );
  dumpSubClass(105, 0, blockdata);
  printBlock(blockdata);

  dumpSubClass(107, 0, blockdata);
  chprintf(stream, "\n\rCurrent deadband: %d\n\r", blockdata[1] );

  i2cAcquireBus(driver);
  // seal up the gas gauge
  gg_set( GG_CMD_CNTL, GG_CODE_SEAL ); 
  i2cReleaseBus(driver);

}
orchard_command("ggdump", cmd_ggdump);

void cmd_ggsearch(BaseSequentialStream *chp, int argc, char *argv[]) {
  uint16_t subclass;
  uint8_t block;
  uint8_t  blockdata[33];
  uint16_t value;
  int i;
  
  if( argc != 1 ) {
    chprintf(chp, "Usage: ggsearch [value]\n\r");
  }

  value = strtoul(argv[0], NULL, 0);

  i2cAcquireBus(driver);
  // unseal the device by writing the unseal command twice
  gg_set(GG_CMD_CNTL, GG_CODE_UNSEAL);
  gg_set(GG_CMD_CNTL, GG_CODE_UNSEAL);
  i2cReleaseBus(driver);

  for( subclass = 0; subclass < 256; subclass++ ) {
    if( (subclass % 32) == 0 )
      chprintf(chp, "\r\n");
    chprintf(chp, ".");
    for( block = 0; block < 5; block++ ) {
      dumpSubClass((uint8_t) subclass, block, blockdata);
      for( i = 0; i < 31; i++ ) {
	if( (blockdata[i] == ((value >> 8) & 0xFF)) &&
	    (blockdata[i+1] == (value & 0xFF)) ) {
	  chprintf(chp, "\n\rMatch at subclass %d block %d offset %d\n\r",
		    subclass, block, i );
	}
      }
    }
  }

  i2cAcquireBus(driver);
  // seal up the gas gauge
  gg_set( GG_CMD_CNTL, GG_CODE_SEAL ); 
  i2cReleaseBus(driver);
}
orchard_command("ggsearch", cmd_ggsearch);

void cmd_ggpeek(BaseSequentialStream *chp, int argc, char *argv[]) {
  uint8_t subclass;
  uint8_t block;
  uint8_t  blockdata[33];
  
  if( argc != 2 ) {
    chprintf(chp, "Usage: ggpeek [subclass] [block]\n\r");
  }

  subclass = strtoul(argv[0], NULL, 0);
  block = strtoul(argv[1], NULL, 0);

  i2cAcquireBus(driver);
  // unseal the device by writing the unseal command twice
  gg_set(GG_CMD_CNTL, GG_CODE_UNSEAL);
  gg_set(GG_CMD_CNTL, GG_CODE_UNSEAL);
  i2cReleaseBus(driver);

  dumpSubClass(subclass, block, blockdata);

  chprintf(chp, "\n\rsub %d block %d as int16:", subclass, block);
  printBlock(blockdata);
  chprintf(chp, "\n\rsub %d block %d as uint8:", subclass, block);
  printBlock8(blockdata);

  i2cAcquireBus(driver);
  // seal up the gas gauge
  gg_set( GG_CMD_CNTL, GG_CODE_SEAL ); 
  i2cReleaseBus(driver);
}
orchard_command("ggpeek", cmd_ggpeek);
 
void cmd_ggfix(BaseSequentialStream *chp, int argc, char *argv[]) {
  (void) chp;
  (void) argc;
  (void) argv;
  
  ggCheckUpdate(1); // call with force update
  
}
orchard_command("ggfix", cmd_ggfix);

OrchardTestResult test_gasgauge(const char *my_name, OrchardTestType test_type) {
  (void) my_name;
  int16_t ret;

  switch(test_type) {
  case orchardTestPoweron:
  case orchardTestTrivial:
    i2cAcquireBus(driver);
    gg_set(GG_CMD_CNTL, GG_CODE_DEVTYPE);
    gg_get(GG_CMD_CNTL, &ret);
    i2cReleaseBus(driver);
    if( ret != 0x0421 ) {
      return orchardResultFail;
    } else {
      return orchardResultPass;
    }
    break;
  default:
    return orchardResultNoTest;
  }
  
  return orchardResultNoTest;
}
orchard_test("gasgauge", test_gasgauge);
