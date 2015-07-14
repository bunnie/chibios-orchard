
#include "ch.h"
#include "hal.h"
#include "i2c.h"

#include "orchard.h"
#include "gasgauge.h"

#include "chprintf.h"
#include "userconfig.h"

#include "orchard-test.h"
#include "test-audit.h"

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
  // data offset is computed by (parameter * 32)

  for( i = 0; i < 33; i++ ) {
    gg_get_byte( GG_EXT_BLKDATABSE + i, &(blockdata[i]) );
  }

  designCapacity = blockdata[11] | (blockdata[10] << 8);

  gg_set( GG_CMD_CNTL, GG_CODE_RESET );

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

// this function is to be used only once
// returns the previously recorded design capacity
uint16_t setDesignCapacity(uint16_t mAh, uint16_t mWh) {
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
  // data offset is computed by (parameter * 32)

  for( i = 0; i < 33; i++ ) {
    gg_get_byte( GG_EXT_BLKDATABSE + i, &(blockdata[i]) );
  }

  designCapacity = blockdata[11] | (blockdata[10] << 8);
  chprintf( stream, "Debug: current design capacity: %dmAh checksum: %02x\n\r", designCapacity, blockdata[32] );
  designCapacity = blockdata[13] | (blockdata[12] << 8);
  chprintf( stream, "Debug: current design energy: %dmWh checksum: %02x\n\r", designCapacity, blockdata[32] );

  blockdata[11] = mAh & 0xFF;
  blockdata[10] = (mAh >> 8) & 0xFF;
  blockdata[13] = mWh & 0xFF;
  blockdata[12] = (mWh >> 8) & 0xFF;

  compute_checksum(blockdata);
  
  // commit the new data and checksum
  for( i = 0; i < 33; i++ ) {
    gg_set_byte( GG_EXT_BLKDATABSE + i, blockdata[i] );
  }
  chprintf( stream, "Debug: new design capacity: %dmAh checksum: %02x\n\r", mAh, blockdata[32]);

  gg_set( GG_CMD_CNTL, GG_CODE_RESET );

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

#define GG_BM_CAPACITY 4000   // 4000 mAh
#define GG_BM_ENERGY 14800    // 4000 mAh * 3.7V = 14800 mWh

void ggCheckUpdate(void) {
  int16_t flags;
  uint16_t designCapacity, designEnergy;
  uint8_t  blockdata[33];
  uint8_t  i;
  uint32_t starttime, curtime;
  uint8_t timeout = 0;
  char uistr[32];

  i2cAcquireBus(driver);
  // unseal the device by writing the unseal command twice
  gg_set(GG_CMD_CNTL, GG_CODE_UNSEAL);
  gg_set(GG_CMD_CNTL, GG_CODE_UNSEAL);

  // set configuration update command
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
      i2cAcquireBus(driver);
      gg_set( GG_CMD_CNTL, GG_CODE_RESET );
      i2cReleaseBus(driver);

      // this basically happens only if you reset the device twice rapidly in sequence.
      // so it's pretty rare
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

  i2cAcquireBus(driver);
  gg_set_byte(GG_EXT_BLKDATACTL, 0x00);    // enable block data memory control
  gg_set_byte(GG_EXT_BLKDATACLS, 0x52);    // set data class to 0x52 - state subclass

  gg_set_byte(GG_EXT_BLKDATAOFF, 0x00);    // set the block data offset
  // data offset is computed by (parameter * 32)

  for( i = 0; i < 33; i++ ) {
    gg_get_byte( GG_EXT_BLKDATABSE + i, &(blockdata[i]) );
  }
  i2cReleaseBus(driver);

  designCapacity = blockdata[11] | (blockdata[10] << 8);
  chprintf( stream, "ggUpdate: current design capacity: %dmAh checksum: %02x\n\r", designCapacity, blockdata[32] );
  designEnergy = blockdata[13] | (blockdata[12] << 8);
  chprintf( stream, "ggUpdate: current design energy: %dmWh checksum: %02x\n\r", designEnergy, blockdata[32] );
  if( (designCapacity != GG_BM_CAPACITY) || (designEnergy != GG_BM_ENERGY) ) {
    // fix the capacity and energy settings...
    designCapacity = GG_BM_CAPACITY;
    designEnergy = GG_BM_ENERGY;
    
    blockdata[11] = designCapacity & 0xFF;
    blockdata[10] = (designCapacity >> 8) & 0xFF;
    blockdata[13] = designEnergy & 0xFF;
    blockdata[12] = (designEnergy >> 8) & 0xFF;
    
    compute_checksum(blockdata);

    i2cAcquireBus(driver);
    // commit the new data and checksum
    for( i = 0; i < 33; i++ ) {
      gg_set_byte( GG_EXT_BLKDATABSE + i, blockdata[i] );
    }
    i2cReleaseBus(driver);
    chprintf( stream, "ggUpdate: updated design capacity: %dmAh energy: %dmWh checksum: %02x\n\r", designCapacity, designEnergy, blockdata[32]);
  } else {
    chprintf( stream, "ggUpdate: Seems we are already updated, skipping update...\n\r" );
  }
  
  i2cAcquireBus(driver);
  gg_set( GG_CMD_CNTL, GG_CODE_RESET );
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
      
      i2cAcquireBus(driver);
      gg_set( GG_CMD_CNTL, GG_CODE_RESET );
      i2cReleaseBus(driver);
    }
  } while( (flags & 0x10) && (timeout < 4) );
  if( timeout >= 4 ) {
    chprintf( stream, "ggCheckUpdate ERROR: Couldn't reset gasgauge. Sealing anyways.\n\r" );
  }
    
  i2cAcquireBus(driver);
  // seal up the gas gauge
  gg_set( GG_CMD_CNTL, GG_CODE_SEAL ); 
  i2cReleaseBus(driver);

  chprintf( stream, "ggUpdate: update complete...\n\r" );
}

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
