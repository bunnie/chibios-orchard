
#include "ch.h"
#include "hal.h"
#include "i2c.h"

#include "orchard.h"
#include "gasgauge.h"

#include "chprintf.h"

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

void compute_checksum(uint8_t *blockdata) {
  uint8_t i;
  uint8_t sum = 0;

  for( i = 0; i < 32; i++ ) {
    sum += blockdata[i];
  }

  blockdata[32] = 255 - sum;
}

// this function is to be used only once
// returns the previously recorded design capacity
uint16_t setDesignCapacity(uint16_t mAh) {
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

  blockdata[11] = mAh & 0xFF;
  blockdata[10] = (mAh >> 8) & 0xFF;

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

OrchardTestResult test_gasgauge(const char *my_name, OrchardTestType test_type) {

  switch(test_type) {
  default:
    auditUpdate(my_name, test_type, orchardResultNoTest);
  }
  
  return orchardResultNoTest;
}
orchard_test("gasgauge", test_gasgauge);
