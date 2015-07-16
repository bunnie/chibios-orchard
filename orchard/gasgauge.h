
int16_t ggVoltage(void);
int16_t ggAvgCurrent(void);
int16_t ggAvgPower(void);
int16_t ggRemainingCapacity(void);
int16_t ggStateofCharge(void);
int16_t ggStateofChargeUnpatch(void);
int16_t ggFullChargeCap(void);
int16_t ggFullAvailableCap(void);
int16_t ggNomAvailableCap(void);
uint16_t ggCtlStat(void);
uint16_t ggFlags(void);

uint16_t setDesignCapacity(uint16_t mAh, uint16_t mWh, uint16_t termV, uint16_t taper);
uint16_t getDesignCapacity(void);
void ggCheckUpdate(void);

void ggSetHibernate(void);

void ggStart(I2CDriver *i2cp);

int16_t ggRemainingUnfiltered(void);
int16_t ggCapFiltered(void);
int16_t ggFullChargeCapUnfiltered(void);
int16_t ggFullChargeCapFiltered(void);
int16_t ggStateofChargeUnfiltered(void);

// word-width commands
#define GG_CMD_CNTL  0x00
#define GG_CMD_TEMP  0x02     // 0.1 degrees K, of battery
#define GG_CMD_VOLT  0x04     // (mV)
#define GG_CMD_FLAG  0x06
#define GG_CMD_NOM_CAP  0x08  // nominal available capacity (mAh)
#define GG_CMD_FULL_CAP 0x0A  // full available capacity (mAh)
#define GG_CMD_RM       0x0C  // remaining capacity (mAh)
#define GG_CMD_FCC      0x0E  // full charge capacity (mAh)
#define GG_CMD_AVGCUR   0x10  // (mA)
#define GG_CMD_SBYCUR   0x12  // standby current (mA)
#define GG_CMD_MAXCUR   0x14  // max load current (mA)
#define GG_CMD_AVGPWR   0x18  // average power (mW)
#define GG_CMD_SOC      0x1C  // state of charge in %
#define GG_CMD_INTTEMP  0x1E  // temperature of gg IC
#define GG_CMD_SOH      0x20  // state of health, num / %
#define GG_CMD_RM_UN    0x28  // remaining capacity unfiltered
#define GG_CMD_CAP      0x2A  // remaining capacity filtered
#define GG_CMD_FCC_UN   0x2C  // full charge capacity unfiltered
#define GG_CMD_FCC_F    0x2E  // full charge capacity filtered
#define GG_CMD_SOC_UN   0x30  // state of charge unfiltered

// single-byte extended commands
#define GG_EXT_BLKDATACTL  0x61  // block data control
#define GG_EXT_BLKDATACLS  0x3E  // block data class
#define GG_EXT_BLKDATAOFF  0x3F  // block data offset
#define GG_EXT_BLKDATACHK  0x60  // block data checksum
#define GG_EXT_BLKDATABSE  0x40  // block data base

// control command codes
#define GG_CODE_CTLSTAT 0x0000
#define GG_CODE_DEVTYPE 0x0001
#define GG_CODE_UNSEAL  0x8000
#define GG_CODE_SEAL    0x0020
#define GG_CODE_CFGUPDATE 0x0013
#define GG_CODE_HARDRESET 0x0041
#define GG_CODE_RESET   0x0042
#define GG_CODE_SET_HIB 0x0011
#define GG_CODE_CLR_HIB 0x0012
#define GG_CODE_EXIT_CFGUPDATE 0x0043
#define GG_CODE_EXIT_RESIM 0x0044

#define GG_UPDATE_INTERVAL_MS 1000
