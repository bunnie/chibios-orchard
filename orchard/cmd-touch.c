
#include "ch.h"
#include "hal.h"

#include "orchard.h"
#include "orchard-shell.h"
#include "captouch.h"

#include <stdlib.h>

static int should_stop(void) {
  uint8_t bfr[1];
  return chnReadTimeout(serialDriver, bfr, sizeof(bfr), 1);
}

static void cmd_touch(BaseSequentialStream *buf, int argc, char **argv) {

  uint16_t val;

  (void)argc;
  (void)argv;

  chprintf(buf, "Captouch value: \n\r");
  while( !should_stop() ) {
    val = captouchRead();
    chprintf(buf, "0x%04x\r", val);
  }
  chprintf(buf, "\r\n");
  
}

orchard_command("touch", cmd_touch);

static void cmd_tdbg(BaseSequentialStream *buf, int argc, char **argv) {
  (void)buf;
  uint8_t adr;
  uint8_t dat;

  if(argc == 1 ) {
    adr = (uint8_t ) strtoul(argv[0], NULL, 16);
    captouchPrint(adr);
  } else if( argc == 2 ) {
    adr = (uint8_t ) strtoul(argv[0], NULL, 16);
    dat = (uint8_t ) strtoul(argv[1], NULL, 16);
    captouchSet(adr, dat);
  } else {
    captouchDebug();
  }
}
orchard_command("td", cmd_tdbg);


static void cmd_tcal(BaseSequentialStream *buf, int argc, char **argv) {
  (void)buf;
  (void)argc;
  (void)argv;

  captouchRecal();
}
orchard_command("tcal", cmd_tcal);
