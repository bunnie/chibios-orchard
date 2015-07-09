#include "orchard-app.h"
#include "orchard-test.h"

#include "radio.h"

static uint32_t test_rxdat;

static void testmode_start(OrchardAppContext *context) {
  (void)context;
  
  orchardTestPrompt("Testing...", "", 0);
}

static void testmode_event(OrchardAppContext *context,
			   const OrchardAppEvent *event) {
  (void) context;
  (void) event;

  // do nothing -- but this function has to exist for the app to be running
  // if you optimize it out with a NULL function, the app quits immediately
  return;
}

orchard_app("~testmode", NULL, testmode_start, testmode_event, NULL);

#if 0  // comment out for production units
static void test_peer_handler(uint8_t prot, uint8_t src, uint8_t dst,
                              uint8_t length, const void *data) {

  (void)length;
  (void)prot;
  (void)src;
  (void)dst;

  test_rxdat = *((uint32_t *) data);

  chEvtBroadcast(&radio_app);
}

static void testpeer_start(OrchardAppContext *context) {
  (void)context;
  
  radioSetHandler(radioDriver, radio_prot_dut_to_peer, test_peer_handler);
  orchardTestPrompt("Now a test peer", "reboot to stop.", 0);
}

static void testpeer_event(OrchardAppContext *context,
                           const OrchardAppEvent *event) {
  (void)context;
  char datstr[16];
  
  if (event->type == radioEvent) {
    chprintf(stream, "received %08x, rebroadcasting...\n\r", test_rxdat);
    chsnprintf(datstr, sizeof(datstr), "%08x", test_rxdat);
    orchardTestPrompt("received:", datstr, 0);
    radioAcquire(radioDriver);
    radioSend(radioDriver, RADIO_BROADCAST_ADDRESS, radio_prot_peer_to_dut,
              4, &test_rxdat);
    radioRelease(radioDriver);
  }
}

orchard_app("~testpeer", NULL, testpeer_start, testpeer_event, NULL);
#endif
