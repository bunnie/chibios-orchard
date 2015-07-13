#include "led.h"
#include "orchard-app.h"
#include "gpiox.h"
#include "gasgauge.h"

static uint8_t orig_shift;

static void redraw_ui(void) {
  char tmp[] = "Powerbank mode";
  char uiStr[32];
  
  coord_t width;
  coord_t height;
  font_t font;

  orchardGfxStart();
  // draw the title bar
  font = gdispOpenFont("fixed_5x8");
  width = gdispGetWidth();
  height = gdispGetFontMetric(font, fontHeight);

  gdispClear(Black);
  gdispFillArea(0, 0, width, height, White);
  gdispDrawStringBox(0, 0, width, height,
                     tmp, font, Black, justifyCenter);


  // 1st line left
  gdispDrawStringBox(0, height*1, width, height,
		     "USB power source active", font, White, justifyLeft);
  
  // 2nd line left
  gdispDrawStringBox(0, height*2, width, height,
		     "Exit to turn off", font, White, justifyLeft);
  
  // 4th line left
  chsnprintf(uiStr, sizeof(uiStr), "Capacity: %d%%", ggStateofCharge());
  gdispDrawStringBox(0, height*4, width, height,
		     uiStr, font, White, justifyLeft);

  // 6th line left
  chsnprintf(uiStr, sizeof(uiStr), "Draw: %dmA", ggAvgCurrent());
  gdispDrawStringBox(0, height*6, width, height,
		     uiStr, font, White, justifyLeft);

  // 6th line right
  chsnprintf(uiStr, sizeof(uiStr), "Volt: %dmV", ggVoltage());
  gdispDrawStringBox(width/2, height*6, width/2, height,
		     uiStr, font, White, justifyRight);

  // 7th line left
  chsnprintf(uiStr, sizeof(uiStr), "Pwr: %dmW", ggAvgPower());
  gdispDrawStringBox(0, height*7, width, height,
		     uiStr, font, White, justifyLeft);

  // 7th line right
  chsnprintf(uiStr, sizeof(uiStr), "Tank: %dmAh", ggRemainingCapacity());
  gdispDrawStringBox(width/2, height*7, width/2, height,
		     uiStr, font, White, justifyRight);

  
  gdispFlush();
  orchardGfxEnd();

}

static uint32_t powerbank_init(OrchardAppContext *context) {

  (void)context;
  return 0;
}

static void powerbank_start(OrchardAppContext *context) {

  (void)context;

  // 100 ms timer
  orchardAppTimer(context, 100 * 1000 * 1000, true);

  orig_shift = getShift();
  setShift(6);

  gpioxSetPadMode(GPIOX, usbOutPad, GPIOX_OUT_PUSHPULL | GPIOX_VAL_LOW);
  
  redraw_ui();
}

static void powerbank_event(OrchardAppContext *context, const OrchardAppEvent *event) {

  (void)context;

  if (event->type == keyEvent) {
    if ( (event->key.flags == keyDown) && (event->key.code == keyLeft) ) {
      // don't do anything, this is a mis-feature
    } else if ((event->key.flags == keyDown) && (event->key.code == keySelect)) {
      // do we do anything on select?? probs not
    }
  } else if (event->type == timerEvent) {
    redraw_ui();
  }
}

static void powerbank_exit(OrchardAppContext *context) {

  (void)context;
  
  gpioxSetPadMode(GPIOX, usbOutPad, GPIOX_OUT_PUSHPULL | GPIOX_VAL_HIGH);
  setShift(orig_shift);
}

orchard_app("Powerbank mode", powerbank_init, powerbank_start, powerbank_event, powerbank_exit);
