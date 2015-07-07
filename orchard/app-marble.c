#include "ch.h"
#include "hal.h"
#include "orchard-app.h"

#include "gfx.h"
#include "led.h"
#include "accel.h"
#include "captouch.h"
#include "orchard-ui.h"

#define TILT_THRESH 100
#define TILT_RATE   128
#define BALL_SIZE  6

struct accel_data d, dref;

static void redraw_ui(void) {
  static coord_t x = 60, y = 28;
  static coord_t xo, yo;
  static uint8_t changed = 0;
  coord_t width, height;

  width = gdispGetWidth();    // these are thread-safe, don't lock around them
  height = gdispGetHeight();

  accelPoll(&d);
  d.x = (d.x + 2048) & 0xFFF;
  d.y = (d.y + 2048) & 0xFFF;
  
  // x values go up as you tilt to the right
  // y vaules go up as you tilt toward the bottom
  xo = x; yo = y;
  if( d.x > (dref.x + TILT_THRESH) ) {
    x += (d.x - dref.x) / TILT_RATE;
    changed = 1;
  } else if( d.x < (dref.x - TILT_THRESH) ) {
    x -= (dref.x -d.x) / TILT_RATE;
    changed = 1;
  }
  if( x > (width - BALL_SIZE))
    x = width - BALL_SIZE;
  if( x < BALL_SIZE )
    x = BALL_SIZE;
    
  if( d.y > (dref.y + TILT_THRESH) ) {
    y += (d.y - dref.y) / TILT_RATE;
    changed = 1;
  } else if( d.y < (dref.y - TILT_THRESH) ) {
    y -= (dref.y -d.y) / TILT_RATE;
    changed = 1;
  }
  if( y > (height - BALL_SIZE))
    y = height - BALL_SIZE;
  if( y < BALL_SIZE )
    y = BALL_SIZE;
  
  orchardGfxStart();
  if( changed ) {
    // blank out the old ball
    changed  = 0;
    gdispFillCircle(xo, yo, BALL_SIZE, Black);
  }

  // draw the new ball position
  gdispFillCircle(x, y, BALL_SIZE, White);
  gdispFlush();
  orchardGfxEnd();
}


static uint32_t marble_init(OrchardAppContext *context) {

  (void)context;
  return 0;
}

static void marble_start(OrchardAppContext *context) {

  (void)context;
  
  accelPoll(&dref);  // seed accelerometer values
  dref.x = (dref.x + 2048) & 0xFFF;
  dref.y = (dref.y + 2048) & 0xFFF;

  orchardGfxStart();
  gdispClear(Black);
  gdispFlush();
  orchardGfxEnd();

  orchardAppTimer(context, 20 * 1000, true); // screen refresh timer at once every 20ms
}

static void marble_event(OrchardAppContext *context, const OrchardAppEvent *event) {

  (void)context;

  if (event->type == keyEvent) {
    if (event->key.flags == keyDown) {
      if( event->key.code == keySelect ) {
	// center the accelerometer reference
	accelPoll(&dref);
	dref.x = (dref.x + 2048) & 0xFFF;
	dref.y = (dref.y + 2048) & 0xFFF;
      }
    }
  } else if (event->type == timerEvent) {
    redraw_ui();
  }
}

static void marble_exit(OrchardAppContext *context) {

  (void)context;
}

orchard_app("Marble", marble_init, marble_start, marble_event, marble_exit);
