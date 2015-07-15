#include "orchard-app.h"
#include "orchard-ui.h"

#include "analog.h"
#include "gasgauge.h"
#include "charger.h"

#include <string.h>
#include <stdlib.h>

static int32_t celcius;
static uint16_t usbn, usbp;
static usbStat usbStatus = usbStatNC;

typedef enum chargerOverrideType {
  chargerNoOverride = 0,
  charger500 = 1,
  charger1500 = 2,
  chargerOverrideTotal = 3,
} chargerOverrideType;

static chargerOverrideType override = chargerNoOverride;

// flag argument should probably be turned into an enum in a refactor
// flag = 0 means normal UI
// flag = 1 means first-time UI (indicate ADC is measuring)
// flag = 2 means confirm override
static void redraw_ui(uint8_t flag) {
  char tmp[] = "Charger status";
  char uiStr[32];
  
  coord_t width;
  coord_t height;
  font_t font;
  color_t draw_color = White;

  orchardGfxStart();
  // draw the title bar
  font = gdispOpenFont("fixed_5x8");
  width = gdispGetWidth();
  height = gdispGetFontMetric(font, fontHeight);

  gdispClear(Black);
  gdispFillArea(0, 0, width, height, White);
  gdispDrawStringBox(0, 0, width, height,
                     tmp, font, Black, justifyCenter);

  // 1st line: CPU temp
  if( flag == 1 ) {
    chsnprintf(uiStr, sizeof(uiStr), "CPU Temp: measuring...");
  } else {
    chsnprintf(uiStr, sizeof(uiStr), "CPU Temp: %d.%dC", celcius / 1000, 
	       abs((celcius % 1000) / 100));
  }
  gdispDrawStringBox(0, height, width, height,
		     uiStr, font, White, justifyLeft);

  // 2nd line left: USB-
  if( flag == 1 ) {
    chsnprintf(uiStr, sizeof(uiStr), "USB-: measuring...");
  } else {
    chsnprintf(uiStr, sizeof(uiStr), "USB-: 0x%04x", usbn);
  }
  gdispDrawStringBox(0, height*2, width, height,
		     uiStr, font, White, justifyLeft);

  // 3rd line left: USB+
  if( flag == 1) {
    chsnprintf(uiStr, sizeof(uiStr), "USB+: measuring...");
  } else {
    chsnprintf(uiStr, sizeof(uiStr), "USB+: 0x%04x", usbp);
  }
  gdispDrawStringBox(0, height*3, width, height,
		     uiStr, font, White, justifyLeft);


  if( !(flag == 1) ) {
    // 2nd line right
    chsnprintf(uiStr, sizeof(uiStr), "Detected:");
    gdispDrawStringBox(width / 2, height*2, width / 2, height,
		       uiStr, font, White, justifyRight);

    // 3rd line right
    switch(usbStatus) {
    case usbStatNC:
      chsnprintf(uiStr, sizeof(uiStr), "Unplugged");
      break;
    case usbStat500:
      chsnprintf(uiStr, sizeof(uiStr), "USB Host");
      break;
    case usbStat1500:
      chsnprintf(uiStr, sizeof(uiStr), "1.5A chgr");
      break;
    default:
      chsnprintf(uiStr, sizeof(uiStr), "Error");
    }
    gdispDrawStringBox(width / 2, height*3, width / 2, height,
		       uiStr, font, White, justifyRight);

  }
  
  // 4th line left
  chsnprintf(uiStr, sizeof(uiStr), "Draw: %dmA", ggAvgCurrent());
  gdispDrawStringBox(0, height*4, width, height,
		     uiStr, font, White, justifyLeft);

  // 4th line right
  chsnprintf(uiStr, sizeof(uiStr), "Volt: %dmV", ggVoltage());
  gdispDrawStringBox(width/2, height*4, width/2, height,
		     uiStr, font, White, justifyRight);

  // 5th line left
  chsnprintf(uiStr, sizeof(uiStr), "Pwr: %dmW", ggAvgPower());
  gdispDrawStringBox(0, height*5, width, height,
		     uiStr, font, White, justifyLeft);

  // 5th line right
  chsnprintf(uiStr, sizeof(uiStr), "Tank: %dmAh", ggRemainingCapacity());
  gdispDrawStringBox(width/2, height*5, width/2, height,
		     uiStr, font, White, justifyRight);

  // 6th line left
  chsnprintf(uiStr, sizeof(uiStr), "Capacity: %d%%", ggStateofCharge());
  gdispDrawStringBox(0, height*6, width, height,
		     uiStr, font, White, justifyLeft);

  // 6th line right
  //  chsnprintf(uiStr, sizeof(uiStr), "Cap: %d\%", ggRemainingCapacity());
  //  gdispDrawStringBox(width/2, height*6, width/2, height,
  //		     uiStr, font, White, justifyRight);

  // 7th line left
  if( flag != 2 ) 
    chsnprintf(uiStr, sizeof(uiStr), "Override:");
  else
    chsnprintf(uiStr, sizeof(uiStr), "Overriding!!");
  gdispDrawStringBox(0, height*7, width, height,
		     uiStr, font, White, justifyLeft);

  chsnprintf(uiStr, sizeof(uiStr), "500");
  if( override == charger500 ) {
    gdispFillArea(width/2, height*7, width/4, height, White);
    draw_color = Black;
  } else {
    draw_color = White;
  }
  gdispDrawStringBox(width/2, height*7, width/4, height,
		     uiStr, font, draw_color, justifyCenter);

  chsnprintf(uiStr, sizeof(uiStr), "1500");
  if( override == charger1500 ) {
    gdispFillArea(width/2 + width/4, height*7, width/4, height, White);
    draw_color = Black;
  } else {
    draw_color = White;
  }
  gdispDrawStringBox(width/2 + width/4, height*7, width/4, height,
		     uiStr, font, draw_color, justifyCenter);
  
  gdispFlush();
  orchardGfxEnd();
}

static uint32_t charging_init(OrchardAppContext *context) {

  (void)context;

  return 0;
}

static void charging_start(OrchardAppContext *context) {
  
  analogUpdateTemperature(); 
  analogUpdateUsbStatus(); 
  orchardAppTimer(context, 500 * 1000 * 1000, true); // update UI every 500 ms
  
  redraw_ui(1);
  // all this app does is launch a text entry box and store the name
}

void charging_event(OrchardAppContext *context, const OrchardAppEvent *event) {

  (void)context;
  adcsample_t *buf;
  
  if (event->type == keyEvent) {
    if ( (event->key.flags == keyDown) && (event->key.code == keyLeft) ) {
      // orchardAppExit();  this is probably a misfeature
    } else if ((event->key.flags == keyDown) && (event->key.code == keyCW)) {
      override = (override + 1) % chargerOverrideTotal;

      redraw_ui(0);
    } else if ((event->key.flags == keyDown) && (event->key.code == keyCCW)) {
      if( override > 0 )
	override = override - 1;
      else
	override = chargerOverrideTotal - 1;

      redraw_ui(0);
    } else if ((event->key.flags == keyDown) && (event->key.code == keySelect)) {
      if( override == charger500 ) {
	chargerForce500();
	chargerSetTargetCurrent(500);
      } else if( override == charger1500 ) {
	chargerForce1500();
	chargerSetTargetCurrent(1500);
      }
      if( override != chargerNoOverride )
	redraw_ui(2);
    }
  } else if (event->type == timerEvent) {
    analogUpdateTemperature(); // the actual value won't update probably until the UI is redrawn...
    analogUpdateUsbStatus(); 
    redraw_ui(0);
  } else if( event->type == adcEvent) {
    if( event->adc.code == adcCodeTemp ) {
      celcius = analogReadTemperature();
    } else if( event->adc.code == adcCodeUsbdet ) {
      buf = analogReadUsbRaw();
      usbn = (uint16_t) buf[0];
      usbp = (uint16_t) buf[1];
      
      usbStatus = analogReadUsbStatus();
    }
  }
}

static void charging_exit(OrchardAppContext *context) {

  (void)context;
}

orchard_app("Charging settings", charging_init, charging_start, charging_event, charging_exit);


