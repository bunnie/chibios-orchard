#include <stdio.h>

#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "orchard.h"
#include "orchard-app.h"

#include "storage.h"
#include "genes.h"
#include "gasgauge.h"

struct launcher_list_item {
  const char        *name;
  const OrchardApp  *entry;
};

struct launcher_list {
  unsigned int selected;
  unsigned int total;
  struct launcher_list_item items[0];
};

static uint32_t last_ui_time = 0;
#define BLINKY_DEFAULT_DELAY 60000 // in milliseconds

static void redraw_list(struct launcher_list *list) {

  char tmp[20];
  uint8_t i;
  const struct genes *family;

  coord_t width;
  coord_t height;
  coord_t totalheight;
  coord_t header_height;
  font_t font;
  uint8_t visible_apps;
  uint8_t app_modulus;
  uint8_t max_list;
  int32_t ui_timeout;

  ui_timeout = (BLINKY_DEFAULT_DELAY - (chVTGetSystemTime() - last_ui_time)) / 1000;

  family = (const struct genes *) storageGetData(GENE_BLOCK);
  //  chsnprintf(tmp, sizeof(tmp), "%d of %d apps", list->selected + 1, list->total);
  
  orchardGfxStart();
  // draw title bar
  font = gdispOpenFont("fixed_5x8");
  width = gdispGetWidth();
  height = gdispGetFontMetric(font, fontHeight);
  header_height = height;

  gdispClear(Black);
  gdispFillArea(0, 0, width, height, White);

  chsnprintf(tmp, sizeof(tmp), "%s", family->name);
  gdispDrawStringBox(0, 0, width, height,
                     tmp, font, Black, justifyLeft);
  chsnprintf(tmp, sizeof(tmp), "%d %d%%", ui_timeout, ggStateofCharge());
  gdispDrawStringBox(0, 0, width, height,
                     tmp, font, Black, justifyRight);

  // draw app list
  width = gdispGetWidth();
  height = gdispGetFontMetric(font, fontHeight);

  totalheight = gdispGetHeight();
  visible_apps = (uint8_t) (totalheight - header_height) / height;
  
  app_modulus = (uint8_t) list->selected / visible_apps;
  
  max_list = (app_modulus + 1) * visible_apps;
  if( max_list > list->total )
    max_list = list->total;
  
  for (i = app_modulus * visible_apps; i < max_list; i++) {
    color_t draw_color = White;
    
    if (i == list->selected) {
      gdispFillArea(0, header_height + (i - app_modulus * visible_apps) * height,
		    width, height, White);
      draw_color = Black;
    }
    
    gdispDrawStringBox(0, header_height + (i - app_modulus * visible_apps) * height,
                       width, height,
                       list->items[i].name, font, draw_color, justifyCenter);
  }
  gdispCloseFont(font);
  gdispFlush();
  orchardGfxEnd();
}

static uint32_t launcher_init(OrchardAppContext *context) {

  (void)context;
  unsigned int total_apps = 0;
  const OrchardApp *current;

  /* Rebuild the app list */
  current = orchard_app_start();
  while (current->name) {
    total_apps++;
    current++;
  }

  return sizeof(struct launcher_list)
       + (total_apps * sizeof(struct launcher_list_item));
}

static void launcher_start(OrchardAppContext *context) {

  struct launcher_list *list = (struct launcher_list *)context->priv;
  const OrchardApp *current;

  /* Rebuild the app list */
  current = orchard_app_start();
  list->total = 0;
  while (current->name) {
    list->items[list->total].name = current->name;
    list->items[list->total].entry = current;
    list->total++;
    current++;
  }

  list->selected = 3;

  last_ui_time = chVTGetSystemTime();
  redraw_list(list);
}

void launcher_event(OrchardAppContext *context, const OrchardAppEvent *event) {
  struct launcher_list *list = (struct launcher_list *)context->priv;
  int32_t ui_timeout;
  const OrchardApp *led_app;

  ui_timeout = (BLINKY_DEFAULT_DELAY - (chVTGetSystemTime() - last_ui_time)) / 1000;

  if (event->type == keyEvent) {
    last_ui_time = chVTGetSystemTime();
    if ((event->key.flags == keyDown) && (event->key.code == keyCW)) {
      list->selected++;
      if (list->selected >= list->total)
        list->selected = 0;
    }
    else if ((event->key.flags == keyDown) && (event->key.code == keyCCW)) {
      list->selected--;
      if (list->selected >= list->total)
        list->selected = list->total - 1;
    }
    else if ((event->key.flags == keyDown) && (event->key.code == keySelect)) {
      orchardAppRun(list->items[list->selected].entry);
    }
    redraw_list(list);
  } else if (event->type == adcEvent) {
    // to update % charge state based on USB detect status ping
    redraw_list(list);
  }

  if( (ui_timeout <= 0) || (ui_timeout > (BLINKY_DEFAULT_DELAY / 1000)) ) {
    led_app = orchardAppByName("Blinkies!");
    if( led_app != NULL )
      orchardAppRun(led_app);
  }
}

static void launcher_exit(OrchardAppContext *context) {

  (void)context;
}

const OrchardApp _orchard_app_list_launcher
__attribute__((unused, aligned(4), section(".chibi_list_app_1_launcher"))) = {
  "Launcher", 
  launcher_init,
  launcher_start,
  launcher_event,
  launcher_exit,
};
