#include "orchard-app.h"
#include "orchard-ui.h"

#include "storage.h"
#include "genes.h"

#include <string.h>

struct OrchardUiContext textUiContext;

static char myname[TEXTENTRY_MAXLEN + 1];
  
static void redraw_ui(void) {
  char tmp[] = "Enter your name";
  
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

  gdispFlush();
  orchardGfxEnd();
}

static void draw_confirmation(void) {
  coord_t width;
  coord_t height;
  font_t font;
  char tmp[32];
  char tmp2[32];
  const struct genes *family;
  struct genes newFamily;

  redraw_ui();  // clear the text entry area
  family = (const struct genes *) storageGetData(GENE_BLOCK);

  if( strlen(myname) < 2 ) {
    chsnprintf(tmp, sizeof(tmp), "Name too short. Reset to:");
    strncpy(myname, family->name, GENE_NAMELENGTH); // reset to original name
  } else {
    chsnprintf(tmp, sizeof(tmp), "Your name is now:");
  }
  chsnprintf(tmp2, sizeof(tmp2), "Press select to continue.");
  
  orchardGfxStart();
  font = gdispOpenFont("fixed_5x8");
  width = gdispGetWidth();
  height = gdispGetFontMetric(font, fontHeight);

  gdispDrawStringBox(0, height, width, height,
                     tmp, font, White, justifyCenter);

  gdispDrawStringBox(0, height * 2, width, height,
                     myname, font, White, justifyCenter);
  
  gdispDrawStringBox(0, height * 4, width, height,
                     tmp2, font, White, justifyCenter);
  
  gdispFlush();
  orchardGfxEnd();
  
  family = (const struct genes *) storageGetData(GENE_BLOCK);
  memcpy(&newFamily, family, sizeof(struct genes));
  strncpy(newFamily.name, myname, GENE_NAMELENGTH);
  newFamily.name[GENE_NAMELENGTH - 1] = '\0'; // enforce the null terminator
  
  storagePatchData(GENE_BLOCK, (uint32_t *) &newFamily, GENE_OFFSET, sizeof(struct genes));
}

static uint32_t name_init(OrchardAppContext *context) {

  (void)context;

  return 0;
}

static void name_start(OrchardAppContext *context) {
  const OrchardUi *textUi;
  const struct genes *family;
  
  family = (const struct genes *) storageGetData(GENE_BLOCK);
  strncpy(myname, family->name, GENE_NAMELENGTH); // populate myname with an initial value
  
  redraw_ui();
  // all this app does is launch a text entry box and store the name
  textUi = getUiByName("textentry");
  textUiContext.itemlist = NULL;
  if( textUi != NULL ) {
    context->instance->uicontext = &textUiContext;
    context->instance->ui = textUi;
  }
  textUi->start(context);
  
}

void name_event(OrchardAppContext *context, const OrchardAppEvent *event) {

  (void)context;
  
  if (event->type == keyEvent) {
    if ( (event->key.flags == keyDown) && (event->key.code == keySelect) ) {
      orchardAppExit();
    }
  } else if( event->type == uiEvent ) {
    if(( event->ui.code == uiComplete ) && ( event->ui.flags == uiOK )) {
      strncpy(myname, (char *) context->instance->ui_result, TEXTENTRY_MAXLEN + 1);
    }
    chprintf( stream, "My name is: %s\n\r", myname );

    draw_confirmation();
  }
}

static void name_exit(OrchardAppContext *context) {

  (void)context;
}

orchard_app("Set your name", name_init, name_start, name_event, name_exit);


