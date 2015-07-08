#ifndef __ORCHARD_APP_H__
#define __ORCHARD_APP_H__

#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "shell.h"
#include "orchard.h"
#include "gfx.h"
#include "orchard-ui.h"
#include "orchard-events.h"

struct _OrchardApp;
typedef struct _OrchardApp OrchardApp;
struct _OrchardAppContext;
typedef struct _OrchardAppContext OrchardAppContext;
struct orchard_app_instance;

/* Emitted to an app when it's about to be terminated */
extern event_source_t orchard_app_terminate;

/* Emitted to the system after the app has terminated */
extern event_source_t orchard_app_terminated;

// Emitted to the system after a UI dialog is completed
extern event_source_t ui_completed;

void orchardAppInit(void);
void orchardAppRestart(void);
void orchardAppWatchdog(void);
const OrchardApp *orchardAppByName(const char *name);
void orchardAppRun(const OrchardApp *app);
void orchardAppExit(void);
void orchardAppTimer(const OrchardAppContext *context,
                     uint32_t usecs,
                     bool repeating);
void friendsSort(void);
const char **friendsGet(void);
void friendsLock(void);
void friendsUnlock(void);
uint8_t friendCount(void);
uint8_t getMutationRate(void);
#define MAX_FRIENDS  100   // max # of friends to track

typedef struct _OrchardAppContext {
  struct orchard_app_instance *instance;
  uint32_t                    priv_size;
  void                        *priv;
} OrchardAppContext;

typedef struct _OrchardApp {
  char *name;
  uint32_t (*init)(OrchardAppContext *context);
  void (*start)(OrchardAppContext *context);
  void (*event)(OrchardAppContext *context, const OrchardAppEvent *event);
  void (*exit)(OrchardAppContext *context);
} OrchardApp;

typedef struct orchard_app_instance {
  const OrchardApp      *app;
  const OrchardApp      *next_app;
  OrchardAppContext     *context;
  thread_t              *thr;
  uint32_t              keymask;
  virtual_timer_t       timer;
  uint32_t              timer_usecs;
  bool                  timer_repeating;
  const OrchardUi       *ui;
  OrchardUiContext      *uicontext;
  uint32_t              ui_result;
} orchard_app_instance;

#define orchard_app_start()                                                   \
({                                                                            \
  static char start[0] __attribute__((unused,                                 \
    aligned(4), section(".chibi_list_app_1")));                               \
  (const OrchardApp *)&start;                                                 \
})

#define orchard_app(_name, _init, _start, _event, _exit)                        \
  const OrchardApp _orchard_app_list_##_init##_start##_event##_exit           \
  __attribute__((unused, aligned(4), section(".chibi_list_app_2_" # _event # _start # _init # _exit))) =  \
     { _name, _init, _start, _event, _exit }

#define orchard_app_end()                                                     \
  const OrchardApp _orchard_app_list_final                                    \
  __attribute__((unused, aligned(4), section(".chibi_list_app_3_end"))) =     \
     { NULL, NULL, NULL, NULL, NULL }

#define ORCHARD_APP_PRIO (LOWPRIO + 2)

#endif /* __ORCHARD_APP_H__ */
