#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "orchard.h"
#include "orchard-app.h"
#include "orchard-events.h"
#include "orchard-math.h"
#include "captouch.h"
#include "orchard-ui.h"
#include "analog.h"
#include "charger.h"
#include "paging.h"
#include "led.h"
#include "accel.h"
#include "genes.h"
#include "storage.h"
#include "radio.h"
#include "TransceiverReg.h"
#include "gasgauge.h"
#include "userconfig.h"

#include "shell.h" // for friend testing function
#include "orchard-shell.h" // for friend testing function

#define SEXTEST 0
#define SEX_TURNAROUND_TIME 6000 // 6 seconds for sex -- mostly to give sender time to "shake it"

extern uint8_t sex_running;  // from app-default.c
extern uint8_t sex_done;

orchard_app_end();

static const OrchardApp *orchard_app_list;
static virtual_timer_t run_launcher_timer;
static bool run_launcher_timer_engaged;
#define RUN_LAUNCHER_TIMEOUT MS2ST(500)

orchard_app_instance instance;  // the one and in fact only instance of any orchard app

typedef enum _DirIntent {
  dirNone = 0x0,
  dirCW = 0x1,
  dirCCW = 0x2,
} DirIntent;

static struct jogdial_state {
  int8_t lastpos;
  DirIntent direction_intent;
  uint32_t lasttime;
} jogdial_state;
#define DWELL_THRESH  500  // time to spend in one state before direction intent is null

event_source_t orchard_app_terminated;
event_source_t orchard_app_terminate;
event_source_t timer_expired;
event_source_t ui_completed;

static virtual_timer_t keycollect_timer;
static event_source_t keycollect_timeout;
static uint16_t  captouch_collected_state = 0;

#define COLLECT_INTERVAL 50  // time to collect events for multi-touch gesture
#define TRACK_INTERVAL 1  // trackpad debounce in ms
static unsigned long track_time;

static virtual_timer_t chargecheck_timer;
static event_source_t chargecheck_timeout;
#define CHARGECHECK_INTERVAL 1000 // time between checking state of USB pins

static virtual_timer_t ping_timer;
static event_source_t ping_timeout;
#define PING_MIN_INTERVAL  5000 // base time between pings
#define PING_RAND_INTERVAL 2000 // randomization zone for pings
static char *friends[MAX_FRIENDS]; // array of pointers to friends' names; first byte is priority metric
#define FRIENDS_INIT_CREDIT  4  // defines how long a friend record stays around before expiration
// max level of credit a friend can have; defines how long a record can stay around
// once a friend goes away. Roughly equal to
// 2 * (PING_MIN_INTERVAL + PING_RAND_INTERVAL / 2 * MAX_CREDIT) milliseconds
#define FRIENDS_MAX_CREDIT   12
#define FRIENDS_SORT_HYSTERESIS 4 

static uint8_t cleanup_state = 0;
mutex_t friend_mutex;

static uint8_t ui_override = 0;

void friend_cleanup(void);

#define MAIN_MENU_MASK  ((1 << 11) | (1 << 0))
#define MAIN_MENU_VALUE ((1 << 11) | (1 << 0))

static usbStat lastUSBstatus = usbStatUnknown;

static void handle_radio_page(eventid_t id) {
  (void) id;
  uint8_t oldfx;

  ui_override = 1;
  oldfx = effectsGetPattern();
  effectsSetPattern(effectsNameLookup("strobe"));
  radioPagePopup();
  effectsSetPattern(oldfx);
  ui_override = 0;
}

static void handle_ping_timeout(eventid_t id) {
  (void) id;
  const struct genes *family;
  family = (const struct genes *) storageGetData(GENE_BLOCK);

  radioAcquire(radioDriver);
  radioSend(radioDriver, RADIO_BROADCAST_ADDRESS, radio_prot_ping,
	    strlen(family->name) + 1, family->name);
  radioRelease(radioDriver);
    
  // cleanup every other ping we send, to make sure friends that are
  // nearby build up credit over time to max credits

  // if the system is unstable, tweak this parameter to reduce the
  // clean-up rate
  if( cleanup_state ) {
    friend_cleanup();
    chEvtBroadcast(&radio_app);
  }
  cleanup_state = !cleanup_state;
}

char *friend_lookup(char *name) {
  int i;

  osalMutexLock(&friend_mutex);
  for( i = 0; i < MAX_FRIENDS; i++ ) {
    if( friends[i] != NULL ) {
      if(0 == strncmp(&(friends[i][1]), name, GENE_NAMELENGTH)) {
	osalMutexUnlock(&friend_mutex);
	return friends[i];
      }
    }
  }
  osalMutexUnlock(&friend_mutex);
  return NULL;
}

uint8_t friendCount(void) {
  int i;
  uint8_t count = 0;
  
  osalMutexLock(&friend_mutex);
  for( i = 0; i < MAX_FRIENDS; i++ ) {
    if( friends[i] != NULL )
      count++;
  }
  osalMutexUnlock(&friend_mutex);
  
  return count;
}

char *friend_add(char *name) {
  char *record;
  uint32_t i;

  record = friend_lookup(name);
  if( record != NULL )
    return record;  // friend already exists, don't add it again

  osalMutexLock(&friend_mutex);
  for( i = 0; i < MAX_FRIENDS; i++ ) {
    if( friends[i] == NULL ) {
      friends[i] = (char *) chHeapAlloc(NULL, GENE_NAMELENGTH + 2); // space for NULL + metric byte
      if( friends[i] != NULL ) {
	friends[i][0] = FRIENDS_INIT_CREDIT;
	strncpy(&(friends[i][1]), name, GENE_NAMELENGTH);
	osalMutexUnlock(&friend_mutex);
	return friends[i];
      }
    }
  }
  osalMutexUnlock(&friend_mutex);

  // if we got here, we couldn't add the friend because we ran out of space
  return NULL;
}

// generate a one-time list of random names and populate the friend list
// for testing only
void cmd_friendlocal(BaseSequentialStream *chp, int argc, char *argv[]) {
  char tempName[GENE_NAMELENGTH];
  uint32_t i;
  uint32_t total;
  
  if( argc != 1 ) {
    chprintf(chp, "usage: friendlocal <friendcount>\n\r");
  }
  total = strtoul(argv[0], NULL, 0);

  for(i = 0; i < total; i++ ) {
    generateName(tempName);
    friend_add(tempName);
  }
}
orchard_command("friendlocal", cmd_friendlocal);

// to be called periodically to decrement credits and de-alloc friends we haven't seen in a while
void friend_cleanup(void) {
  uint32_t i;

  osalMutexLock(&friend_mutex);
  for( i = 0; i < MAX_FRIENDS; i++ ) {
    if( friends[i] == NULL )
      continue;

    friends[i][0]--;
    if( friends[i][0] == 0 ) {
      chHeapFree(friends[i]);
      friends[i] = NULL;
    }
  }
  osalMutexUnlock(&friend_mutex);
}

int friend_comp(const void *a, const void *b) {
  char **mya;
  char **myb;
  
  mya = (char **)a;
  myb = (char **)b;

  if( *mya == NULL && *myb == NULL )
    return 0;

  if( *mya == NULL )
    return 1;

  if( *myb == NULL )
    return -1;

  if( ((*mya)[0] != (*myb)[0]) &&
      (((*mya)[0] < (FRIENDS_MAX_CREDIT - FRIENDS_SORT_HYSTERESIS)) ||
       ((*myb)[0] < (FRIENDS_MAX_CREDIT - FRIENDS_SORT_HYSTERESIS))) ) {
    return (*mya)[0] > (*myb)[0] ? -1 : 1;
  } else {
    // sort alphabetically from here
    return strncmp(&((*mya)[1]), &((*myb)[1]), GENE_NAMELENGTH + 1);
  }
}

void friendsSort(void) {
  osalMutexLock(&friend_mutex);
  qsort(friends, MAX_FRIENDS, sizeof(char *), friend_comp);
  osalMutexUnlock(&friend_mutex);
}

void friendsLock(void) {
  osalMutexLock(&friend_mutex);
}

void friendsUnlock(void) {
  osalMutexUnlock(&friend_mutex);
}

const char **friendsGet(void) {
  return (const char **) friends;   // you shouldn't modify this record outside of here, hence const
}

static void radio_ping_received(uint8_t prot, uint8_t src, uint8_t dst,
                                   uint8_t length, const void *data) {
  (void) prot;
  (void) src;
  (void) dst;
  (void) length;
  char *friend;

  friend = friend_lookup((char *) data);
  if( friend == NULL )
    friend = friend_add((char *) data);

  if( friend == NULL ) // we simply can't take any more friends, drop new requests until old ones expire
    return;

  if(friend[0] < FRIENDS_MAX_CREDIT)
    friend[0]++;

  chEvtBroadcast(&radio_app);
}

static void meiosis(genome *gamete, const genome *haploidM, const genome *haploidP) {
  uint32_t xover = rand();
  
  // create a gamete by picking chromosomes randomly from one parent or the other
  if( xover & 1 ) {
    gamete->cd_period = haploidM->cd_period;
    gamete->cd_rate = haploidM->cd_rate;
    gamete->cd_dir = haploidM->cd_dir;
  } else {
    gamete->cd_period = haploidP->cd_period;
    gamete->cd_rate = haploidP->cd_rate;
    gamete->cd_dir = haploidP->cd_dir;
  }
  xover >>= 1;
      
  if( xover & 1 )
    gamete->sat = haploidM->sat;
  else
    gamete->sat = haploidP->sat;
  xover >>= 1;
      
  if( xover & 1 ) {
    gamete->hue_ratedir = haploidM->hue_ratedir;
    gamete->hue_base = haploidM->hue_base;
    gamete->hue_bound = haploidM->hue_bound;
  }	else {
    gamete->hue_ratedir = haploidP->hue_ratedir;
    gamete->hue_base = haploidP->hue_base;
    gamete->hue_bound = haploidP->hue_bound;
  }
  xover >>= 1;

  if( xover & 1 )
    gamete->lin = haploidM->lin;
  else
    gamete->lin = haploidP->lin;
  xover >>= 1;
      
  if( xover & 1 )
    gamete->strobe = haploidM->strobe;
  else
    gamete->strobe = haploidP->strobe;
  xover >>= 1;
  
  if( xover & 1 )
    gamete->accel = haploidM->accel;
  else
    gamete->accel = haploidP->accel;
  xover >>= 1;
      
  if( xover & 1 )
    gamete->nonlin = haploidM->nonlin;
  else
    gamete->nonlin = haploidP->nonlin;
  xover >>= 1;
      
  if( xover & 1 )
    strncpy( gamete->name, haploidM->name, GENE_NAMELENGTH );
  else
    strncpy( gamete->name, haploidP->name, GENE_NAMELENGTH );
}

static uint8_t mfunc(uint8_t gene, uint8_t bits, uint32_t r) {
  return gray_decode(gray_encode(gene) ^ (bits << ((r >> 8) & 0x7)) );
}

static void mutate(genome *gamete, uint8_t mutation_rate) {
  uint32_t r;
  uint8_t bits;
  char genName[GENE_NAMELENGTH];

  // amplify mutation rate
  if( mutation_rate < 128 )
    bits = 1;
  else if( mutation_rate < 245 )
    bits = 3;
  else
    bits = 7;  // radioactive levels of mutation
  
  r = rand();
  if( (r & 0xFF) < mutation_rate ) {
    gamete->cd_period = mfunc(gamete->cd_period, bits, r) % 6;
  }
  
  r = rand();
  if( (r & 0xFF) < mutation_rate ) {
    gamete->cd_rate = mfunc(gamete->cd_rate, bits, r);
  }

  r = rand();
  if( (r & 0xFF) < mutation_rate ) {
    gamete->cd_dir = mfunc(gamete->cd_dir, bits, r);
  }

  r = rand();
  if( (r & 0xFF) < mutation_rate ) {
    gamete->sat = mfunc(gamete->sat, bits, r);
  }

  r = rand();
  if( (r & 0xFF) < mutation_rate ) {
    gamete->hue_ratedir = mfunc(gamete->hue_ratedir, bits, r);
  }

  r = rand();
  if( (r & 0xFF) < mutation_rate ) {
    gamete->hue_base = mfunc(gamete->hue_base, bits, r);
  }

  r = rand();
  if( (r & 0xFF) < mutation_rate ) {
    gamete->hue_bound = mfunc(gamete->hue_bound, bits, r);
  }

  r = rand();
  if( (r & 0xFF) < mutation_rate ) {
    gamete->lin = mfunc(gamete->lin, bits, r);
  }

  r = rand();
  if( (r & 0xFF) < mutation_rate ) {
    gamete->strobe = mfunc(gamete->strobe, bits, r);
  }

  r = rand();
  if( (r & 0xFF) < mutation_rate ) {
    gamete->accel = mfunc(gamete->accel, bits, r);
  }

  r = rand();
  if( (r & 0xFF) < mutation_rate ) {
    gamete->nonlin = mfunc(gamete->nonlin, bits, r);
  }

  r = rand();
  if( (r & 0xFF) < mutation_rate ) {
    generateName(genName);
    strncpy(gamete->name, genName, GENE_NAMELENGTH);
  }
}

static void handle_radio_sex_ack(uint8_t prot, uint8_t src, uint8_t dst,
                                   uint8_t length, const void *data) {
  (void) prot;
  (void) src;
  (void) dst;
  (void) length;
  
  genome *sperm;
  genome egg;
  uint8_t mutation_rate;
  struct genes *newfam;
  const struct genes *oldfam;
  int i;
  uint8_t curfam = 0;
  char *target;
  
  if( !sex_running )  // check to avoid people from sending acks without reqs
    return;
  
  oldfam = (const struct genes *) storageGetData(GENE_BLOCK);
  target = (char *)(data + sizeof(genome));
  if(strncmp(target, oldfam->name, GENE_NAMELENGTH) != 0)
    return; // I was expecting sex, someone acked, but actually I was hearing my neighbors doing it

  configIncSexResponses(); // record # times we've had sex
  newfam =  (struct genes *) chHeapAlloc(NULL, sizeof(struct genes));
  osalDbgAssert( newfam != NULL, "couldn't allocate space for the new family\n\r" );
  
  sperm = (genome *)data;
  mutation_rate = getMutationRate();
  
  newfam->signature = GENE_SIGNATURE;
  newfam->version = GENE_VERSION;
  strncpy(newfam->name, oldfam->name, GENE_NAMELENGTH);
#ifdef WHOLE_FAMILY
  (void) curfam;
  for( i = 0; i < GENE_FAMILYSIZE; i++ ) {
    meiosis(&egg, &(oldfam->haploidM[i]), &(oldfam->haploidP[i]));

    mutate(&egg, mutation_rate);
    mutate(sperm, mutation_rate);
    memcpy(&(newfam->haploidM[i]), &egg, sizeof(genome));
    memcpy(&(newfam->haploidP[i]), sperm, sizeof(genome));
  }
#else
  // ASSUME: current effect is in fact an Lg-series effect...
  curfam = effectsCurName()[2] - '0';
  meiosis(&egg, &(oldfam->haploidM[curfam]), &(oldfam->haploidP[curfam]));
  for( i = 0; i < GENE_FAMILYSIZE; i++ ) {
    if( i != curfam ) {
      // preserve other family members
      memcpy(&(newfam->haploidM[i]), &(oldfam->haploidM[i]), sizeof(genome));
      memcpy(&(newfam->haploidP[i]), &(oldfam->haploidP[i]), sizeof(genome));
    } else {
      // just make one new baby
      mutate(&egg, mutation_rate);
      mutate(sperm, mutation_rate);
      memcpy(&(newfam->haploidM[i]), &egg, sizeof(genome));
      memcpy(&(newfam->haploidP[i]), sperm, sizeof(genome));
    }
  }
#endif
  
  storagePatchData(GENE_BLOCK, (uint32_t *) newfam, GENE_OFFSET, sizeof(struct genes));
  chHeapFree(newfam);
  sex_done = 1;
}

#if SEXTEST
void handle_radio_sex_req(uint8_t prot, uint8_t src, uint8_t dst,
                                   uint8_t length, const void *data) {
#else
static void handle_radio_sex_req(uint8_t prot, uint8_t src, uint8_t dst,
                                   uint8_t length, const void *data) {
#endif
  (void) prot;
  (void) src;
  (void) dst;
  (void) length;
  const struct genes *family;
  uint8_t family_member = 0;
  genome  gamete;
  const userconfig *config;
  uint8_t consent = 0;
  char *who;
  char  response[sizeof(genome) + GENE_NAMELENGTH + 1];

  family = (const struct genes *) storageGetData(GENE_BLOCK);

  if( strncmp((char *)data, family->name, GENE_NAMELENGTH) == 0 ) {
    who = &(((char *)data)[strlen(family->name)+1]);
    config = getConfig();
    if( config->cfg_autosex == 0 ) {
      // UI prompt and escape with return if denied
      ui_override = 1;
      consent = getConsent(who);
      chThdSleepMilliseconds(300); // clear event queues
      ui_override = 0;
      if( !consent )
	return;
    } else {
      chThdSleepMilliseconds(SEX_TURNAROUND_TIME); // wait a little bit before responding to sex query
    }
    configIncSexResponses(); // record # times we've had sex
    // sex with me!
    if( strncmp(effectsCurName(), "Lg", 2) == 0 ) {
      // and it's a generated light pattern!
      
      family_member = effectsCurName()[2] - '0';
	
      // silly biologists, they should have called it create_gamete
      meiosis(&gamete, &(family->haploidM[family_member]),
	      &(family->haploidP[family_member]));

#if SEXTEST
      handle_radio_sex_ack(radio_prot_sex_ack, 255, 255, sizeof(genome), &gamete);
#else
      memcpy(response, &gamete, sizeof(genome));
      strncpy(&(response[sizeof(genome)]), who, GENE_NAMELENGTH);
      radioSend(radioDriver, RADIO_BROADCAST_ADDRESS, radio_prot_sex_ack,
		sizeof(response), response);
#endif
    }
  }
}


static void handle_charge_state(eventid_t id) {
  (void)id;
  usbStat usbStatus;
  // this was dispatched because a usbdet_rdy event was received
  struct accel_data accel; // for entropy call

  usbStatus = analogReadUsbStatus();
  if( lastUSBstatus != usbStatus ) {
    lastUSBstatus = usbStatus;
    switch(usbStatus) {
    case usbStatNC:
      // not connected. Let's turn on the boost subsystem.
      // chprintf(stream, "DEBUG: going boost\n\r" );
      chargerChargeIntent(0);
      chargerBoostIntent(1);
      break;
    case usbStat500:
      // chprintf(stream, "DEBUG: charging at 500mA\n\r" );
      chargerBoostIntent(0);
      chargerForce500();
      chargerSetTargetCurrent(500);
      chargerChargeIntent(1);
      break;
    case usbStat1500:
      // chprintf(stream, "DEBUG: charging at 1500mA\n\r" );
      chargerBoostIntent(0);
      chargerForce1500();
      chargerSetTargetCurrent(1500);
      chargerChargeIntent(1);
      break;
    default:
      ;
      // this is an internal error but what can we do about it? strings are expensive in 128k
    }
  }

  // whenever this system task runs, add some entropy to the random number pool...
  accelPoll(&accel);
  addEntropy(accel.x ^ accel.y ^ accel.z);

  // flush config data if it's changed
  configLazyFlush();
  
  // check if battery is too low, and shut down if it is
  if( ggVoltage() < 3250 ) {  // 3.3V (3300mV) is threshold for OLED failure; 50mV for margin
    chargerShipMode();  // requires plugging in to re-active battery
  }
}

static void handle_chargecheck_timeout(eventid_t id) {
  (void)id;

  // this kicks off an asynchronous ADC request that results in a usbdet_rdy event
  analogUpdateUsbStatus();
}

static int captouch_to_key(uint8_t code) {
  if (code == 11)
    return keyLeft;
  if (code == 0)
    return keyRight;
#if KEY_LAYOUT == LAYOUT_BM
  if (code == 5)
    return keySelect;
#elif KEY_LAYOUT == LAYOUT_BC1
  if (code == 2)
    return keySelect;
#endif
  return code;
}

static void run_launcher(void *arg) {

  (void)arg;

  chSysLockFromISR();
  /* Launcher is the first app in the list */
  instance.next_app = orchard_app_list;
  instance.thr->p_flags |= CH_FLAG_TERMINATE;
  chEvtBroadcastI(&orchard_app_terminate);
  run_launcher_timer_engaged = false;
  chSysUnlockFromISR();
}

static void poke_run_launcher_timer(eventid_t id) {

  (void)id;

  uint32_t val = captouchRead();
  if (run_launcher_timer_engaged) {
    /* Timer is engaged, but both buttons are still held.  Do nothing.*/
    if ((val & MAIN_MENU_MASK) == MAIN_MENU_VALUE)
      return;

    /* One or more buttons was released.  Cancel the timer.*/
    chVTReset(&run_launcher_timer);
    run_launcher_timer_engaged = false;
  }
  else {
    /* Timer not engaged, and the magic sequene isn't held.  Do nothing.*/
    if ((val & MAIN_MENU_MASK) != MAIN_MENU_VALUE)
      return;

    /* Start the sequence to go to the main menu when we're done.*/
    run_launcher_timer_engaged = true;
    chVTSet(&run_launcher_timer, RUN_LAUNCHER_TIMEOUT, run_launcher, NULL);
  }
}

static int8_t jog_raw_to_position(uint32_t raw) {
#if KEY_LAYOUT == LAYOUT_BM
  // hex codes from top, going clockwise
  // 80 40 10 08 04 02 400 200 100 
  switch(raw) {
  case 0x80:
    return 0;
  case (0x80 | 0x40):
    return 1;
  case 0x40:
    return 2;
  case (0x40 | 0x10):
    return 3;
  case 0x10:
    return 4;
  case (0x10 | 0x08):
    return 5;
  case 0x08:
    return 6;
  case (0x08 | 0x04):
    return 7;
  case 0x04:
    return 8;
  case (0x04 | 0x02):
    return 9;
  case 0x02:
    return 10;
  case (0x02 | 0x400):
    return 11;
  case 0x400:
    return 12;
  case (0x400 | 0x200):
    return 13;
  case 0x200:
    return 14;
  case (0x200 | 0x100):
    return 15;
  case 0x100:
    return 16;
  case (0x100 | 0x80):
    return 17;
    
  default:
    return -1; // error case
  }
#elif KEY_LAYOUT == LAYOUT_BC1
  // hex codes from top, going clockwise
  // 
  switch(raw) {
  case 0x200:
    return 0;
  case (0x200 | 0x2):
    return 1;
  case 0x2:
    return 2;
  case (0x2 | 0x8):
    return 3;
  case 0x8:
    return 4;
  case (0x08 | 0x10):
    return 5;
  case 0x10:
    return 6;
  case (0x10 | 0x20):
    return 7;
  case 0x20:
    return 8;
  case (0x20 | 0x40):
    return 9;
  case 0x40:
    return 10;
  case (0x40 | 0x80):
    return 11;
  case 0x80:
    return 12;
  case (0x80 | 0x100):
    return 13;
  case 0x100:
    return 14;
  case (0x100 | 0x400):
    return 15;
  case 0x400:
    return 16;
  case (0x400 | 0x200):
    return 17;
    
  default:
    return -1; // error case
  }
#endif
}

// current state + previous state => direction
// problem is if we jitter back and forth, direction intent is wrong

// so compute using:
// current state + previous state + direction intent + time => new direction intent
// direction intent can be none, CW, or CCW.

// time is used to retire direction state, if you stay in a quadrant for
// longer than a certain amount of time, direction intent should be reset to none

// returns 0 if there is no dial tracking event
static uint8_t track_dial(uint32_t raw) {
  uint32_t curtime = chVTGetSystemTime();
  int8_t curpos = jog_raw_to_position(raw);
  int8_t posdif;

  if( raw == 0 ) {
    // reset to untouched state
    jogdial_state.lastpos = -1;
    jogdial_state.lasttime = curtime;
    jogdial_state.direction_intent = dirNone;
    return 0;
  }
  
  if( curpos == -1 ) {
    // still touched, but can't make sense of it. clear direction intent,
    // set the time, but don't change from the last known position
    jogdial_state.lasttime = curtime;
    jogdial_state.direction_intent = dirNone;
    return 0;
  }

  if( jogdial_state.lastpos == -1 ) {
    // we're starting from untouched state
    jogdial_state.lastpos = curpos;
    jogdial_state.lasttime = curtime;
    jogdial_state.direction_intent = dirNone;
    return 0;
  }
  
  posdif = curpos - jogdial_state.lastpos;
  
  // check to see if we've made a significant state change from before
  if( abs(posdif) <= 1 ||
      abs(posdif) == 17 ) {
    // we haven't changed since before
    if (curtime - jogdial_state.lasttime > DWELL_THRESH)
      jogdial_state.direction_intent = dirNone;

    // don't update lastpos either, so we can't "drift" around the clock
    return 0; // no event update
    
  } else {
    // now check if we're going CW or CCW
    // (16 -> 17, 16 -> 0), (17 -> 0, 17 -> 1) are both CW
    if( ((posdif > 1) && (posdif < 14)) ||
	(posdif <= -14) ) {
      jogdial_state.direction_intent = dirCW;
      jogdial_state.lastpos = curpos;
      jogdial_state.lasttime = curtime;
      return 1;
    }

    if( (posdif < -1) ||
	(posdif >= 14) ) {
      jogdial_state.direction_intent = dirCCW;
      jogdial_state.lastpos = curpos;
      jogdial_state.lasttime = curtime;
      return 1;
    }
  }
  
  // all cases already handled with a return
  return 0;
}

static void ui_complete_cleanup(eventid_t id) {
  (void)id;
  OrchardAppEvent evt;
  
  // unhook the UI patch so key & dial events pass into the app
  instance.ui = NULL;

  evt.type = uiEvent;
  evt.ui.code = uiComplete;
  evt.ui.flags = uiOK;
  if( !ui_override )
    instance.app->event(instance.context, &evt);  
}

static void run_keycollect_timer(void *arg) {
  (void)arg;
  
  chSysLockFromISR();
  chEvtBroadcastI(&keycollect_timeout);
  chSysUnlockFromISR();
}

static void run_chargecheck(void *arg) {
  (void)arg;

  chSysLockFromISR();
  chEvtBroadcastI(&chargecheck_timeout);
  chVTSetI(&chargecheck_timer, MS2ST(CHARGECHECK_INTERVAL), run_chargecheck, NULL);
  chSysUnlockFromISR();
}

static void run_ping(void *arg) {
  (void)arg;

  chSysLockFromISR();
  chEvtBroadcastI(&ping_timeout);
  chVTSetI(&ping_timer, MS2ST(PING_MIN_INTERVAL + rand() % PING_RAND_INTERVAL), run_ping, NULL);
  chSysUnlockFromISR();
}

static void key_event_timer(eventid_t id) {
  (void)id;
  captouch_collected_state |= captouchRead(); // accumulate events

  // (re)set a timer to collect accumulated events...
  chVTReset(&keycollect_timer);
  chVTSet(&keycollect_timer, MS2ST(COLLECT_INTERVAL), run_keycollect_timer, NULL);
}

static void adc_temp_event(eventid_t id) {
  (void) id;
  OrchardAppEvent evt;

  evt.type = adcEvent;
  evt.adc.code = adcCodeTemp;
  if( !ui_override )
    instance.app->event(instance.context, &evt);
}

static void adc_mic_event(eventid_t id) {
  (void) id;
  OrchardAppEvent evt;

  evt.type = adcEvent;
  evt.adc.code = adcCodeMic;
  if( !ui_override )
    instance.app->event(instance.context, &evt);
}

static void accel_bump_event(eventid_t id) {
  (void) id;
  OrchardAppEvent evt;
  
  evt.type = accelEvent;
  evt.accel.code = accelCodeBump;
  if( !ui_override )
    instance.app->event(instance.context, &evt);
}

static void adc_usb_event(eventid_t id) {
  (void) id;
  OrchardAppEvent evt;

  evt.type = adcEvent;
  evt.adc.code = adcCodeUsbdet;
  if( !ui_override )
    instance.app->event(instance.context, &evt);
}

static void radio_app_event(eventid_t id) {
  (void)id;
  OrchardAppEvent evt;

  evt.type = radioEvent;
  if( !ui_override )
    instance.app->event(instance.context, &evt);
}

static void key_event(eventid_t id) {
  (void)id;
  uint32_t val = captouch_collected_state;
  uint32_t i;
  OrchardAppEvent evt;

  if( ui_override )
    return;
  
  if (!instance.app->event) {
    captouch_collected_state = 0;
    return;
  }

  /* No key changed */
  if (instance.keymask == val) {
    captouch_collected_state = 0;
    return;
  }

  // don't send main menu requests on to the app
  if( (val & MAIN_MENU_MASK) == MAIN_MENU_VALUE ) { 
    captouch_collected_state = 0;
    return;
  }
    
  for (i = 0; i < 16; i++) {
    uint8_t code = captouch_to_key(i);

    /* Code is a wheel event */
    if (code < 0x80)
      continue;

    if ((val & (1 << i)) && !(instance.keymask & (1 << i))) {
      evt.type = keyEvent;
      evt.key.code = code;
      evt.key.flags = keyDown;
      if( instance.ui == NULL )
	instance.app->event(instance.context, &evt);
      else
	instance.ui->event(instance.context, &evt);
    }
    if (!(val & (1 << i)) && (instance.keymask & (1 << i))) {
      evt.type = keyEvent;
      evt.key.code = code;
      evt.key.flags = keyUp;
      if( instance.ui == NULL )
	instance.app->event(instance.context, &evt);
      else
	instance.ui->event(instance.context, &evt);
    }
  }
  
  instance.keymask = val;

  // reset the collective state to 0
  captouch_collected_state = 0;
}

// handle jogdial events (in parallel to key events)
static void dial_event(eventid_t id) {
  (void)id;
  uint32_t val = captouchRead();
  unsigned int curtime;
  OrchardAppEvent evt;

  if( ui_override )
    return;
  
  if (!instance.app->event)
    return;

  /* No key changed */
  if (instance.keymask == val)
    return;

  curtime = chVTGetSystemTime();
  // track dial above the debouncer for smooth scrolling
  if( track_dial(val) ) {
    // debounce this slightly, to avoid adding lag to the track dial responsivitiy
    if( !((curtime - track_time) < TRACK_INTERVAL) ) {
      track_time = curtime;
    
      evt.type = keyEvent;
      evt.key.flags = keyDown;
      if( jogdial_state.direction_intent == dirCW )
	evt.key.code = keyCW;
      else
	evt.key.code = keyCCW;

      if( instance.ui == NULL )
	instance.app->event(instance.context, &evt);
      else
	instance.ui->event(instance.context, &evt);
    }
  }
  
}

static void terminate(eventid_t id) {

  (void)id;
  OrchardAppEvent evt;

  if (!instance.app->event)
    return;

  evt.type = appEvent;
  evt.app.event = appTerminate;
  instance.app->event(instance.context, &evt);
  chThdTerminate(instance.thr);
}

static void timer_event(eventid_t id) {

  (void)id;
  OrchardAppEvent evt;

  if (!instance.app->event)
    return;

  evt.type = timerEvent;
  evt.timer.usecs = instance.timer_usecs;
  if( !ui_override )
    instance.app->event(instance.context, &evt);

  if (instance.timer_repeating)
    orchardAppTimer(instance.context, instance.timer_usecs, true);
}

static void timer_do_send_message(void *arg) {

  (void)arg;
  chSysLockFromISR();
  chEvtBroadcastI(&timer_expired);
  chSysUnlockFromISR();
}

const OrchardApp *orchardAppByName(const char *name) {
  const OrchardApp *current;

  current = orchard_app_start();
  while(current->name) {
    if( !strncmp(name, current->name, 16) ) {
      return current;
    }
    current++;
  }
  return NULL;
}

void orchardAppRun(const OrchardApp *app) {
  instance.next_app = app;
  chThdTerminate(instance.thr);
  chEvtBroadcast(&orchard_app_terminate);
}

void orchardAppExit(void) {
  instance.next_app = orchard_app_start();  // the first app is the launcher
  chThdTerminate(instance.thr);
  chEvtBroadcast(&orchard_app_terminate);
}

void orchardAppTimer(const OrchardAppContext *context,
                     uint32_t usecs,
                     bool repeating) {

  if (!usecs) {
    chVTReset(&context->instance->timer);
    context->instance->timer_usecs = 0;
    return;
  }

  context->instance->timer_usecs = usecs;
  context->instance->timer_repeating = repeating;
  chVTSet(&context->instance->timer, US2ST(usecs), timer_do_send_message, NULL);
}

static THD_WORKING_AREA(waOrchardAppThread, 0x900); // more stack
static THD_FUNCTION(orchard_app_thread, arg) {

  (void)arg;
  struct orchard_app_instance *instance = arg;
  struct evt_table orchard_app_events;
  OrchardAppContext app_context;

  ui_override = 0;
  memset(&app_context, 0, sizeof(app_context));
  instance->context = &app_context;
  app_context.instance = instance;
  
  // set UI elements to null
  instance->ui = NULL;
  instance->uicontext = NULL;
  instance->ui_result = 0;

  chRegSetThreadName("Orchard App");

  instance->keymask = captouchRead();

  evtTableInit(orchard_app_events, 32);
  evtTableHook(orchard_app_events, radio_app, radio_app_event);
  evtTableHook(orchard_app_events, ui_completed, ui_complete_cleanup);
  evtTableHook(orchard_app_events, captouch_changed, key_event_timer);
  evtTableHook(orchard_app_events, captouch_changed, dial_event);
  evtTableHook(orchard_app_events, keycollect_timeout, key_event);
  evtTableHook(orchard_app_events, orchard_app_terminate, terminate);
  evtTableHook(orchard_app_events, timer_expired, timer_event);
  evtTableHook(orchard_app_events, celcius_rdy, adc_temp_event);
  evtTableHook(orchard_app_events, mic_rdy, adc_mic_event);
  evtTableHook(orchard_app_events, usbdet_rdy, adc_usb_event);
  evtTableHook(orchard_app_events, accel_bump, accel_bump_event);

  if (instance->app->init)
    app_context.priv_size = instance->app->init(&app_context);
  else
    app_context.priv_size = 0;

  /* Allocate private data on the stack (word-aligned) */
  uint32_t priv_data[app_context.priv_size / 4];
  if (app_context.priv_size) {
    memset(priv_data, 0, sizeof(priv_data));
    app_context.priv = priv_data;
  }
  else
    app_context.priv = NULL;

  if (instance->app->start)
    instance->app->start(&app_context);
  if (instance->app->event) {
    {
      OrchardAppEvent evt;
      evt.type = appEvent;
      evt.app.event = appStart;
      instance->app->event(instance->context, &evt);
    }
    while (!chThdShouldTerminateX())
      chEvtDispatch(evtHandlers(orchard_app_events), chEvtWaitOne(ALL_EVENTS));
  }

  chVTReset(&instance->timer);

  if (instance->app->exit)
    instance->app->exit(&app_context);

  instance->context = NULL;

  /* Set up the next app to run when the orchard_app_terminated message is
     acted upon.*/
  if (instance->next_app)
    instance->app = instance->next_app;
  else
    instance->app = orchard_app_list;
  instance->next_app = NULL;

  chVTReset(&run_launcher_timer);
  run_launcher_timer_engaged = false;

  evtTableUnhook(orchard_app_events, accel_bump, accel_bump_event);
  evtTableUnhook(orchard_app_events, usbdet_rdy, adc_usb_event);
  evtTableUnhook(orchard_app_events, mic_rdy, adc_mic_event);
  evtTableUnhook(orchard_app_events, celcius_rdy, adc_temp_event);
  evtTableUnhook(orchard_app_events, timer_expired, timer_event);
  evtTableUnhook(orchard_app_events, orchard_app_terminate, terminate);
  evtTableUnhook(orchard_app_events, keycollect_timeout, key_event);
  evtTableUnhook(orchard_app_events, captouch_changed, dial_event);
  evtTableUnhook(orchard_app_events, captouch_changed, key_event_timer);
  evtTableUnhook(orchard_app_events, ui_completed, ui_complete_cleanup);
  evtTableUnhook(orchard_app_events, radio_app, radio_app_event);

  /* Atomically broadcasting the event source and terminating the thread,
     there is not a chSysUnlock() because the thread terminates upon return.*/
  chSysLock();
  chEvtBroadcastI(&orchard_app_terminated);
  chThdExitS(MSG_OK);
}

void orchardAppInit(void) {
  int i;

  orchard_app_list = orchard_app_start();
  instance.app = orchard_app_list;
  chEvtObjectInit(&orchard_app_terminated);
  chEvtObjectInit(&orchard_app_terminate);
  chEvtObjectInit(&timer_expired);
  chEvtObjectInit(&keycollect_timeout);
  chEvtObjectInit(&chargecheck_timeout);
  chEvtObjectInit(&ping_timeout);
  chEvtObjectInit(&ui_completed);
  chVTReset(&instance.timer);

  /* Hook this outside of the app-specific runloop, so it runs even if
     the app isn't listening for events.*/
  evtTableHook(orchard_events, captouch_changed, poke_run_launcher_timer);
  
  // usb detection and charge state management is also meta to the apps
  // sequence of events:
  // 0. timer chargecheck_timer is set for CHARGECHECK_INTERVAL
  // 1. timer chargecheck_timer times out, and run_chargecheck callback is executed
  // 2. run_chargecheck callback issues a chargecheck_timeout event and re-schedules itself
  // 3. event sytsem receives chargecheck_timeout event and  dispatches handle_chargecheck_timeout
  // 4. handle_chargecheck_timeout issues an analogUpdateUsbStatus() call and exits
  // 5. analogUpdateUsbStatus() eventually results in a usbdet_rdy event
  // 6. usbdet_rdy event dispatches into the handle_charge_state event handler
  // 7. handle_charge_state runs all the logic for managing charge state

  // Steps 0-7 create a periodic timer that polls the USB D+/D- pin state so we can
  // make a determination of how to correctly set the charger/boost state
  // It's complicated because both the timer and the D+/D- detetion are asynchronous
  // and you have to use events to poke operations that can't happen in interrupt contexts!
  evtTableHook(orchard_events, usbdet_rdy, handle_charge_state);
  evtTableHook(orchard_events, chargecheck_timeout, handle_chargecheck_timeout);

  evtTableHook(orchard_events, radio_page, handle_radio_page);
  evtTableHook(orchard_events, ping_timeout, handle_ping_timeout);
  radioSetHandler(radioDriver, radio_prot_ping, radio_ping_received);
  radioSetHandler(radioDriver, radio_prot_sex_req, handle_radio_sex_req );
  radioSetHandler(radioDriver, radio_prot_sex_ack, handle_radio_sex_ack );

  chVTReset(&chargecheck_timer);
  chVTSet(&chargecheck_timer, MS2ST(CHARGECHECK_INTERVAL), run_chargecheck, NULL);

  chVTReset(&ping_timer);
  chVTSet(&ping_timer, MS2ST(PING_MIN_INTERVAL + rand() % PING_RAND_INTERVAL), run_ping, NULL);

  jogdial_state.lastpos = -1; 
  jogdial_state.direction_intent = dirNone;
  jogdial_state.lasttime = chVTGetSystemTime();

  for( i = 0; i < MAX_FRIENDS; i++ ) {
    friends[i] = NULL;
  }
  osalMutexObjectInit(&friend_mutex);
}

void orchardAppRestart(void) {

  /* Recovers memory of the previous application. */
  if (instance.thr) {
    osalDbgAssert(chThdTerminatedX(instance.thr), "App thread still running");
    chThdRelease(instance.thr);
    instance.thr = NULL;
  }

  instance.thr = chThdCreateStatic(waOrchardAppThread,
                                   sizeof(waOrchardAppThread),
                                   ORCHARD_APP_PRIO,
                                   orchard_app_thread,
                                   (void *)&instance);
}
