#include <stdio.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "orchard.h"
#include "gfx.h"
#include "orchard-ui.h"
#include "captouch.h"

#include "orchard-test.h"
#include "test-audit.h"

static const TestRoutine *first_test;

orchard_test_end();

void orchardTestInit(void) {
  first_test = orchard_test_start();
  auditStart();  // start / initialize the test audit log
}

void orchardListTests(BaseSequentialStream *chp) {
  const TestRoutine *current;

  current = orchard_test_start();
  chprintf(chp, "Available tests:\n\r" );
  while (current->test_name) {
    chprintf(chp, "%s\r\n", current->test_name);
    current++;
  }
}

const TestRoutine *orchardGetTestByName(const char *name) {
  const TestRoutine *current;

  current = orchard_test_start();
  while (current->test_name) {
    if (!strncmp(name, current->test_name, 16))
      return current;
    current++;
  }
  return NULL;
}

OrchardTestResult orchardTestRun(const TestRoutine *test, uint32_t test_type) {

  return test->test_function(test->test_name, test_type);
}

OrchardTestResult orchardTestRunByName(const char *name, uint32_t test_type) {

  const TestRoutine *test = orchardGetTestByName(name);
  if (!test)
    return orchardResultNoTest;
  return orchardTestRun(test, test_type);
}

void orchardTestRunAll(BaseSequentialStream *chp, OrchardTestType test_type) {
  const TestRoutine *cur_test;
  OrchardTestResult test_result;
  char prompt[24];
  uint32_t auditval;

  cur_test = orchard_test_start();
  while(cur_test->test_function) {
    test_result = cur_test->test_function(cur_test->test_name, test_type);
    if( test_type != orchardTestPoweron ) {
      auditUpdate(cur_test->test_name, test_type, test_result);
    } else {
      switch(test_result) {
      case orchardResultPass:
        break;
      case orchardResultFail:
        chprintf(chp, "TEST: %s subystem failed test with code %d\n\r",
                 cur_test->test_name, test_result);
        break;
      case orchardResultNoTest:
        chprintf(chp, "TEST: reminder: write test for subystem %s\n\r",
                 cur_test->test_name );
        break;
      case orchardResultUnsure:
        chprintf(stream, "TEST: %s subystem not testable with test type %d\n\r",
                 cur_test->test_name, test_result, test_type);
        break;
      default:
        // lolwut?
        break;
      }
    }
    cur_test++;
  }
  if( test_type == orchardTestInteractive ) {
    auditval = auditCheck(test_type);
    if( auditval > 0xFFFF ) {
      // failure
      chsnprintf(prompt, sizeof(prompt), "0x%x", auditval);
      orchardTestPrompt( "FAIL", prompt, -120 );
    } else {
      // pass
      chsnprintf(prompt, sizeof(prompt), "0x%x", auditval);
      orchardTestPrompt( "TEST DONE", prompt, -120 );
    }
  }
}

// used to draw UI prompts for tests to the screen
// print up to 2 lines of text
// interaction_delay specifies how long we should wait before we declare failure
//   0 means don't delay
//   negative numbers delay that amount of seconds, but don't print a timeout failure
//    (meant for prompts that need to be read, but not interacted with)
OrchardTestResult orchardTestPrompt(char *line1, char *line2,
                                    int8_t interaction_delay) {
  coord_t width;
  coord_t height;
  font_t font;
  uint32_t val;
  char timer[16];
  uint32_t starttime;
  uint32_t curtime, updatetime;
  OrchardTestResult result = orchardResultUnsure;
  uint8_t countdown;

  val = captouchRead();
  countdown = (uint8_t) abs(interaction_delay);

  orchardGfxStart();
  font = gdispOpenFont("ui2");
  width = gdispGetWidth();
  height = gdispGetFontMetric(font, fontHeight);

  gdispClear(Black);

  gdispDrawStringBox(0, height * 2, width, height,
                     line1, font, White, justifyCenter);

  gdispDrawStringBox(0, height * 3, width, height,
                     line2, font, White, justifyCenter);

  if( interaction_delay != 0 ) {
    chsnprintf(timer, sizeof(timer), "%d", countdown);
    gdispDrawStringBox(0, height * 4, width, height,
                       timer, font, White, justifyCenter);
    countdown--;
  }

  gdispFlush();

  starttime = chVTGetSystemTime();
  updatetime = starttime + 1000;
  if (interaction_delay != 0) {
    while(1) {
      curtime = chVTGetSystemTime();
      if ((val != captouchRead())) {
        result = orchardResultPass;
        break;
      }
      if ((curtime - starttime) > ((uint32_t) abs(interaction_delay) * 1000)) {
        result = orchardResultFail;
        break;
      }

      if (curtime > updatetime) {
        chsnprintf(timer, sizeof(timer), "%d", countdown);
        gdispFillArea(0, height * 4, width, height, Black);

        gdispDrawStringBox(0, height * 4, width, height,
               timer, font, White, justifyCenter);
        gdispFlush();
        countdown--;
        updatetime += 1000;
      }
    }
  }

  if (result == orchardResultFail) {
    if( interaction_delay >= 0 ) {
      chsnprintf(timer, sizeof(timer), "timeout!");
      gdispFillArea(0, height * 4, width, height, Black);
      
      gdispDrawStringBox(0, height * 4, width, height,
                       timer, font, White, justifyCenter);

      gdispFlush();
      chThdSleepMilliseconds(2000);
    }
  }

  gdispCloseFont(font);
  orchardGfxEnd();

  return result;
}
