#include "ch.h"
#include "hal.h"
#include "orchard.h"

#include "storage.h"
#include "userconfig.h"

#include <string.h>

static userconfig config_cache;

// reading is easy, just return an immutable structure
const userconfig *getConfig(void) {
  return (const userconfig *) &config_cache;
}

// writing is harder, wrap in helper functions
// why? because we want the flexibility to not use
// a RAM-only cache if the config structure starts to
// grow pretty big. For now we can do ramcache because
// it's small, but later on we may have to refactor this
// implementation, so make the API future-proof.
void configIncSexInitiations(void) {
  config_cache.sex_initiations++;
}

void configIncSexResponses(void) {
  config_cache.sex_responses++;
}

void configSetAutosex(void) {
  config_cache.cfg_autosex = 1;
}

void configSetClearAutosex(void) {
  config_cache.cfg_autosex = 0;
}

void configToggleAutosex(void) {
  config_cache.cfg_autosex = !config_cache.cfg_autosex;
}

void configGgPatched(void) {
  config_cache.gg_hotfix = 1;
}

void configFlush(void) {
  storagePatchData(CONFIG_BLOCK, (uint32_t *) &config_cache, CONFIG_OFFSET, sizeof(struct userconfig));
}

void configLazyFlush(void) {
  const struct userconfig *config;

  config = (const struct userconfig *) storageGetData(CONFIG_BLOCK);
  if( memcmp( config, &config_cache, sizeof(struct userconfig) ) != 0) {
    storagePatchData(CONFIG_BLOCK, (uint32_t *) &config_cache, CONFIG_OFFSET, sizeof(struct userconfig));
  }
}

static void init_config(uint32_t block) {
  struct userconfig config;

  config.signature = CONFIG_SIGNATURE;
  config.version = CONFIG_VERSION;

  config.sex_initiations = 0;
  config.sex_responses = 0;
  config.cfg_autosex = 0;   // deny rapid breeding by default
  config.gg_hotfix = 0xFFFFFFFF;

  storagePatchData(block, (uint32_t *) &config, CONFIG_OFFSET, sizeof(struct userconfig));
}

void configStart(void) {
  const struct userconfig *config;

  config = (const struct userconfig *) storageGetData(CONFIG_BLOCK);

  if( config->signature != CONFIG_SIGNATURE ) {
    init_config(CONFIG_BLOCK);
    config = (const struct userconfig *) storageGetData(CONFIG_BLOCK);
  } else if( config->version != CONFIG_VERSION ) {
    init_config(CONFIG_BLOCK);
    config = (const struct userconfig *) storageGetData(CONFIG_BLOCK);
  }

  memcpy( &config_cache, config, sizeof(userconfig) ); // copy configuration to volatile cache
}
