#define CONFIG_SIGNATURE  0x55434647  // UCFG
#define CONFIG_BLOCK      1
#define CONFIG_OFFSET     0
#define CONFIG_VERSION    0

typedef struct userconfig {
  uint32_t  signature;
  uint32_t  version;
  uint32_t  sex_initiations; // number of times we've initiated sex
  uint32_t  sex_responses;   // numbef or times others have initiated sex
  uint32_t  cfg_autosex;     // set if sex automatically allowed
} userconfig;

void configStart(void);

const userconfig *getConfig(void);
void configIncSexInitiations(void);
void configIncSexResponses(void);
void configSetAutosex(void);
void configClearAutoSex(void);
void configToggleAutosex(void);
void configFlush(void); // call on power-down to flush config state

void configLazyFlush(void);  // call periodically to sync state, but only when dirty
