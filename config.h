#include "hardware/flash.h"
#define OFFSET_DIV 8

/// These three parameter are connected
#define FREQUENCY_MHZ 200
#define OFFSET_MIN (FREQUENCY_MHZ * 9200 / 300)
#define OFFSET_MAX (FREQUENCY_MHZ * 10300 / 300)

#define VER_HI 200
#define VER_LO 74

bool is_configured();
void init_config();
bool is_inited();
void erase_config();
void add_boot_record(int offset);
int find_best_record(int * max_weight);
bool fast_check();
int get_weigth(int offset);

#define CONFIG_START 0x8000 // change to 0x100000 when compiling in "default mode"
#define CONFIG_IDX(offset) (((offset - OFFSET_MIN) / OFFSET_DIV) + 1)
#define CONFIG_CNT CONFIG_IDX(OFFSET_MAX)
#define OFFSET_CNT (CONFIG_CNT-1)
#define CONFIG_SIZE  (CONFIG_CNT * 256)

#define flash_cfg ((const uint8_t *) (XIP_BASE + CONFIG_START))
