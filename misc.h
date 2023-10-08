#define PIX_blu 0x00003F
#define PIX_yel 0x151500
#define PIX_whi 0x111111

#define PIX_b 0x00000F

enum vreg_en {
    VREG_EN_ENABLED = 0b1,
    VREG_EN_DISABLED = 0b0,

    VREG_EN_DEFAULT = VREG_EN_ENABLED,
};

enum vreg_hiz {
    VREG_HIZ_ENABLED = 0b1,
    VREG_HIZ_DISABLED = 0b0,

    VREG_HIZ_DEFAULT = VREG_HIZ_ENABLED,
};

enum bod_en {
    BOD_EN_ENABLED = 0b1,
    BOD_EN_DISABLED = 0b0,

    BOD_EN_DEFAULT = BOD_EN_ENABLED,
};

enum syscfg_mempowerdown_state {
    SYSCFG_MEMPOWERDOWN_STATE_TRUE = 0b1,
    SYSCFG_MEMPOWERDOWN_STATE_FALSE = 0b0,

    SYSCFG_MEMPOWERDOWN_STATE_DEFAULT = SYSCFG_MEMPOWERDOWN_STATE_FALSE,
};

enum syscfg_mempowerdown {
    SYSCFG_MEMPOWERDOWN_SRAM0,
    SYSCFG_MEMPOWERDOWN_SRAM1,
    SYSCFG_MEMPOWERDOWN_SRAM2,
    SYSCFG_MEMPOWERDOWN_SRAM3,
    SYSCFG_MEMPOWERDOWN_SRAM4,
    SYSCFG_MEMPOWERDOWN_SRAM5,
    SYSCFG_MEMPOWERDOWN_USB,
    SYSCFG_MEMPOWERDOWN_ROM,
};

void vreg_enable(enum vreg_en en);

void bod_enable(enum bod_en en);

void syscfg_mempowerdown(enum syscfg_mempowerdown name, enum syscfg_mempowerdown_state state);

void put_pixel(uint32_t pixel_grb);

void halt_with_error(uint32_t err, uint32_t bits);

void gpio_disable_input_output(int pin);

void gpio_enable_input_output(int pin);

void finish_pins_except_leds();

void reset_cpu();