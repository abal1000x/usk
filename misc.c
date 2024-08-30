#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "pins.h"
#include <string.h>
#include <hardware/xosc.h>
#include <hardware/rosc.h>
#include <hardware/pll.h>
#include "hardware/vreg.h"
#include "ws2812.pio.h"
#include "board_detect.h"
#include "misc.h"
#include "board_detect.h"
#include "hardware/structs/syscfg.h"

extern int ws_pio_offset;

#define BLINK_TIME 700
#define SHORT_TIME ( BLINK_TIME * 2 / 10 )
#define SHORT_PAUSE_TIME ((BLINK_TIME - SHORT_TIME) / 2)
#define LONG_TIME ( BLINK_TIME * 8 / 10 )
#define LONG_PAUSE_TIME ((BLINK_TIME - LONG_TIME) / 2)
#define PAUSE_BETWEEN 2000
#define PAUSE_BEFORE 750
#define CODE_REPEATS 3

#define GPIO_OD PADS_BANK0_GPIO0_OD_BITS
#define GPIO_IE PADS_BANK0_GPIO0_IE_BITS
#define GPIO_OD_PD (GPIO_OD | PADS_BANK0_GPIO0_PDE_BITS)

#define VREG_VOLTAGE_0_80 (VREG_VOLTAGE_0_85 - 1)

void vreg_enable(enum vreg_en en) {
    hw_write_masked(&vreg_and_chip_reset_hw->vreg, ((uint)en) << VREG_AND_CHIP_RESET_VREG_EN_LSB, VREG_AND_CHIP_RESET_VREG_EN_BITS);
}

void vreg_hiz(enum vreg_hiz hiz) {
    hw_write_masked(&vreg_and_chip_reset_hw->vreg, ((uint)hiz) << VREG_AND_CHIP_RESET_VREG_HIZ_LSB, VREG_AND_CHIP_RESET_VREG_HIZ_BITS);
}

void bod_enable(enum bod_en en) {
    hw_write_masked(&vreg_and_chip_reset_hw->bod, ((uint)en) << VREG_AND_CHIP_RESET_BOD_EN_LSB, VREG_AND_CHIP_RESET_BOD_EN_BITS);
}

void syscfg_mempowerdown(enum syscfg_mempowerdown name, enum syscfg_mempowerdown_state state) {
    uint32_t lsb, bits;
    switch (name) {
        case SYSCFG_MEMPOWERDOWN_SRAM0:
            lsb = SYSCFG_MEMPOWERDOWN_SRAM0_LSB;
            bits = SYSCFG_MEMPOWERDOWN_SRAM0_BITS;
            break;
        case SYSCFG_MEMPOWERDOWN_SRAM1:
            lsb = SYSCFG_MEMPOWERDOWN_SRAM1_LSB;
            bits = SYSCFG_MEMPOWERDOWN_SRAM1_BITS;
            break;
        case SYSCFG_MEMPOWERDOWN_SRAM2:
            lsb = SYSCFG_MEMPOWERDOWN_SRAM2_LSB;
            bits = SYSCFG_MEMPOWERDOWN_SRAM2_BITS;
            break;
        case SYSCFG_MEMPOWERDOWN_SRAM3:
            lsb = SYSCFG_MEMPOWERDOWN_SRAM3_LSB;
            bits = SYSCFG_MEMPOWERDOWN_SRAM3_BITS;
            break;
        case SYSCFG_MEMPOWERDOWN_SRAM4:
            lsb = SYSCFG_MEMPOWERDOWN_SRAM4_LSB;
            bits = SYSCFG_MEMPOWERDOWN_SRAM4_BITS;
            break;
        case SYSCFG_MEMPOWERDOWN_SRAM5:
            lsb = SYSCFG_MEMPOWERDOWN_SRAM5_LSB;
            bits = SYSCFG_MEMPOWERDOWN_SRAM5_BITS;
            break;
        case SYSCFG_MEMPOWERDOWN_USB:
            lsb = SYSCFG_MEMPOWERDOWN_USB_LSB;
            bits = SYSCFG_MEMPOWERDOWN_USB_BITS;
            break;
        case SYSCFG_MEMPOWERDOWN_ROM:
            lsb = SYSCFG_MEMPOWERDOWN_ROM_LSB;
            bits = SYSCFG_MEMPOWERDOWN_ROM_BITS;
            break;
    }
    hw_write_masked(&syscfg_hw->mempowerdown, ((uint)state) << lsb, bits);
}

void __no_inline_not_in_flash_func(zzz)() {
    uint src_hz = 6.5 * MHZ;
    clock_configure(clk_ref,
                    CLOCKS_CLK_REF_CTRL_SRC_VALUE_ROSC_CLKSRC_PH,
                    0, // No aux mux
                    src_hz,
                    src_hz);

    clock_configure(clk_sys,
                    CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLK_REF,
                    0,
                    src_hz,
                    src_hz);

    clock_stop(clk_usb);
    clock_stop(clk_adc);

    clock_configure(clk_rtc,
                    0, // No GLMUX
                    CLOCKS_CLK_RTC_CTRL_AUXSRC_VALUE_ROSC_CLKSRC_PH,
                    src_hz,
                    46875);

    clock_configure(clk_peri,
                    0,
                    CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLK_SYS,
                    src_hz,
                    src_hz);

    pll_deinit(pll_sys);
    pll_deinit(pll_usb);

    uint32_t address = (uint32_t) zzz;
    if (!(
        (SRAM_STRIPED_BASE <= address && address < SRAM_STRIPED_END) ||
        (SRAM0_BASE <= address && address < SRAM1_BASE)
    )) {
        syscfg_mempowerdown(SYSCFG_MEMPOWERDOWN_SRAM0, SYSCFG_MEMPOWERDOWN_STATE_TRUE);
    }
    if (!(
        (SRAM_STRIPED_BASE <= address && address < SRAM_STRIPED_END) ||
        (SRAM1_BASE <= address && address < SRAM2_BASE)
    )) {
        syscfg_mempowerdown(SYSCFG_MEMPOWERDOWN_SRAM1, SYSCFG_MEMPOWERDOWN_STATE_TRUE);
    }
    if (!(
        (SRAM_STRIPED_BASE <= address && address < SRAM_STRIPED_END) ||
        (SRAM2_BASE <= address && address < SRAM3_BASE)
    )) {
        syscfg_mempowerdown(SYSCFG_MEMPOWERDOWN_SRAM2, SYSCFG_MEMPOWERDOWN_STATE_TRUE);
    }
    if (!(
        (SRAM_STRIPED_BASE <= address && address < SRAM_STRIPED_END) ||
        (SRAM3_BASE <= address && address < SRAM4_BASE)
    )) {
        syscfg_mempowerdown(SYSCFG_MEMPOWERDOWN_SRAM3, SYSCFG_MEMPOWERDOWN_STATE_TRUE);
    }
    if (!(SRAM4_BASE <= address && address < SRAM5_BASE)) {
        syscfg_mempowerdown(SYSCFG_MEMPOWERDOWN_SRAM4, SYSCFG_MEMPOWERDOWN_STATE_TRUE);
    }
    if (!(SRAM5_BASE <= address && address < SRAM_END)) {
        syscfg_mempowerdown(SYSCFG_MEMPOWERDOWN_SRAM5, SYSCFG_MEMPOWERDOWN_STATE_TRUE);
    }
    syscfg_mempowerdown(SYSCFG_MEMPOWERDOWN_USB, SYSCFG_MEMPOWERDOWN_STATE_TRUE);
    syscfg_mempowerdown(SYSCFG_MEMPOWERDOWN_ROM, SYSCFG_MEMPOWERDOWN_STATE_TRUE);

    vreg_set_voltage(VREG_VOLTAGE_0_80);
    xosc_disable();
    bod_enable(BOD_EN_DISABLED);
    vreg_enable(VREG_EN_ENABLED);
//    vreg_hiz(VREG_HIZ_ENABLED);
    rosc_disable();
    while(1);
}

void finish_pins_except_leds() {
    for(int pin = 0; pin <= 29; pin += 1) {
        if (pin == led_pin() || pin == pwr_pin())
            continue;
        if (pin == PIN_GLI_PICO || pin == PIN_GLI_XIAO || pin == PIN_GLI_WS || pin == PIN_GLI_ITSY)
        {
            gpio_pull_down(pin);
        }
        else
        {
            gpio_disable_pulls(pin);
        }
        gpio_disable_input_output(pin);
    }
}

void finish_pins_leds() {
    if (!is_tiny())
    {
        gpio_disable_input_output(led_pin());
    }
    gpio_disable_input_output(pwr_pin());
}

void halt_with_error(uint32_t err, uint32_t bits)
{
    finish_pins_except_leds();
    pio_set_sm_mask_enabled(pio0, 0xF, false);
    pio_set_sm_mask_enabled(pio1, 0xF, false);
    set_sys_clock_khz(48000, true);
    vreg_set_voltage(VREG_VOLTAGE_0_95);
    if (bits != 1)
    {
        put_pixel(0);
        sleep_ms(PAUSE_BEFORE);
    }
    for(int j = 0; j < CODE_REPEATS; j++)
    {
        for(int i = 0; i < bits; i++)
        {
            bool is_long = err & (1 << (bits - i - 1));
            sleep_ms(is_long ? LONG_PAUSE_TIME : SHORT_PAUSE_TIME);
            bool success = bits == 1 && is_long == 0;
            if (success)
                put_pixel(PIX_whi);
            else
                put_pixel(PIX_yel);
            sleep_ms(is_long ? LONG_TIME : success ? SHORT_TIME * 2 : SHORT_TIME);
            put_pixel(0);
            if (i != bits - 1 || j != CODE_REPEATS - 1)
                sleep_ms(is_long ? LONG_PAUSE_TIME : SHORT_PAUSE_TIME);
            if (i == bits - 1 && j != CODE_REPEATS - 1)
                sleep_ms(PAUSE_BETWEEN);
        }
        // first write case, do not repeat this kind of error code
        if (bits == 1)
            break;
    }
    finish_pins_leds();
    zzz();
}

void put_pixel(uint32_t pixel_grb)
{
    static bool led_enabled = false;
    if (is_pico())
    {
        gpio_init(led_pin());
        if (pixel_grb) {
            gpio_set_dir(led_pin(), true);
            gpio_put(led_pin(), 1);
        }
        return;
    }
    ws2812_program_init(pio0, 3, ws_pio_offset, led_pin(), 800000, true);
    if (!led_enabled && pwr_pin() != 31)
    {
        led_enabled = true;
        gpio_init(pwr_pin());
        gpio_set_drive_strength(pwr_pin(), GPIO_DRIVE_STRENGTH_12MA);
        gpio_set_dir(pwr_pin(), true);
        gpio_put(pwr_pin(), 1);
        sleep_us(200);
    }
    pio_sm_put_blocking(pio0, 3, pixel_grb << 8u);
    sleep_us(50);
    pio_sm_set_enabled(pio0, 3, false);
    if (!is_tiny())
    {
        gpio_init(led_pin());
    }
}

void gpio_disable_input_output(int pin)
{
    uint32_t pad_reg = 0x4001c000 + 4 + pin*4;
    *(uint32_t*)(pad_reg + 0x2000) = GPIO_OD;
    *(uint32_t*)(pad_reg + 0x3000) = GPIO_IE;
}

void gpio_enable_input_output(int pin)
{
    uint32_t pad_reg = 0x4001c000 + 4 + pin*4;
    *(uint32_t*)(pad_reg + 0x3000) = GPIO_OD;
    *(uint32_t*)(pad_reg + 0x2000) = GPIO_IE;
}

void reset_cpu() {
    gpio_enable_input_output(PIN_RST);
    gpio_pull_up(PIN_RST);
    sleep_us(1000);
    gpio_init(PIN_RST);
    gpio_set_dir(PIN_RST, true);
    sleep_us(2000);
    gpio_deinit(PIN_RST);
    gpio_disable_pulls(PIN_RST);
    gpio_disable_input_output(PIN_RST);
}
