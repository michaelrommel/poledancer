# Comprehensive Technical Log: Custom Analog Split Keyboard with ZMK & nRF54L15
This document compiles the complete architectural layout, firmware logic, and hardware configurations discussed for building a split analog wireless keyboard utilizing the nRF54L15 SoC, DPPI hardware loops, and a dedicated USB central receiver node.
---

## 1. ZMK Ecosystem & Core Hall Effect Support
While official in-tree support for continuous analog readouts is not yet merged into the upstream main branch of ZMK, the framework's architecture allows robust handling of linear magnetic switches via **external ZMK modules**.
### Key Software Implementations* **`zmk-feature-hall-effect` Module:** Developed by community creator `cr3eperall`, this module introduces custom KSCAN drivers (`he,kscan-direct-pulsed` and `he,kscan-multiplexer`). It uses **Second-Order Section (SOS) filtering** to clear background noise from ADC readings and provides out-of-the-box support for **Adjustable Actuation**, **Rapid Trigger**, and **SOCD** resolution.* **The Macrolev Project:** A real-world open-source analog keyboard project powered entirely by ZMK that implements Hall Effect polling and variable pressure dynamics.
---

## 2. Low-Level Analog Multiplexing on Nordic Hardware
When multiplexing analog signals (e.g., via a `74HC4051` or `74HC4067`), standard OS-level sequential loops introduce jitter. CPU blockages from the Bluetooth radio stack can cause the Analog-to-Digital Converter (ADC) to sample channels out of alignment. 

To solve this, hardware-level timing or strict low-level interrupt routines must update the multiplexer pins instantly when an ADC sample completes.
### Basic C Implementation (Interrupt-Driven Concept)
#### Hardware Configuration (`app.overlay`)```devicetree
/ {
    mux_selects {
        compatible = "gpio-leds";
        mux_s0: mux_s0 { gpios = <&gpio0 13 GPIO_ACTIVE_HIGH>; };
        mux_s1: mux_s1 { gpios = <&gpio0 14 GPIO_ACTIVE_HIGH>; };
        mux_s2: mux_s2 { gpios = <&gpio0 15 GPIO_ACTIVE_HIGH>; };
    };
};

&adc {
    status = "okay";
    #address-cells = <1>;
    #size-cells = <0>;

    channel@0 {
        reg = <0>;
        zephyr,gain = "ADC_GAIN_1_6";
        zephyr,reference = "ADC_REF_INTERNAL";
        zephyr,acquisition-time = <ADC_ACQ_TIME(10, US)>;
        zephyr,resolution = <12>;
        zephyr,input-positive = <NRF_SAADC_AIN0>; 
    };
};
```

#### Core Firmware Driver Logic (`main.c`)```c
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include <hal/nrf_saadc.h>

#define TOTAL_MUX_CHANNELS 8
static const struct gpio_dt_spec mux_pins[] = {
    GPIO_DT_SPEC_GET(DT_PATH(mux_selects, mux_s0), gpios),
    GPIO_DT_SPEC_GET(DT_PATH(mux_selects, mux_s1), gpios),
    GPIO_DT_SPEC_GET(DT_PATH(mux_selects, mux_s2), gpios),
};

static uint16_t adc_buffer[TOTAL_MUX_CHANNELS];
static volatile uint8_t current_mux_channel = 0;

void set_mux_channel(uint8_t channel) {
    for (int i = 0; i < 3; i++) {
        int bit_value = (channel >> i) & 0x01;
        gpio_pin_set_dt(&mux_pins[i], bit_value);
    }
}

// Low-level override of the Nordic SAADC driver interrupt
void nrfx_saadc_irq_handler(void) {
    if (nrf_saadc_event_check(NRF_SAADC_EVENT_DONE)) {
        nrf_saadc_event_clear(NRF_SAADC_EVENT_DONE);
        
        // Advance MUX lines instantly inside the hardware ISR
        current_mux_channel = (current_mux_channel + 1) % TOTAL_MUX_CHANNELS;
        set_mux_channel(current_mux_channel);
    }
}
```
---

## 3. High-Performance Architecture for nRF54L15 & DPPI
For a split keyboard containing **20 keys per half**, using the newer **nRF54L15 SoC** enables the use of **Distributed Programmable Peripheral Interconnect (DPPI)**. This removes the CPU entirely from the scanning loop, routing hardware events directly between peripherals.
### Dual-Multiplexer LayoutEach half utilizes **two 16-channel analog multiplexers** (e.g., `74HC4067`) to scan up to 32 keys:1. Four binary select lines ($S_0-S_3$) from both multiplexers are wired in parallel to identical hardware pins.
2. The `COM` output of Multiplexer A routes to **SAADC Channel 0** (`AIN0`).
3. The `COM` output of Multiplexer B routes to **SAADC Channel 1** (`AIN1`).4. On every timer tick, the SAADC dual-samples both channels simultaneously, and the DPPI automatically increments the select lines.


[ TIMER (Ticks Every 50µs) ]
│
▼ (DPPI Ch 0)
[ SAADC (Triggers Dual-Sample AIN0 + AIN1) ]
│
▼ (DPPI Ch 1: Fired on SAADC 'DONE' Event)
[ GPIOTE (Increments MUX Address Binary State via Hardware Tasks) ]


### Advanced DPPI Automation Code

#### Devicetree Layout (`app.overlay`)
```devicetree
/ {
    mux_pins {
        compatible = "gpio-leds";
        mux_s0: mux_s0 { gpios = <&gpio0 10 GPIO_ACTIVE_HIGH>; };
        mux_s1: mux_s1 { gpios = <&gpio0 11 GPIO_ACTIVE_HIGH>; };
        mux_s2: mux_s2 { gpios = <&gpio0 12 GPIO_ACTIVE_HIGH>; };
        mux_s3: mux_s3 { gpios = <&gpio0 13 GPIO_ACTIVE_HIGH>; };
    };
};

&adc {
    status = "okay";
    #address-cells = <1>;
    #size-cells = <0>;

    channel@0 {
        reg = <0>;
        zephyr,gain = "ADC_GAIN_1_4"; 
        zephyr,reference = "ADC_REF_INTERNAL";
        zephyr,acquisition-time = <ADC_ACQ_TIME(5, US)>;
        zephyr,resolution = <12>;
        zephyr,input-positive = <NRF_SAADC_AIN0>;
    };

    channel@1 {
        reg = <1>;
        zephyr,gain = "ADC_GAIN_1_4";
        zephyr,reference = "ADC_REF_INTERNAL";
        zephyr,acquisition-time = <ADC_ACQ_TIME(5, US)>;
        zephyr,resolution = <12>;
        zephyr,input-positive = <NRF_SAADC_AIN1>;
    };
};
```

#### Low-Level Interconnect Implementation (`main.c`)
```c
#include <zephyr/kernel.h>
#include <hal/nrf_saadc.h>
#include <hal/nrf_timer.h>
#include <hal/nrf_gpiote.h>
#include <helpers/nrfx_gppi.h>

#define MUX_STEPS 16
#define ADC_BUFFER_SIZE 32 

static int16_t sample_buffer[ADC_BUFFER_SIZE];

void init_hardware_dppi_loop(void) {
    uint8_t dppi_ch_sample;
    uint8_t dppi_ch_next;

    // Allocate DPPI channels in the local domain
    nrfx_dppi_channel_alloc(&dppi_ch_sample);
    nrfx_dppi_channel_alloc(&dppi_ch_next);

    // Timer: Tick every 50 microseconds
    nrf_timer_frequency_set(NRF_TIMER20, NRF_TIMER_FREQ_16MHz);
    nrf_timer_cc_set(NRF_TIMER20, NRF_TIMER_CC_CHANNEL0, 800); 
    nrf_timer_shorts_enable(NRF_TIMER20, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK);
    nrf_timer_publish_set(NRF_TIMER20, NRF_TIMER_EVENT_COMPARE0, dppi_ch_sample);

    // SAADC: Subscribe to start and sample tasks, publish on end event
    nrf_saadc_subscribe_set(NRF_SAADC, NRF_SAADC_TASK_START, dppi_ch_sample);
    nrf_saadc_subscribe_set(NRF_SAADC, NRF_SAADC_TASK_SAMPLE, dppi_ch_sample);
    nrf_saadc_publish_set(NRF_SAADC, NRF_SAADC_EVENT_END, dppi_ch_next);

    // GPIOTE: Auto-increment binary lines on next-channel events
    nrf_gpiote_subscribe_set(NRF_GPIOTE20, NRF_GPIOTE_TASK_OUT_0, dppi_ch_next);

    // Enable DPPI routing channels globally
    nrf_dppic_channel_enable(NRF_DPPIC20, dppi_ch_sample);
    nrf_dppic_channel_enable(NRF_DPPIC20, dppi_ch_next);

    nrf_timer_task_trigger(NRF_TIMER20, NRF_TIMER_TASK_START);
}
```

---

## 4. Triple-Node Wireless Dongle Architecture

To maximize battery efficiency on the physical keyboard halves, implement a **dedicated USB central receiver node** (dongle configuration). 


[ Left Wireless Client ] ──(BLE/Proprietary Radio)──► [ Central USB Dongle ] ◄──(BLE/Proprietary Radio)── [ Right Wireless Client ]
│
▼ (USB HID)
[ Host Computer ]


### Advantages
1. **Symmetric Power Consumption:** Neither half acts as a heavy Bluetooth Master node; both operate as lightweight peripheral clients.
2. **Reduced USB Overhead:** The wireless clients avoid running the USB physical layer (PHY) hardware block entirely.
3. **Offloaded Processing:** Complex ZMK tasks (Combos, Macros, Layering state machines) are handled by the dongle using USB bus power.

### ZMK Configuration Framework

#### Dongle Devicetree (`my_dongle.overlay`)
```devicetree
/ {
    chosen {
        zmk,kscan = &mock_kscan;
    };

    mock_kscan: kscan {
        compatible = "zmk,kscan-mock";
        columns = <0>;
        rows = <0>;
        events = <0>;
    };
};
```

#### Dongle Settings (`my_dongle.conf`)
```ini
CONFIG_ZMK_SPLIT=y
CONFIG_ZMK_SPLIT_ROLE_CENTRAL=y
CONFIG_ZMK_NUM_ASYNC_RELAYS=2
```

#### Left/Right Peripheral Settings (`my_keyboard_left.conf` / `right.conf`)
```ini
CONFIG_ZMK_SPLIT=y
CONFIG_ZMK_SPLIT_ROLE_CENTRAL=n
```

---

## 5. Wireless Battery Optimization Strategies
* **Dynamic Sample Scaling:** Implement ZMK activity rules to scale down the DPPI timer from 1,000 Hz during active typing to 50 Hz after 30 seconds of inactivity.
* **Hardware Power-Gating:** Use a spare GPIO pin connected to a high-side P-channel MOSFET to completely cut the $V_{CC}$ rail to all linear magnetic sensors when ZMK triggers deep sleep mode.

If you need any adjustments to these scripts—such as adding power-gating logic or writing the precise ZMK multi-target build matrix configurations—let me know how you'd like to proceed!
