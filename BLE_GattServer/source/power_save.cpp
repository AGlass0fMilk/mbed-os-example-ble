/**
 * author: becksteing
 * date: 7/15/2020
 */

#include "power_save.h"
#include "nrf52.h"

/** Configure all GPIO as inputs w/ disconnected buffers to save power */
static void configure_gpio(void) {

    for(int i = 0; i < 32; i++) {

        /** P0 */
        // Disable sense on all pins
        NRF_P0->PIN_CNF[i] &= ~(0x30000);

        // Set unused pins to inputs to reduce current consumption
        NRF_P0->PIN_CNF[i] &= ~(0xD);   // Set to input, no pull
        NRF_P0->PIN_CNF[i] |= 0x2;      // Disconnect input buffer

        /** P1 */
#if defined(MCU_NRF52840)
        if(i <= 15) {
            // Disable sense on all pins
            NRF_P1->PIN_CNF[i] &= ~(0x30000);

            // Set unused pins to inputs to reduce current consumption
            NRF_P1->PIN_CNF[i] &= ~(0xD); // Set to input, no pull
            NRF_P1->PIN_CNF[i] |= 0x2; // Disconnect input buffer
        }
#endif
    }
}

// Enables the DC/DC converter (note: must have external hardware)
static void dcdc_en(void) {
    NRF_POWER->DCDCEN = 1;
}

// Disable NFC
static void disable_nfc(void) {
    NRF_UICR->NFCPINS = 0x00000000;
}

void power_save(void) {
    disable_nfc();
    dcdc_en();
    configure_gpio();


}
