#include <avr/io.h>
#include <avr/interrupt.h>
#include "disk_defs.h"

extern "C" {
    void run_disk_controller_asm(void);
}

#if defined(ARDUINO)
void setup() {
    TIMSK0 = 0;   /* Disable Timer0 interrupts (no millis/micros jitter) */
    UCSR0B = 0;   /* Disable USART0 */
    cli();        /* Disable global interrupts for hard real-time 6502 bus timing */

    run_disk_controller_asm();
}

void loop() {
}
#else
int main(void) {
    TIMSK0 = 0;
    UCSR0B = 0;
    cli();

    run_disk_controller_asm();
    return 0;
}
#endif
