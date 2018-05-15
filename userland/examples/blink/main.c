#include <led.h>
#include <timer.h>

int main(void) {


  // Blink the LEDs in a binary count pattern and scale
  // to the number of LEDs on the board.
  while(1){

    led_on(0);
    // This delay uses an underlying timer in the kernel.
    delay_ms(500);
    led_off(0);
    delay_ms(500);
  }
}
