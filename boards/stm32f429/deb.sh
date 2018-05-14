arm-none-eabi-gdb -x .gdbinit target/thumbv7em-none-eabi/release/stm32f429zi 0x8000000 add-symbol-file ../../userland/examples/blink/build/cortex-m4/cortex-m4.elf 0x8080000
