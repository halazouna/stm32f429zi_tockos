//! 
//! 
//! Tock Kernel for STM32F429ZI Discovery.
//! 
//! 
//! 
//! Pin Configuration
//! 
//! LED: LED1 = PG_13, LED2 = PG.14
//! 
//! 

#![no_std]
#![no_main]
#![feature(asm, const_fn, lang_items, compiler_builtins_lib, const_cell_new)]
#![deny(missing_docs)]

extern crate capsules;
extern crate compiler_builtins;
#[allow(unused_imports)]
#[macro_use(debug, debug_gpio, static_init)]
extern crate kernel;
extern crate stm32f429;

//use capsules::virtual_alarm::VirtualMuxAlarm;
use stm32f429::rcc;


/// Support routines for debugging I/O.
///
/// Note: Use of this module will trample any other USART3 configuration.
#[macro_use]
pub mod io;

// State for loading and holding applications.
// How should the kernel respond when a process faults.
const FAULT_RESPONSE: kernel::process::FaultResponse = kernel::process::FaultResponse::Panic;

const NUM_PROCS: usize = 8;

#[link_section = ".app_memory"]
static mut APP_MEMORY: [u8; 131072] = [0; 131072];

static mut PROCESSES: [Option<kernel::Process<'static>>; NUM_PROCS] = [None, None, None, None, None, None, None, None];


/// Supported drivers by the platform
pub struct Platform {
    gpio: &'static capsules::gpio::GPIO<'static, stm32f429::gpio::GPIOPin>,
    led: &'static capsules::led::LED<'static, stm32f429::gpio::GPIOPin>,
    ipc: kernel::ipc::IPC,
}


impl kernel::Platform for Platform {
    fn with_driver<F, R>(&self, driver_num: usize, f: F) -> R
    where
        F: FnOnce(Option<&kernel::Driver>) -> R,
    {
        match driver_num {
            capsules::gpio::DRIVER_NUM => f(Some(self.gpio)),
            capsules::led::DRIVER_NUM => f(Some(self.led)),
            kernel::ipc::DRIVER_NUM => f(Some(&self.ipc)),
            _ => f(None),
        }
    }
}


/// Entry point in the vector table called on hard reset.
#[no_mangle]
pub unsafe fn reset_handler() {
	stm32f429::init();

	rcc::set_clock(rcc::CrystalClock::Clock16MHz, rcc::Clock::Clock168MHz);

	let gpio_pins = static_init!(
        [&'static stm32f429::gpio::GPIOPin; 2],
        [
            &stm32f429::gpio::PA[3], // Bottom right header on DK board
            &stm32f429::gpio::PA[5], // Bottom right header on DK board
        ]
    );

    let gpio = static_init!(
        capsules::gpio::GPIO<'static, stm32f429::gpio::GPIOPin>,
        capsules::gpio::GPIO::new(gpio_pins)
    );
    for pin in gpio_pins.iter() {
        pin.set_client(gpio);
    } 

    // # LEDs
    let led_pins = static_init!(
        [(&'static stm32f429::gpio::GPIOPin, capsules::led::ActivationMode); 2],
        [
            (
                &stm32f429::gpio::PG[13],
                capsules::led::ActivationMode::ActiveHigh
            ),
            (
                &stm32f429::gpio::PG[14],
                capsules::led::ActivationMode::ActiveHigh
            )
        ]
    );
    let led = static_init!(
        capsules::led::LED<'static, stm32f429::gpio::GPIOPin>,
        capsules::led::LED::new(led_pins)
    );

    let platform = Platform {
        led: led,
        gpio: gpio,
        ipc: kernel::ipc::IPC::new(),
    };

    let mut chip = stm32f429::chip::STM32F429ZI::new();

    

    //debug!("Initialization complete. Entering main loop\r");
    extern "C" {
        /// Beginning of the ROM region containing app images.
        static _sapps: u8;
    }
    kernel::process::load_processes(
        &_sapps as *const u8,
        &mut APP_MEMORY,
        &mut PROCESSES,
        FAULT_RESPONSE,
    );

    kernel::main(&platform, &mut chip, &mut PROCESSES, &platform.ipc);

}
