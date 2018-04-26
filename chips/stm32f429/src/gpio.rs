use self::Pin::*;
use core::cell::Cell;
use vol_cell::VolatileCell;
use kernel::hil;
use core::ops::{Index, IndexMut};

pub const GPIO_BASE: usize = 0x40020000;
const SIZE: usize = 0x400;
pub const GPIOA_BASE: usize = GPIO_BASE + 0 * SIZE;
pub const GPIOB_BASE: usize = GPIO_BASE + 1 * SIZE;
pub const GPIOC_BASE: usize = GPIO_BASE + 2 * SIZE;
pub const GPIOD_BASE: usize = GPIO_BASE + 3 * SIZE;
pub const GPIOE_BASE: usize = GPIO_BASE + 4 * SIZE;
pub const GPIOF_BASE: usize = GPIO_BASE + 5 * SIZE;
pub const GPIOG_BASE: usize = GPIO_BASE + 6 * SIZE;
pub const GPIOH_BASE: usize = GPIO_BASE + 7 * SIZE;
pub const GPIOI_BASE: usize = GPIO_BASE + 8 * SIZE;
pub const GPIOJ_BASE: usize = GPIO_BASE + 9 * SIZE;
pub const GPIOK_BASE: usize = GPIO_BASE + 10 * SIZE;





#[derive(Debug, Copy, Clone)]
pub enum PinMode {
    Input  = 0x0,
    Output = 0x1,
    AF     = 0x2,
    Analog = 0x3
}


#[repr(C)]
struct GPIO_reg {
	MODER: VolatileCell<u32>,    		
	OTYPER: VolatileCell<u32>,   		
	OSPEEDR: VolatileCell<u32>,  		
	PUPDR: VolatileCell<u32>,    		
	IDR: VolatileCell<u32>,      		
	ODR: VolatileCell<u32>,      		
	BSR: VolatileCell<u16>,     		
    BRR: VolatileCell<u16>,            
	LCKR: VolatileCell<u32>,     		
	AFR: [VolatileCell<u32>; 2],   	
}

#[repr(C)]
struct EXTI_reg {
	IMR: VolatileCell<u32>,    
	EMR: VolatileCell<u32>,    
	RTSR: VolatileCell<u32>,   
	FTSR: VolatileCell<u32>,   
	SWIER: VolatileCell<u32>,  
	PR: VolatileCell<u32>,     
}

#[derive(Copy,Clone)]
#[cfg_attr(rustfmt, rustfmt_skip)]
pub enum Pin {
    PA00, PA01, PA02, PA03, PA04, PA05, PA06, PA07,
    PA08, PA09, PA10, PA11, PA12, PA13, PA14, PA15,


    PB00, PB01, PB02, PB03, PB04, PB05, PB06, PB07,
    PB08, PB09, PB10, PB11, PB12, PB13, PB14, PB15,


    PC00, PC01, PC02, PC03, PC04, PC05, PC06, PC07,
    PC08, PC09, PC10, PC11, PC12, PC13, PC14, PC15,

    PD00, PD01, PD02, PD03, PD04, PD05, PD06, PD07,
    PD08, PD09, PD10, PD11, PD12, PD13, PD14, PD15,


    PE00, PE01, PE02, PE03, PE04, PE05, PE06, PE07,
    PE08, PE09, PE10, PE11, PE12, PE13, PE14, PE15,


    PF00, PF01, PF02, PF03, PF04, PF05, PF06, PF07,
    PF08, PF09, PF10, PF11, PF12, PF13, PF14, PF15,

    PG00, PG01, PG02, PG03, PG04, PG05, PG06, PG07,
    PG08, PG09, PG10, PG11, PG12, PG13, PG14, PG15,
}

pub struct Port {
    port: *mut GPIO_reg,
    pins: [GPIOPin; 16],
}

impl Index<usize> for Port {
    type Output = GPIOPin;

    fn index(&self, index: usize) -> &GPIOPin {
        &self.pins[index]
    }
}

impl IndexMut<usize> for Port {
    fn index_mut(&mut self, index: usize) -> &mut GPIOPin {
        &mut self.pins[index]
    }
}


impl Port {
    pub fn handle_interrupt(&self) {
        let port: &GPIO_reg = unsafe {&*self.port};
    }
}


/// Port A
pub static mut PA: Port = Port {
    port: (GPIOA_BASE + 0 * SIZE) as *mut GPIO_reg,
    pins: [
        GPIOPin::new(PA00),
        GPIOPin::new(PA01),
        GPIOPin::new(PA02),
        GPIOPin::new(PA03),
        GPIOPin::new(PA04),
        GPIOPin::new(PA05),
        GPIOPin::new(PA06),
        GPIOPin::new(PA07),
        GPIOPin::new(PA08),
        GPIOPin::new(PA09),
        GPIOPin::new(PA10),
        GPIOPin::new(PA11),
        GPIOPin::new(PA12),
        GPIOPin::new(PA13),
        GPIOPin::new(PA14),
        GPIOPin::new(PA15),
    ],
};

/// Port B
pub static mut PB: Port = Port {
    port: (GPIOB_BASE + 0 * SIZE) as *mut GPIO_reg,
    pins: [
        GPIOPin::new(PB00),
        GPIOPin::new(PB01),
        GPIOPin::new(PB02),
        GPIOPin::new(PB03),
        GPIOPin::new(PB04),
        GPIOPin::new(PB05),
        GPIOPin::new(PB06),
        GPIOPin::new(PB07),
        GPIOPin::new(PB08),
        GPIOPin::new(PB09),
        GPIOPin::new(PB10),
        GPIOPin::new(PB11),
        GPIOPin::new(PB12),
        GPIOPin::new(PB13),
        GPIOPin::new(PB14),
        GPIOPin::new(PB15),
    ],
};

/// Port C
pub static mut PC: Port = Port {
    port: (GPIOC_BASE + 0 * SIZE) as *mut GPIO_reg,
    pins: [
        GPIOPin::new(PC00),
        GPIOPin::new(PC01),
        GPIOPin::new(PC02),
        GPIOPin::new(PC03),
        GPIOPin::new(PC04),
        GPIOPin::new(PC05),
        GPIOPin::new(PC06),
        GPIOPin::new(PC07),
        GPIOPin::new(PC08),
        GPIOPin::new(PC09),
        GPIOPin::new(PC10),
        GPIOPin::new(PC11),
        GPIOPin::new(PC12),
        GPIOPin::new(PC13),
        GPIOPin::new(PC14),
        GPIOPin::new(PC15),
    ],
};

/// Port D
pub static mut PD: Port = Port {
    port: (GPIOD_BASE + 0 * SIZE) as *mut GPIO_reg,
    pins: [
        GPIOPin::new(PD00),
        GPIOPin::new(PD01),
        GPIOPin::new(PD02),
        GPIOPin::new(PD03),
        GPIOPin::new(PD04),
        GPIOPin::new(PD05),
        GPIOPin::new(PD06),
        GPIOPin::new(PD07),
        GPIOPin::new(PD08),
        GPIOPin::new(PD09),
        GPIOPin::new(PD10),
        GPIOPin::new(PD11),
        GPIOPin::new(PD12),
        GPIOPin::new(PD13),
        GPIOPin::new(PD14),
        GPIOPin::new(PD15),
    ],
};

/// Port E
pub static mut PE: Port = Port {
    port: (GPIOE_BASE + 0 * SIZE) as *mut GPIO_reg,
    pins: [
        GPIOPin::new(PE00),
        GPIOPin::new(PE01),
        GPIOPin::new(PE02),
        GPIOPin::new(PE03),
        GPIOPin::new(PE04),
        GPIOPin::new(PE05),
        GPIOPin::new(PE06),
        GPIOPin::new(PE07),
        GPIOPin::new(PE08),
        GPIOPin::new(PE09),
        GPIOPin::new(PE10),
        GPIOPin::new(PE11),
        GPIOPin::new(PE12),
        GPIOPin::new(PE13),
        GPIOPin::new(PE14),
        GPIOPin::new(PE15),
    ],
};

/// Port F
pub static mut PF: Port = Port {
    port: (GPIOF_BASE + 0 * SIZE) as *mut GPIO_reg,
    pins: [
        GPIOPin::new(PF00),
        GPIOPin::new(PF01),
        GPIOPin::new(PF02),
        GPIOPin::new(PF03),
        GPIOPin::new(PF04),
        GPIOPin::new(PF05),
        GPIOPin::new(PF06),
        GPIOPin::new(PF07),
        GPIOPin::new(PF08),
        GPIOPin::new(PF09),
        GPIOPin::new(PF10),
        GPIOPin::new(PF11),
        GPIOPin::new(PF12),
        GPIOPin::new(PF13),
        GPIOPin::new(PF14),
        GPIOPin::new(PF15),
    ],
};

/// Port G
pub static mut PG: Port = Port {
    port: (GPIOG_BASE + 0 * SIZE) as *mut GPIO_reg,
    pins: [
        GPIOPin::new(PG00),
        GPIOPin::new(PG01),
        GPIOPin::new(PG02),
        GPIOPin::new(PG03),
        GPIOPin::new(PG04),
        GPIOPin::new(PG05),
        GPIOPin::new(PG06),
        GPIOPin::new(PG07),
        GPIOPin::new(PG08),
        GPIOPin::new(PG09),
        GPIOPin::new(PG10),
        GPIOPin::new(PG11),
        GPIOPin::new(PG12),
        GPIOPin::new(PG13),
        GPIOPin::new(PG14),
        GPIOPin::new(PG15),
    ],
};

pub struct GPIOPin {
    port: *mut GPIO_reg,
    pin: u32,
    pin_mask: u32,
    client_data: Cell<usize>,
    client: Cell<Option<&'static hil::gpio::Client>>,
}

impl GPIOPin {
    const fn new(pin: Pin) -> GPIOPin {
        GPIOPin {
            port: (GPIO_BASE + ((pin as usize) / 16) * SIZE) as *mut GPIO_reg,
            pin: pin as u32,
            pin_mask: 1 << ((pin as u32) & 16),
            client_data: Cell::new(0),
            client: Cell::new(None),
        }
    }

    pub fn set_client<C: hil::gpio::Client>(&self, client: &'static C) {
        self.client.set(Some(client));
    }

    pub fn handle_interrupt(&self) {

    }


}

impl hil::gpio::PinCtl for GPIOPin {
    fn set_input_mode(&self, mode: hil::gpio::InputMode) {
        let conf = match mode {
            hil::gpio::InputMode::PullUp => 1,
            hil::gpio::InputMode::PullDown => 2,
            hil::gpio::InputMode::PullNone => 0,
        };
        let port: &GPIO_reg = unsafe{ &*self.port};
        port.PUPDR.mask_set(0b11, (self.pin as u32) << 1, conf as u32);
    }
}

impl hil::gpio::Pin for GPIOPin {
    /// Configure the GPIO pin as an output pin.
    fn make_output(&self) {
        let port: &GPIO_reg = unsafe{ &*self.port};
        port.MODER.mask_set(0b11, (self.pin as u32) << 1, PinMode::Output as u32);
    }

    /// Configure the GPIO pin as an input pin.
    fn make_input(&self) {
        let port: &GPIO_reg = unsafe{ &*self.port};
        port.MODER.mask_set(0b11, (self.pin as u32) << 1, PinMode::Input as u32);
    }

    /// Disable the GPIO pin and put it into its lowest power
    /// mode.
    fn disable(&self) {
        let port: &GPIO_reg = unsafe{ &*self.port};
        hil::gpio::PinCtl::set_input_mode(self, hil::gpio::InputMode::PullNone);
    }

    /// Set the GPIO pin high. It must be an output.
    fn set(&self) {
        let port: &GPIO_reg = unsafe{ &*self.port};
        port.ODR.set(self.pin_mask)
    }   

    /// Set the GPIO pin low. It must be an output.
    fn clear(&self) {
        let port: &GPIO_reg = unsafe{ &*self.port};
        port.BSR.set(self.pin_mask as u16);
        port.BRR.set(self.pin_mask as u16);
    }

    /// Toggle the GPIO pin. It must be an output.
    fn toggle(&self) {
        let port: &GPIO_reg = unsafe{ &*self.port};
        let odr = port.ODR.get();

        port.BSR.check((!odr & self.pin_mask) as u16);
        port.BRR.check((odr & self.pin_mask) as u16);
    }

    /// Get the current state of an input GPIO pin.
    fn read(&self) -> bool {
        let port: &GPIO_reg = unsafe{ &*self.port};
        (port.IDR.get() & self.pin_mask) > 0
    }

    fn enable_interrupt(&self, client_data: usize, mode: hil::gpio::InterruptMode) {
        
    }

    fn disable_interrupt(&self) {
        
    }
}