
use cortexm4;
use gpio;
use timer;
use nvic::*;
use kernel::Chip;



pub struct STM32F429ZI {
	pub mpu: cortexm4::mpu::MPU,
	pub systick: cortexm4::systick::SysTick,
}

impl STM32F429ZI {
	pub unsafe fn new() -> STM32F429ZI {
		STM32F429ZI {
            mpu: cortexm4::mpu::MPU::new(),
            systick: cortexm4::systick::SysTick::new(),
        }
	}
}

impl Chip for STM32F429ZI {
	type MPU = cortexm4::mpu::MPU;
	type SysTick = cortexm4::systick::SysTick;

	fn service_pending_interrupts(&mut self) {
		unsafe {
			while let Some(interrupt) = cortexm4::nvic::next_pending() {
				match interrupt {
					EXTI0_IRQn 			=> gpio::PA.handle_interrupt(),
					EXTI1_IRQn 			=> gpio::PB.handle_interrupt(),
					EXTI2_IRQn 			=> gpio::PC.handle_interrupt(),
					EXTI3_IRQn 			=> gpio::PD.handle_interrupt(),
					EXTI4_IRQn 			=> gpio::PE.handle_interrupt(),
					EXTI9_5_IRQn		=> gpio::PF.handle_interrupt(),
					EXTI15_10_IRQn		=> gpio::PG.handle_interrupt(),
					TIM5_IRQn           => timer::TIM_ALARM.handle_interrupt(),
					_ => debug!("NvicIdx not supported by Tock"),
				}
				let n = cortexm4::nvic::Nvic::new(interrupt);
	    	    n.clear_pending();
	            n.enable();
			}
		}
	}

	fn has_pending_interrupts(&self) -> bool {
        unsafe { cortexm4::nvic::has_pending() }
    }

    fn mpu(&self) -> &cortexm4::mpu::MPU {
        &self.mpu
    }

    fn systick(&self) -> &cortexm4::systick::SysTick {
        &self.systick
    }

}