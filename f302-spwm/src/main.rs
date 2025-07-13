#![no_std]
#![no_main]

use core::cell::UnsafeCell;
use core::ops::Not;
use core::panic::PanicInfo;
use cortex_m_rt::entry;
use stm32f3::stm32f302::gpioc::moder::MODE;

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {}
}

// PC: 9, 8, 6, 5

struct PeripheralSingletone {
    holder: UnsafeCell<Option<stm32f3::stm32f302::Peripherals>>,
}

unsafe impl Sync for PeripheralSingletone {}

impl PeripheralSingletone {
    fn set(&self, peripherals: Option<stm32f3::stm32f302::Peripherals>) {
        unsafe {
            *self.holder.get() = peripherals;
        }
    }

    fn get_peripheral(&self) -> &stm32f3::stm32f302::Peripherals {
        unsafe { self.holder.get().as_ref().unwrap().as_ref().unwrap() }
    }
}

static PERIPHERAL_SINGLETON: PeripheralSingletone = PeripheralSingletone {
    holder: UnsafeCell::new(None),
};

#[entry]
fn main() -> ! {
    unsafe {
        PERIPHERAL_SINGLETON.set(Some(stm32f3::stm32f302::Peripherals::steal()));
    }

    PERIPHERAL_SINGLETON
        .get_peripheral()
        .RCC
        .ahbenr()
        .modify(|_, w| w.iopcen().set_bit());
    PERIPHERAL_SINGLETON
        .get_peripheral()
        .GPIOC
        .moder()
        .modify(|_, w| w.moder9().set(MODE::Output as u8));

    let mut toggle = false;

    loop {
        for _ in 0..50 {}

        if toggle.not() {
            PERIPHERAL_SINGLETON
                .get_peripheral()
                .GPIOC
                .bsrr()
                .write(|w| w.bs9().set_bit());
        } else {
            PERIPHERAL_SINGLETON
                .get_peripheral()
                .GPIOC
                .bsrr()
                .write(|w| w.br9().set_bit());
        }

        toggle = toggle.not();
    }
}
