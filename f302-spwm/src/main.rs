#![no_std]
#![no_main]

use core::cell::UnsafeCell;
use core::ops::Not;
use core::panic::PanicInfo;
use cortex_m::peripheral::NVIC;
use cortex_m_rt::entry;
use stm32f3::stm32f302::{Interrupt, gpioc::moder::MODE, interrupt};

use spwm::Spwm;
use stm32f3::stm32f302::rcc::cfgr::SW;

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {}
}

// PC: 9, 8, 6, 5
struct UnsafeSingleton<T> {
    holder: UnsafeCell<Option<T>>,
}

impl<T> UnsafeSingleton<T> {
    fn set(&self, peripherals: Option<T>) {
        unsafe {
            *self.holder.get() = peripherals;
        }
    }

    fn get_peripheral(&self) -> &T {
        unsafe { self.holder.get().as_ref().unwrap().as_ref().unwrap() }
    }
}

unsafe impl<T> Sync for UnsafeSingleton<T> {}

static PERIPHERAL_SINGLETON: UnsafeSingleton<stm32f3::stm32f302::Peripherals> = UnsafeSingleton {
    holder: UnsafeCell::new(None),
};
static CORE_PERIPHERAL_SINGLETON: UnsafeSingleton<cortex_m::Peripherals> = UnsafeSingleton {
    holder: UnsafeCell::new(None),
};
static TIM15_SOFTWARE_PWM: UnsafeSingleton<Spwm> = UnsafeSingleton {
    holder: UnsafeCell::new(None),
};

fn clock_init() {
    let peripheral = PERIPHERAL_SINGLETON.get_peripheral();

    if peripheral.RCC.cr().read().hsirdy().is_not_ready() {
        peripheral.RCC.cr().modify(|_, w| w.hsion().set_bit());
        while peripheral.RCC.cr().read().hsirdy().is_not_ready() {}
    }

    peripheral.RCC.cfgr().modify(|_, w| w.sw().variant(SW::Hsi));
}

fn gpio_init() {
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
}

fn tim15_init() {
    let peripheral = PERIPHERAL_SINGLETON.get_peripheral();

    peripheral
        .RCC
        .apb2enr()
        .modify(|_, w| w.tim15en().set_bit());

    // enable CC1IE interrupt
    peripheral.TIM15.dier().write(|w| w.uie().set_bit());
    // 8 MHz / 80 = 100 kHz
    peripheral
        .TIM15
        .arr()
        .write(|w| unsafe { w.arr().bits(80) });
    // enable TIM15
    peripheral.TIM15.cr1().write(|w| w.cen().set_bit());
    unsafe { NVIC::unmask(Interrupt::TIM1_BRK_TIM15) };
}

#[entry]
fn main() -> ! {
    unsafe {
        PERIPHERAL_SINGLETON.set(Some(stm32f3::stm32f302::Peripherals::steal()));
        CORE_PERIPHERAL_SINGLETON.set(Some(cortex_m::Peripherals::steal()));
    }

    clock_init();
    gpio_init();
    tim15_init();

    // let mut toggle = false;

    loop {
        cortex_m::asm::wfi();
    }
}

#[interrupt]
fn TIM1_BRK_TIM15() {
    static mut TOGGLE: bool = false;
    let peripheral = PERIPHERAL_SINGLETON.get_peripheral();
    let tim15 = &peripheral.TIM15;
    let gpioc = &peripheral.GPIOC;

    tim15.sr().write(|w| w.uif().clear_bit());

    let current_state = *TOGGLE;
    *TOGGLE = current_state.not();

    if current_state {
        gpioc.bsrr().write(|w| w.bs9().set_bit());
        // gpioc.bsrr().write(|w| w.br9().set_bit());
    } else {
        // gpioc.bsrr().write(|w| w.bs9().set_bit());
        gpioc.bsrr().write(|w| w.br9().set_bit());
    }
}
