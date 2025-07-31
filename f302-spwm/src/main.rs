#![no_std]
#![no_main]

use core::cell::UnsafeCell;
use core::ops::Not;
use core::panic::PanicInfo;
use cortex_m::peripheral::NVIC;
use cortex_m_rt::entry;
use stm32f3::stm32f302::{Interrupt, gpioc::moder::MODE, interrupt};

use spwm::{Spwm, SpwmState};
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

    fn get_ref(&self) -> &T {
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
    let peripheral = PERIPHERAL_SINGLETON.get_ref();

    if peripheral.RCC.cr().read().hsirdy().is_not_ready() {
        peripheral.RCC.cr().modify(|_, w| w.hsion().set_bit());
        while peripheral.RCC.cr().read().hsirdy().is_not_ready() {}
    }

    peripheral.RCC.cfgr().modify(|_, w| w.sw().variant(SW::Hsi));
}

fn gpio_init() {
    PERIPHERAL_SINGLETON
        .get_ref()
        .RCC
        .ahbenr()
        .modify(|_, w| w.iopcen().set_bit());
    PERIPHERAL_SINGLETON.get_ref().GPIOC.moder().modify(|_, w| {
        w.moder9()
            .set(MODE::Output as u8)
            .moder8()
            .set(MODE::Output as u8)
    });
}

fn tim15_init() {
    let peripheral = PERIPHERAL_SINGLETON.get_ref();

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

fn pc9_period_callback() {}

fn pc9_on_off_callback(state: &SpwmState) {
    let peripheral = PERIPHERAL_SINGLETON.get_ref();
    let gpioc = &peripheral.GPIOC;

    match state {
        SpwmState::On => {
            gpioc.bsrr().write(|w| w.bs9().set_bit());
        }
        SpwmState::Off => {
            gpioc.bsrr().write(|w| w.br9().set_bit());
        }
    }
}

fn pc8_period_callback() {}

fn pc8_on_off_callback(state: &SpwmState) {
    let peripheral = PERIPHERAL_SINGLETON.get_ref();
    let gpioc = &peripheral.GPIOC;

    match state {
        SpwmState::On => {
            gpioc.bsrr().write(|w| w.bs8().set_bit());
        }
        SpwmState::Off => {
            gpioc.bsrr().write(|w| w.br8().set_bit());
        }
    }
}

#[entry]
fn main() -> ! {
    unsafe {
        PERIPHERAL_SINGLETON.set(Some(stm32f3::stm32f302::Peripherals::steal()));
        CORE_PERIPHERAL_SINGLETON.set(Some(cortex_m::Peripherals::steal()));
        TIM15_SOFTWARE_PWM.set(Some(Spwm::new(100_000, None, None)));
    }

    // 100 kHz = 10 us
    // 10 Hz = 100 ms = 100000 us
    // 100 kHz / 10 Hz = 10000 tick -> 10000 ticks * 10 us = 100000 us = 100 ms
    // channel #0
    let _ = TIM15_SOFTWARE_PWM.get_ref().set_channel_frequency(0, 10);
    let _ = TIM15_SOFTWARE_PWM
        .get_ref()
        .set_channel_on_off_callback(0, pc9_on_off_callback);
    let _ = TIM15_SOFTWARE_PWM
        .get_ref()
        .set_channel_period_callback(0, pc9_period_callback);
    let _ = TIM15_SOFTWARE_PWM.get_ref().set_channel_duty_cycle(0, 90);
    let _ = TIM15_SOFTWARE_PWM.get_ref().enable(0);
    // channel #1
    let _ = TIM15_SOFTWARE_PWM.get_ref().set_channel_frequency(1, 240);
    let _ = TIM15_SOFTWARE_PWM
        .get_ref()
        .set_channel_on_off_callback(1, pc8_on_off_callback);
    let _ = TIM15_SOFTWARE_PWM
        .get_ref()
        .set_channel_period_callback(1, pc8_period_callback);
    let _ = TIM15_SOFTWARE_PWM.get_ref().set_channel_duty_cycle(1, 50);
    let _ = TIM15_SOFTWARE_PWM.get_ref().enable(1);

    clock_init();
    gpio_init();
    tim15_init();

    loop {
        cortex_m::asm::wfi();
    }
}

#[interrupt]
fn TIM1_BRK_TIM15() {
    let peripheral = PERIPHERAL_SINGLETON.get_ref();
    let tim15 = &peripheral.TIM15;

    tim15.sr().write(|w| w.uif().clear_bit());
    TIM15_SOFTWARE_PWM.get_ref().irq_handler();
}
