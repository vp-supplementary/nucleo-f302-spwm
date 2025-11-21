#![no_std]
#![no_main]

use core::cell::UnsafeCell;
use core::panic::PanicInfo;
use cortex_m::peripheral::NVIC;
use cortex_m_rt::entry;
use stm32f3::stm32f302::{Interrupt, gpioc::moder::MODE, interrupt};

use spwm::{ChannelId, Spwm, SpwmState};
use stm32f3::stm32f302::flash::acr::LATENCY;
use stm32f3::stm32f302::rcc::cfgr::{HPRE, PLLMUL, PLLSRC, PPRE1, SW};

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {}
}

enum AppPwmChannel {
    Channel0,
    Channel1,
    Channel2,
    Channel3,
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

    fn with_mut<R>(&self, f: impl FnOnce(&mut T) -> R) -> R {
        unsafe {
            let slot = (*self.holder.get())
                .as_mut()
                .expect("UnsafeSingleton::with_mut called before set");
            f(slot)
        }
    }
}

unsafe impl<T> Sync for UnsafeSingleton<T> {}

static PERIPHERAL_SINGLETON: UnsafeSingleton<stm32f3::stm32f302::Peripherals> = UnsafeSingleton {
    holder: UnsafeCell::new(None),
};
static CORE_PERIPHERAL_SINGLETON: UnsafeSingleton<cortex_m::Peripherals> = UnsafeSingleton {
    holder: UnsafeCell::new(None),
};
static TIM15_SOFTWARE_PWM: UnsafeSingleton<Spwm<4>> = UnsafeSingleton {
    holder: UnsafeCell::new(None),
};

fn clock_init() {
    let peripheral = PERIPHERAL_SINGLETON.get_ref();

    if peripheral.RCC.cr().read().hsirdy().is_not_ready() {
        peripheral.RCC.cr().modify(|_, w| w.hsion().set_bit());
        while peripheral.RCC.cr().read().hsirdy().is_not_ready() {}
    }

    peripheral.RCC.cfgr().modify(|_, w| w.sw().variant(SW::Hsi));
    peripheral
        .FLASH
        .acr()
        .modify(|_, w| w.latency().variant(LATENCY::Ws2));
    peripheral.RCC.cfgr().modify(|_, w| {
        w.pllsrc()
            .variant(PLLSRC::HsiDiv2)
            .pllmul()
            .variant(PLLMUL::Mul16)
    });
    peripheral.RCC.cr().modify(|_, w| w.pllon().set_bit());

    while peripheral.RCC.cr().read().pllrdy().is_not_ready() {}

    peripheral
        .RCC
        .cfgr()
        .modify(|_, w| w.hpre().variant(HPRE::Div1).sw().variant(SW::Pll));

    while !peripheral.RCC.cfgr().read().sws().is_pll() {}

    peripheral
        .RCC
        .cfgr()
        .modify(|_, w| w.ppre1().variant(PPRE1::Div2).ppre2().variant(PPRE1::Div1));
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
            .moder6()
            .set(MODE::Output as u8)
            .moder5()
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
    // 64 MHz / 640 = 100 kHz
    peripheral
        .TIM15
        .arr()
        .write(|w| unsafe { w.arr().bits(640) });
    // enable TIM15
    peripheral.TIM15.cr1().write(|w| w.cen().set_bit());
    unsafe { NVIC::unmask(Interrupt::TIM1_BRK_TIM15) };
}

fn period_callback() {}

fn gpio_toggle(state: &SpwmState, channel: &AppPwmChannel) {
    let peripheral = PERIPHERAL_SINGLETON.get_ref();

    let (gpio_reg, gpio_id) = match channel {
        AppPwmChannel::Channel0 => (&peripheral.GPIOC, 9),
        AppPwmChannel::Channel1 => (&peripheral.GPIOC, 8),
        AppPwmChannel::Channel2 => (&peripheral.GPIOC, 6),
        AppPwmChannel::Channel3 => (&peripheral.GPIOC, 5),
    };

    match state {
        SpwmState::On => {
            gpio_reg.bsrr().write(|w| w.bs(gpio_id).set_bit());
        }
        SpwmState::Off => {
            gpio_reg.bsrr().write(|w| w.br(gpio_id).set_bit());
        }
    }
}

fn create_pwm_channels() {
    TIM15_SOFTWARE_PWM.with_mut(|pwm| {
        // 100 kHz = 10 us
        // 10 Hz = 100 ms = 100000 us
        // 100 kHz / 10 Hz = 10000 tick -> 10000 ticks * 10 us = 100000 us = 100 ms
        // channel #0
        let frequency = 10;
        let channel_0 = pwm
            .create_channel()
            .freq_hz(frequency)
            .duty_cycle(90)
            .on_off_callback(|state| {
                gpio_toggle(state, &AppPwmChannel::Channel0);
            })
            .period_callback(period_callback)
            .build();
        // channel #1
        let frequency = 1000;
        let channel_1 = pwm
            .create_channel()
            .freq_hz(frequency)
            .duty_cycle(55)
            .on_off_callback(|state| {
                gpio_toggle(state, &AppPwmChannel::Channel1);
            })
            .period_callback(period_callback)
            .build();
        // channel #2
        let frequency = 500;
        let channel_2 = pwm
            .create_channel()
            .freq_hz(frequency)
            .duty_cycle(50)
            .on_off_callback(|state| {
                gpio_toggle(state, &AppPwmChannel::Channel2);
            })
            .period_callback(period_callback)
            .build();
        // channel #3
        let frequency = 250;
        let channel_3 = pwm
            .create_channel()
            .freq_hz(frequency)
            .duty_cycle(64)
            .on_off_callback(|state| {
                gpio_toggle(state, &AppPwmChannel::Channel3);
            })
            .period_callback(period_callback)
            .build();

        let mut arr: [ChannelId; 4] = [0; 4];
        let result = pwm.register_channel(channel_0.unwrap());
        assert!(result.is_ok(), "Failed to register channel #0");
        arr[0] = result.unwrap();
        let result = pwm.register_channel(channel_1.unwrap());
        assert!(result.is_ok(), "Failed to register channel #1");
        arr[1] = result.unwrap();
        let result = pwm.register_channel(channel_2.unwrap());
        assert!(result.is_ok(), "Failed to register channel #2");
        arr[2] = result.unwrap();
        let result = pwm.register_channel(channel_3.unwrap());
        assert!(result.is_ok(), "Failed to register channel #3");
        arr[3] = result.unwrap();

        for channel_id in arr {
            let channel = pwm.get_channel(channel_id);

            if let Some(channel) = channel {
                let result = channel.enable();
                assert!(result.is_ok(), "Failed to enable channel");
            }
        }
    });
}

#[entry]
fn main() -> ! {
    let spwm: Spwm<4> = Spwm::new(100_000);

    unsafe {
        PERIPHERAL_SINGLETON.set(Some(stm32f3::stm32f302::Peripherals::steal()));
        CORE_PERIPHERAL_SINGLETON.set(Some(cortex_m::Peripherals::steal()));
        TIM15_SOFTWARE_PWM.set(Some(spwm));
    }

    create_pwm_channels();
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
