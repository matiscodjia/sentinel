#![no_std]
#![no_main]

use panic_halt as _;
use stm32f4xx_hal::{
    pac,
    prelude::*,
    timer::Channel1,
};
use cortex_m_rt::entry;

#[entry]
fn main() -> ! {
    // 1. Matériel
    let dp = pac::Peripherals::take().unwrap();

    // 2. Horloges
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.use_hse(25.MHz()).freeze();

    // 3. GPIOs
    let gpioa = dp.GPIOA.split(); // Pour le Servo (PA6)
    let gpioc = dp.GPIOC.split(); // Pour la LED (PC13)

    // --- CONFIG SERVO (PA6 / TIM3) ---
    // On s'assure d'utiliser l'Alternate Function 2 (AF2) pour TIM3
    let servo_pin = gpioa.pa6.into_alternate::<2>();

    // --- CONFIG LED (PC13) ---
    let mut led = gpioc.pc13.into_push_pull_output();

    // 4. PWM avec l'API ChannelBuilder (stm32f4xx-hal 0.17+)
    let channels = Channel1::new(servo_pin);
    let mut ch1 = dp.TIM3.pwm_hz(channels, 50.Hz(), &clocks).split();
    ch1.enable();

    // 5. Bornes (Calculs)
    let max = ch1.get_max_duty();
    // Valeurs sûres pour voir un mouvement large
    let min_pos = max / 40; // ~0°
    let max_pos = max / 8;  // ~180°

    let mut delay = dp.TIM1.delay_us(&clocks);

    // 6. Boucle de Test
    loop {
        // Mouvement A + LED ON
        ch1.set_duty(min_pos);
        led.set_low(); // LED ON (Sur la Black Pill, Low = Allumé)
        delay.delay_ms(1000_u32);

        // Mouvement B + LED OFF
        ch1.set_duty(max_pos);
        led.set_high(); // LED OFF
        delay.delay_ms(1000_u32);
    }
}
