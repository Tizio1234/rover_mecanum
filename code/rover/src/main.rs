#![no_std]
#![no_main]

mod fmt;

use core::cell::RefCell;

#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_stm32::{
    gpio::{Level, Output, OutputType, Speed},
    time::khz,
    timer::simple_pwm,
};
use embassy_time::{Duration, Timer};
use embedded_hal_02::PwmPin;
use fmt::info;

use rover_lib::{DrivePower, Motor, MyMotor};

struct PwmWrapper<'a, C, T, D, P: embedded_hal_02::Pwm<Channel = C, Time = T, Duty = D>> {
    pwm: &'a RefCell<P>,
    channel: C,
}

impl<'a, C: Copy, T, D, P> PwmWrapper<'a, C, T, D, P>
where
    P: embedded_hal_02::Pwm<Channel = C, Time = T, Duty = D>,
{
    pub fn new(pwm: &'a RefCell<P>, channel: C) -> Self {
        Self { pwm, channel }
    }
}

impl<'a, C: Copy, T, D, P: embedded_hal_02::Pwm<Channel = C, Time = T, Duty = D>> PwmPin
    for PwmWrapper<'a, C, T, D, P>
{
    type Duty = D;

    fn disable(&mut self) {
        self.pwm.borrow_mut().disable(self.channel);
    }
    fn enable(&mut self) {
        self.pwm.borrow_mut().enable(self.channel);
    }

    fn get_duty(&self) -> Self::Duty {
        self.pwm.borrow_mut().get_duty(self.channel)
    }
    fn get_max_duty(&self) -> Self::Duty {
        self.pwm.borrow_mut().get_max_duty()
    }
    fn set_duty(&mut self, duty: Self::Duty) {
        self.pwm.borrow_mut().set_duty(self.channel, duty);
    }
}

impl<C, T, D, P: embedded_hal_02::Pwm<Channel = C, Time = T, Duty = D>>
    embedded_hal_1::pwm::ErrorType for PwmWrapper<'_, C, T, D, P>
{
    type Error = embedded_hal_1::pwm::ErrorKind;
}

impl<'a, C: Copy, T, D, P> embedded_hal_1::pwm::SetDutyCycle for PwmWrapper<'a, C, T, D, P>
where
    D: TryFrom<u16> + Into<u16>,
    P: embedded_hal_02::Pwm<Channel = C, Time = T, Duty = D>,
{
    fn max_duty_cycle(&self) -> u16 {
        self.get_max_duty().into()
    }
    fn set_duty_cycle(&mut self, duty: u16) -> Result<(), Self::Error> {
        self.set_duty(duty.try_into().map_err(|_| Self::Error::Other)?);
        Ok(())
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    let mut led = Output::new(p.PA5, Level::High, Speed::Low);

    let ch1_pin = simple_pwm::PwmPin::new_ch1(p.PA8, OutputType::PushPull);
    let ch2_pin = simple_pwm::PwmPin::new_ch2(p.PA9, OutputType::PushPull);
    let ch3_pin = simple_pwm::PwmPin::new_ch3(p.PA10, OutputType::PushPull);
    let ch4_pin = simple_pwm::PwmPin::new_ch4(p.PA11, OutputType::PushPull);
    let pwm = RefCell::new(simple_pwm::SimplePwm::new(
        p.TIM1,
        Some(ch1_pin),
        Some(ch2_pin),
        Some(ch3_pin),
        Some(ch4_pin),
        khz(1),
        Default::default(),
    ));

    let mut ch1 = PwmWrapper::new(&pwm, embassy_stm32::timer::Channel::Ch1);
    // let mut ch2 = PwmWrapper::new(&pwm, embassy_stm32::timer::Channel::Ch2);

    ch1.enable();

    let dir_0 = Output::new(p.PB4, Level::Low, Speed::Low);
    let dir_1 = Output::new(p.PB5, Level::Low, Speed::Low);

    let mut motor = MyMotor::new(ch1, dir_0, dir_1, embedded_hal_1::digital::PinState::High);

    loop {
        info!("Hello, World!");
        led.set_high();
        motor.drive(DrivePower::new(0.5)).unwrap();
        Timer::after(Duration::from_millis(500)).await;

        led.set_low();
        motor.drive(DrivePower::new(1.0)).unwrap();
        Timer::after(Duration::from_millis(500)).await;
    }
}
