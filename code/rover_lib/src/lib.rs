#![no_std]

use embedded_hal::{
    digital::{OutputPin, PinState},
    pwm::SetDutyCycle,
};
use uom::si::f32::Angle;

pub enum Direction {
    ClockWise,
    AntiClockWise,
}

pub trait Motor {
    type Error;

    fn drive(&mut self, power: u8, dir: Direction) -> Result<(), Self::Error>;
    fn neutral(&mut self) -> Result<(), Self::Error>;
}

pub trait MecanumRobot {
    type Error;

    fn drive(&mut self, power: u8, theta: Angle, turn: i8) -> Result<(), Self::Error>;
}

pub struct MyMotor<P: SetDutyCycle, O: OutputPin> {
    pwm: P,
    dir_0: O,
    dir_1: O,
    dir_active: PinState,
    dir_passive: PinState,
}

impl<P: SetDutyCycle, O: OutputPin> MyMotor<P, O> {
    pub fn new(pwm: P, dir_0: O, dir_1: O, dir_active: PinState, dir_passive: PinState) -> Self {
        Self {
            pwm,
            dir_0,
            dir_1,
            dir_active,
            dir_passive,
        }
    }
}

impl<P: SetDutyCycle, O: OutputPin> Motor for MyMotor<P, O> {
    type Error = ();

    fn drive(&mut self, power: u8, dir: Direction) -> Result<(), Self::Error> {
        let (dir_0, dir_1) = match dir {
            Direction::ClockWise => (self.dir_active, self.dir_passive),
            Direction::AntiClockWise => (self.dir_passive, self.dir_active),
        };
        
        self.dir_0.set_state(dir_0).map_err(|_| ())?;
        self.dir_1.set_state(dir_1).map_err(|_| ())?;
        self.pwm
            .set_duty_cycle_fraction(power as u16, u8::MAX as u16)
            .map_err(|_| ())?;

        Ok(())
    }
    fn neutral(&mut self) -> Result<(), ()> {
        self.pwm.set_duty_cycle_fully_off().map_err(|_| ())?;
        self.dir_0.set_state(self.dir_passive).map_err(|_| ())?;
        self.dir_1.set_state(self.dir_passive).map_err(|_| ())?;

        Ok(())
    }
}
