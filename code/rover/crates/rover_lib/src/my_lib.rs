use embedded_hal_1::{
    digital::{OutputPin, PinState},
    pwm::SetDutyCycle,
};
// use uom::si::f32::Angle;

use crate::iface::{DrivePower, FourWheeledRobot, Motor};

pub struct MyMotor<P, O0, O1> {
    pwm: P,
    dir_0: O0,
    dir_1: O1,
    dir_active: PinState,
    dir_passive: PinState,
}
trait Opposite {
    fn opposite(&self) -> Self;
}

impl Opposite for PinState {
    fn opposite(&self) -> Self {
        match self {
            Self::High => Self::Low,
            Self::Low => Self::High,
        }
    }
}

impl<P, O0, O1> MyMotor<P, O0, O1> {
    pub fn new(pwm: P, dir_0: O0, dir_1: O1, dir_active: PinState) -> Self {
        Self {
            pwm,
            dir_0,
            dir_1,
            dir_active,
            dir_passive: dir_active.opposite(),
        }
    }
}


impl<P: SetDutyCycle, O0: OutputPin, O1: OutputPin> Motor for MyMotor<P, O0, O1> {
    type Error = ();

    fn drive(&mut self, power: DrivePower) -> Result<(), Self::Error> {
        let power = power.inner();

        let dirs = if power >= 0.0 {
            (self.dir_active, self.dir_passive)
        } else {
            (self.dir_passive, self.dir_active)
        };

        self.dir_0.set_state(dirs.0).map_err(|_| ())?;
        self.dir_1.set_state(dirs.1).map_err(|_| ())?;
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

pub struct MyFourWheelRobot<FL, FR, BL, BR> {
    fl: FL,
    fr: FR,
    bl: BL,
    br: BR,
}

impl<FL, FR, BL, BR> MyFourWheelRobot<FL, FR, BL, BR> {
    pub fn new(fl: FL, fr: FR, bl: BL, br: BR) -> Self {
        Self { fl, fr, bl, br }
    }
}

impl<FL: Motor, FR: Motor, BL: Motor, BR: Motor> FourWheeledRobot for MyFourWheelRobot<FL, FR, BL, BR> {
    type Error = ();

    fn drive(
        &mut self,
        fl: DrivePower,
        fr: DrivePower,
        bl: DrivePower,
        br: DrivePower,
    ) -> Result<(), Self::Error> {
        self.fl.drive(fl).map_err(|_| ())?;
        self.fr.drive(fr).map_err(|_| ())?;
        self.bl.drive(bl).map_err(|_| ())?;
        self.br.drive(br).map_err(|_| ())?;

        Ok(())
    }
    fn neutral(&mut self) -> Result<(), Self::Error> {
        self.fl.neutral().map_err(|_|())?;
        self.fr.neutral().map_err(|_|())?;
        self.bl.neutral().map_err(|_|())?;
        self.br.neutral().map_err(|_|())?;

        Ok(())
    }
}
