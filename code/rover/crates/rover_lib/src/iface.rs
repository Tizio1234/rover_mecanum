pub use uom::si::f32::Angle;

#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct MotorPower(f32);

impl MotorPower {
    pub const MAX: f32 = 1.0;
    pub const MIN: f32 = -Self::MAX;

    pub fn new(inner: f32) -> Self {
        Self(inner.clamp(Self::MIN, Self::MAX))
    }

    pub fn inner(&self) -> f32 {
        self.0
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct MecanumPower(f32);

impl MecanumPower {
    pub const MAX: f32 = 1.0;
    pub const MIN: f32 = 0.0;

    pub fn new(inner: f32) -> Self {
        Self(inner.clamp(Self::MIN, Self::MAX))
    }

    pub fn inner(&self) -> f32 {
        self.0
    }
}

pub trait Motor {
    type Error: core::error::Error;

    fn drive(&mut self, power: MotorPower) -> Result<(), Self::Error>;
    fn neutral(&mut self) -> Result<(), Self::Error>;
}

pub trait FourWheeledRobot {
    type Error: core::error::Error;

    fn drive(
        &mut self,
        fl: MotorPower,
        fr: MotorPower,
        bl: MotorPower,
        br: MotorPower,
    ) -> Result<(), Self::Error>;
    fn neutral(&mut self) -> Result<(), Self::Error>;
}

#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Turn(f32);

impl Turn {
    pub const MAX: f32 = 1.0;
    pub const MIN: f32 = -Self::MAX;

    pub fn new(turn: f32) -> Self {
        Self(turn.clamp(Self::MIN, Self::MAX))
    }

    pub fn inner(&self) -> f32 {
        self.0
    }
}

pub trait MecanumRobot {
    type Error: core::error::Error;

    fn drive(&mut self, power: MecanumPower, theta: Angle, turn: Turn) -> Result<(), Self::Error>;
    fn neutral(&mut self) -> Result<(), Self::Error>;
    fn control(&mut self, ctrl: MecanumControl) -> Result<(), Self::Error> {
        match ctrl {
            MecanumControl::Neutral => self.neutral(),
            MecanumControl::Drive(p, th, tu) => self.drive(p, th, tu),
        }
    }
}

pub enum MecanumControl {
    Neutral,
    Drive(MecanumPower, Angle, Turn),
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FWRMerror<E> {
    Mecanum,
    Internal(E),
}

impl<E: core::fmt::Debug> core::fmt::Display for FWRMerror<E> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        core::fmt::Debug::fmt(&self, f)
    }
}
impl<E: core::error::Error> core::error::Error for FWRMerror<E> {}

impl<T: FourWheeledRobot> MecanumRobot for T {
    type Error = FWRMerror<T::Error>;

    fn drive(&mut self, power: MecanumPower, theta: Angle, turn: Turn) -> Result<(), Self::Error> {
        let power = power.inner();
        let theta = theta.get::<uom::si::angle::radian>() - core::f32::consts::FRAC_PI_4;
        let turn = turn.inner();

        let fl = MotorPower::new(power * libm::cosf(theta) + turn);
        let fr = MotorPower::new(power * libm::sinf(theta) - turn);
        let bl = MotorPower::new(power * libm::sinf(theta) + turn);
        let br = MotorPower::new(power * libm::cosf(theta) - turn);

        FourWheeledRobot::drive(self, fl, fr, bl, br)
            .map_err(|e| <Self as MecanumRobot>::Error::Internal(e))
    }
    fn neutral(&mut self) -> Result<(), Self::Error> {
        self.neutral()
            .map_err(|e| <Self as MecanumRobot>::Error::Internal(e))
    }
}
