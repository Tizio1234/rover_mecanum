pub use uom::si::f32::Angle;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DrivePower(f32);

impl DrivePower {
    pub const MAX: f32 = 1.0;
    pub const MIN: f32 = -Self::MAX;

    pub fn new(inner: f32) -> Self {
        Self(inner.clamp(Self::MIN, Self::MAX))
    }

    pub fn inner(&self) -> f32 {
        self.0
    }
}

pub trait Motor {
    type Error: core::error::Error;

    fn drive(&mut self, power: DrivePower) -> Result<(), Self::Error>;
    fn neutral(&mut self) -> Result<(), Self::Error>;
}

pub trait FourWheeledRobot {
    type Error: core::error::Error;

    fn drive(
        &mut self,
        fl: DrivePower,
        fr: DrivePower,
        bl: DrivePower,
        br: DrivePower,
    ) -> Result<(), Self::Error>;
    fn neutral(&mut self) -> Result<(), Self::Error>;
}

#[derive(Debug, Clone, Copy, PartialEq)]
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

    fn drive(&mut self, power: DrivePower, theta: Angle, turn: Turn) -> Result<(), Self::Error>;
    fn neutral(&mut self) -> Result<(), Self::Error>;
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

    fn drive(&mut self, power: DrivePower, theta: Angle, turn: Turn) -> Result<(), Self::Error> {
        let power = power.inner();
        let theta = theta.get::<uom::si::angle::radian>() - core::f32::consts::FRAC_PI_4;
        let turn = turn.inner();

        let fl = DrivePower::new(power * libm::cosf(theta) + turn);
        let fr = DrivePower::new(power * libm::sinf(theta) - turn);
        let bl = DrivePower::new(power * libm::sinf(theta) + turn);
        let br = DrivePower::new(power * libm::cosf(theta) - turn);

        FourWheeledRobot::drive(self, fl, fr, bl, br)
            .map_err(|e| <Self as MecanumRobot>::Error::Internal(e))
    }
    fn neutral(&mut self) -> Result<(), Self::Error> {
        self.neutral()
            .map_err(|e| <Self as MecanumRobot>::Error::Internal(e))
    }
}
