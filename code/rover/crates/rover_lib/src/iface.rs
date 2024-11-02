use uom::si::f32::Angle;

pub struct DrivePower(f32);

impl DrivePower {
    pub const MAX: f32 = 1.0;
    pub const MIN: f32 = -1.0;

    pub fn new(inner: f32) -> Self {
        Self(inner.clamp(Self::MIN, Self::MAX))
    }

    pub fn inner(&self) -> f32 {
        self.0
    }
}

pub trait Motor {
    type Error;

    fn drive(&mut self, power: DrivePower) -> Result<(), Self::Error>;
    fn neutral(&mut self) -> Result<(), Self::Error>;
}

pub trait FourWheeledRobot {
    type Error;

    fn drive(
        &mut self,
        fl: DrivePower,
        fr: DrivePower,
        bl: DrivePower,
        br: DrivePower,
    ) -> Result<(), Self::Error>;
    fn neutral(&mut self) -> Result<(), Self::Error>;
}

pub struct Turn(f32);

impl Turn {
    pub const MAX: f32 = 1.0;
    pub const MIN: f32 = -1.0;

    pub fn new(turn: f32) -> Self {
        Self(turn.clamp(Self::MIN, Self::MAX))
    }

    pub fn inner(&self) -> f32 {
        self.0
    }
}

pub trait MecanumRobot: FourWheeledRobot {
    type Error;

    fn drive(
        &mut self,
        power: DrivePower,
        theta: Angle,
        turn: Turn,
    ) -> Result<(), <Self as MecanumRobot>::Error>;
}

impl<T> MecanumRobot for T
where
    T: FourWheeledRobot,
{
    type Error = T::Error;

    fn drive(
        &mut self,
        power: DrivePower,
        theta: Angle,
        turn: Turn,
    ) -> Result<(), <Self as MecanumRobot>::Error> {
        let power = power.inner();
        let theta = theta.get::<uom::si::angle::radian>() - core::f32::consts::FRAC_PI_4;
        let turn = turn.inner();

        let fl = DrivePower::new(power * libm::cosf(theta) + turn);
        let fr = DrivePower::new(power * libm::sinf(theta) - turn);
        let bl = DrivePower::new(power * libm::sinf(theta) + turn);
        let br = DrivePower::new(power * libm::cosf(theta) - turn);

        self.drive(fl, fr, bl, br)
    }
}
