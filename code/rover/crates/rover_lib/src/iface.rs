#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub enum Direction {
    #[default]
    Clockwise,
    CounterClockwise,
}

pub trait Motor {
    type Error;

    fn drive(&mut self, power: u8, dir: Direction) -> Result<(), Self::Error>;
    fn neutral(&mut self) -> Result<(), Self::Error>;
}
