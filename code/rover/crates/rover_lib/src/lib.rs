#![no_std]

pub mod iface;
pub mod my_lib;

pub use iface::{Motor, DrivePower, FourWheeledRobot, MecanumRobot, Turn};
pub use my_lib::{MyMotor, MyFourWheelRobot};
