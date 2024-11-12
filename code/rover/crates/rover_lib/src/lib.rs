#![no_std]

pub mod iface;
pub mod my_lib;

pub use iface::{Angle, MotorPower, FourWheeledRobot, MecanumRobot, Motor, Turn};
pub use my_lib::{MyFourWheelRobot, MyMotor};
