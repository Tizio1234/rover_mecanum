#![no_std]

pub mod iface;
pub mod my_lib;

pub use iface::{Motor, DrivePower};
pub use my_lib::MyMotor;
