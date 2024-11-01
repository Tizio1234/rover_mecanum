#![no_std]

pub mod iface;
pub mod my_lib;

pub use iface::{Direction, Motor};
pub use my_lib::MyMotor;
