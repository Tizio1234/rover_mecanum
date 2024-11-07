#![no_std]
#![no_main]

mod fmt;

extern crate alloc;

use alloc::{boxed::Box, rc::Rc};
use cobs::CobsDecoder;
use embedded_alloc::LlffHeap as Heap;
use serde_json::Value;

#[global_allocator]
static HEAP: Heap = Heap::empty();

use core::cell::RefCell;

#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_stm32::{
    bind_interrupts,
    exti::{Channel, ExtiInput},
    gpio::{AnyPin, Input, Output, Pin},
    peripherals,
    timer::simple_pwm,
    usart::{self, BufferedUart},
};
use embassy_time::{Duration, Timer};
use embedded_hal_02::PwmPin;

use fmt::info;

use embedded_io::Write;
use embedded_io_async::BufRead;

use rover_lib::{
    iface::FWRMerror, my_lib::MyFourWheelRobotError, Angle, DrivePower, FourWheeledRobot,
    MecanumRobot, Motor, MyFourWheelRobot, MyMotor, Turn,
};

struct PwmWrapper<C, T, D, P: embedded_hal_02::Pwm<Channel = C, Time = T, Duty = D>> {
    pwm: Rc<RefCell<P>>,
    channel: C,
}

impl<C, T, D, P> PwmWrapper<C, T, D, P>
where
    P: embedded_hal_02::Pwm<Channel = C, Time = T, Duty = D>,
{
    pub fn new(pwm: Rc<RefCell<P>>, channel: C) -> Self {
        Self { pwm, channel }
    }
}

impl<C: Copy, T, D, P: embedded_hal_02::Pwm<Channel = C, Time = T, Duty = D>> PwmPin
    for PwmWrapper<C, T, D, P>
{
    type Duty = D;

    fn disable(&mut self) {
        self.pwm.borrow_mut().disable(self.channel);
    }
    fn enable(&mut self) {
        self.pwm.borrow_mut().enable(self.channel);
    }

    fn get_duty(&self) -> Self::Duty {
        self.pwm.borrow_mut().get_duty(self.channel)
    }
    fn get_max_duty(&self) -> Self::Duty {
        self.pwm.borrow_mut().get_max_duty()
    }
    fn set_duty(&mut self, duty: Self::Duty) {
        self.pwm.borrow_mut().set_duty(self.channel, duty);
    }
}

impl<C, T, D, P: embedded_hal_02::Pwm<Channel = C, Time = T, Duty = D>>
    embedded_hal_1::pwm::ErrorType for PwmWrapper<C, T, D, P>
{
    type Error = embedded_hal_1::pwm::ErrorKind;
}

impl<C: Copy, T, D, P> embedded_hal_1::pwm::SetDutyCycle for PwmWrapper<C, T, D, P>
where
    D: TryFrom<u16> + Into<u16>,
    P: embedded_hal_02::Pwm<Channel = C, Time = T, Duty = D>,
{
    fn max_duty_cycle(&self) -> u16 {
        self.get_max_duty().into()
    }
    fn set_duty_cycle(&mut self, duty: u16) -> Result<(), Self::Error> {
        self.set_duty(duty.try_into().map_err(|_| Self::Error::Other)?);
        Ok(())
    }
}

#[embassy_executor::task]
async fn button_task(
    button: ExtiInput<'static, AnyPin>,
    mut robot: Box<dyn MecanumRobot<Error = FWRMerror<MyFourWheelRobotError>>>,
) {
    generic_button_task(button, robot.as_mut()).await;
}

async fn generic_button_task<'a, E: core::error::Error>(
    mut button: ExtiInput<'a, AnyPin>,
    robot: &'a mut dyn (MecanumRobot<Error = E>),
) {
    loop {
        button.wait_for_low().await;
        info!("making robot go forward");
        robot
            .drive(
                DrivePower::new(1.0),
                Angle::new::<uom::si::angle::radian>(core::f32::consts::FRAC_PI_2),
                Turn::new(0.0),
            )
            .unwrap();

        button.wait_for_high().await;
        info!("putting robot in neutral");
        robot.neutral().unwrap();
    }
}

bind_interrupts!(struct Irqs {
    USART2 => usart::BufferedInterruptHandler<peripherals::USART2>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    // allocator
    {
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 4096;
        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
    }

    let pwm = {
        use embassy_stm32::{gpio::OutputType, time::khz, timer::Channel};
        use simple_pwm::PwmPin;

        let channels = (
            Some(PwmPin::new_ch1(p.PA8, OutputType::PushPull)),
            Some(PwmPin::new_ch2(p.PA9, OutputType::PushPull)),
            Some(PwmPin::new_ch3(p.PA10, OutputType::PushPull)),
            Some(PwmPin::new_ch4(p.PA11, OutputType::PushPull)),
        );

        let mut pwm = simple_pwm::SimplePwm::new(
            p.TIM1,
            channels.0,
            channels.1,
            channels.2,
            channels.3,
            khz(1),
            Default::default(),
        );

        pwm.enable(Channel::Ch1);
        pwm.enable(Channel::Ch2);
        pwm.enable(Channel::Ch3);
        pwm.enable(Channel::Ch4);

        Rc::new(RefCell::new(pwm))
    };

    let robot = {
        use embassy_stm32::{
            gpio::{Level, Speed},
            timer::Channel,
        };
        use embedded_hal_1::digital::PinState;

        MyFourWheelRobot::new(
            MyMotor::new(
                PwmWrapper::new(Rc::clone(&pwm), Channel::Ch1),
                Output::new(p.PC4, Level::Low, Speed::Low),
                Output::new(p.PB13, Level::Low, Speed::Low),
                PinState::High,
            ),
            MyMotor::new(
                PwmWrapper::new(Rc::clone(&pwm), Channel::Ch2),
                Output::new(p.PB14, Level::Low, Speed::Low),
                Output::new(p.PB15, Level::Low, Speed::Low),
                PinState::High,
            ),
            MyMotor::new(
                PwmWrapper::new(Rc::clone(&pwm), Channel::Ch3),
                Output::new(p.PB1, Level::Low, Speed::Low),
                Output::new(p.PB2, Level::Low, Speed::Low),
                PinState::High,
            ),
            MyMotor::new(
                PwmWrapper::new(Rc::clone(&pwm), Channel::Ch4),
                Output::new(p.PB12, Level::Low, Speed::Low),
                Output::new(p.PC5, Level::Low, Speed::Low),
                PinState::High,
            ),
        )
    };

    let button: ExtiInput<'static, AnyPin> = ExtiInput::new(
        Input::new(p.PC13.degrade(), embassy_stm32::gpio::Pull::Up),
        p.EXTI13.degrade(),
    );
    spawner.spawn(button_task(button, Box::new(robot))).unwrap();

    const RX_SIZE: usize = 128;

    let mut tx_buf = [0u8; 32];
    let mut rx_buf = [0u8; RX_SIZE];

    let buf_usart = BufferedUart::new(
        p.USART2,
        Irqs,
        p.PA3,
        p.PA2,
        &mut tx_buf,
        &mut rx_buf,
        usart::Config::default(),
    )
    .unwrap();

    let (mut tx, mut rx) = buf_usart.split();

    loop {
        let mut decode_out = [0u8; RX_SIZE];
        let mut decoder = CobsDecoder::new(&mut decode_out);
        let size = loop {
            let buf = rx.fill_buf().await.unwrap();
            let len = buf.len();

            _ = writeln!(tx, "received raw: {:?}", buf);

            match decoder.push(buf) {
                Ok(Some((n, _))) => {
                    rx.consume(len);
                    break Some(n);
                }
                Ok(None) => {}
                Err(_) => {
                    rx.consume(len);
                    break None;
                }
            }
        };

        if let Some(size) = size {
            let packet_raw = &decode_out[..size];

            match serde_json::from_slice::<Value>(packet_raw) {
                Ok(v) => {
                    _ = writeln!(tx, "p: {}, th: {}, tu: {}", v["p"], v["th"], v["tu"]);
                }
                Err(_) => {}
            }

            match core::str::from_utf8(packet_raw) {
                Ok(s) => {
                    _ = writeln!(tx, "utf8 packet: {s}");
                }
                Err(_) => _ = writeln!(tx, "raw packet: {:?}", packet_raw),
            }
        } else {
            _ = writeln!(tx, "error decoding");
        }
    }
}
