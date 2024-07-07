#![no_std]
#![no_main]

use bme280::i2c::AsyncBME280;
use defmt::{debug, error, info, warn};
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_executor::InterruptExecutor;
use embassy_futures::{
    join::{join, join5},
    select::{select, Either},
};
use embassy_stm32::{
    bind_interrupts, dma::NoDma, exti::ExtiInput, gpio::{AnyPin, Input, Level, Output, Pull, Speed}, i2c, interrupt::{self, typelevel::TIM4, InterruptExt}, peripherals::{
        self, DMA1_CH4, DMA1_CH5, DMA1_CH6, DMA1_CH7, DMA2_CH0, DMA2_CH1, DMA2_CH3, DMA2_CH6, EXTI9, I2C1, PA11, PA12, PA2, PA3, PA5, PA6, PA7, PA8, PA9, PB3, PB4, PB6, PB8, PB9, PC13, PC4, SPI1, TIM1, TIM3, USART2, USART6
    }, time::Hertz, timer::simple_pwm::{PwmPin, SimplePwm}, usart, Peripheral, PeripheralRef
};
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    channel::{Receiver, Sender},
};
use embassy_time::{Delay, Duration, Instant, Timer};
use embedded_hal_async::i2c::I2c;
use icm20948_async::{AccRange, GyrUnit, Icm20948};
use num_traits::real::Real;
use static_cell::StaticCell;

use defmt_rtt as _;
use panic_probe as _;

type MUTEX = CriticalSectionRawMutex;

//use core::sync::atomic::AtomicU32;
//static SIGNAL_A: AtomicU32 = AtomicU32::new(0);
//static SIGNAL_B: AtomicU32 = AtomicU32::new(900);
//static SIGNAL_C: AtomicU32 = AtomicU32::new(900);

#[embassy_stm32::interrupt]
fn I2C2_EV() {
    unsafe { EXECUTOR_HIGH.on_interrupt() };
}

#[embassy_stm32::interrupt]
fn I2C2_ER() {
    unsafe { EXECUTOR_NORMAL.on_interrupt() };
}

#[embassy_stm32::interrupt]
fn I2C3_EV() {
    unsafe { EXECUTOR_LOW.on_interrupt() };
}

// Bind the ebassy interrupt handlers
bind_interrupts!(struct Irqs {
    //USB => usb::InterruptHandler<peripherals::USB>;
    USART6 => usart::InterruptHandler<peripherals::USART6>;
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
});

// DMA/hardware interrupts > HIGH > NORMAL > LOW
static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();
static EXECUTOR_NORMAL: InterruptExecutor = InterruptExecutor::new();
static EXECUTOR_LOW: InterruptExecutor = InterruptExecutor::new();

static IR_CHANNEL_RECEIVE: StaticCell<IrChannelReceive> = StaticCell::new();
static IR_CHANNEL_TRANSMIT: StaticCell<IrChannelTransmit> = StaticCell::new();

#[global_allocator]
static HEAP: embedded_alloc::Heap = embedded_alloc::Heap::empty();

#[cortex_m_rt::entry]
fn main() -> ! {
    {
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 1024;
        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
    }

    let p = embassy_stm32::init(Default::default());

    // Every time our IR receiver gets an event, send an event on this channel with the timing info
    let ir_recv_channel = IR_CHANNEL_RECEIVE.init(IrChannelReceive::new());
    let ir_recv_rx = ir_recv_channel.receiver();
    let ir_recv_tx = ir_recv_channel.sender();

    // If we want to transmit, send an event on this chanel
    let ir_transmit_channel = IR_CHANNEL_TRANSMIT.init(IrChannelTransmit::new());
    let ir_transmit_rx = ir_transmit_channel.receiver();
    let ir_transmit_tx = ir_transmit_channel.sender();

    let ir_input = InputIRLoop {
        pin: p.PA9.into_ref(),
        exti: p.EXTI9.into_ref(),
        tx: ir_recv_tx,
    };

    let ir_output_task = OutputIRLoop {
        ir_output_timer: p.TIM1.into_ref(),
        ir_output_pin: p.PA8.into_ref(),

        rx: ir_transmit_rx,
    };

    let uart = USart {
        dma_tx: p.DMA2_CH6.into_ref(),
        dma_rx: p.DMA2_CH1.into_ref(),
        tx: p.PA11.into_ref(),
        rx: p.PA12.into_ref(),

        uart: p.USART6.into_ref(),
    };

    let main_input = InputMainLoop {
        cs_spi1: p.PB6.into_ref(),
        cs_spi2: p.PC4.into_ref(),

        i2c_channel: p.I2C1.into_ref(),
        i2c_sda: p.PB9.into_ref(),
        i2c_scl: p.PB8.into_ref(),
        i2c_dma_tx: p.DMA1_CH7.into_ref(),
        //i2c_dma_rx: p.DMA1_CH5.into_ref(),

        spi_channel: p.SPI1.into_ref(),
        spi_sck: p.PB3.into_ref(),
        spi_mosi: p.PA7.into_ref(),
        spi_miso: p.PA6.into_ref(),
        spi_dma_tx: p.DMA2_CH3.into_ref(),
        spi_dma_rx: p.DMA2_CH0.into_ref(),

        uart,

        button: p.PC13.into_ref(),

        onboard_led_pin: p.PA5.into_ref(),

        other_output_timer: p.TIM3.into_ref(),
        other_led_pin: p.PB4.into_ref(),

        rx: ir_recv_rx,
        ir_transmit_tx,
    };

    // We use I2C flags for our event loops
    // This could be any set of interrupts that are not used by the peripherals
    drop(p.I2C2);
    drop(p.I2C3);

    // Setup our two executors
    {
        let chan = embassy_stm32::pac::interrupt::I2C2_EV;
        chan.set_priority(embassy_stm32::interrupt::Priority::P2);
        let spawner = EXECUTOR_HIGH.start(chan);
        spawner.spawn(high_prio_loop(ir_output_task)).unwrap();

        let chan = embassy_stm32::pac::interrupt::I2C2_ER;
        chan.set_priority(embassy_stm32::interrupt::Priority::P3);
        let spawner = EXECUTOR_NORMAL.start(chan);
        spawner.spawn(normal_prio_loop(ir_input)).unwrap();

        let chan = embassy_stm32::pac::interrupt::I2C3_EV;
        chan.set_priority(embassy_stm32::interrupt::Priority::P8);
        let spawner = EXECUTOR_LOW.start(chan);
        spawner.spawn(low_prio_loop(main_input)).unwrap();
    }

    // Wait for interrupts
    loop {
        cortex_m::asm::wfi();
    }
}

#[derive(Default, Clone, Debug, defmt::Format)]
struct TimingCharistics {
    initial_low: u32,
    initial_high: u32,

    bit_high_zero: u32,
    bit_high_one: u32,
    bit_low: u32,
}

type IrReceiveType = (Duration, Duration);
const IR_RECEIVE_COUNT: usize = 100;
type IrChannelReceive = embassy_sync::channel::Channel<MUTEX, IrReceiveType, IR_RECEIVE_COUNT>;

struct IrTransmitType {
    timing: TimingCharistics,
    data: u32,
}

const IR_TRANSMIT_COUNT: usize = 10;
type IrChannelTransmit = embassy_sync::channel::Channel<MUTEX, IrTransmitType, IR_TRANSMIT_COUNT>;

struct OutputIRLoop {
    ir_output_timer: PeripheralRef<'static, TIM1>,
    ir_output_pin: PeripheralRef<'static, PA8>,

    rx: Receiver<'static, MUTEX, IrTransmitType, IR_TRANSMIT_COUNT>,
}

struct InputIRLoop {
    //ir_timer_a: PeripheralRef<'static, TIM4>,
    //ir_timer_b: PeripheralRef<'static, TIM5>,

    pin: PeripheralRef<'static, PA9>,
    exti: PeripheralRef<'static, EXTI9>,
    tx: Sender<'static, MUTEX, IrReceiveType, IR_RECEIVE_COUNT>,
}

#[embassy_executor::task]
async fn high_prio_loop(ins: OutputIRLoop) {
    debug!("Running IR Transmitter program");
    // D6 / PB10 = TIM2CH3
    // D5 / PB4 = TIM3CH1
    // D3 / PB3 = TIM2CH2
    //
    // D7 / PA8 = TIM1CH1
    let mut pwm = embassy_stm32::timer::simple_pwm::SimplePwm::new(
        ins.ir_output_timer,
        Some(PwmPin::new_ch1(
            ins.ir_output_pin,
            embassy_stm32::gpio::OutputType::OpenDrain,
        )),
        None,
        None,
        None,
        Hertz::khz(38),
        embassy_stm32::timer::CountingMode::EdgeAlignedUp,
    );

    // When the IR receiver does not detect anything, it will output a high signal
    // When it detects a signal, it will output a low signal
    //
    // So, a "1" from our perspective (transmitter) is a low signal for the receiver.

    async fn set0(pwm: &mut SimplePwm<'_, TIM1>) {
        pwm.set_duty(embassy_stm32::timer::Channel::Ch1, 0);
    }

    async fn set1(pwm: &mut SimplePwm<'_, TIM1>) {
        let max = pwm.get_max_duty();
        pwm.set_duty(embassy_stm32::timer::Channel::Ch1, max / 2);
    }

    // Send a `0` bit: short low pulse, short high pulse
    async fn send0(pwm: &mut SimplePwm<'_, TIM1>, chars: &TimingCharistics) {
        set1(pwm).await;
        Timer::after_micros(chars.bit_low.into()).await;
        set0(pwm).await;
        Timer::after_micros(chars.bit_high_zero.into()).await;
    }

    // Send a `0` bit: short low pulse, long high pulse
    async fn send1(pwm: &mut SimplePwm<'_, TIM1>, chars: &TimingCharistics) {
        set1(pwm).await;
        Timer::after_micros(chars.bit_low.into()).await;
        set0(pwm).await;
        Timer::after_micros(chars.bit_high_one.into()).await;
    }

    set0(&mut pwm).await;
    pwm.enable(embassy_stm32::timer::Channel::Ch1);
    loop {
        let IrTransmitType { timing: char, data } = ins.rx.receive().await;

        join(set1(&mut pwm), Timer::after_micros(char.initial_low.into())).await;
        join(
            set0(&mut pwm),
            Timer::after_micros(char.initial_high.into()),
        )
        .await;

        for _ in 0..1 {
            let range = if data < 0x0001_0000 { 16 } else { 32 };
            for i in (0..range).rev() {
                let bit = (data >> i) & 1;
                if bit == 1 {
                    send1(&mut pwm, &char).await;
                } else {
                    send0(&mut pwm, &char).await;
                }
            }

            send0(&mut pwm, &char).await;
            Timer::after_millis(20).await;
        }

        Timer::after_millis(50).await;
    }
}

#[embassy_executor::task]
async fn normal_prio_loop(ins: InputIRLoop) {
    debug!("Running IR Program");
    let led_in = Input::new(ins.pin, Pull::None);
    let mut led_in = ExtiInput::new(led_in, ins.exti);

    //let tim = ins.ir_timer_a;

    let mut t2 = Instant::now();
    loop {
        led_in.wait_for_low().await;
        let t1 = Instant::now();
        let time_high = t1 - t2;
        led_in.wait_for_high().await;
        t2 = Instant::now();
        let time_low = t2 - t1;

        let _ = ins.tx.try_send((time_high, time_low));
    }
}

struct InputMainLoop {
    cs_spi1: PeripheralRef<'static, PB6>,
    cs_spi2: PeripheralRef<'static, PC4>,

    i2c_channel: PeripheralRef<'static, I2C1>,
    i2c_sda: PeripheralRef<'static, PB9>,
    i2c_scl: PeripheralRef<'static, PB8>,
    i2c_dma_tx: PeripheralRef<'static, DMA1_CH7>,
    //i2c_dma_rx: PeripheralRef<'static, DMA1_CH5>,

    // let spi = embassy_stm32::spi::Spi::new(p.SPI1, p.PB3, p.PA7, p.PA6, p.DMA2_CH3, p.DMA2_CH0, cfg);
    spi_channel: PeripheralRef<'static, SPI1>,
    spi_sck: PeripheralRef<'static, PB3>,
    spi_mosi: PeripheralRef<'static, PA7>,
    spi_miso: PeripheralRef<'static, PA6>,
    spi_dma_tx: PeripheralRef<'static, DMA2_CH3>,
    spi_dma_rx: PeripheralRef<'static, DMA2_CH0>,

    uart: USart,

    onboard_led_pin: PeripheralRef<'static, PA5>,

    other_output_timer: PeripheralRef<'static, TIM3>,
    other_led_pin: PeripheralRef<'static, PB4>,

    button: PeripheralRef<'static, PC13>,

    rx: Receiver<'static, MUTEX, IrReceiveType, IR_RECEIVE_COUNT>,
    ir_transmit_tx: Sender<'static, MUTEX, IrTransmitType, IR_TRANSMIT_COUNT>,
}

struct USart {
    uart: PeripheralRef<'static, USART6>,
    rx: PeripheralRef<'static, PA12>,
    tx: PeripheralRef<'static, PA11>,
    dma_tx: PeripheralRef<'static, DMA2_CH6>,
    dma_rx: PeripheralRef<'static, DMA2_CH1>,
}

#[embassy_executor::task]
async fn idle_prio_loop(pin: AnyPin) {
    debug!("Running Idle Program");
    let mut led = Output::new(pin, Level::Low, Speed::High);

    loop {
        led.set_high();
        Timer::after_millis(10).await;
        led.set_low();
        Timer::after_millis(985).await;
    }
}

#[embassy_executor::task]
async fn low_prio_loop(ins: InputMainLoop) {
    debug!("Running main Program");
    let cs_icm20948 = Output::new(ins.cs_spi1, Level::High, Speed::High);

    let shared_uart_bus = {
        let mut cfg = embassy_stm32::usart::Config::default();
        cfg.baudrate = 9600;

        let mut uart_tx = embassy_stm32::usart::Uart::new(
            ins.uart.uart,
            ins.uart.rx,
            ins.uart.tx,
            Irqs,
            ins.uart.dma_tx,
            ins.uart.dma_rx,
            cfg,
        ).unwrap();

        //let gps = adafruit_gps::Gps::new(port, baud_rate)

        info!("waiting for uart");
        loop {
            let mut buf = [0; 256];
            let len = uart_tx.read_until_idle(&mut buf).await.unwrap();
            let buf = &buf[..len];
            let buf = core::str::from_utf8(buf).unwrap();
            info!("UART data len: {}\n{}", len, buf);
        }
        return;
    };

    // As long as we use DMA, we can issue the messages in the low prio loop
    let shared_i2c_bus = {
        let spd = Hertz::hz(250_000);
        let mut cfg = embassy_stm32::i2c::Config::default();
        cfg.sda_pullup = true;
        cfg.scl_pullup = true;

        let i2c = embassy_stm32::i2c::I2c::new(
            ins.i2c_channel,
            ins.i2c_scl,
            ins.i2c_sda,
            Irqs,
            NoDma,
            //ins.i2c_dma_tx,
            NoDma,
            //ins.i2c_dma_rx,
            spd,
            cfg,
        );
        embassy_sync::mutex::Mutex::<CriticalSectionRawMutex, _>::new(i2c)
    };

    /*

    let shared_spi_bus = {
        let mut cfg = embassy_stm32::spi::Config::default();
        cfg.frequency = Hertz::hz(1_000_000); // up to 7mhz
        cfg.bit_order = embassy_stm32::spi::BitOrder::MsbFirst;
        cfg.mode = embassy_stm32::spi::MODE_0;
        let spi = embassy_stm32::spi::Spi::new(
            ins.spi_channel,
            ins.spi_sck,
            ins.spi_mosi,
            ins.spi_miso,
            ins.spi_dma_tx,
            ins.spi_dma_rx,
            cfg,
        );
        embassy_sync::mutex::Mutex::<CriticalSectionRawMutex, _>::new(spi)
    };

    let _test = {
        let mut bus = I2cDevice::new(&shared_i2c_bus);
        const MAX_DEVICES: usize = 128;
        let mut connected = heapless::Vec::<u8, MAX_DEVICES>::new();
        for i in 0..128 {
            let r = bus.write(i, &[0x00]).await;
            if r.is_ok() {
                connected
                    .push(i)
                    .expect("Sorry, too many I2C devices conncted");
            }
        }

        info!("Connected devices (hex): {:x}", connected);
    };

    // BME280 example
    let get_sensor_bme = async {
        let bus = I2cDevice::new(&shared_i2c_bus);
        let mut bme = AsyncBME280::new_secondary(bus);
        let bme_init_res = bme.init(&mut Delay).await;
        if bme_init_res.is_err() {
            error!("BME280 init failed!");
            return None;
        }

        let m = bme.measure(&mut Delay).await.unwrap();
        let temp_f = m.temperature * 9.0 / 5.0 + 32.0;
        warn!(
            "BME: \n\tpressure: {}\n\ttemp: {}C ({}f)\n\thumid: {}",
            m.pressure, m.temperature, temp_f, m.humidity
        );

        Some(bme)
    };

    // ICM20948 example
    let get_sensor_imu_i2c = async {
        let bus = I2cDevice::new(&shared_i2c_bus);
        match Icm20948::new_i2c(bus, Delay)
            .gyr_unit(GyrUnit::Dps)
            .gyr_dlp(icm20948_async::GyrDlp::Hz24)
            .acc_range(AccRange::Gs8)
            .set_address(0x69)
            .initialize_9dof()
            .await
        {
            Ok(mut icm) => {
                let stuff = icm.read_9dof().await.unwrap();
                warn!(
                    "ICM20948: \n acc: {} {} {}\n gyr: {} {} {}\n mag: {} {} {}",
                    stuff.acc.x,
                    stuff.acc.y,
                    stuff.acc.z,
                    stuff.gyr.x,
                    stuff.gyr.y,
                    stuff.gyr.z,
                    stuff.mag.x,
                    stuff.mag.y,
                    stuff.mag.z
                );
                Some(icm)
            }
            Err(_) => {
                error!("ICM20948 i2c init failed!");
                None
            }
        }
    };

    //let cs_icm20948 = Output::new(p.PA3, Level::High, Speed::High);
    let imu_bus =
        embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice::new(&shared_spi_bus, cs_icm20948);
    let get_sensor_imu_spi = async {
        match Icm20948::new_spi(imu_bus, Delay)
            .gyr_unit(GyrUnit::Dps)
            .gyr_dlp(icm20948_async::GyrDlp::Hz24)
            .acc_range(AccRange::Gs8)
            .initialize_6dof()
            .await
        {
            Ok(mut icm) => {
                let stuff = icm.read_6dof().await.unwrap();
                warn!(
                    "ICM20948 spi: \n acc: {} {} {}\n gyr: {} {} {}\n",
                    stuff.acc.x, stuff.acc.y, stuff.acc.z, stuff.gyr.x, stuff.gyr.y, stuff.gyr.z
                ); //, stuff.mag.x, stuff.mag.y, stuff.mag.z);
                Some(icm)
            }
            Err(e) => {
                match e {
                    icm20948_async::IcmError::BusError(_e) => {
                        error!("ICM20948 spi init failed: Bus error")
                    }
                    icm20948_async::IcmError::ImuSetupError => {
                        error!("ICM20948 spi init failed: IMU setup error")
                    }
                    icm20948_async::IcmError::MagSetupError => {
                        error!("ICM20948 spi init failed: MAG setup error")
                    }
                };
                None
            }
        }
    };

    let get_sensor_6dof = async {
        let mut bus = I2cDevice::new(&shared_i2c_bus);
        let mut sensor = ism330dhcx::Ism330Dhcx::new_with_address(&mut bus, 0x6b)
            .await
            .unwrap();

        // Initializing sensor
        // =======================================

        let i2c = &mut bus;

        // =======================================
        // CTRL3_C

        sensor.ctrl3c.set_boot(i2c, true).await.unwrap();
        sensor.ctrl3c.set_bdu(i2c, true).await.unwrap();
        sensor.ctrl3c.set_if_inc(i2c, true).await.unwrap();

        // =======================================
        // CTRL9_XL

        sensor.ctrl9xl.set_den_x(i2c, true).await.unwrap();
        sensor.ctrl9xl.set_den_y(i2c, true).await.unwrap();
        sensor.ctrl9xl.set_den_z(i2c, true).await.unwrap();
        sensor.ctrl9xl.set_device_conf(i2c, true).await.unwrap();

        // =======================================
        // CTRL1_XL

        sensor
            .ctrl1xl
            .set_accelerometer_data_rate(i2c, ism330dhcx::ctrl1xl::Odr_Xl::Hz52)
            .await
            .unwrap();

        sensor
            .ctrl1xl
            .set_chain_full_scale(i2c, ism330dhcx::ctrl1xl::Fs_Xl::G4)
            .await
            .unwrap();
        sensor.ctrl1xl.set_lpf2_xl_en(i2c, true).await.unwrap();

        // =======================================
        // CTRL2_G

        sensor
            .ctrl2g
            .set_gyroscope_data_rate(i2c, ism330dhcx::ctrl2g::Odr::Hz52)
            .await
            .unwrap();

        sensor
            .ctrl2g
            .set_chain_full_scale(i2c, ism330dhcx::ctrl2g::Fs::Dps500)
            .await
            .unwrap();

        // =======================================
        // CTRL7_G

        sensor.ctrl7g.set_g_hm_mode(i2c, true).await.unwrap();

        let mut fusion = datafusion_imu::Fusion::new(0.05, 20.0, 50);
        fusion.set_mode(datafusion_imu::Mode::Dof6);

        let (temp, gyr, acc) = (
            sensor.get_temperature(i2c).await.unwrap(),
            sensor.get_gyroscope(i2c).await.unwrap().as_dps(),
            sensor.get_accelerometer(i2c).await.unwrap().as_m_ss(),
        );

        fusion.set_data_dof6(acc[0] as f32, acc[1] as f32, acc[2] as f32, gyr[0] as f32, gyr[1] as f32, gyr[2] as f32);
        fusion.init();

        let mut now = Instant::now();
        // washington DC 2024
        fusion.set_declination(-10.45);
        loop {

            let (temp, gyr, acc) = (
                sensor.get_temperature(i2c).await.unwrap(),
                sensor.get_gyroscope(i2c).await.unwrap().as_dps(),
                sensor.get_accelerometer(i2c).await.unwrap().as_m_ss(),
            );

            let dt = now.elapsed().as_micros() as f32 / 1_000_000.0;
            now = Instant::now();

            fusion.set_data_dof6(acc[0] as f32, acc[1] as f32, acc[2] as f32, gyr[0] as f32, gyr[1] as f32, gyr[2] as f32);
            fusion.step(dt);

            let angle_x = fusion.get_x_angle();
            let angle_y = fusion.get_y_angle();
            let angle_z = fusion.get_z_angle();
            let dist = fusion.get_final_distance();

            defmt::info!(
                "ism330dhcx:
    Temperature: {}C
    Accelerometer: {:?}
    Gyroscope: {:?}
    Fusion: ({}°, {}°, {}°) {}cm",
                temp, acc, gyr,
                angle_x, angle_y, angle_z,
                dist
            );


            Timer::after_millis(50000).await;
        }
    };

    let normal_out = async {
        let mut pwm_a = embassy_stm32::timer::simple_pwm::SimplePwm::new(
            ins.other_output_timer,
            Some(PwmPin::new_ch1(
                ins.other_led_pin,
                embassy_stm32::gpio::OutputType::OpenDrain,
            )),
            None,
            None,
            None,
            Hertz::hz(100),
            embassy_stm32::timer::CountingMode::EdgeAlignedUp,
        );

        let max = pwm_a.get_max_duty();
        let chan = embassy_stm32::timer::Channel::Ch1;
        pwm_a.set_duty(chan, max);
        pwm_a.enable(chan);
        let mut f = 0.0;
        loop {
            for _ in 0..100 {
                f += 0.03;
                let loc = (f.sin() + 1.0) / 2.0;
                let loc = loc * loc * loc * loc;

                let loc = loc * 0.4 + 0.01;
                pwm_a.set_duty(chan, ((max as f32) * loc) as u16);
                Timer::after_millis(50).await;
            }

            // pwm_a.set_duty(chan, max);
            // Timer::after_millis(15).await;
            // pwm_a.set_duty(chan, 0);
            // Timer::after_millis(0).await;
            // pwm_a.set_duty(chan, max / 15);
            // Timer::after_millis(10).await;
        }
    };

    let button = async {
        let button = embassy_stm32::gpio::Input::new(ins.button, Pull::Up);
        loop {
            // we use a pullup resistor, so the button is active low
            while button.is_high() {
                Timer::after_millis(10).await;
                //Timer::after_millis(1000).await;
            }
            info!("Button pressed!");

            //let data: u32 = 0b0000_0000_0000_0000_1100_0000_0111_1000; // vol up
            ins.ir_transmit_tx
                .send(IrTransmitType {
                    timing: TimingCharistics {
                        initial_low: 9000,
                        initial_high: 4500,

                        bit_low: 630,
                        bit_high_zero: 530,
                        bit_high_one: 1641,
                    },
                    data: !0b1110_0110_0000_1001_0110_0111_1001_1000,
                })
                .await;

            while button.is_low() {
                Timer::after_millis(10).await;
            }

            info!("Button released!");
        }
    };

    let prints = async {
        let mut onboard_led =
            embassy_stm32::gpio::Output::new(ins.onboard_led_pin, Level::Low, Speed::Low);
        loop {
            //println!("new_ticks {}, {:?}", read.len(), read.get(0..20));
            let mut ir_recv_buf = heapless::Vec::<u32, 2048>::new();
            let i = 0;
            //let mut start = Instant::now();
            loop {
                if ir_recv_buf.is_full() {
                    break;
                }

                match select(ins.rx.receive(), Timer::after_millis(10)).await {
                    // We got a IR packet
                    Either::First(a) => {
                        ir_recv_buf.push(a.0.as_micros() as u32).unwrap();
                        ir_recv_buf.push(a.1.as_micros() as u32).unwrap();
                    }
                    // Timeout
                    Either::Second(_) => {
                        if ir_recv_buf.len() > 0 {
                            break;
                        } else {
                            ir_recv_buf.clear();
                            Timer::after_millis(250).await;
                        }
                    }
                }
            }
            onboard_led.set_high();
            //let time = Instant::now().duration_since(start);
            info!("ticks: {}", ir_recv_buf.len());

            let mut totals = TimingCharistics::default();
            let mut counts = TimingCharistics::default();

            if ir_recv_buf.len() < 5 {
                continue;
            }
            let _first = ir_recv_buf[0];
            let inital_low = ir_recv_buf[1];
            let initial_high = ir_recv_buf[2];

            let mut last_high_time = 0;

            let mut has_discerned = false;
            let mut unknown_total = 0;
            let mut unknown_count = 0;

            let chunk_iter = ir_recv_buf[3..].chunks_exact(2);

            let mut total_low_time = 0;
            let total_low_count = chunk_iter.len();

            // this is the actual data coming out of the IR remote in the form of a bitstream
            let mut data = [false; 32];
            for (i, x) in chunk_iter.enumerate() {
                let low_time = x[0];
                let high_time = x[1];

                total_low_time += low_time;

                if last_high_time == 0 {
                    last_high_time = high_time;
                } else if !has_discerned {
                    let diff = high_time.abs_diff(last_high_time);
                    if diff < 500 {
                        unknown_total += high_time;
                        unknown_count += 1;
                    } else {
                        has_discerned = true;
                        if high_time > last_high_time {
                            totals.bit_high_zero = unknown_total;
                            counts.bit_high_zero = unknown_count;
                            totals.bit_high_one = high_time;
                            counts.bit_high_one = 1;
                        } else {
                            totals.bit_high_one = unknown_total;
                            counts.bit_high_one = unknown_count;
                            totals.bit_high_zero = high_time;
                            counts.bit_high_zero = 1;
                            for k in 0..(unknown_count as usize) {
                                data[k] = true;
                            }
                        }
                    }
                } else {
                    let diff_one =
                        (totals.bit_high_one / counts.bit_high_one.max(1)).abs_diff(high_time);
                    let diff_zero =
                        (totals.bit_high_zero / counts.bit_high_zero.max(1)).abs_diff(high_time);

                    if diff_one < diff_zero {
                        totals.bit_high_one += high_time;
                        counts.bit_high_one += 1;
                        data[i] = true;
                    } else {
                        totals.bit_high_zero += high_time;
                        counts.bit_high_zero += 1;
                    }
                }
            }

            let final_char = TimingCharistics {
                initial_high: inital_low,
                initial_low: initial_high,

                bit_low: total_low_time / (total_low_count as u32).max(1),
                bit_high_zero: totals.bit_high_zero / counts.bit_high_zero.max(1),
                bit_high_one: totals.bit_high_one / counts.bit_high_one.max(1),
            };

            debug!("ticks: {:?} {:b}", final_char, data);

            Timer::after_millis(100).await;
            onboard_led.set_low();
        }
    };

    // Blinking LED example
    //println!("Starting blinking program");
    //_spawner.spawn(blink(p.PA5.degrade())).unwrap();

    //let bme = get_sensor_bme.await;
    //let imu = get_sensor_imu_i2c.await;
    //let imu_spi = get_sensor_imu_spi.await;

    //Timer::after_millis(100).await;

    join5(button, prints, get_sensor_bme, get_sensor_6dof, normal_out).await;
    //servo.await;
    //let ptr = shared_spi_bus.lock().await;
    */
}
