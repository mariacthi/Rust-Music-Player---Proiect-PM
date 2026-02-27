// COD DE HELLO WORLD SA VADA PICO-UL
// // +---------------------------------------------------------------------------+
// // |                             PM/MA lab skel                                |
// // +---------------------------------------------------------------------------+

// //! By default, this app prints a "Hello world" message with `defmt`.

// #![no_std]
// #![no_main]

// use embassy_executor::Spawner;
// // use embassy_net::StackResources;
// use embassy_time::{Duration, Timer};
// //use static_cell::StaticCell;
// use {defmt_rtt as _, panic_probe as _};

// // Use the logging macros provided by defmt.
// use defmt::*;

// // Import interrupts definition module
// mod irqs;

// // const SOCK: usize = 4;
// // static RESOURCES: StaticCell<StackResources<SOCK>> = StaticCell::<StackResources<SOCK>>::new();

// #[embassy_executor::main]
// async fn main(_spawner: Spawner) {
//     // Get a handle to the RP's peripherals.
//     //let peripherals = embassy_rp::init(Default::default());


//     // Default config for dynamic IP address
//     //let config = embassy_net::Config::dhcpv4(Default::default());


//     info!("Hello world!");

//     let delay = Duration::from_secs(1);
//     loop {
//         Timer::after(delay).await;
//     }
// }




// // DISPLAY
// #![no_std]
// #![no_main]

// use defmt::*;
// use embassy_executor::Spawner;
// use embassy_rp::{
//     bind_interrupts,
//     i2c::{I2c, InterruptHandler, Config as I2cConfig},
//     peripherals::I2C0,
// };
// use embassy_time::{Duration, Timer};
// use embedded_hal_async::i2c::I2c as AsyncI2c;
// use {defmt_rtt as _, panic_probe as _};

// bind_interrupts!(struct Irqs {
//     I2C0_IRQ => InterruptHandler<I2C0>;
// });

// const OLED_ADDR: u8 = 0x3C;

// #[embassy_executor::main]
// async fn main(_spawner: Spawner) {

//     let p = embassy_rp::init(Default::default());

//     let mut cfg = I2cConfig::default();
//     cfg.frequency = 400_000;
//     let mut i2c = I2c::new_async(p.I2C0, p.PIN_5, p.PIN_4, Irqs, cfg);

//     const INIT_CMDS: &[u8] = &[
//         0xAE,       // display off
//         0x20, 0x00, // memory mode = horizontal
//         0xA1,       // segment remap
//         0xC8,       // COM scan dec
//         0x81, 0x7F, // contrast
//         0xA6,       // normal display
//         0xA8, 0x3F, // multiplex = 64
//         0xD3, 0x00, // display offset
//         0xD5, 0x80, // clock divide
//         0xD9, 0xF1, // pre‑charge
//         0xDA, 0x12, // com pins
//         0xDB, 0x40, // vcom detect
//         0x8D, 0x14, // charge pump on
//         0xAF,       // display on
//     ];

//     send_cmds(&mut i2c, INIT_CMDS).await.unwrap();

//  const HELLO_WORLD: [u8; 60] = [
//     // H
//     0x7E, 0x10, 0x10, 0x10, 0x7E,
//     // e
//     0x38, 0x54, 0x54, 0x54, 0x18,
//     // l
//     0x00, 0x24, 0x7E, 0x40, 0x00,
//     // l
//     0x00, 0x24, 0x7E, 0x40, 0x00,
//     // o
//     0x38, 0x44, 0x44, 0x44, 0x38,
//     // spațiu
//     0x00, 0x00, 0x00, 0x00, 0x00,
//     // w
//     0x7C, 0x02, 0x0C, 0x02, 0x7C,
//     // o
//     0x38, 0x44, 0x44, 0x44, 0x38,
//     // r
//     0x7C, 0x08, 0x04, 0x04, 0x08,
//     // l
//     0x00, 0x24, 0x7E, 0x40, 0x00,
//     // d
//     0x38, 0x44, 0x44, 0x44, 0x78,
//     // !
//     0x00,0x00,0x5F,0x00,0x00,
// ];


//     send_cmds(&mut i2c, &[0x21, 0, HELLO_WORLD.len() as u8 - 1,
//                       0x22, 0, 0]).await.unwrap();
//    let mut data = [0u8; 1 + 60];
//     data[0] = 0x40;
//     data[1..].copy_from_slice(&HELLO_WORLD);
//     i2c.write(OLED_ADDR, &data).await.unwrap();

//     info!("Hello world afișat prin I²C async!");

//     loop {
//         Timer::after(Duration::from_secs(1)).await;
//     }
// }

// async fn send_cmds<I: AsyncI2c<Error = E>, E>(
//     i2c: &mut I,
//     cmds: &[u8],
// ) -> Result<(), E> {
//     let mut buf: [u8; 2] = [0; 2];
//     for &c in cmds {
//         buf[0] = 0x00;
//         buf[1] = c;
//         i2c.write(OLED_ADDR, &buf).await?;
//     }
//     Ok(())
// }

// display - cod mai bun
// #![no_std]
// #![no_main]

// use defmt::*;
// use embassy_executor::Spawner;
// use embassy_rp::{
//     bind_interrupts,
//     i2c::{I2c, InterruptHandler, Config as I2cConfig},
//     peripherals::I2C0,
// };
// use embassy_time::{Duration, Timer};
// use {defmt_rtt as _, panic_probe as _};

// use ssd1306::{
//     mode::BufferedGraphicsMode,
//     prelude::*,
//     I2CDisplayInterface,
//     Ssd1306,
// };
// use embedded_graphics::{
//     mono_font::{ascii::FONT_10X20, MonoTextStyleBuilder},
//     pixelcolor::BinaryColor,
//     prelude::*,
//     primitives::Rectangle,
//     text::{Baseline, Text},
// };

// bind_interrupts!(struct Irqs {
//     I2C0_IRQ => InterruptHandler<I2C0>;
// });

// #[embassy_executor::main]
// async fn main(_spawner: Spawner) {
//     let p = embassy_rp::init(Default::default());

//     // configure I²C la 400 kHz
//     let mut cfg = I2cConfig::default();
//     cfg.frequency = 400_000;
//     let i2c = I2c::new_async(p.I2C0, p.PIN_5, p.PIN_4, Irqs, cfg);

//     // construim interfața I²C pentru driver
//     let interface = I2CDisplayInterface::new(i2c);

//     // inițializăm driver-ul în BufferedGraphicsMode
//     let mut display: Ssd1306<_, _, BufferedGraphicsMode<_>> = Ssd1306::new(
//         interface,
//         DisplaySize128x64,
//         DisplayRotation::Rotate0,
//     )
//     .into_buffered_graphics_mode();

//     // dacă vrei suport async pentru init(), în Cargo.toml:
//     // ssd1306 = { version = "0.10", default-features = false, features = ["async"] }
//     display.init().unwrap();

//     // curățăm ecranul
//     display.clear_buffer();

//     // // definim stilul textului
//     // let text_style = MonoTextStyleBuilder::new()
//     //     .font(&FONT_10X20)
//     //     .text_color(BinaryColor::On)
//     //     .build();

//     // // desenăm textul-dinamic
//     // let mesaj = "Hello World \n Orice cuvant merge!";
//     // Text::with_baseline(mesaj, Point::new(0, 0), text_style, Baseline::Top)
//     //     .draw(&mut display)
//     //     .unwrap();

// //    let font = &FONT_10X20;
// // let style = MonoTextStyleBuilder::new()
// //     .font(font)
// //     .text_color(BinaryColor::On)
// //     .build();

// // // şir cu salt de linie
// // let mesaj = "Linia 1: Salut\nLinia 2: București\nLinia 3: OLED!";
// // // înălţimea caracterului + un mic spaţiu între linii
// // let line_height = font.character_size.height as i32 + 2;

// // for (i, linie) in mesaj.lines().enumerate() {
// //     Text::with_baseline(
// //         linie,
// //         Point::new(0, i as i32 * line_height),
// //         style,
// //         Baseline::Top,
// //     )
// //     .draw(&mut display)
// //     .unwrap();
// // }


//     // // trimitem buffer-ul la OLED
//     // display.flush().unwrap();

//     // info!("Text afișat corect!") ;

//     // loop {
//     //     Timer::after(Duration::from_secs(1)).await;
//     // }
//     let font = &FONT_10X20;
//     let text_style = MonoTextStyleBuilder::new()
//         .font(font)
//         .text_color(BinaryColor::On)
//         .build();
//     let mesaj = "Hello World! Asta este o linie foarte lunga care nu incape!";
//     // calculăm lățimea aproximativă (monospace):
//     let char_w = font.character_size.width as i32;
//     let text_w = (mesaj.len() as i32) * char_w;
//     // poziția verticală (sus)
//     let y = 0;

//     // pornim dincolo de marginea din dreapta
//     let mut x = 128;

//     loop {
//         display.clear_buffer();
//         // desenăm textul la offset-ul curent
//         Text::with_baseline(
//             mesaj,
//             Point::new(x, y),
//             text_style,
//             Baseline::Top,
//         )
//         .draw(&mut display)
//         .unwrap();
//         display.flush().unwrap();

//         // mutăm cu un pixel la stânga
//         x -= 1;
//         // dacă a dispărut complet la stânga, resetăm
//         if x < -text_w {
//             x = 128;
//         }

//         // pauză de 50ms — ajustează ca să fie mai rapid/lent
//         Timer::after(Duration::from_millis(1)).await;
//     }
// }


// SENZOR

// #![no_std]
// #![no_main]

// use defmt::*;
// use embassy_executor::Spawner;
// use embassy_time::{Duration, Timer, Instant};
// use {defmt_rtt as _, panic_probe as _};

// use embassy_rp::gpio::{Input, Output, Pull, Level};

// #[embassy_executor::main]
// async fn main(_spawner: Spawner) {
//     let p = embassy_rp::init(Default::default());

//     // ↙ adaptează dacă vrei alți pini
//     let mut trig = Output::new(p.PIN_15, Level::Low); // GPIO2  → TRIG
//     let echo     = Input::new(p.PIN_14, Pull::None);  // GPIO3  ← ECHO (prin divizor)

//     loop {
//         // 1. impuls TRIG: HIGH 10 µs
//         trig.set_high();
//         Timer::after(Duration::from_micros(10)).await;
//         trig.set_low();

//         // 2. așteaptă frontul ascendent pe ECHO (timeout 25 ms ≈ 4 m)
//         let start_wait = Instant::now();
//         while echo.is_low() {
//             if Instant::now() - start_wait > Duration::from_millis(60) {
//                 warn!("No echo (timeout on rising edge)");
//                 continue; // sari la următoarea măsură
//             }
//         }

//         // 3. măsoară lungimea impulsului HIGH
//         let t0 = Instant::now();
//         while echo.is_high() {}          // buclă tight (durata max ~25 ms)
//         let us = (Instant::now() - t0).as_micros() as u32;

//         // 4. convertește: dist[cm] = t[µs] / 58
//         let cm = us / 58;
//         info!("Dist: {} cm  ({} µs)", cm, us);

//         Timer::after(Duration::from_millis(100)).await; // 10 Hz
//     }
// }

// SENZOR HC-SR04
// #![no_std]
// #![no_main]

// use defmt::*;
// use embassy_executor::Spawner;
// use embassy_time::{Duration, Timer, Instant};
// use {defmt_rtt as _, panic_probe as _};

// use embassy_rp::gpio::{Input, Output, Pull, Level};

// #[embassy_executor::main]
// async fn main(_spawner: Spawner) {
//     let p = embassy_rp::init(Default::default());

//     // TRIG: GP15, ECHO: GP14 cu pull-down intern
//     let mut trig = Output::new(p.PIN_20, Level::Low);
//     let mut echo = Input::new(p.PIN_21, Pull::None);

//     info!("Încep măsurători…");

//     'measure: loop {
//         // 0) așteaptă ca ECHO să fie LOW (stare idle)

//         echo.wait_for_low().await;
//         // mic delay pentru stabilitate (2 µs)
//         Timer::after(Duration::from_micros(2)).await;

//         // 1) Trimite un impuls de 10 µs pe TRIG
//         trig.set_high();
//         Timer::after(Duration::from_micros(10)).await;
//         trig.set_low();

//         // 2) Așteaptă front ascendent cu timeout 25 ms
//         let t_start = Instant::now();
//         echo.wait_for_high().await;
//         // while echo.is_low() {
//         //     if Instant::now() - t_start > Duration::from_millis(25) {
//         //         warn!("No echo (timeout front ascendent)");
//         //         // așteaptă puțin să nu spam-uie log-ul
//         //         Timer::after(Duration::from_millis(100)).await;
//         //         continue 'measure; // reintră în măsurătoare
//         //     }
//         // }

//         // 3) Măsoară durata semnalului HIGH
//         let t0 = Instant::now();
//         while echo.is_high() {}
//         let pulse_us = (Instant::now() - t0).as_micros() as f32;

//         // 4) Convertește în centimetri:
//         //     dist(cm) = pulse_time(µs) / 58.0
//         // (spec HC-SR04: ~58 µs per cm tur-retur)
//         let distance_cm = pulse_us / 58.0;

//         info!("Dist: {} cm ({} µs)", distance_cm, pulse_us as u32);

//         // 10 măsurători pe secundă
//         Timer::after(Duration::from_millis(1000)).await;
//     }
// }

//CARD SD
// #![no_std]
// #![no_main]

// use {defmt_rtt as _, panic_probe as _};
// use defmt::info;
// use core::fmt::Write;
// use embassy_executor::Spawner;
// use embassy_time::{Delay, Duration, Timer};

// use embassy_rp::{
//     bind_interrupts,
//     gpio::{Level, Output},
//     peripherals::SPI0,
//     spi::{self, Blocking, Spi},
// };
// use embedded_hal_1::spi::SpiDevice;
// use embassy_embedded_hal::shared_bus::blocking::spi::SpiDeviceWithConfig;
// use embassy_sync::blocking_mutex::{Mutex, raw::NoopRawMutex};
// use embedded_sdmmc::{SdCard, VolumeManager, Mode, TimeSource, Timestamp, VolumeIdx};
// use static_cell::StaticCell;

// bind_interrupts!(struct Irqs { /*  nothing  */ });

// struct DummyTime;
// impl TimeSource for DummyTime {
//     fn get_timestamp(&self) -> Timestamp {
//         Timestamp { year_since_1970: 54, zero_indexed_month: 0, zero_indexed_day: 1,
//                     hours: 0, minutes: 0, seconds: 0 }
//     }
// }

// static BUS: StaticCell<
//     Mutex<NoopRawMutex, core::cell::RefCell<Spi<'static, SPI0, Blocking>>>
// > = StaticCell::new();

// type SdCardDev = SdCard<
//     SpiDeviceWithConfig<
//         'static,
//         NoopRawMutex,
//         Spi<'static, SPI0, Blocking>,
//         Output<'static>,
//     >,
//     Delay,
// >;
// type FsMgr = VolumeManager<SdCardDev, DummyTime>;
// static FS: StaticCell<FsMgr> = StaticCell::new();

// #[embassy_executor::main]
// async fn main(_spawner: Spawner) {
//     let p = embassy_rp::init(Default::default());

//     let spi_cfg = spi::Config::default();
//     let spi_raw = Spi::new_blocking(p.SPI0, p.PIN_18, p.PIN_19, p.PIN_16, spi_cfg);
//     let bus = BUS.init(Mutex::new(core::cell::RefCell::new(spi_raw)));

//     let sd_cs = Output::new(p.PIN_17, Level::High);
//     let sd_spi_dev = {
//         let mut cfg = spi::Config::default();
//         cfg.frequency = 400_000;
//         SpiDeviceWithConfig::new(bus, sd_cs, cfg)
//     };

//     let sdcard = SdCard::new(sd_spi_dev, Delay);
//     let fs = FS.init(VolumeManager::new(sdcard, DummyTime {}));

//     match fs.device().num_bytes() {
//         Ok(sz) => defmt::info!("Card OK – {} bytes ({} MiB)", sz, sz / 1_048_576),
//         Err(e) => panic!("SD detect error: {:?}", e),
//     }

//     fs.device().spi(|spi_dev| {

//         let mut config = spi::Config::default();
//         config.frequency = 8_000_000;
//         config.phase = embassy_rp::spi::Phase::CaptureOnFirstTransition;
//         config.polarity = embassy_rp::spi::Polarity::IdleLow;

//         spi_dev.set_config(config);
//         info!("Successfully increased SD card speed to 8MHz");
//     });

//     let mut vol = fs.open_volume(VolumeIdx(0)).expect("open_volume");
//     let mut root = vol.open_root_dir().expect("open_root");

//     defmt::info!("Root dir:");

//     root.iterate_dir(|e| {
//         // let name_str = e.name.base_name();
//         // defmt::info!("  {}", name_str);
// 		let raw: &[u8] = &e.name.base_name();

// 		let end = raw.iter()
// 					.rposition(|&b| b != b' ')
// 					.map(|p| p + 1)          // poziția după ultimul char valid
// 					.unwrap_or(0);           // numai spații ⇒ nume gol

// 		if let Ok(txt) = core::str::from_utf8(&raw[..end]) {
// 			defmt::info!("  {}", txt);
// 		} else {
// 			defmt::warn!("  <nume nevalid>");
// 		}
//     });

//     defmt::info!("Test SD terminat");
//     loop { Timer::after(Duration::from_secs(1)).await; }
// }

// difuzor
// #![no_std]
// #![no_main]

// use defmt::*;
// use embassy_executor::Spawner;
// use embassy_rp::bind_interrupts;
// use embassy_rp::pio::{InterruptHandler, Pio};
// use embassy_rp::pio_programs::i2s::{PioI2sOut, PioI2sOutProgram};
// use embassy_time::{Timer, Duration};
// use micromath::F32Ext;
// use {defmt_rtt as _, panic_probe as _};

// bind_interrupts!(struct Irqs {
//     PIO0_IRQ_0 => InterruptHandler<embassy_rp::peripherals::PIO0>;
// });

// /// nota = 0,4 s; la 44 100 Hz => 17 640 esantioane stereo
// const MAX_SAMPLES: usize = 18_000;

// static mut AUDIO_BUF: [u32; MAX_SAMPLES] = [0; MAX_SAMPLES];

// #[embassy_executor::main]
// async fn main(_spawner: Spawner) {
//     info!("I2S demo 22 k / 44 k");

//     let p = embassy_rp::init(Default::default());
//     let Pio { mut common, sm0, .. } = Pio::new(p.PIO0, Irqs);

//     const SAMPLE_RATE: u32 = 44_100;        // sau 22_050


//     let program = PioI2sOutProgram::new(&mut common);
//     let mut i2s = PioI2sOut::new(
//         &mut common,
//         sm0,
//         p.DMA_CH0,
//         p.PIN_9,  // DIN
//         p.PIN_10, // BCLK
//         p.PIN_11, // LRCK
//         SAMPLE_RATE,
//         16, /* bits */
//         2,  /* stereo */
//         &program,
//     );



//     let melody = [
// 		261.63, 261.63, 392.00, 392.00, 440.00, 440.00, 392.00,
//         349.23, 349.23, 329.63, 329.63, 293.66, 293.66, 261.63,
//     ];
// 	Timer::after(Duration::from_millis(100)).await;

//     for &note in &melody {

//         let nsamp = genereaza_beep(note, 0.4, SAMPLE_RATE, 1.0);
//         i2s.write(unsafe { &AUDIO_BUF[..nsamp] }).await;
//         Timer::after(Duration::from_millis(60)).await;


// 	}

//     info!("End!");
//     loop { Timer::after(Duration::from_secs(1)).await; }
// }

// fn genereaza_beep(freq: f32, duration_s: f32, sr: u32, volume: f32) -> usize {
//     let nsamp = (duration_s * sr as f32) as usize;
//     let nsamp = nsamp.min(MAX_SAMPLES);               // max 18 000 samples

//     let amp = (i16::MAX as f32 * volume) as i16;
//     let fade = 200;                                   // 200 fade

//     unsafe {
//         for i in 0..nsamp {
//             // fade-in / fade-out
//             let env = if i < fade {
//                 i as f32 / fade as f32
//             } else if i > nsamp - fade {
//                 (nsamp - i) as f32 / fade as f32
//             } else {
//                 1.0
//             };
//             let t = i as f32 / sr as f32;
//             let val = (f32::sin(3.0 * core::f32::consts::PI * freq * t) * amp as f32 * env) as i16;
//             AUDIO_BUF[i] = ((val as u16 as u32) << 16) | (val as u16 as u32);
//         }
//     }
//     nsamp
// }


// card sd + play pe difuzor
// #![no_std]
// #![no_main]

// use {defmt_rtt as _, panic_probe as _};
// use defmt::info;
// use embassy_executor::Spawner;
// use embassy_time::{Delay, Duration, Timer};

// use embassy_rp::{
//     bind_interrupts,
//     gpio::{Level, Output},
//     pio::{InterruptHandler, Pio},
//     peripherals::SPI0,
//     spi::{self, Blocking, Spi},
// };
// use embassy_rp::pio_programs::i2s::{PioI2sOut, PioI2sOutProgram};

// use embedded_sdmmc::{SdCard, VolumeManager, Mode, TimeSource, Timestamp, VolumeIdx, File, Error, SdCardError};
// use embassy_embedded_hal::shared_bus::blocking::spi::SpiDeviceWithConfig;
// use embassy_sync::blocking_mutex::{Mutex, raw::NoopRawMutex};
// use static_cell::StaticCell;

// // Map PIO interrupt for I2S
// bind_interrupts!(struct Irqs {
//     PIO0_IRQ_0 => InterruptHandler<embassy_rp::peripherals::PIO0>;
// });

// // Dummy timestamp for FAT
// struct DummyTime;
// impl TimeSource for DummyTime {
//     fn get_timestamp(&self) -> Timestamp {
//         Timestamp {
//             year_since_1970: 54,
//             zero_indexed_month: 0,
//             zero_indexed_day: 1,
//             hours: 0,
//             minutes: 0,
//             seconds: 0,
//         }
//     }
// }

// // Shared SPI bus for SD
// static BUS: StaticCell<Mutex<NoopRawMutex, core::cell::RefCell<Spi<'static, SPI0, Blocking>>>> = StaticCell::new();

// type SdCardDev = SdCard<
//     SpiDeviceWithConfig<'static, NoopRawMutex, Spi<'static, SPI0, Blocking>, Output<'static>>,
//     Delay,
// >;
// type FsMgr = VolumeManager<SdCardDev, DummyTime>;
// static FS: StaticCell<FsMgr> = StaticCell::new();

// // DMA buffer: 1024 stereo frames (4 KiB)
// const CHUNK_FRAMES: usize = 1024;
// static mut AUDIO_BUF: [u32; CHUNK_FRAMES * 2] = [0; CHUNK_FRAMES * 2];

// #[embassy_executor::main]
// async fn main(_spawner: Spawner) {
//     info!("WAV player SD+I2S demo");

//     // Init peripherals
//     let p = embassy_rp::init(Default::default());

//     // --- I2S via PIO ---
//     let Pio { mut common, sm0, .. } = Pio::new(p.PIO0, Irqs);
//     let program = PioI2sOutProgram::new(&mut common);
//     let mut i2s = PioI2sOut::new(
//         &mut common, sm0, p.DMA_CH0,
//         p.PIN_9, p.PIN_10, p.PIN_11,
//         44_100, 16, 2, &program,
//     );

//     // --- SD card setup over SPI ---
//     let spi_cfg = spi::Config::default();
//     let spi_raw = Spi::new_blocking(p.SPI0, p.PIN_18, p.PIN_19, p.PIN_16, spi_cfg);
//     let bus = BUS.init(Mutex::new(core::cell::RefCell::new(spi_raw)));
//     let sd_cs = Output::new(p.PIN_17, Level::High);
//     let mut sd_spi_cfg = spi::Config::default();
//     sd_spi_cfg.frequency = 400_000;
//     let sd_spi_dev = SpiDeviceWithConfig::new(bus, sd_cs, sd_spi_cfg);

//     let sdcard = SdCard::new(sd_spi_dev, Delay);
//     let mut fs = FS.init(VolumeManager::new(sdcard, DummyTime {}));

//     // Check card and speed up
//     let sz = loop {
//         match fs.device().num_bytes() {
//             Ok(sz) => break sz,
//             Err(e) => {
//                 defmt::warn!("Eroare SD init: {:?}, retry...", e);
//             }
//         }
//         Timer::after(Duration::from_millis(100)).await;
//     };
//     defmt::info!("Card OK – {} MiB", sz / 1_048_576);
//     // let sz = fs.device().num_bytes().unwrap();
//     // info!("Card OK – {} MiB", sz / 1_048_576);

//     fs.device().spi(|dev| {
//         let mut fast = spi::Config::default();
//         fast.frequency = 8_000_000;
//         fast.phase = embassy_rp::spi::Phase::CaptureOnFirstTransition;
//         fast.polarity = embassy_rp::spi::Polarity::IdleLow;
//         dev.set_config(fast);
//         info!("SD speed ↑ to 8 MHz");
//     });

//     // Mount and list root directory
//     let mut vol = fs.open_volume(VolumeIdx(0)).unwrap();
//     let mut root = vol.open_root_dir().unwrap();
//     info!("Root dir:");
//     root.iterate_dir(|e| {
//         let raw: &[u8] = &e.name.base_name();
//         let end = raw.iter().rposition(|&b| b != b' ').map(|p| p + 1).unwrap_or(0);
//         if let Ok(txt) = core::str::from_utf8(&raw[..end]) {
//             info!("  {}", txt);
//         } else {
//             defmt::warn!("<invalid>");
//         }
//     });

//         // Open WAV file directly from root directory
//     let mut wav = root.open_file_in_dir("N&0024~7.wav", Mode::ReadOnly).unwrap();
//     // Skip WAV header
//     wav.seek_from_start(44).unwrap();

//     // Short delay before playback
//     Timer::after(Duration::from_millis(100)).await;

//     const VOLUME: f32 = 1.0;
//     let mut read_buf = [0u8; CHUNK_FRAMES * 4];
//     loop {
//         let n = wav.read(&mut read_buf).unwrap(); // ([docs.rs](https://docs.rs/embedded-sdmmc/latest/x86_64-pc-windows-msvc/embedded_sdmmc/struct.File.html))
//         if n == 0 { break; }
//         let frames = n / 4;
//         unsafe {
//             for i in 0..frames {
//                 let base = i * 4;
//                 // let l = i16::from_le_bytes([read_buf[base], read_buf[base + 1]]);
//                 // let r = i16::from_le_bytes([read_buf[base + 2], read_buf[base + 3]]);

//                 // AUDIO_BUF[i] = ((l as u16 as u32) << 16) | (r as u16 as u32);
//                 let l_raw = i16::from_le_bytes([read_buf[base], read_buf[base + 1]]);
//                 let r_raw = i16::from_le_bytes([read_buf[base + 2], read_buf[base + 3]]);

//                 // let l = ((l_raw as i32 * VOLUME_FP) >> FIXED_POINT_SHIFT) as i16;
//                 // let r = ((r_raw as i32 * VOLUME_FP) >> FIXED_POINT_SHIFT) as i16;
//                 let l = (l_raw as f32 * VOLUME) as i16;
//                 let r = (r_raw as f32 * VOLUME) as i16;
//                 // sign-extend + împachetare corectă
//                 let l_u = (l as i32 as u32) << 16;
//                 let r_u = (r as i32 as u32) & 0xFFFF;
//                 AUDIO_BUF[i] = l_u | r_u;

//                 unsafe {
//                     AUDIO_BUF[i] = ((l as u16 as u32) << 16) | (r as u16 as u32);
//                 }
//             }
//         }
//         i2s.write(unsafe { &AUDIO_BUF[..frames] }).await;
//     }

//     info!("Playback done");
//     loop { Timer::after(Duration::from_secs(1)).await; }
// }

// CITIT DE PE CARD SD + PLAY LA DIFUZOR LA O PIESA + AFISARE PE DISPLAY
// #![no_std]
// #![no_main]

// use {defmt_rtt as _, panic_probe as _};
// use defmt::info;
// use embassy_executor::Spawner;
// use embassy_time::{Delay, Duration, Timer};

// use embassy_rp::{
//     bind_interrupts,
//     gpio::{Level, Output},
//     i2c::{I2c, InterruptHandler as I2cIrq, Config as I2cConfig, Async as I2cAsync},
//     pio::{InterruptHandler as PioIrq, Pio},
//     peripherals::{I2C0, SPI0},
//     spi::{self, Blocking, Spi},
// };
// use embassy_rp::pio_programs::i2s::{PioI2sOut, PioI2sOutProgram};

// use embedded_sdmmc::{SdCard, VolumeManager, Mode, TimeSource, Timestamp, VolumeIdx};
// use embassy_embedded_hal::shared_bus::blocking::spi::SpiDeviceWithConfig;
// use embassy_sync::blocking_mutex::{Mutex, raw::NoopRawMutex};
// use static_cell::StaticCell;
// use ssd1306::{
//     mode::BufferedGraphicsMode,
//     prelude::*,
//     I2CDisplayInterface,
//     Ssd1306,
// };

// use embedded_graphics::{
//     mono_font::{ascii::FONT_10X20, MonoTextStyleBuilder},
//     pixelcolor::BinaryColor,
//     prelude::*,
//     text::{Baseline, Text},
// };

// // Map PIO interrupt for I2S
// bind_interrupts!(struct Irqs {
//     PIO0_IRQ_0 => PioIrq<embassy_rp::peripherals::PIO0>;
// });

// bind_interrupts!(struct I2cIrqs {
//     I2C0_IRQ => I2cIrq<I2C0>;
// });
// // Dummy timestamp for FAT
// struct DummyTime;
// impl TimeSource for DummyTime {
//     fn get_timestamp(&self) -> Timestamp {
//         Timestamp {
//             year_since_1970: 54,
//             zero_indexed_month: 0,
//             zero_indexed_day: 1,
//             hours: 0,
//             minutes: 0,
//             seconds: 0,
//         }
//     }
// }

// // Shared SPI bus for SD
// static BUS: StaticCell<Mutex<NoopRawMutex, core::cell::RefCell<Spi<'static, SPI0, Blocking>>>> = StaticCell::new();

// type SdCardDev = SdCard<
//     SpiDeviceWithConfig<'static, NoopRawMutex, Spi<'static, SPI0, Blocking>, Output<'static>>,
//     Delay,
// >;
// type FsMgr = VolumeManager<SdCardDev, DummyTime>;
// static FS_MGR: StaticCell<FsMgr> = StaticCell::new();

// // DMA buffer: 1024 stereo frames (4 KiB)
// const CHUNK_FRAMES: usize = 1024;
// static mut AUDIO_BUF: [u32; CHUNK_FRAMES * 2] = [0; CHUNK_FRAMES * 2];

// #[embassy_executor::task]
// async fn display_task(  i2c: I2c<'static, I2C0, I2cAsync>,
//     mesaj: &'static str)
// {
//     let interface = I2CDisplayInterface::new(i2c);
//     // inițializăm driver-ul în BufferedGraphicsMode
//     let mut display: Ssd1306<_, _, BufferedGraphicsMode<_>> = Ssd1306::new(
//         interface,
//         DisplaySize128x64,
//         DisplayRotation::Rotate0,
//     )
//     .into_buffered_graphics_mode();
//     display.init().unwrap();

//     // curățăm ecranul
//     display.clear_buffer();


//     let font = &FONT_10X20;
//     let text_style = MonoTextStyleBuilder::new()
//         .font(font)
//         .text_color(BinaryColor::On)
//         .build();

//     // calculăm lățimea aproximativă (monospace):
//     let char_w = font.character_size.width as i32;
//     let text_w = (mesaj.len() as i32) * char_w;
//     // poziția verticală (sus)
//     let y = 0;

//     // pornim dincolo de marginea din dreapta
//     let mut x = 128;

//     loop {
//         display.clear_buffer();
//         // desenăm textul la offset-ul curent
//         Text::with_baseline(
//             mesaj,
//             Point::new(x, y),
//             text_style,
//             Baseline::Top,
//         )
//         .draw(&mut display)
//         .unwrap();
//         display.flush().unwrap();

//         // mutăm cu un pixel la stânga
//         x -= 1;
//         // dacă a dispărut complet la stânga, resetăm
//         if x < -text_w {
//             x = 128;
//         }

//         // pauză de 50ms — ajustează ca să fie mai rapid/lent
//         Timer::after(Duration::from_millis(1)).await;
//     }
// }

// #[embassy_executor::main]
// async fn main(spawner: Spawner) {
//     info!("WAV player SD+I2S demo");

//     // Init peripherals
//     let p = embassy_rp::init(Default::default());

//     // --- I2S via PIO ---
//     let Pio { mut common, sm0, .. } = Pio::new(p.PIO0, Irqs);
//     let program = PioI2sOutProgram::new(&mut common);
//     let mut i2s = PioI2sOut::new(
//         &mut common, sm0, p.DMA_CH0,
//         p.PIN_9, p.PIN_10, p.PIN_11,
//         44_100, 16, 2, &program,
//     );

//     // --- SD card setup over SPI ---
//     let spi_cfg = spi::Config::default();
//     let spi_raw = Spi::new_blocking(p.SPI0, p.PIN_18, p.PIN_19, p.PIN_16, spi_cfg);
//     let bus = BUS.init(Mutex::new(core::cell::RefCell::new(spi_raw)));
//     let sd_cs = Output::new(p.PIN_17, Level::High);
//     let mut sd_spi_cfg = spi::Config::default();
//     sd_spi_cfg.frequency = 400_000;
//     let sd_spi_dev = SpiDeviceWithConfig::new(bus, sd_cs, sd_spi_cfg);

//     let sdcard = SdCard::new(sd_spi_dev, Delay);
//     let fs = FS_MGR.init(VolumeManager::new(sdcard, DummyTime {}));

//     // Check card and speed up
//     let sz = loop {
//         match fs.device().num_bytes() {
//             Ok(sz) => break sz,
//             Err(e) => {
//                 defmt::warn!("Eroare SD init: {:?}, retry...", e);
//             }
//         }
//         Timer::after(Duration::from_millis(100)).await;
//     };
//     defmt::info!("Card OK – {} MiB", sz / 1_048_576);

//     fs.device().spi(|dev| {
//         let mut fast = spi::Config::default();
//         fast.frequency = 8_000_000;
//         fast.phase = embassy_rp::spi::Phase::CaptureOnFirstTransition;
//         fast.polarity = embassy_rp::spi::Polarity::IdleLow;
//         dev.set_config(fast);
//         info!("SD speed ↑ to 8 MHz");
//     });

//     // Mount and list root directory
//     let mut vol = fs.open_volume(VolumeIdx(0)).unwrap();
//     let mut root = vol.open_root_dir().unwrap();
//     info!("Root dir:");
//     root.iterate_dir(|e| {
//         let raw: &[u8] = &e.name.base_name();
//         let end = raw.iter().rposition(|&b| b != b' ').map(|p| p + 1).unwrap_or(0);
//         if let Ok(txt) = core::str::from_utf8(&raw[..end]) {
//             info!("  {}", txt);
//         } else {
//             defmt::warn!("<invalid>");
//         }
//     });

//         // Open WAV file directly from root directory
//     let mut wav = root.open_file_in_dir("N&0024~7.wav", Mode::ReadOnly).unwrap();

//     let mut cfg = I2cConfig::default();
//     cfg.frequency = 400_000;
//     let i2c: I2c<'static, I2C0, I2cAsync>  = I2c::new_async(p.I2C0, p.PIN_5, p.PIN_4, I2cIrqs, cfg);
//     // În main(), după init-display și definirea style/message:
//     let mesaj = "N&D - Vino la mine";
//     spawner.spawn(display_task(i2c, mesaj)).unwrap();
// // Apoi continui în paralel cu redarea WAV-ului...

//     // Skip WAV header
//     wav.seek_from_start(44).unwrap();

//     // Short delay before playback
//     Timer::after(Duration::from_millis(100)).await;

//     const VOLUME: f32 = 1.0;
//     let mut read_buf = [0u8; CHUNK_FRAMES * 4];
//     loop {
//         let n = wav.read(&mut read_buf).unwrap(); // ([docs.rs](https://docs.rs/embedded-sdmmc/latest/x86_64-pc-windows-msvc/embedded_sdmmc/struct.File.html))
//         if n == 0 { break; }
//         let frames = n / 4;
//         unsafe {
//             for i in 0..frames {
//                 let base = i * 4;
//                 // let l = i16::from_le_bytes([read_buf[base], read_buf[base + 1]]);
//                 // let r = i16::from_le_bytes([read_buf[base + 2], read_buf[base + 3]]);

//                 // AUDIO_BUF[i] = ((l as u16 as u32) << 16) | (r as u16 as u32);
//                 let l_raw = i16::from_le_bytes([read_buf[base], read_buf[base + 1]]);
//                 let r_raw = i16::from_le_bytes([read_buf[base + 2], read_buf[base + 3]]);

//                 // let l = ((l_raw as i32 * VOLUME_FP) >> FIXED_POINT_SHIFT) as i16;
//                 // let r = ((r_raw as i32 * VOLUME_FP) >> FIXED_POINT_SHIFT) as i16;
//                 let l = (l_raw as f32 * VOLUME) as i16;
//                 let r = (r_raw as f32 * VOLUME) as i16;
//                 // sign-extend + împachetare corectă
//                 let l_u = (l as i32 as u32) << 16;
//                 let r_u = (r as i32 as u32) & 0xFFFF;
//                 AUDIO_BUF[i] = ((l as u16 as u32) << 16) | (r as u16 as u32);
//             }
//         }
//         i2s.write(unsafe { &AUDIO_BUF[..frames] }).await;
//     }

//     info!("Playback done");
//     loop { Timer::after(Duration::from_secs(1)).await; }
// }

// CARD SD + AMPLFICATOR SI DIFUZOR + DISPLAY + SENZOR
// #![no_std]
// #![no_main]
// #![allow(static_mut_refs)]
// use {defmt_rtt as _, panic_probe as _};
// use defmt::info;
// use embassy_executor::Spawner;
// use embassy_time::{Delay, Duration, Timer, Instant};

// use embassy_rp::{
//     bind_interrupts, gpio::{AnyPin, Input, Level, Output, Pull}, i2c::{Async as I2cAsync, Config as I2cConfig, I2c, InterruptHandler as I2cIrq}, peripherals::{I2C0, PIO0, SPI0}, pio::{InterruptHandler as PioIrq, Pio}, spi::{self, Blocking, Spi}, Peripherals
// };
// use embassy_rp::pio_programs::i2s::{PioI2sOut, PioI2sOutProgram};

// use embedded_sdmmc::{SdCard, VolumeManager, Mode, TimeSource, Timestamp, VolumeIdx};
// use embassy_embedded_hal::shared_bus::blocking::spi::SpiDeviceWithConfig;
// use embassy_sync::blocking_mutex::{Mutex, raw::NoopRawMutex};
// use static_cell::StaticCell;
// use ssd1306::{
//     mode::BufferedGraphicsMode,
//     prelude::*,
//     I2CDisplayInterface,
//     Ssd1306,
// };
// use core::sync::atomic::AtomicBool;
// use core::sync::atomic::Ordering;
// use core::sync::atomic::AtomicU8;

// use embedded_graphics::{
//     mono_font::{ascii::FONT_10X20, MonoTextStyleBuilder},
//     pixelcolor::BinaryColor,
//     prelude::*,
//     text::{Baseline, Text},
// };

// // Map PIO interrupt for I2S
// bind_interrupts!(struct Irqs {
//     PIO0_IRQ_0 => PioIrq<embassy_rp::peripherals::PIO0>;
// });

// bind_interrupts!(struct I2cIrqs {
//     I2C0_IRQ => I2cIrq<I2C0>;
// });
// // Dummy timestamp for FAT
// struct DummyTime;
// impl TimeSource for DummyTime {
//     fn get_timestamp(&self) -> Timestamp {
//         Timestamp {
//             year_since_1970: 54,
//             zero_indexed_month: 0,
//             zero_indexed_day: 1,
//             hours: 0,
//             minutes: 0,
//             seconds: 0,
//         }
//     }
// }

// // Shared SPI bus for SD
// static BUS: StaticCell<Mutex<NoopRawMutex, core::cell::RefCell<Spi<'static, SPI0, Blocking>>>> = StaticCell::new();

// type SdCardDev = SdCard<
//     SpiDeviceWithConfig<'static, NoopRawMutex, Spi<'static, SPI0, Blocking>, Output<'static>>,
//     Delay,
// >;
// type FsMgr = VolumeManager<SdCardDev, DummyTime>;
// static FS_MGR: StaticCell<FsMgr> = StaticCell::new();


// use embassy_sync::channel::Channel;

// const CHUNK_FRAMES: usize = 2048;

// // 1) One big static array of 2 buffers
// static mut BUFS: [[u32; CHUNK_FRAMES]; 2] = [
//     [0; CHUNK_FRAMES],
//     [0; CHUNK_FRAMES],
// ];

// // 2) Channel that carries buffer indices 0 or 1
// static mut BUF_CH: Channel<NoopRawMutex, u8, 2> = Channel::new();

// #[embassy_executor::task]
// async fn reader_task(sd_spi_dev: SpiDeviceWithConfig<'static, NoopRawMutex, Spi<'static, SPI0, Blocking>, Output<'static>>) {
//     // skip WAV header
//      let sdcard = SdCard::new(sd_spi_dev, Delay);
//     let fs = FS_MGR.init(VolumeManager::new(sdcard, DummyTime {}));

//     // Check card and speed up
//     let sz = loop {
//         match fs.device().num_bytes() {
//             Ok(sz) => break sz,
//             Err(e) => {
//                 defmt::warn!("Eroare SD init: {:?}, retry...", e);
//             }
//         }
//         Timer::after(Duration::from_millis(100)).await;
//     };
//     defmt::info!("Card OK – {} MiB", sz / 1_048_576);

//     fs.device().spi(|dev| {
//         let mut fast = spi::Config::default();
//         fast.frequency = 8_000_000;
//         fast.phase = embassy_rp::spi::Phase::CaptureOnFirstTransition;
//         fast.polarity = embassy_rp::spi::Polarity::IdleLow;
//         dev.set_config(fast);
//         info!("SD speed ↑ to 8 MHz");
//     });

//     // Mount and list root directory
//     let mut vol = fs.open_volume(VolumeIdx(0)).unwrap();
//     let mut root = vol.open_root_dir().unwrap();
//     info!("Root dir:");
//     root.iterate_dir(|e| {
//         let raw: &[u8] = &e.name.base_name();
//         let end = raw.iter().rposition(|&b| b != b' ').map(|p| p + 1).unwrap_or(0);
//         if let Ok(txt) = core::str::from_utf8(&raw[..end]) {
//             info!("  {}", txt);
//         } else {
//             defmt::warn!("<invalid>");
//         }
//     });
//     let mut wav = root.open_file_in_dir("N&0024~7.wav", Mode::ReadOnly).unwrap();

//     wav.seek_from_start(44).unwrap();


//     let mut VOLUME: f32 = 1.0;
//     let mut next: u8  = 0;
//     let mut spare: u8 = 1;
//     let mut read_buf = [0u8; CHUNK_FRAMES * 4];

//     loop {
//         VOLUME = VOLUME_LEVEL.load(Ordering::SeqCst) as f32 / 10.0;
//         while PAUSED.load(Ordering::SeqCst) {
//             Timer::after(Duration::from_millis(100)).await;
//         }
//         let n = wav.read(&mut read_buf).unwrap(); // ([docs.rs](https://docs.rs/embedded-sdmmc/latest/x86_64-pc-windows-msvc/embedded_sdmmc/struct.File.html))
//         if n == 0 { break; }
//         let frames = n / 4;
//         unsafe {
//             for i in 0..frames {
//                 let buf = &mut BUFS[next as usize];
//                 let base = i * 4;
//                 // let l = i16::from_le_bytes([read_buf[base], read_buf[base + 1]]);
//                 // let r = i16::from_le_bytes([read_buf[base + 2], read_buf[base + 3]]);

//                 // AUDIO_BUF[i] = ((l as u16 as u32) << 16) | (r as u16 as u32);
//                 let l_raw = i16::from_le_bytes([read_buf[base], read_buf[base + 1]]);
//                 let r_raw = i16::from_le_bytes([read_buf[base + 2], read_buf[base + 3]]);

//                 // let l = ((l_raw as i32 * VOLUME_FP) >> FIXED_POINT_SHIFT) as i16;
//                 // let r = ((r_raw as i32 * VOLUME_FP) >> FIXED_POINT_SHIFT) as i16;
//                 let l = (l_raw as f32 * VOLUME * 2.0) as i16;
//                 let r = (r_raw as f32 * VOLUME * 2.0) as i16;
//                 // sign-extend + împachetare corectă
//                 let l_u = (l as i32 as u32) << 16;
//                 let r_u = (r as i32 as u32) & 0xFFFF;
//                 buf[i] = ((l_u as u16 as u32) << 16) | (r_u as u16 as u32);
//             }
//         }
//         // hand it off to the writer
//         unsafe { BUF_CH.send(next).await; }

//         // swap next & spare
//         core::mem::swap(&mut next, &mut spare);
//     }
// }

// #[embassy_executor::task]
// async fn writer_task(mut i2s: PioI2sOut<'static, PIO0, 0>) {
//     loop {
//         // wait for next buffer index
//         let idx = unsafe { BUF_CH.receive().await };
//         // safely borrow the finished buffer and write it out
//         unsafe {
//             let buf = &BUFS[idx as usize];
//             let frames = CHUNK_FRAMES; // or track actual frame count if last chunk is short
//             i2s.write(&buf[..frames]).await;
//         }
//     }
// }



// // DMA buffer: 1024 stereo frames (4 KiB)
// // const CHUNK_FRAMES: usize = 2048; // 8K frames for better performance
// static mut AUDIO_BUF: [u32; CHUNK_FRAMES * 2] = [0; CHUNK_FRAMES * 2];
// static PAUSED: AtomicBool = AtomicBool::new(false);
// static VOLUME_LEVEL: AtomicU8 = AtomicU8::new(10); // 0..=10 -> scale 0.0..=1.0

// #[embassy_executor::task]
// async fn button_task(
//     mut btn: Input<'static>,
// ) {
//     // Wait for button press (falling edge), debounce, toggle paused
//     loop {
//         btn.wait_for_falling_edge().await;
//         // debounce
//         Timer::after(Duration::from_millis(50)).await;
//         // toggle
//         let new = !PAUSED.load(Ordering::SeqCst);
//         PAUSED.store(new, Ordering::SeqCst);
//         info!("Play/Pause toggled: {}", new);
//         // wait release
//         btn.wait_for_rising_edge().await;
//         Timer::after(Duration::from_millis(50)).await;
//     }
// }

// #[embassy_executor::task]
// async fn button_volup_task(
//     mut btn: Input<'static>,
// ) {
//     loop {
//         btn.wait_for_falling_edge().await;
//         Timer::after(Duration::from_millis(50)).await;
//         let new = (VOLUME_LEVEL.load(Ordering::SeqCst) + 1).min(10);
//                 VOLUME_LEVEL.store(new, Ordering::SeqCst);
//         info!("Volume UP: {}", new);
//         btn.wait_for_rising_edge().await;
//         Timer::after(Duration::from_millis(50)).await;
//     }
// }

// #[embassy_executor::task]
// async fn button_voldown_task(
//     mut btn: Input<'static>,
// ) {
//     loop {
//         btn.wait_for_falling_edge().await;
//         Timer::after(Duration::from_millis(50)).await;
//         let new = VOLUME_LEVEL.load(Ordering::SeqCst).saturating_sub(1);
//                 VOLUME_LEVEL.store(new, Ordering::SeqCst);
//         info!("Volume DOWN: {}", new);
//         btn.wait_for_rising_edge().await;
//         Timer::after(Duration::from_millis(50)).await;
//     }
// }

// #[embassy_executor::task]
// async fn button_next_task(
//     mut btn: Input<'static>,
// ) {
//     loop {
//         btn.wait_for_falling_edge().await;
//         Timer::after(Duration::from_millis(50)).await;
//         info!("Next button pressed");
//         btn.wait_for_rising_edge().await;
//         Timer::after(Duration::from_millis(50)).await;
//     }
// }
// #[embassy_executor::task]
// async fn button_prev_task(
//     mut btn: Input<'static>,
// ) {
//     loop {
//         btn.wait_for_falling_edge().await;
//         Timer::after(Duration::from_millis(50)).await;
//         info!("Prev button pressed");
//         btn.wait_for_rising_edge().await;
//         Timer::after(Duration::from_millis(50)).await;
//     }
// }


// #[embassy_executor::task]
// async fn display_task(  i2c: I2c<'static, I2C0, I2cAsync>,
//     mesaj: &'static str)
// {
//     let interface = I2CDisplayInterface::new(i2c);
//     // inițializăm driver-ul în BufferedGraphicsMode
//     let mut display: Ssd1306<_, _, BufferedGraphicsMode<_>> = Ssd1306::new(
//         interface,
//         DisplaySize128x64,
//         DisplayRotation::Rotate0,
//     )
//     .into_buffered_graphics_mode();
//     display.init().unwrap();

//     // curățăm ecranul
//     display.clear_buffer();


//     let font = &FONT_10X20;
//     let text_style = MonoTextStyleBuilder::new()
//         .font(font)
//         .text_color(BinaryColor::On)
//         .build();

//     // calculăm lățimea aproximativă (monospace):
//     let char_w = font.character_size.width as i32;
//     let text_w = (mesaj.len() as i32) * char_w;
//     // poziția verticală (sus)
//     let y = 0;

//     // pornim dincolo de marginea din dreapta
//     let mut x = 128;

//     loop {
//         display.clear_buffer();
//         // desenăm textul la offset-ul curent
//         Text::with_baseline(
//             mesaj,
//             Point::new(x, y),
//             text_style,
//             Baseline::Top,
//         )
//         .draw(&mut display)
//         .unwrap();
//         display.flush().unwrap();

//         // mutăm cu un pixel la stânga
//         x -= 1;
//         // dacă a dispărut complet la stânga, resetăm
//         if x < -text_w {
//             x = 128;
//         }

//         // pauză de 50ms — ajustează ca să fie mai rapid/lent
//         Timer::after(Duration::from_millis(1)).await;
//     }
// }



// /// Măsoară o singură valoare de distanță și o returnează în cm.
// async fn measure_distance(
//     trig: &mut Output<'_>,
//     echo: &mut Input<'_>,
// ) -> f32 {
//     echo.wait_for_low().await;
//     Timer::after(Duration::from_micros(2)).await;

//     trig.set_high();
//     Timer::after(Duration::from_micros(10)).await;
//     trig.set_low();

//     // așteaptă front ascendent
//     echo.wait_for_rising_edge().await;
//     // măsoară cât e HIGH
//     let t0 = Instant::now();
//     echo.wait_for_falling_edge().await;
//     let pulse_us = (Instant::now() - t0).as_micros() as f32;

//     pulse_us / 58.0
// }


// #[embassy_executor::main]
// async fn main(spawner: Spawner) {
//     info!("WAV player SD+I2S demo");

//     // Init peripherals
//     let p = embassy_rp::init(Default::default());

//     // --- I2S via PIO ---
//     let Pio { mut common, sm0, .. } = Pio::new(p.PIO0, Irqs);
//     let program = PioI2sOutProgram::new(&mut common);
//     let mut i2s = PioI2sOut::new(
//         &mut common, sm0, p.DMA_CH0,
//         p.PIN_9, p.PIN_10, p.PIN_11,
//         44_100, 16, 2, &program,
//     );

//     // --- SD card setup over SPI ---
//     let spi_cfg = spi::Config::default();
//     let spi_raw = Spi::new_blocking(p.SPI0, p.PIN_18, p.PIN_19, p.PIN_16, spi_cfg);
//     let bus = BUS.init(Mutex::new(core::cell::RefCell::new(spi_raw)));
//     let sd_cs = Output::new(p.PIN_17, Level::High);
//     let mut sd_spi_cfg = spi::Config::default();
//     sd_spi_cfg.frequency = 400_000;
//     let sd_spi_dev = SpiDeviceWithConfig::new(bus, sd_cs, sd_spi_cfg);

//     let btn = Input::new(p.PIN_3, Pull::Up);
//     let btn_up = Input::new(p.PIN_2, Pull::Up);   // vol up
//     let btn_dn = Input::new(p.PIN_13, Pull::Up);   // vol down
//     let btn_next = Input::new(p.PIN_14, Pull::Up);   // next
//     let btn_prev = Input::new(p.PIN_15, Pull::Up);   // prev
//     //let btn_shuffle = Input::new(p.PIN_12, Pull::Up);   // shuffle


//     let mut trig = Output::new(p.PIN_20, Level::Low);
//     let mut echo = Input::new(p.PIN_21, Pull::None);

//     const COIN_THRESHOLD_CM: f32 = 7.0;
//     info!("Aștept monedă...");
//     loop {
//         let d = measure_distance(&mut trig, &mut echo).await;
//         info!("Dist: {} cm", d);
//         if d < COIN_THRESHOLD_CM {
//             info!("Monedă detectată ({} cm), pornesc muzica!", d);
//             break;
//         }
//         // mic delay între măsurători
//         Timer::after(Duration::from_millis(200)).await;
//     }


//     let mut cfg = I2cConfig::default();
//     cfg.frequency = 400_000;
//     let i2c: I2c<'static, I2C0, I2cAsync>  = I2c::new_async(p.I2C0, p.PIN_5, p.PIN_4, I2cIrqs, cfg);
//     // În main(), după init-display și definirea style/message:
//     let mesaj = "N&D - Vino la mine";
//     spawner.spawn(display_task(i2c, mesaj)).unwrap();
//     spawner.spawn(button_task(btn)).unwrap();
//     spawner.spawn(button_volup_task(btn_up)).unwrap();
//     spawner.spawn(button_voldown_task(btn_dn)).unwrap();
//     spawner.spawn(button_next_task(btn_next)).unwrap();
//     spawner.spawn(button_prev_task(btn_prev)).unwrap();
// // Apoi continui în paralel cu redarea WAV-ului...
//     spawner.spawn(reader_task(sd_spi_dev)).unwrap();
//     spawner.spawn(writer_task(i2s)).unwrap();
//     // Skip WAV header
//    // wav.seek_from_start(44).unwrap();

//     // Short delay before playback
//    // Timer::after(Duration::from_millis(100)).await;



//     // loop {
//     //     VOLUME = VOLUME_LEVEL.load(Ordering::SeqCst) as f32 / 10.0;
//     //     while PAUSED.load(Ordering::SeqCst) {
//     //         Timer::after(Duration::from_millis(100)).await;
//     //     }
//     //     let n = wav.read(&mut read_buf).unwrap(); // ([docs.rs](https://docs.rs/embedded-sdmmc/latest/x86_64-pc-windows-msvc/embedded_sdmmc/struct.File.html))
//     //     if n == 0 { break; }
//     //     let frames = n / 4;
//     //     unsafe {
//     //         for i in 0..frames {
//     //             let buf = &mut BUFS[next as usize];
//     //             let base = i * 4;
//     //             // let l = i16::from_le_bytes([read_buf[base], read_buf[base + 1]]);
//     //             // let r = i16::from_le_bytes([read_buf[base + 2], read_buf[base + 3]]);

//     //             // AUDIO_BUF[i] = ((l as u16 as u32) << 16) | (r as u16 as u32);
//     //             let l_raw = i16::from_le_bytes([read_buf[base], read_buf[base + 1]]);
//     //             let r_raw = i16::from_le_bytes([read_buf[base + 2], read_buf[base + 3]]);

//     //             // let l = ((l_raw as i32 * VOLUME_FP) >> FIXED_POINT_SHIFT) as i16;
//     //             // let r = ((r_raw as i32 * VOLUME_FP) >> FIXED_POINT_SHIFT) as i16;
//     //             let l = (l_raw as f32 * VOLUME) as i16;
//     //             let r = (r_raw as f32 * VOLUME) as i16;
//     //             // sign-extend + împachetare corectă
//     //             let l_u = (l as i32 as u32) << 16;
//     //             let r_u = (r as i32 as u32) & 0xFFFF;
//     //             buf[i] = ((l as u16 as u32) << 16) | (r as u16 as u32);
//     //         }
//     //     }
//     //     // hand it off to the writer
//     //     unsafe { BUF_CH.send(next).await; }

//     //     // swap next & spare
//     //     core::mem::swap(&mut next, &mut spare);
//     // }

// }