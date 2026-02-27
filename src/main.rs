#![no_std]
#![no_main]
#![allow(static_mut_refs)]

use {defmt_rtt as _, panic_probe as _};
use defmt::info;
use embassy_executor::Spawner;
use embassy_time::{Delay, Duration, Timer, Instant};
use embassy_futures::join::join;
use embassy_rp::{
    bind_interrupts,
    gpio::{Input, Level, Output, Pull},
    i2c::{Async as I2cAsync, Config as I2cConfig, I2c, InterruptHandler as I2cIrq},
    peripherals::{I2C0, PIO0, SPI0},
    pio::{InterruptHandler as PioIrq, Pio},
    pio_programs::i2s::{PioI2sOut, PioI2sOutProgram},
    spi::{self, Blocking, Spi}
};

use embedded_sdmmc::{SdCard, VolumeManager, Mode, TimeSource, Timestamp, VolumeIdx};
use embassy_embedded_hal::shared_bus::blocking::spi::SpiDeviceWithConfig;
use embassy_sync::{
    blocking_mutex::{Mutex, raw::NoopRawMutex},
    channel::Channel,
};
use static_cell::StaticCell;
use core::sync::atomic::{AtomicBool, AtomicU32, AtomicU8, AtomicUsize, Ordering};

use embedded_graphics::{
    mono_font::{ascii::FONT_10X20, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use ssd1306::{ mode::BufferedGraphicsMode,
    prelude::*,
    I2CDisplayInterface,
    Ssd1306,};
use heapless::{Vec, String};


// ——— Dummy timestamp for FAT —————————————————————————————————————
struct DummyTime;
impl TimeSource for DummyTime {
    fn get_timestamp(&self) -> Timestamp {
        Timestamp { year_since_1970: 54, zero_indexed_month: 0,
                    zero_indexed_day: 1, hours: 0, minutes: 0, seconds: 0 }
    }
}

// ——— Shared SPI bus for SD —————————————————————————————————————
static BUS: StaticCell<Mutex<NoopRawMutex, core::cell::RefCell<Spi<'static, SPI0, Blocking>>>> = StaticCell::new();

type SdCardDev = SdCard<
    SpiDeviceWithConfig<'static, NoopRawMutex, Spi<'static, SPI0, Blocking>, Output<'static>>,
    Delay,
>;
type FsMgr = VolumeManager<SdCardDev, DummyTime>;
static FS_MGR: StaticCell<FsMgr> = StaticCell::new();


// ——— Audio constants & buffer ——————————————————————————————————
const CHUNK_FRAMES: usize = 2048;

// ——— Playback control ———————————————————————————————————————
static PAUSED: AtomicBool     = AtomicBool::new(false);
static VOLUME_LEVEL: AtomicU8 = AtomicU8::new(10); // 0..=10
static mut CONTROL_CH: Channel<NoopRawMutex, TrackCommand, 2> = Channel::new();
static CURRENT_TRACK: AtomicUsize = AtomicUsize::new(0);

static SHUFFLE: AtomicBool     = AtomicBool::new(false);
static RNG_SEED: AtomicU32     = AtomicU32::new(0xDEADBEEF);

#[derive(Copy, Clone)]
enum TrackCommand { Next, Prev }
static mut DISPLAY_CH: Channel<NoopRawMutex, String<32>, 2> = Channel::new();
// ——— Bind interrupts ————————————————————————————————————————
bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => PioIrq<embassy_rp::peripherals::PIO0>;
});
bind_interrupts!(struct I2cIrqs {
    I2C0_IRQ => I2cIrq<I2C0>;
});

// ——— Button tasks ————————————————————————————————————————

#[embassy_executor::task]
async fn button_playpause(mut btn: Input<'static>) {
    loop {
        btn.wait_for_falling_edge().await;
        Timer::after(Duration::from_millis(50)).await;
        let new = !PAUSED.load(Ordering::SeqCst);
        PAUSED.store(new, Ordering::SeqCst);
        info!("Paused = {}", new);
        btn.wait_for_rising_edge().await;
        Timer::after(Duration::from_millis(50)).await;
    }
}

#[embassy_executor::task]
async fn button_volup(mut btn: Input<'static>) {
    loop {
        btn.wait_for_falling_edge().await;
        Timer::after(Duration::from_millis(50)).await;
        let v = (VOLUME_LEVEL.load(Ordering::SeqCst) + 1).min(10);
        VOLUME_LEVEL.store(v, Ordering::SeqCst);
        info!("Volume = {}", v);
        btn.wait_for_rising_edge().await;
        Timer::after(Duration::from_millis(50)).await;
    }
}

#[embassy_executor::task]
async fn button_voldown(mut btn: Input<'static>) {
    loop {
        btn.wait_for_falling_edge().await;
        Timer::after(Duration::from_millis(50)).await;
        let v = VOLUME_LEVEL.load(Ordering::SeqCst).saturating_sub(1);
        VOLUME_LEVEL.store(v, Ordering::SeqCst);
        info!("Volume = {}", v);
        btn.wait_for_rising_edge().await;
        Timer::after(Duration::from_millis(50)).await;
    }
}

#[embassy_executor::task]
async fn button_next(mut btn: Input<'static>) {
    loop {
        btn.wait_for_falling_edge().await;
        Timer::after(Duration::from_millis(50)).await;
        unsafe { CONTROL_CH.send(TrackCommand::Next).await; }
        btn.wait_for_rising_edge().await;
        Timer::after(Duration::from_millis(50)).await;
    }
}

#[embassy_executor::task]
async fn button_prev(mut btn: Input<'static>) {
    loop {
        btn.wait_for_falling_edge().await;
        Timer::after(Duration::from_millis(50)).await;
        unsafe { CONTROL_CH.send(TrackCommand::Prev).await; }
        btn.wait_for_rising_edge().await;
        Timer::after(Duration::from_millis(50)).await;
    }
}

#[embassy_executor::task]
async fn button_shuffle(mut btn: Input<'static>) {
    loop {
        btn.wait_for_falling_edge().await;
        Timer::after(Duration::from_millis(50)).await;
        let new = !SHUFFLE.load(Ordering::SeqCst);
        SHUFFLE.store(new, Ordering::SeqCst);
        info!("Shuffle");
        btn.wait_for_rising_edge().await;
        Timer::after(Duration::from_millis(50)).await;
    }
}


// ——— Display scrolling message ——————————————————————————————

#[embassy_executor::task]
async fn display_task(i2c: I2c<'static, I2C0, I2cAsync>) {
    let interface = I2CDisplayInterface::new(i2c);
     let mut display: Ssd1306<_, _, BufferedGraphicsMode<_>> = Ssd1306::new(
        interface,
        DisplaySize128x64,
        DisplayRotation::Rotate0,
    )
    .into_buffered_graphics_mode();
    display.init().unwrap();
    display.clear_buffer();
    display.flush().unwrap();
    let style = MonoTextStyleBuilder::new()
        .font(&FONT_10X20)
        .text_color(BinaryColor::On)
        .build();


   // let mut mesaj = String::<32>::new();
    let mut mesaj = unsafe { DISPLAY_CH.receive().await };
   loop {
        let text = mesaj.as_str();
        let char_w = FONT_10X20.character_size.width as i32;
        let text_w = (text.len() as i32) * char_w;
        let y = 0;
        let mut x = 128;

        loop {
            if let Ok(nou) = unsafe {DISPLAY_CH.try_receive() } {
                mesaj = nou;
                break;
            }
            display.clear_buffer();
            Text::with_baseline(text, Point::new(x, y), style, Baseline::Top)
                .draw(&mut display).unwrap();
            display.flush().unwrap();
            x -= 1;
            if x < -text_w {
                x = 128;

            }
            Timer::after(Duration::from_millis(100)).await;
        }
    }
}

// ——— Playback task: scan, open & stream WAVs ——————————————————————

#[embassy_executor::task]
async fn playback_task(
    sd_spi_dev: SpiDeviceWithConfig<'static, NoopRawMutex, Spi<'static, SPI0, Blocking>, Output<'static>>,
    mut i2s: PioI2sOut<'static, PIO0, 0>,
) {
    let sdcard = SdCard::new(sd_spi_dev, Delay);
    let fs = FS_MGR.init(VolumeManager::new(sdcard, DummyTime {}));

    let sz = loop {
        match fs.device().num_bytes() {
            Ok(sz) => break sz,
            Err(e) => {
                defmt::warn!("Eroare SD init: {:?}, retry...", e);
            }
        }
        Timer::after(Duration::from_millis(100)).await;
    };
    defmt::info!("Card OK – {} MiB", sz / 1_048_576);

    fs.device().spi(|dev| {
        let mut fast = spi::Config::default();
        fast.frequency = 8_000_000;
        fast.phase = embassy_rp::spi::Phase::CaptureOnFirstTransition;
        fast.polarity = embassy_rp::spi::Polarity::IdleLow;
        dev.set_config(fast);
        info!("SD speed ↑ to 8 MHz");
    });

    let mut vol = fs.open_volume(VolumeIdx(0)).unwrap();
    let mut root = vol.open_root_dir().unwrap();

    let mut file_list: Vec<String<32>, 16> = Vec::new();

    root.iterate_dir(|e| {
        let ext = e.name.extension();

        if ext.len() == 3 {
            if ext[0] == b'W' && ext[1] == b'A' && ext[2] == b'V' {
            let mut name: String<32> = String::new();
            for &c in e.name.base_name().iter() {
                if c == b' ' { break; }
                name.push(c as char).unwrap();
            }
            name.push('.').unwrap();
            name.push('W').unwrap();
            name.push('A').unwrap();
            name.push('V').unwrap();
            file_list.push(name).ok();
        }}
    });

    info!("Found {} WAV files", file_list.len());

      loop {
        // selectăm index-ul curent
        let idx = CURRENT_TRACK.load(Ordering::SeqCst) % file_list.len();
        let fname = &file_list[idx];
        //let mut name = String::<32>::new();
        let name = match fname.as_str() {
            "INA901~5.WAV" => {
                let mut s = String::<32>::new();
                s.push_str("1. Inna - Sun is Up").unwrap();
                s
            }
            "IRA9F4~5.WAV" => {
                let mut s = String::<32>::new();
                s.push_str("2. Irina Rimes - Ba Ba Ba").unwrap();
                s
            }
            "MAAADB~5.WAV" => {
                let mut s = String::<32>::new();
                s.push_str("3. Maximilian - Sophie").unwrap();
                s
            }
            "VAABE5~5.WAV" => {
                let mut s = String::<32>::new();
                s.push_str("4. Vama - Perfect fara tine").unwrap();
                s
            }
            "N&0024~7.WAV" => {
                let mut s = String::<32>::new();
                s.push_str("6. N&D - Vino la mine").unwrap();
                s
            }
            "AC015C~7.WAV" => {
                let mut s = String::<32>::new();
                s.push_str("7.Activ - Doar cu tine").unwrap();
                s
            }
            "VUACE8~5.WAV" => {
                let mut s = String::<32>::new();
                s.push_str("8. Vunk - Asa, si?").unwrap();
                s
            }
            "DOD9D5~7.WAV" => {
                let mut s = String::<32>::new();
                s.push_str("5. DO RE MI").unwrap();
                s
            }
            other => {
                let mut s = String::<32>::new();
                s.push_str("ceva").unwrap();
                s
            }
        };

        unsafe { DISPLAY_CH.send(name).await; }
        info!("Opening track[{}]: {}", idx, fname);
        let mut wav = root.open_file_in_dir(fname.as_str(), Mode::ReadOnly).unwrap();
        wav.seek_from_start(44).unwrap();

        let mut read_bufs = [
            [0u8; CHUNK_FRAMES * 4],
            [0u8; CHUNK_FRAMES * 4],
        ];
        static mut audio_bufs: [[u32; CHUNK_FRAMES]; 2] = [[0; CHUNK_FRAMES]; 2];

        async fn process_volume(
            raw: &[u8],
            out: &mut [u32],
            frames: usize,
        ) {
            let lvl = VOLUME_LEVEL.load(Ordering::SeqCst) as i32;
            for i in 0..frames {
                let base = i * 4;
                let l = i16::from_le_bytes([raw[base], raw[base + 1]]);
                let r = i16::from_le_bytes([raw[base + 2], raw[base + 3]]);
                let l = ((l as i32 * lvl) / 10) as u16 as u32;
                let r = ((r as i32 * lvl) / 10) as u16 as u32;
                out[i] = (l << 16) | r;
            }
            Timer::after(Duration::from_micros(1)).await;

        }
        let mut cur = 0;
        let mut next = 1;

        let n0 = wav.read(&mut read_bufs[cur]).unwrap();
        let frames0 = n0 / 4;
process_volume(&read_bufs[cur], unsafe { &mut audio_bufs[cur] }, frames0).await;

loop {
    if let Ok(cmd) = unsafe { CONTROL_CH.try_receive() } {
        let cur = CURRENT_TRACK.load(Ordering::SeqCst);
                let next_idx = match cmd {
                    TrackCommand::Next => {

                        if SHUFFLE.load(Ordering::SeqCst) {
                            rand_index(file_list.len())
                        } else {

                            (cur + 1) % file_list.len()
                        }
                    }
                    TrackCommand::Prev => {
                        if SHUFFLE.load(Ordering::SeqCst) {
                            rand_index(file_list.len())
                        } else {
                            if cur == 0 {
                                file_list.len() - 1
                            } else {
                                cur - 1
                            }
                        }
                    }
                };

            CURRENT_TRACK.store(next_idx, Ordering::SeqCst);
            info!("Received command");
            break;
            }

            if PAUSED.load(Ordering::SeqCst) {
                Timer::after(Duration::from_millis(100)).await;
                continue;
            }

            let read_fut = async {
                let n = wav.read(&mut read_bufs[next]).unwrap();
                let f = n / 4;
                process_volume(&read_bufs[next], unsafe { &mut audio_bufs[next] }, f).await;
                f
            };

            let write_fut = i2s.write(unsafe { &audio_bufs[cur] });


            let (frames_read, _) = join(read_fut, write_fut).await;


            if frames_read == 0 {
                let next_idx = if SHUFFLE.load(Ordering::SeqCst) {
                    rand_index(file_list.len())
                } else {
                    (idx + 1) % file_list.len()
                };
                CURRENT_TRACK.store(next_idx, Ordering::SeqCst);
                break;
            }

            core::mem::swap(&mut cur, &mut next);
        }
    }
}

fn rand_index(len: usize) -> usize {
    let old = RNG_SEED.load(Ordering::SeqCst);
    let new = old.wrapping_mul(1664525).wrapping_add(1013904223);
    RNG_SEED.store(new, Ordering::SeqCst);
    (new as usize) % len
}

async fn measure_distance(
    trig: &mut Output<'_>,
    echo: &mut Input<'_>,
) -> f32 {
    echo.wait_for_low().await;
    Timer::after(Duration::from_micros(2)).await;
    trig.set_high();
    Timer::after(Duration::from_micros(10)).await;
    trig.set_low();
    echo.wait_for_rising_edge().await;
    let t0 = Instant::now();
    echo.wait_for_falling_edge().await;
    let pulse_us = (Instant::now() - t0).as_micros() as f32;
    pulse_us / 58.0
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Music Player starting...");

    let p = embassy_rp::init(Default::default());

    // — I2S via PIO —
    let Pio { mut common, sm0, .. } = Pio::new(p.PIO0, Irqs);
    let program = PioI2sOutProgram::new(&mut common);
    let i2s = PioI2sOut::new(
        &mut common, sm0, p.DMA_CH0,
        p.PIN_9, p.PIN_10, p.PIN_11,
        44_100, 16, 2, &program,
    );

    // — SD over SPI —
    let spi_cfg = spi::Config::default();
    let spi_raw = Spi::new_blocking(p.SPI0, p.PIN_18, p.PIN_19, p.PIN_16, spi_cfg);
    let bus = BUS.init(Mutex::new(core::cell::RefCell::new(spi_raw)));
    let sd_cs = Output::new(p.PIN_17, Level::High);
    let mut sd_spi_cfg = spi::Config::default();
    sd_spi_cfg.frequency = 400_000;
    let sd_spi_dev = SpiDeviceWithConfig::new(bus, sd_cs, sd_spi_cfg);

    // — Buttons —
    let btn_play = Input::new(p.PIN_3, Pull::Up);
    let btn_volup = Input::new(p.PIN_2, Pull::Up);
    let btn_voldn = Input::new(p.PIN_13, Pull::Up);
    let btn_next = Input::new(p.PIN_14, Pull::Up);
    let btn_prev = Input::new(p.PIN_15, Pull::Up);
    let btn_shuffle = Input::new(p.PIN_6, Pull::Up);

    // — Display I2C —
    let mut i2c_cfg = I2cConfig::default();
    i2c_cfg.frequency = 400_000;
    let i2c = I2c::new_async(p.I2C0, p.PIN_5, p.PIN_4, I2cIrqs, i2c_cfg);

    let mut trig = Output::new(p.PIN_20, Level::Low);
    let mut echo = Input::new(p.PIN_21, Pull::None);

    const COIN_THRESHOLD_CM: f32 = 6.0;
    info!("Aștept monedă...");
    loop {
        let d = measure_distance(&mut trig, &mut echo).await;
        info!("Dist: {} cm", d);
        if d < COIN_THRESHOLD_CM {
            info!("Monedă detectată ({} cm), pornesc muzica!", d);
            break;
        }
        Timer::after(Duration::from_millis(200)).await;
    }

    spawner.spawn(button_playpause(btn_play)).unwrap();
    spawner.spawn(button_volup(btn_volup)).unwrap();
    spawner.spawn(button_voldown(btn_voldn)).unwrap();
    spawner.spawn(button_next(btn_next)).unwrap();
    spawner.spawn(button_prev(btn_prev)).unwrap();
    spawner.spawn(button_shuffle(btn_shuffle)).unwrap();
    spawner.spawn(display_task(i2c)).unwrap();
    spawner.spawn(playback_task(sd_spi_dev, i2s)).unwrap();
}
