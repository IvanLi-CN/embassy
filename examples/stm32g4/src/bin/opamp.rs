#![no_std]
#![no_main]

use core::ptr::write_volatile;

use defmt::info;
use embassy_executor::Spawner;
use embassy_stm32::adc::{Adc, AdcChannel, AnyAdcChannel, SampleTime};
use embassy_stm32::dac::{self, Dac, DacCh1};
use embassy_stm32::opamp::OpAmp;
use embassy_stm32::peripherals::ADC1;
use embassy_stm32::Config;
use embassy_stm32::Peripheral;
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

const VREFBUF_BASE: u32 = 0x40010030;
const VREFBUF_CSR_ADDR: *mut u32 = (VREFBUF_BASE + 0x00) as *mut u32;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.pll = Some(Pll {
            source: PllSource::HSI,
            prediv: PllPreDiv::DIV1,
            mul: PllMul::MUL18,
            divp: None,
            divq: Some(PllQDiv::DIV6),
            // Main system clock at 48 MHz
            divr: Some(PllRDiv::DIV2),
        });
        config.rcc.mux.adc12sel = mux::Adcsel::SYS;
        config.rcc.mux.adc345sel = mux::Adcsel::SYS;
        config.rcc.apb2_pre = APBPrescaler::DIV2;
        config.rcc.sys = Sysclk::PLL1_R;
    }
    let p = embassy_stm32::init(config);
    info!("Hello World!");

    unsafe {
        write_volatile(VREFBUF_CSR_ADDR, 0x0000_0021 as u32);
    }

    let mut dac3 = Dac::new_internal(p.DAC3, p.DMA1_CH1, p.DMA1_CH2);
    let mut adc1 = Adc::new(p.ADC1);

    dac3.set(dac::DualValue::Bit12Left(2047, 2047));

    let mut adc_dma = p.DMA2_CH1;
    let mut adc_buffer = [0u16; 2];

    let mut opamp1 = OpAmp::new(p.OPAMP1, embassy_stm32::opamp::OpAmpSpeed::Normal);
    opamp1.calibrate();
    let mut out_pin_ch: AnyAdcChannel<ADC1> = unsafe { p.PA2.clone_unchecked().degrade_adc() };
    let opamp1_standalone = opamp1.buffer_dac(p.PA2);
    let mut vsn_rmt_ch: AnyAdcChannel<ADC1> = opamp1_standalone.degrade_adc();


    loop {
        for i in 0..8 {
            let target = i * 512;
            dac3.set(dac::DualValue::Bit12Left(target, target));

            adc1.read(
                &mut adc_dma,
                [
                    (&mut out_pin_ch, SampleTime::CYCLES640_5),
                    (&mut vsn_rmt_ch, SampleTime::CYCLES640_5),
                ]
                .into_iter(),
                &mut adc_buffer,
            )
            .await;

            Timer::after_millis(1000).await;

            defmt::println!(
                "target: {}, \tpin: {}, \tvsn: {}", 
                target,
                adc_buffer[0],
                adc_buffer[1]
            );
        }

        Timer::after_millis(1000).await;
    }
}
