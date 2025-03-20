//! Operational Amplifier (OPAMP)
#![macro_use]

use embassy_hal_internal::{into_ref, PeripheralRef};

use crate::pac::opamp::vals::*;
use crate::Peripheral;

/// Gain
#[allow(missing_docs)]
#[derive(Clone, Copy)]
#[cfg(not(opamp_g4))]
pub enum OpAmpGain {
    Mul1,
    Mul2,
    Mul4,
    Mul8,
    Mul16,
}

/// Represents the programmable gain settings for an operational amplifier.
///
/// This enum defines various gain configurations that can be set for an operational amplifier
/// in an STM32G4 microcontroller. Each variant corresponds to a specific gain setting,
/// which can be either non-inverting or inverting, and may include additional features such as
/// filtering or specific pin usage for input or biasing.
#[derive(Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg(opamp_g4)]
pub enum OpAmpGain {
    /// Non-inverting internal gain = 2
    Mul2,
    /// Non-inverting internal gain = 4
    Mul4,
    /// Non-inverting internal gain = 8
    Mul8,
    /// Non-inverting internal gain = 16
    Mul16,
    /// Non-inverting internal gain = 32
    Mul32,
    /// Non-inverting internal gain = 64
    Mul64,
    /// Inverting gain = -1 / Non-inverting gain = 2 with VINM0 pin for input or bias
    Inv1OrMul2Vinm0,
    /// Inverting gain = -3 / Non-inverting gain = 4 with VINM0 pin for input or bias
    Inv3OrMul4Vinm0,
    /// Inverting gain = -7 / Non-inverting gain = 8 with VINM0 pin for input or bias
    Inv7OrMul8Vinm0,
    /// Inverting gain = -15 / Non-inverting gain = 16 with VINM0 pin for input or bias
    Inv15OrMul16Vinm0,
    /// Inverting gain = -31 / Non-inverting gain = 32 with VINM0 pin for input or bias
    Inv31OrMul32Vinm0,
    /// Inverting gain = -63 / Non-inverting gain = 64 with VINM0 pin for input or bias
    Inv63OrMul64Vinm0,
    /// Non-inverting gain = 2 with filtering on VINM0 pin
    Mul2FilterVinm0,
    /// Non-inverting gain = 4 with filtering on VINM0 pin
    Mul4FilterVinm0,
    /// Non-inverting gain = 8 with filtering on VINM0 pin
    Mul8FilterVinm0,
    /// Non-inverting gain = 16 with filtering on VINM0 pin
    Mul16FilterVinm0,
    /// Non-inverting gain = 32 with filtering on VINM0 pin
    Mul32FilterVinm0,
    /// Non-inverting gain = 64 with filtering on VINM0 pin
    Mul64FilterVinm0,
    /// Inverting gain = -1 / Non-inverting gain = 2 with VINM0 pin for input or bias, VINM1 pin for filtering
    Inv1OrMul2Vinm0Vinm1,
    /// Inverting gain = -3 / Non-inverting gain = 4 with VINM0 pin for input or bias, VINM1 pin for filtering
    Inv3OrMul4Vinm0Vinm1,
    /// Inverting gain = -7 / Non-inverting gain = 8 with VINM0 pin for input or bias, VINM1 pin for filtering
    Inv7OrMul8Vinm0Vinm1,
    /// Inverting gain = -15 / Non-inverting gain = 16 with VINM0 pin for input or bias, VINM1 pin for filtering
    Inv15OrMul16Vinm0Vinm1,
    /// Inverting gain = -31 / Non-inverting gain = 32 with VINM0 pin for input or bias, VINM1 pin for filtering
    Inv31OrMul32Vinm0Vinm1,
    /// Inverting gain = -63 / Non-inverting gain = 64 with VINM0 pin for input or bias, VINM1 pin for filtering
    Inv63OrMul64Vinm0Vinm1,
}

#[cfg(opamp_g4)]
impl Into<u8> for OpAmpGain {
    fn into(self) -> u8 {
        match self {
            OpAmpGain::Mul2 => 0b00000,
            OpAmpGain::Mul4 => 0b00001,
            OpAmpGain::Mul8 => 0b00010,
            OpAmpGain::Mul16 => 0b00011,
            OpAmpGain::Mul32 => 0b00100,
            OpAmpGain::Mul64 => 0b00101,
            // 00110: Not used
            // 00111: Not used
            OpAmpGain::Inv1OrMul2Vinm0 => 0b01000,
            OpAmpGain::Inv3OrMul4Vinm0 => 0b01001,
            OpAmpGain::Inv7OrMul8Vinm0 => 0b01010,
            OpAmpGain::Inv15OrMul16Vinm0 => 0b01011,
            OpAmpGain::Inv31OrMul32Vinm0 => 0b01100,
            OpAmpGain::Inv63OrMul64Vinm0 => 0b01101,
            // 01110: Not used
            // 01111: Not used
            OpAmpGain::Mul2FilterVinm0 => 0b10000,
            OpAmpGain::Mul4FilterVinm0 => 0b10001,
            OpAmpGain::Mul8FilterVinm0 => 0b10010,
            OpAmpGain::Mul16FilterVinm0 => 0b10011,
            OpAmpGain::Mul32FilterVinm0 => 0b10100,
            OpAmpGain::Mul64FilterVinm0 => 0b10101,
            // 10110: Not used
            // 10111: Not used
            OpAmpGain::Inv1OrMul2Vinm0Vinm1 => 0b11000,
            OpAmpGain::Inv3OrMul4Vinm0Vinm1 => 0b11001,
            OpAmpGain::Inv7OrMul8Vinm0Vinm1 => 0b11010,
            OpAmpGain::Inv15OrMul16Vinm0Vinm1 => 0b11011,
            OpAmpGain::Inv31OrMul32Vinm0Vinm1 => 0b11100,
            OpAmpGain::Inv63OrMul64Vinm0Vinm1 => 0b11101,
            // 11110: Not used
            // 11111: Not used
        }
    }
}

/// Speed
#[allow(missing_docs)]
#[derive(Clone, Copy)]
pub enum OpAmpSpeed {
    Normal,
    HighSpeed,
}

#[cfg(opamp_g4)]
impl From<OpAmpSpeed> for crate::pac::opamp::vals::Opahsm {
    fn from(v: OpAmpSpeed) -> Self {
        match v {
            OpAmpSpeed::Normal => crate::pac::opamp::vals::Opahsm::NORMAL,
            OpAmpSpeed::HighSpeed => crate::pac::opamp::vals::Opahsm::HIGH_SPEED,
        }
    }
}

/// OpAmp external outputs, wired to a GPIO pad.
///
/// This struct can also be used as an ADC input.
pub struct OpAmpOutput<'d, T: Instance> {
    _inner: &'d OpAmp<'d, T>,
}

/// OpAmp internal outputs, wired directly to ADC inputs.
///
/// This struct can be used as an ADC input.
#[cfg(opamp_g4)]
pub struct OpAmpInternalOutput<'d, T: Instance> {
    _inner: &'d OpAmp<'d, T>,
}

/// OpAmp inputs, connected to a GPIO pad.
///
/// This struct can also be used as a DAC output.
pub struct OpAmpInput<'d, T: Instance> {
    _inner: &'d OpAmp<'d, T>,
}

/// OpAmp driver.
pub struct OpAmp<'d, T: Instance> {
    _inner: PeripheralRef<'d, T>,
}

impl<'d, T: Instance> OpAmp<'d, T> {
    /// Create a new driver instance.
    ///
    /// Does not enable the opamp, but does set the speed mode on some families.
    pub fn new(opamp: impl Peripheral<P = T> + 'd, #[cfg(opamp_g4)] speed: OpAmpSpeed) -> Self {
        into_ref!(opamp);

        #[cfg(opamp_g4)]
        T::regs().csr().modify(|w| {
            w.set_opahsm(speed.into());
        });

        Self { _inner: opamp }
    }

    /// Configure the OpAmp as a buffer for the provided input pin,
    /// outputting to the provided output pin, and enable the opamp.
    ///
    /// The input pin is configured for analogue mode but not consumed,
    /// so it may subsequently be used for ADC or comparator inputs.
    ///
    /// The output pin is held within the returned [`OpAmpOutput`] struct,
    /// preventing it being used elsewhere. The `OpAmpOutput` can then be
    /// directly used as an ADC input. The opamp will be disabled when the
    /// [`OpAmpOutput`] is dropped.
    #[cfg(not(opamp_g4))]
    pub fn buffer_ext(
        &mut self,
        in_pin: impl Peripheral<P = impl NonInvertingPin<T> + crate::gpio::Pin>,
        out_pin: impl Peripheral<P = impl OutputPin<T> + crate::gpio::Pin>,
        gain: OpAmpGain,
    ) -> OpAmpOutput<'_, T> {
        into_ref!(in_pin);
        into_ref!(out_pin);
        in_pin.set_as_analog();
        out_pin.set_as_analog();

        // PGA_GAIN value may have different meaning in different MCU serials, use with caution.
        let (vm_sel, pga_gain) = match gain {
            OpAmpGain::Mul1 => (0b11, 0b00),
            OpAmpGain::Mul2 => (0b10, 0b00),
            OpAmpGain::Mul4 => (0b10, 0b01),
            OpAmpGain::Mul8 => (0b10, 0b10),
            OpAmpGain::Mul16 => (0b10, 0b11),
        };

        T::regs().csr().modify(|w| {
            w.set_vp_sel(VpSel::from_bits(in_pin.channel()));
            w.set_vm_sel(VmSel::from_bits(vm_sel));
            w.set_pga_gain(PgaGain::from_bits(pga_gain));
            #[cfg(opamp_g4)]
            w.set_opaintoen(Opaintoen::OUTPUT_PIN);
            w.set_opampen(true);
        });

        OpAmpOutput { _inner: self }
    }

    /// Configure the OpAmp as a buffer for the provided input pin,
    /// outputting to the provided output pin, and enable the opamp.
    ///
    /// If `gain` is `None`, the gain is set to 1.
    ///
    /// The input pin is configured for analogue mode but not consumed,
    /// so it may subsequently be used for ADC or comparator inputs.
    ///
    /// The output pin is held within the returned [`OpAmpOutput`] struct,
    /// preventing it being used elsewhere. The `OpAmpOutput` can then be
    /// directly used as an ADC input. The opamp will be disabled when the
    /// [`OpAmpOutput`] is dropped.
    #[cfg(opamp_g4)]
    pub fn buffer_ext(
        &mut self,
        in_pin: impl Peripheral<P = impl NonInvertingPin<T> + crate::gpio::Pin>,
        out_pin: impl Peripheral<P = impl OutputPin<T> + crate::gpio::Pin>,
        gain: Option<OpAmpGain>,
    ) -> OpAmpOutput<'_, T> {
        into_ref!(in_pin);
        into_ref!(out_pin);
        in_pin.set_as_analog();
        out_pin.set_as_analog();

        let vm_sel = match gain {
            None => VmSel::OUTPUT,
            Some(_) => VmSel::PGA,
        };

        T::regs().csr().modify(|w| {
            w.set_vp_sel(VpSel::from_bits(in_pin.channel()));
            w.set_vm_sel(vm_sel);
            if let Some(gain) = gain {
                w.set_pga_gain(PgaGain::from_bits(gain.into()));
            }
            w.set_opaintoen(Opaintoen::OUTPUT_PIN);
            w.set_opampen(true);
        });

        OpAmpOutput { _inner: self }
    }

    /// Configure the OpAmp as a buffer for the DAC it is connected to,
    /// outputting to the provided output pin, and enable the opamp.
    ///
    /// If `gain` is `None`, the gain is set to 1.
    ///
    /// The output pin is held within the returned [`OpAmpOutput`] struct,
    /// preventing it being used elsewhere. The `OpAmpOutput` can then be
    /// directly used as an ADC input. The opamp will be disabled when the
    /// [`OpAmpOutput`] is dropped.
    #[cfg(opamp_g4)]
    pub fn buffer_dac(
        &mut self,
        out_pin: impl Peripheral<P = impl OutputPin<T> + crate::gpio::Pin>,
    ) -> OpAmpOutput<'_, T> {
        into_ref!(out_pin);
        out_pin.set_as_analog();

        T::regs().csr().modify(|w| {
            use crate::pac::opamp::vals::*;

            w.set_vm_sel(VmSel::OUTPUT);
            w.set_vp_sel(VpSel::DAC3_CH1);
            w.set_opaintoen(Opaintoen::OUTPUT_PIN);
            w.set_opampen(true);
        });

        OpAmpOutput { _inner: self }
    }

    /// Configure the OpAmp as a PGA for the DAC it is connected to,
    /// outputting to the provided output pin, and enable the opamp.
    /// 
    /// 
    ///
    /// The output pin is held within the returned [`OpAmpOutput`] struct,
    /// preventing it being used elsewhere. The `OpAmpOutput` can then be
    /// directly used as an ADC input. The opamp will be disabled when the
    /// [`OpAmpOutput`] is dropped.
    #[cfg(opamp_g4)]
    pub fn pga_dac_ext(
        &mut self,
        in0_pin: impl Peripheral<P = impl InvertingPin<T> + crate::gpio::Pin>,
        in1_pin: Option<impl Peripheral<P = impl InvertingPin<T> + crate::gpio::Pin>>,
        out_pin: impl Peripheral<P = impl OutputPin<T> + crate::gpio::Pin>,
        gain: OpAmpGain,
    ) -> OpAmpOutput<'_, T> {
        into_ref!(in0_pin);
        in0_pin.set_as_analog();
        if let Some(in1_pin) = in1_pin {
            into_ref!(in1_pin);
            in1_pin.set_as_analog();
        }
        into_ref!(out_pin);
        out_pin.set_as_analog();

        T::regs().csr().modify(|w| {
            use crate::pac::opamp::vals::*;

            w.set_pga_gain(PgaGain::from_bits(gain.into()));
            w.set_vm_sel(VmSel::VINM0);
            w.set_vp_sel(VpSel::DAC3_CH1); // DAC3_CHx
            w.set_opaintoen(Opaintoen::OUTPUT_PIN);
            w.set_opampen(true);
        });

        OpAmpOutput { _inner: self }
    }
    /// Configure the OpAmp as a buffer for the provided input pin,
    /// with the output only used internally, and enable the opamp.
    ///
    /// The input pin is configured for analogue mode but not consumed,
    /// so it may be subsequently used for ADC or comparator inputs.
    ///
    /// The returned `OpAmpInternalOutput` struct may be used as an ADC input.
    /// The opamp output will be disabled when it is dropped.
    #[cfg(opamp_g4)]
    pub fn buffer_int(
        &mut self,
        pin: impl Peripheral<P = impl NonInvertingPin<T> + crate::gpio::Pin>,
        gain: Option<OpAmpGain>,
    ) -> OpAmpInternalOutput<'_, T> {
        into_ref!(pin);
        pin.set_as_analog();

        let vm_sel = match gain {
            None => VmSel::OUTPUT,
            Some(_) => VmSel::PGA,
        };

        T::regs().csr().modify(|w| {
            use crate::pac::opamp::vals::*;
            w.set_vp_sel(VpSel::from_bits(pin.channel()));
            w.set_vm_sel(vm_sel);
            if let Some(gain) = gain {
                w.set_pga_gain(PgaGain::from_bits(gain.into()));
            }
            w.set_opaintoen(Opaintoen::ADCCHANNEL);
            w.set_opampen(true);
        });

        OpAmpInternalOutput { _inner: self }
    }
}

impl<'d, T: Instance> Drop for OpAmpOutput<'d, T> {
    fn drop(&mut self) {
        T::regs().csr().modify(|w| {
            w.set_opampen(false);
        });
    }
}

#[cfg(opamp_g4)]
impl<'d, T: Instance> Drop for OpAmpInternalOutput<'d, T> {
    fn drop(&mut self) {
        T::regs().csr().modify(|w| {
            w.set_opampen(false);
        });
    }
}

pub(crate) trait SealedInstance {
    fn regs() -> crate::pac::opamp::Opamp;
}

pub(crate) trait SealedNonInvertingPin<T: Instance> {
    fn channel(&self) -> u8;
}

pub(crate) trait SealedInvertingPin<T: Instance> {
    #[allow(unused)]
    fn channel(&self) -> u8;
}

pub(crate) trait SealedOutputPin<T: Instance> {}

/// Opamp instance trait.
#[allow(private_bounds)]
pub trait Instance: SealedInstance + 'static {}
/// Non-inverting pin trait.
#[allow(private_bounds)]
pub trait NonInvertingPin<T: Instance>: SealedNonInvertingPin<T> {}
/// Inverting pin trait.
#[allow(private_bounds)]
pub trait InvertingPin<T: Instance>: SealedInvertingPin<T> {}
/// Output pin trait.
#[allow(private_bounds)]
pub trait OutputPin<T: Instance>: SealedOutputPin<T> {}

macro_rules! impl_opamp_external_output {
    ($inst:ident, $adc:ident, $ch:expr) => {
        foreach_adc!(
            ($adc, $common_inst:ident, $adc_clock:ident) => {
                impl<'d> crate::adc::SealedAdcChannel<crate::peripherals::$adc>
                    for OpAmpOutput<'d, crate::peripherals::$inst>
                {
                    fn channel(&self) -> u8 {
                        $ch
                    }
                }

                impl<'d> crate::adc::AdcChannel<crate::peripherals::$adc>
                    for OpAmpOutput<'d, crate::peripherals::$inst>
                {
                }
            };
        );
    };
}

foreach_peripheral!(
    (opamp, OPAMP1) => {
        impl_opamp_external_output!(OPAMP1, ADC1, 3);
    };
    (opamp, OPAMP2) => {
        impl_opamp_external_output!(OPAMP2, ADC2, 3);
    };
    (opamp, OPAMP3) => {
        impl_opamp_external_output!(OPAMP3, ADC1, 12);
        impl_opamp_external_output!(OPAMP3, ADC3, 1);
    };
    // OPAMP4 only in STM32G4 Cat 3 devices
    (opamp, OPAMP4) => {
        impl_opamp_external_output!(OPAMP4, ADC1, 11);
        impl_opamp_external_output!(OPAMP4, ADC4, 3);
    };
    // OPAMP5 only in STM32G4 Cat 3 devices
    (opamp, OPAMP5) => {
        impl_opamp_external_output!(OPAMP5, ADC5, 1);
    };
    // OPAMP6 only in STM32G4 Cat 3/4 devices
    (opamp, OPAMP6) => {
        impl_opamp_external_output!(OPAMP6, ADC1, 14);
        impl_opamp_external_output!(OPAMP6, ADC2, 14);
    };
);

#[cfg(opamp_g4)]
macro_rules! impl_opamp_internal_output {
    ($inst:ident, $adc:ident, $ch:expr) => {
        foreach_adc!(
            ($adc, $common_inst:ident, $adc_clock:ident) => {
                impl<'d> crate::adc::SealedAdcChannel<crate::peripherals::$adc>
                    for OpAmpInternalOutput<'d, crate::peripherals::$inst>
                {
                    fn channel(&self) -> u8 {
                        $ch
                    }
                }

                impl<'d> crate::adc::AdcChannel<crate::peripherals::$adc>
                    for OpAmpInternalOutput<'d, crate::peripherals::$inst>
                {
                }
            };
        );
    };
}

#[cfg(opamp_g4)]
foreach_peripheral!(
    (opamp, OPAMP1) => {
        impl_opamp_internal_output!(OPAMP1, ADC1, 13);
    };
    (opamp, OPAMP2) => {
        impl_opamp_internal_output!(OPAMP2, ADC2, 16);
    };
    (opamp, OPAMP3) => {
        impl_opamp_internal_output!(OPAMP3, ADC2, 18);
        // Only in Cat 3/4 devices
        impl_opamp_internal_output!(OPAMP3, ADC3, 13);
    };
    // OPAMP4 only in Cat 3 devices
    (opamp, OPAMP4) => {
        impl_opamp_internal_output!(OPAMP4, ADC5, 5);
    };
    // OPAMP5 only in Cat 3 devices
    (opamp, OPAMP5) => {
        impl_opamp_internal_output!(OPAMP5, ADC5, 3);
    };
    // OPAMP6 only in Cat 3/4 devices
    (opamp, OPAMP6) => {
        // Only in Cat 3 devices
        impl_opamp_internal_output!(OPAMP6, ADC4, 17);
        // Only in Cat 4 devices
        impl_opamp_internal_output!(OPAMP6, ADC3, 17);
    };
);

foreach_peripheral! {
    (opamp, $inst:ident) => {
        impl SealedInstance for crate::peripherals::$inst {
            fn regs() -> crate::pac::opamp::Opamp {
                crate::pac::$inst
            }
        }

        impl Instance for crate::peripherals::$inst {
        }
    };
}

#[allow(unused_macros)]
macro_rules! impl_opamp_vp_pin {
    ($inst:ident, $pin:ident, $ch:expr) => {
        impl crate::opamp::NonInvertingPin<peripherals::$inst> for crate::peripherals::$pin {}
        impl crate::opamp::SealedNonInvertingPin<peripherals::$inst> for crate::peripherals::$pin {
            fn channel(&self) -> u8 {
                $ch
            }
        }
    };
}

#[allow(unused_macros)]
macro_rules! impl_opamp_vn_pin {
    ($inst:ident, $pin:ident, $ch:expr) => {
        impl crate::opamp::InvertingPin<peripherals::$inst> for crate::peripherals::$pin {}
        impl crate::opamp::SealedInvertingPin<peripherals::$inst> for crate::peripherals::$pin {
            fn channel(&self) -> u8 {
                $ch
            }
        }
    };
}

#[allow(unused_macros)]
macro_rules! impl_opamp_vout_pin {
    ($inst:ident, $pin:ident) => {
        impl crate::opamp::OutputPin<peripherals::$inst> for crate::peripherals::$pin {}
        impl crate::opamp::SealedOutputPin<peripherals::$inst> for crate::peripherals::$pin {}
    };
}
