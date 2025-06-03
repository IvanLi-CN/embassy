//! Comparator (COMP)
#![macro_use]

use crate::gpio::{self, AfType, OutputType, Speed};
use crate::pac::comp::vals;
use crate::pac::comp::Comp as CompRegBlock;
use embassy_hal_internal::PeripheralType; // Removed into_ref again
use embassy_sync::waitqueue::AtomicWaker;

use crate::interrupt::typelevel::Binding;
use crate::interrupt::typelevel::Interrupt;
use crate::pac; // Added PAC for EXTI access
use crate::peripherals;
use crate::Peri;

use core::future::Future;
use core::marker::PhantomData;
use core::pin::Pin;
use core::task::{Context, Poll};

fn blocking_delay_us(ms: u32) {
    #[cfg(feature = "time")]
    embassy_time::block_for(embassy_time::Duration::from_millis(ms as u64));
    #[cfg(not(feature = "time"))]
    cortex_m::asm::delay(unsafe { crate::rcc::get_freqs() }.sys.to_hertz().unwrap().0 / 1_000 * ms);
}

/// COMP error
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// Invalid input pin selection for the current COMP instance or mode.
    InvalidInputPin,
    /// The selected VREFINT division is not available or invalid.
    InvalidVrefIntDivision,
    /// The selected DAC channel is not a valid input for this COMP instance.
    InvalidDacInput,
    /// The selected IO pin is not valid for inverting/non-inverting input.
    InvalidIoPin,
    /// The requested power mode is not supported or could not be set.
    InvalidPowerMode,
    /// The requested hysteresis level is not supported.
    InvalidHysteresis,
    /// The blanking source is not valid for this COMP instance.
    InvalidBlankingSource,
    /// Comparator timed out during an operation (e.g., startup).
    Timeout,
    /// An unspecified hardware error occurred.
    Hardware,
}

/// Internal sources for Inverting input selection
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum InvertingInputInternal {
    /// 1/4 VREFINT (INMSEL = 000)
    VrefIntDiv4,
    /// 1/2 VREFINT (INMSEL = 001)
    VrefIntDiv2,
    /// 3/4 VREFINT (INMSEL = 010)
    VrefIntDiv4Mul3,
    /// VREFINT (INMSEL = 011)
    VrefInt,
    /// DAC1 Channel 1 (INMSEL = 101, instance-dependent)
    Dac1Ch1,
    /// DAC1 Channel 2 (INMSEL = 101, instance-dependent)
    Dac1Ch2,
    /// DAC2 Channel 1 (INMSEL = 101, instance-dependent)
    Dac2Ch1,
    /// DAC3 Channel 1 (INMSEL = 100, instance-dependent)
    Dac3Ch1,
    /// DAC3 Channel 2 (INMSEL = 100, instance-dependent)
    Dac3Ch2,
}

// NonInvertingInput only supports IO pins according to Table 196.
// If internal sources were supported, a NonInvertingInputInternal enum would be here.

/// Hysteresis level
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Hysteresis {
    /// No hysteresis (0mV)
    None,
    /// 10mV hysteresis
    Lvl10mV,
    /// 20mV hysteresis
    Lvl20mV,
    /// 30mV hysteresis
    Lvl30mV,
    /// 40mV hysteresis
    Lvl40mV,
    /// 50mV hysteresis
    Lvl50mV,
    /// 60mV hysteresis
    Lvl60mV,
    /// 70mV hysteresis
    Lvl70mV,
}

// Duplicate Hysteresis enum definition removed.

/// Blanking source selection for the comparator output.
///
/// Note: The availability of these options and their corresponding `BLANKSEL` bit values
/// can be specific to the COMP instance (COMP1-COMP7) and the particular STM32G4 variant.
/// Consult the device's Reference Manual (e.g., RM0440, Table "COMPx_CSR register description"
/// and "Blanking sources" - specifically Table 198 for BLANKSEL[2:0] values) for definitive mappings.
/// The `Comp::new` function will map these to the `BLANKSEL` field (3-bits, CSR bits [21:19])
/// using the PAC-generated `set_blanksel` method.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum BlankingSource {
    /// No blanking. (BLANKSEL[2:0] = 0b000, Table 198)
    None,

    // --- TIM1 ---
    /// TIM1 OC5 output. (Refer to Table 198 for BLANKSEL[2:0] mapping per COMP instance)
    Tim1Oc5,

    // --- TIM2 ---
    /// TIM2 OC3 output. (BLANKSEL[2:0] = 0b010, Table 198)
    Tim2Oc3,
    /// TIM2 OC4 output. (Refer to Table 198 for BLANKSEL[2:0] mapping per COMP instance)
    Tim2Oc4,

    // --- TIM3 ---
    /// TIM3 OC3 output. (Refer to Table 198 for BLANKSEL[2:0] mapping per COMP instance)
    Tim3Oc3,
    /// TIM3 OC4 output. (Refer to Table 198 for BLANKSEL[2:0] mapping per COMP instance)
    Tim3Oc4,

    // --- TIM4 ---
    /// TIM4 OC3 output. (Refer to Table 198 for BLANKSEL[2:0] mapping per COMP instance)
    Tim4Oc3,

    // TIM5 outputs are not listed as blanking sources in RM0440 Table 198.

    // --- TIM8 ---
    /// TIM8 OC5 output. (Refer to Table 198 for BLANKSEL[2:0] mapping per COMP instance)
    Tim8Oc5,

    // --- TIM15 ---
    /// TIM15 OC1 output. (Refer to Table 198 for BLANKSEL[2:0] mapping per COMP instance)
    Tim15Oc1,
    /// TIM15 OC2 output. (Refer to Table 198 for BLANKSEL[2:0] mapping per COMP instance)
    Tim15Oc2,

    // TIM16 outputs are not listed as blanking sources in RM0440 Table 198.

    // TIM17 outputs are not listed as blanking sources in RM0440 Table 198.

    // --- TIM20 ---
    // Note: TIM20 might not be available on all G4 variants or for all COMP instances.
    /// TIM20 OC5 output. (Refer to Table 198 for BLANKSEL[2:0] mapping per COMP instance)
    Tim20Oc5,
    // User should verify and extend this enum based on their specific needs and target device's RM.
}
// InvertingInput<PInv> enum removed as its role is replaced by distinct new_* functions.

/// Common configuration for the Comparator.
#[derive(Clone, Copy, Debug)]
pub struct CompCommonConfig {
    /// Hysteresis level for the comparator.
    pub hysteresis: Hysteresis,
    /// Blanking source for the comparator output.
    pub blanking_source: BlankingSource,
    /// Whether the comparator output is inverted.
    pub output_inverted: bool,
    /// Comparator startup delay in microseconds. The constructor will perform this delay.
    pub startup_delay_us: u32,
}

impl CompCommonConfig {
    /// Creates a new `CompCommonConfig` instance.
    ///
    /// # Arguments
    ///
    /// * `hysteresis`: Hysteresis level for the comparator.
    /// * `blanking_source`: Blanking source for the comparator output.
    /// * `output_inverted`: Whether the comparator output is inverted.
    /// * `startup_delay_us`: Comparator startup delay in microseconds. The constructor will perform this delay.
    ///
    /// # Returns
    ///
    /// A new `CompCommonConfig` instance with the specified settings.
    pub fn new(
        hysteresis: Hysteresis,
        blanking_source: BlankingSource,
        output_inverted: bool,
        startup_delay_us: u32,
    ) -> Self {
        Self {
            hysteresis,
            blanking_source,
            output_inverted,
            startup_delay_us,
        }
    }
}
impl Default for CompCommonConfig {
    /// Creates a default `CompCommonConfig`.
    ///
    /// Default values are:
    /// - `hysteresis`: `Hysteresis::None`
    /// - `blanking_source`: `BlankingSource::None`
    /// - `output_inverted`: `false`
    /// - `startup_delay_us`: `5`
    fn default() -> Self {
        Self {
            hysteresis: Hysteresis::None,
            blanking_source: BlankingSource::None,
            output_inverted: false,
            startup_delay_us: 5,
        }
    }
}

/// Detailed status of the Comparator
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CompDetailedStatus {
    /// Current output value of the comparator. True if high, false if low.
    pub output_value: bool,
    /// Indicates if the comparator is currently enabled.
    pub is_enabled: bool,
    /// Indicates if the comparator's CSR register is locked.
    pub is_locked: bool,
}

/// Main Comparator driver struct.
pub struct Comp<'d, T: Instance> {
    _peri: Peri<'d, T>, // Restored original field name
    common_config: CompCommonConfig,
}

impl<'d, T: Instance> Comp<'d, T> {
    /// Creates a new Comparator instance.
    pub fn new(
        peri: Peri<'d, T>,
        _irq: impl Binding<T::Interrupt, CompInterruptHandler<T>> + 'd,
        common_config: CompCommonConfig,
    ) -> Self {
        T::Interrupt::unpend();
        unsafe { T::Interrupt::enable() };
        // into_ref!(peri); // Removed non-existent macro call
        Self {
            _peri: peri, // Assign directly
            common_config,
        }
    }
}

/// Represents an active Comparator peripheral without a configured output pin.
/// Disables the comparator on drop.
pub struct CompActive<'d, T: Instance> {
    _peri: Peri<'d, T>,
    output_inverted: bool,
}

impl<'d, T: Instance> Drop for CompActive<'d, T> {
    fn drop(&mut self) {
        let regs = T::regs_comp();
        if regs.csr().read().lock() == vals::Lock::UNLOCKED {
            regs.csr().modify(|w| {
                w.set_en(vals::En::DISABLED);
            });
        }
    }
}

impl<'d, T: Instance> CompActive<'d, T> {
    // Note: Consider if a constructor for CompActive is needed, or if it's always created by Comp methods.
    // For now, assuming it's created by Comp methods.
    /// Reads the current output level of the comparator.
    ///
    /// Returns `true` if the output is high, `false` if low.
    /// This directly reflects the `VALUE` bit in the COMP_CSR register.
    pub fn output_level(&self) -> bool {
        T::regs_comp().csr().read().value_do_not_set()
    }

    /// Gets the detailed status of the comparator.
    ///
    /// Returns a `CompDetailedStatus` struct containing the output value,
    /// enabled state, and lock state.
    pub fn detailed_status(&self) -> CompDetailedStatus {
        let csr = T::regs_comp().csr().read();
        CompDetailedStatus {
            output_value: csr.value_do_not_set(),
            is_enabled: csr.en() == vals::En::ENABLED,
            is_locked: csr.lock() == vals::Lock::LOCKED,
        }
    }

    /// Locks the comparator configuration.
    ///
    /// Once locked, the comparator configuration (except for enable/disable)
    /// cannot be changed until a system reset.
    pub fn lock(&mut self) {
        T::regs_comp().csr().modify(|w| w.set_lock(vals::Lock::LOCKED));
    }

    /// Enables the comparator.
    ///
    /// This function will only enable the comparator if it is not locked.
    pub fn enable(&mut self) {
        let regs = T::regs_comp();
        if regs.csr().read().lock() == vals::Lock::UNLOCKED {
            regs.csr().modify(|w| w.set_en(vals::En::ENABLED));
        }
    }

    /// Disables the comparator.
    ///
    /// This function will only disable the comparator if it is not locked.
    pub fn disable(&mut self) {
        let regs = T::regs_comp();
        if regs.csr().read().lock() == vals::Lock::UNLOCKED {
            regs.csr().modify(|w| w.set_en(vals::En::DISABLED));
        }
    }
}

/// Represents an active Comparator peripheral with a configured output pin.
/// Disables the comparator and resets the output pin's alternate function on drop.
pub struct CompActiveOut<'d, T: Instance, POut: OutputPin<T> + 'd> {
    _peri: Peri<'d, T>,
    output_pin: Peri<'d, POut>,
    output_inverted: bool,
}

impl<'d, T: Instance, POut: OutputPin<T> + 'd> Drop for CompActiveOut<'d, T, POut> {
    fn drop(&mut self) {
        let regs = T::regs_comp();
        if regs.csr().read().lock() == vals::Lock::UNLOCKED {
            regs.csr().modify(|w| {
                w.set_en(vals::En::DISABLED);
            });
        }
        self.output_pin.set_as_analog();
    }
}

impl<'d, T: Instance, POut: OutputPin<T> + 'd> CompActiveOut<'d, T, POut> {
    // Note: Consider if a constructor for CompActiveOut is needed.
    // For now, assuming it's created by Comp methods.
    /// Reads the current output level of the comparator.
    ///
    /// Returns `true` if the output is high, `false` if low.
    /// This directly reflects the `VALUE` bit in the COMP_CSR register.
    pub fn output_level(&self) -> bool {
        T::regs_comp().csr().read().value_do_not_set()
    }

    /// Gets the detailed status of the comparator.
    ///
    /// Returns a `CompDetailedStatus` struct containing the output value,
    /// enabled state, and lock state.
    pub fn detailed_status(&self) -> CompDetailedStatus {
        let csr = T::regs_comp().csr().read();
        CompDetailedStatus {
            output_value: csr.value_do_not_set(),
            is_enabled: csr.en() == vals::En::ENABLED,
            is_locked: csr.lock() == vals::Lock::LOCKED,
        }
    }

    /// Locks the comparator configuration.
    ///
    /// Once locked, the comparator configuration (except for enable/disable)
    /// cannot be changed until a system reset.
    pub fn lock(&mut self) {
        T::regs_comp().csr().modify(|w| w.set_lock(vals::Lock::LOCKED));
    }

    /// Enables the comparator.
    ///
    /// This function will only enable the comparator if it is not locked.
    pub fn enable(&mut self) {
        let regs = T::regs_comp();
        if regs.csr().read().lock() == vals::Lock::UNLOCKED {
            regs.csr().modify(|w| w.set_en(vals::En::ENABLED));
        }
    }

    /// Disables the comparator.
    ///
    /// This function will only disable the comparator if it is not locked.
    pub fn disable(&mut self) {
        let regs = T::regs_comp();
        if regs.csr().read().lock() == vals::Lock::UNLOCKED {
            regs.csr().modify(|w| w.set_en(vals::En::DISABLED));
        }
    }
}

/// Interrupt handler for COMP instances.
pub struct CompInterruptHandler<T: Instance>(PhantomData<T>);

impl<T: Instance> crate::interrupt::typelevel::Handler<T::Interrupt> for CompInterruptHandler<T> {
    unsafe fn on_interrupt() {
        let exti_line = T::exti_line();
        let comp_idx = T::comp_index();

        let bank = exti_line as usize / 32;
        let line_in_bank_mask = 1 << (exti_line % 32);

        // Critical section to ensure atomic operations on EXTI registers
        critical_section::with(|_| {
            // Check if the specific EXTI line for this COMP instance is pending and enabled
            // This check is important if T::Interrupt is a shared EXTI vector.
            // We assume the interrupt controller already routed to this handler because T::Interrupt fired.
            // The main job here is to service the line: mask, wake, clear.

            let imr_val = pac::EXTI.imr(bank).read().0;
            let pr_val = pac::EXTI.pr(bank).read().0; // Or RPR/FPR for G4

            // Ensure the interrupt for this line was enabled and is pending
            // For G4, check RPR/FPR if PR is not the source of truth for pending status
            #[cfg(any(exti_c0, exti_g0, exti_u0, exti_l5, exti_u5, exti_h5, exti_h50))]
            let is_pending = (pac::EXTI.rpr(bank).read().0 & line_in_bank_mask) != 0
                || (pac::EXTI.fpr(bank).read().0 & line_in_bank_mask) != 0;
            #[cfg(not(any(exti_c0, exti_g0, exti_u0, exti_l5, exti_u5, exti_h5, exti_h50)))]
            let is_pending = (pr_val & line_in_bank_mask) != 0;

            if (imr_val & line_in_bank_mask) != 0 && is_pending {
                // Mask the interrupt line (disable it in IMR)
                pac::EXTI.imr(bank).modify(|r| r.0 &= !line_in_bank_mask);

                COMP_WAKERS[comp_idx].wake();

                // Clear the pending bit by writing 1 to it
                // For G4 series (and others like L5, U5, H5), EXTI pending bits in RPR and FPR are cleared by writing 1.
                #[cfg(any(exti_c0, exti_g0, exti_u0, exti_l5, exti_u5, exti_h5, exti_h50))]
                {
                    pac::EXTI
                        .rpr(bank)
                        .write_value(pac::exti::regs::Lines(line_in_bank_mask));
                    pac::EXTI
                        .fpr(bank)
                        .write_value(pac::exti::regs::Lines(line_in_bank_mask));
                }
                // For other series, PR bits are cleared by writing 1.
                #[cfg(not(any(exti_c0, exti_g0, exti_u0, exti_l5, exti_u5, exti_h5, exti_h50)))]
                pac::EXTI
                    .pr(bank)
                    .write_value(pac::exti::regs::Lines(line_in_bank_mask));
            }
        });
    }
}

// Helper to map BlankingSource to PAC value
fn map_blanking_source_val(blanking_source: BlankingSource, comp_idx: usize) -> Result<vals::Blanksel, Error> {
    match blanking_source {
        BlankingSource::None => Ok(vals::Blanksel::_RESERVED_0), // 000
        BlankingSource::Tim1Oc5 => match comp_idx {
            0..=3 | 6 => Ok(vals::Blanksel::TIM1_OC5), // COMP1-4, COMP7 (BLANKSEL 001)
            _ => Err(Error::InvalidBlankingSource),         // TIM1_OC5 not for COMP5/6 via BLANKSEL 001
        },
        BlankingSource::Tim2Oc3 => match comp_idx {
            0..=2 => Ok(vals::Blanksel::TIM2_OC3), // COMP1-3 (BLANKSEL 010)
            _ => Err(Error::InvalidBlankingSource),
        },
        BlankingSource::Tim2Oc4 => match comp_idx {
            2 | 4 | 5 => Ok(vals::Blanksel::TIM3_OC3_TIM2_OC4_TIM15_OC1), // COMP3,5,6 (BLANKSEL 011)
            3 => Ok(vals::Blanksel::TIM8_OC5_TIM1_OC5_TIM15_OC2),         // COMP4 (BLANKSEL 100)
            _ => Err(Error::InvalidBlankingSource),
        },
        BlankingSource::Tim3Oc3 => match comp_idx {
            0..=1 => Ok(vals::Blanksel::TIM3_OC3_TIM2_OC4_TIM15_OC1), // COMP1-2 (BLANKSEL 011)
            _ => Err(Error::InvalidBlankingSource),
        },
        BlankingSource::Tim3Oc4 => match comp_idx {
            2 | 6 => Ok(vals::Blanksel::TIM8_OC5_TIM1_OC5_TIM15_OC2), // COMP3,7 (BLANKSEL 100)
            3 => Ok(vals::Blanksel::TIM20_OC5_TIM1_OC5),              // COMP4 (BLANKSEL 101)
            4 | 5 => Ok(vals::Blanksel::TIM15_OC1),                   // COMP5,6 (BLANKSEL 110)
            _ => Err(Error::InvalidBlankingSource),
        },
        BlankingSource::Tim4Oc3 => {
            Ok(vals::Blanksel::TIM4_OC3) // BLANKSEL 111
        }
        BlankingSource::Tim8Oc5 => match comp_idx {
            0 | 1 => Ok(vals::Blanksel::TIM8_OC5_TIM1_OC5_TIM15_OC2), // COMP1,2 (BLANKSEL 100)
            2 | 3 => Ok(vals::Blanksel::TIM20_OC5_TIM1_OC5),          // COMP3,4 (BLANKSEL 101)
            4 | 5 => Ok(vals::Blanksel::TIM1_OC5),                    // COMP5,6 (BLANKSEL 001)
            6 => Ok(vals::Blanksel::TIM2_OC3),                        // COMP7 (BLANKSEL 010)
            _ => Err(Error::InvalidBlankingSource),
        },
        BlankingSource::Tim15Oc1 => match comp_idx {
            0 | 1 | 2 => Ok(vals::Blanksel::TIM15_OC1), // COMP1-3 (BLANKSEL 110)
            3 | 6 => Ok(vals::Blanksel::TIM3_OC3_TIM2_OC4_TIM15_OC1), // COMP4,7 (BLANKSEL 011)
            _ => Err(Error::InvalidBlankingSource),
        },
        BlankingSource::Tim15Oc2 => match comp_idx {
            4 | 5 => Ok(vals::Blanksel::TIM8_OC5_TIM1_OC5_TIM15_OC2), // COMP5,6 (BLANKSEL 100)
            _ => Err(Error::InvalidBlankingSource),
        },
        BlankingSource::Tim20Oc5 => match comp_idx {
            0 | 1 | 4 | 5 | 6 => Ok(vals::Blanksel::TIM20_OC5_TIM1_OC5), // COMP1,2,5,6,7 (BLANKSEL 101)
            3 => Ok(vals::Blanksel::TIM15_OC1),                          // COMP4 (BLANKSEL 110)
            _ => Err(Error::InvalidBlankingSource),
        },
    }
}

// Common internal configuration function
fn configure_and_enable_comp<T: Instance>(
    regs: &crate::pac::comp::Comp, // Borrowing regs
    inverting_input_source_val: vals::Inmsel,
    enable_scalen_val: vals::Scalen,
    non_inverting_pin_val: vals::Inpsel,
    common_config: &CompCommonConfig,
) -> Result<(), Error> {
    // Ensure comparator is disabled before configuration
    regs.csr().modify(|w| w.set_en(vals::En::DISABLED));

    let comp_idx = T::comp_index();
    let final_blanksel_val = map_blanking_source_val(common_config.blanking_source, comp_idx)?;

    regs.csr().modify(|w| {
        w.set_inmsel(inverting_input_source_val);
        w.set_scalen(enable_scalen_val);
        w.set_inpsel(non_inverting_pin_val);


        if matches!(inverting_input_source_val, vals::Inmsel::VREFINT_1_4
            | vals::Inmsel::VREFINT_1_2 | vals::Inmsel::VREFINT_3_4
        ) {
            w.set_brgen(vals::Brgen::ENABLED);
            w.set_scalen(vals::Scalen::ENABLED);
        }

        let hyst_val = match common_config.hysteresis {
            Hysteresis::None => vals::Hyst::NONE,
            Hysteresis::Lvl10mV => vals::Hyst::HYST10M,
            Hysteresis::Lvl20mV => vals::Hyst::HYST20M,
            Hysteresis::Lvl30mV => vals::Hyst::HYST30M,
            Hysteresis::Lvl40mV => vals::Hyst::HYST40M,
            Hysteresis::Lvl50mV => vals::Hyst::HYST50M,
            Hysteresis::Lvl60mV => vals::Hyst::HYST60M,
            Hysteresis::Lvl70mV => vals::Hyst::HYST70M,
        };
        w.set_hyst(hyst_val);

        w.set_polarity(if common_config.output_inverted {
            vals::Polarity::INVERTED
        } else {
            vals::Polarity::NON_INVERTED
        });

        w.set_blanksel(final_blanksel_val);
    });

    regs.csr().modify(|w| w.set_en(vals::En::ENABLED));

    if common_config.startup_delay_us > 0 {
        let mut delay_ms = common_config.startup_delay_us / 1000; // Convert microseconds to milliseconds
        if delay_ms == 0 && common_config.startup_delay_us > 0 {
            // If the original delay_us is > 0 but < 1000us,
            // enforce a minimum delay of 1 millisecond as we use a ms-based delay function.
            delay_ms = 1;
        }

        if delay_ms > 0 {
            self::blocking_delay_us(delay_ms); // delay_ms is already u32 if startup_delay_us was u32
        }
    }

    Ok(())
}

impl<'d, T: Instance> Comp<'d, T> {
    /// Creates a new Comparator with an internal inverting input and no dedicated output pin.
    ///
    /// - `inm`: The internal source for the inverting input.
    /// - `non_inverting_pin`: The GPIO pin for the non-inverting input.
    /// - `common_config`: Common comparator settings.
    pub fn int<PNInv>(
        self, // CHANGED: Takes ownership of self
        // _irq parameter removed
        non_inverting_pin: Peri<'d, PNInv>,
        inverting: InvertingInputInternal,
    ) -> Result<CompActive<'d, T>, Error>
    where
        PNInv: NonInvertingPin<T> + 'd,
    {
        non_inverting_pin.set_as_analog();

        let regs = T::regs_comp();

        // Interrupt unpend and enable moved to Comp::new

        let (inm_sel_val, enable_scalen_val) = match inverting {
            InvertingInputInternal::VrefIntDiv4 => (vals::Inmsel::VREFINT_1_4, vals::Scalen::ENABLED),
            InvertingInputInternal::VrefIntDiv2 => (vals::Inmsel::VREFINT_1_2, vals::Scalen::ENABLED),
            InvertingInputInternal::VrefIntDiv4Mul3 => (vals::Inmsel::VREFINT_3_4, vals::Scalen::ENABLED),
            InvertingInputInternal::VrefInt => (vals::Inmsel::VREFINT, vals::Scalen::ENABLED),
            InvertingInputInternal::Dac1Ch1 => (vals::Inmsel::DAC_CH1, vals::Scalen::DISABLED),
            InvertingInputInternal::Dac1Ch2 => (vals::Inmsel::DAC_CH2, vals::Scalen::DISABLED),
            InvertingInputInternal::Dac2Ch1 => (vals::Inmsel::DAC_CH2, vals::Scalen::DISABLED), // Ensure correct mapping for specific DAC instances if needed
            InvertingInputInternal::Dac3Ch1 => (vals::Inmsel::DAC_CH1, vals::Scalen::DISABLED),
            InvertingInputInternal::Dac3Ch2 => (vals::Inmsel::DAC_CH2, vals::Scalen::DISABLED),
        };
        let non_inv_sel_val = vals::Inpsel::from_bits(PNInv::INPSEL_VAL);

        configure_and_enable_comp::<T>(
            &regs,
            inm_sel_val,
            enable_scalen_val,
            non_inv_sel_val,
            &self.common_config,
        )?;

        Ok(CompActive {
            _peri: self._peri, // Use restored field name
            output_inverted: self.common_config.output_inverted,
        })
    }

    /// Creates a new Comparator with an external inverting input pin and no dedicated output pin.
    ///
    /// - `inverting_pin`: The GPIO pin for the inverting input.
    /// - `non_inverting_pin`: The GPIO pin for the non-inverting input.
    /// - `common_config`: Common comparator settings.
    pub fn ext<PInv, PNInv>(
        self, // CHANGED: Takes ownership of self
        // _irq parameter removed
        non_inverting_pin: Peri<'d, PNInv>,
        inverting_pin: Peri<'d, PInv>,
    ) -> Result<CompActive<'d, T>, Error>
    where
        PInv: InvertingPin<T> + 'd,
        PNInv: NonInvertingPin<T> + 'd,
    {
        inverting_pin.set_as_analog();
        non_inverting_pin.set_as_analog();

        let regs = T::regs_comp();
        // Interrupt unpend and enable moved to Comp::new

        let inm_sel_val = vals::Inmsel::from_bits(PInv::INMSEL_VAL);
        let enable_scalen_val = vals::Scalen::DISABLED; // For IO pin inputs, SCALEN is usually disabled
        let non_inv_sel_val = vals::Inpsel::from_bits(PNInv::INPSEL_VAL);

        configure_and_enable_comp::<T>(
            &regs,
            inm_sel_val,
            enable_scalen_val,
            non_inv_sel_val,
            &self.common_config,
        )?;

        Ok(CompActive {
            _peri: self._peri, // Use restored field name
            output_inverted: self.common_config.output_inverted,
        })
    }

    /// Creates a new Comparator with an internal inverting input and a dedicated output pin.
    ///
    /// - `inm`: The internal source for the inverting input.
    /// - `non_inverting_pin`: The GPIO pin for the non-inverting input.
    /// - `output_pin`: The GPIO pin to be used as comparator output.
    /// - `common_config`: Common comparator settings.
    pub fn int_out<PNInv, POut>(
        self, // CHANGED: Takes ownership of self
        // _irq parameter removed
        non_inverting_pin: Peri<'d, PNInv>,
        inverting_pin: InvertingInputInternal,
        output_pin: Peri<'d, POut>,
    ) -> Result<CompActiveOut<'d, T, POut>, Error>
    where
        PNInv: NonInvertingPin<T> + 'd,
        POut: OutputPin<T> + 'd,
    {
        non_inverting_pin.set_as_analog();
        output_pin.set_as_af(POut::AF, AfType::output(OutputType::PushPull, Speed::Medium)); // Configure output pin

        let regs = T::regs_comp();
        // Interrupt unpend and enable moved to Comp::new

        let (inm_sel_val, enable_scalen_val) = match inverting_pin {
            InvertingInputInternal::VrefIntDiv4 => (vals::Inmsel::VREFINT_1_4, vals::Scalen::ENABLED),
            InvertingInputInternal::VrefIntDiv2 => (vals::Inmsel::VREFINT_1_2, vals::Scalen::ENABLED),
            InvertingInputInternal::VrefIntDiv4Mul3 => (vals::Inmsel::VREFINT_3_4, vals::Scalen::ENABLED),
            InvertingInputInternal::VrefInt => (vals::Inmsel::VREFINT, vals::Scalen::ENABLED),
            InvertingInputInternal::Dac1Ch1 => (vals::Inmsel::DAC_CH1, vals::Scalen::DISABLED),
            InvertingInputInternal::Dac1Ch2 => (vals::Inmsel::DAC_CH2, vals::Scalen::DISABLED),
            InvertingInputInternal::Dac2Ch1 => (vals::Inmsel::DAC_CH2, vals::Scalen::DISABLED),
            InvertingInputInternal::Dac3Ch1 => (vals::Inmsel::DAC_CH1, vals::Scalen::DISABLED),
            InvertingInputInternal::Dac3Ch2 => (vals::Inmsel::DAC_CH2, vals::Scalen::DISABLED),
        };
        let non_inv_sel_val = vals::Inpsel::from_bits(PNInv::INPSEL_VAL);

        configure_and_enable_comp::<T>(
            &regs,
            inm_sel_val,
            enable_scalen_val,
            non_inv_sel_val,
            &self.common_config,
        )?;

        Ok(CompActiveOut {
            _peri: self._peri, // Use restored field name
            output_pin,
            output_inverted: self.common_config.output_inverted,
        })
    }

    /// Creates a new Comparator with external inverting and non-inverting input pins, and a dedicated output pin.
    ///
    /// - `inverting_pin`: The GPIO pin for the inverting input.
    /// - `non_inverting_pin`: The GPIO pin for the non-inverting input.
    /// - `output_pin`: The GPIO pin to be used as comparator output.
    /// - `common_config`: Common comparator settings.
    pub fn ext_out<PInv, PNInv, POut>(
        self, // CHANGED: Takes ownership of self
        // _irq parameter removed
        inverting_pin: Peri<'d, PInv>,
        non_inverting_pin: Peri<'d, PNInv>,
        output_pin: Peri<'d, POut>,
    ) -> Result<CompActiveOut<'d, T, POut>, Error>
    where
        PInv: InvertingPin<T> + 'd,
        PNInv: NonInvertingPin<T> + 'd,
        POut: OutputPin<T> + 'd,
    {
        inverting_pin.set_as_analog();
        non_inverting_pin.set_as_analog();
        output_pin.set_as_af(POut::AF, AfType::output(OutputType::PushPull, Speed::Medium)); // Configure output pin

        let regs = T::regs_comp();
        // Interrupt unpend and enable moved to Comp::new

        let inm_sel_val = vals::Inmsel::from_bits(PInv::INMSEL_VAL);
        let enable_scalen_val = vals::Scalen::DISABLED;
        let non_inv_sel_val = vals::Inpsel::from_bits(PNInv::INPSEL_VAL);

        configure_and_enable_comp::<T>(
            &regs,
            inm_sel_val,
            enable_scalen_val,
            non_inv_sel_val,
            &self.common_config,
        )?;

        Ok(CompActiveOut {
            _peri: self._peri, // Use restored field name
            output_pin,
            output_inverted: self.common_config.output_inverted,
        })
    }
}
// Drop implementation ensures the comparator is disabled when the driver goes out of scope.

static COMP_WAKERS: [AtomicWaker; 7] = [const { AtomicWaker::new() }; 7]; // Max 7 COMPs on G4

pub(crate) trait SealedInstance {
    fn regs_comp() -> CompRegBlock; // This assumes a common `Comp` register block type
                                    // For G4, individual COMPs (COMP1, COMP2, etc.) are separate peripherals in `pac`.
                                    // This method should provide access to the register block for the *specific* instance.
    fn comp_index() -> usize; // 0 for COMP1, 1 for COMP2, etc.
}

/// COMP instance trait.
#[allow(private_bounds)]
pub trait Instance: SealedInstance + PeripheralType + 'static {
    // Removed RccPeripheral
    /// Interrupt for this COMP instance. This should correspond to the EXTI line interrupt for the comparator's output.
    type Interrupt: crate::interrupt::typelevel::Interrupt;
    /// EXTI line number for this comparator's output.
    fn exti_line() -> u8;
}

// TODO: Define Pin traits for INP, INM, OUT if they have specific AF modes or configurations for COMP
// Pin traits for COMP inputs/outputs
// These help ensure type safety when selecting pins for comparator functions.
// The actual AF mapping for COMP pins is usually fixed (analog input).
// These traits primarily serve as markers.

/// Trait for a pin that can be used as a non-inverting input for a COMP instance.
pub trait NonInvertingPin<T: Instance>: gpio::Pin {
    /// The INPSEL value (0 or 1) corresponding to this pin for the given COMP instance T.
    const INPSEL_VAL: u8;
}

/// Trait for a pin that can be used as an inverting input for a COMP instance.
pub trait InvertingPin<T: Instance>: gpio::Pin {
    /// The INMSEL value (0b110 or 0b111) corresponding to this pin for the given COMP instance T.
    const INMSEL_VAL: u8;
}

/// Trait for a pin that can be used as an output for a COMP instance.
pub trait OutputPin<T: Instance>: gpio::Pin {
    /// The alternate function number for this pin when used as a COMP output.
    const AF: u8;
}

// Macros to implement pin traits, intended to be called by build.rs generated code.
#[allow(unused_macros)]
macro_rules! impl_comp_vinp_pin {
    ($peri:ident, $pin:ident, $inpsel_val:expr) => {
        // Added $inpsel_val
        impl crate::comp::NonInvertingPin<crate::peripherals::$peri> for crate::peripherals::$pin {
            const INPSEL_VAL: u8 = $inpsel_val;
        }
    };
}

#[allow(unused_macros)]
macro_rules! impl_comp_vinn_pin {
    ($peri:ident, $pin:ident, $inmsel_val:expr) => {
        // Added $inmsel_val
        impl crate::comp::InvertingPin<crate::peripherals::$peri> for crate::peripherals::$pin {
            const INMSEL_VAL: u8 = $inmsel_val;
        }
    };
}

#[allow(unused_macros)]
macro_rules! impl_comp_vout_pin {
    ($peri:ident, $pin:ident, $af:expr) => {
        impl crate::comp::OutputPin<crate::peripherals::$peri> for crate::peripherals::$pin {
            const AF: u8 = $af;
        }
    };
}
// STM32G4 specific implementations
// Assuming COMP1 to COMP7 might exist on some G4s.
// The exact PAC path for COMP registers is `pac::COMPx` which derefs to `Comp`.
// Interrupts for G4 COMPs: COMP1_2_3, COMP4_5_6_7 (example, check specific G4 variant)

macro_rules! impl_comp_instance {
    ($name:ident, $pac_module:ident, $irq:ident, $idx:expr, $exti_line:expr) => {
        impl SealedInstance for peripherals::$name {
            fn regs_comp() -> CompRegBlock {
                crate::pac::$pac_module
            }
            fn comp_index() -> usize {
                $idx
            }
        }
        impl Instance for peripherals::$name {
            type Interrupt = crate::interrupt::typelevel::$irq;
            fn exti_line() -> u8 {
                $exti_line
            }
        }
    };
}

// Example for G4 (actual instances and IRQs need to be verified from stm32-metapac for the target G4 chip)
// This is a placeholder and needs to be accurate for the specific G4 variant being targeted.
// For example, if COMP1, COMP2, COMP3 share an IRQ, and COMP4, COMP5, COMP6, COMP7 share another.
// Let's assume COMP1_2_3 and COMP4_5_6_7 are the IRQ names from metapac for G4.

foreach_peripheral!(
    // EXTI line mappings for G4 (example, verify for specific chip):
    // COMP1 -> EXTI21
    // COMP2 -> EXTI22
    // COMP3 -> EXTI29
    // COMP4 -> EXTI30
    // COMP5 -> EXTI31 (if available)
    // COMP6 -> EXTI32 (if available)
    // COMP7 -> EXTI33 (if available)
    // The IRQ (e.g., COMP1_2_3) is the EXTI IRQ that covers these lines.
    (comp, COMP1) => { impl_comp_instance!(COMP1, COMP1, COMP1_2_3, 0, 21); };
    (comp, COMP2) => { impl_comp_instance!(COMP2, COMP2, COMP1_2_3, 1, 22); };
    (comp, COMP3) => { impl_comp_instance!(COMP3, COMP3, COMP1_2_3, 2, 29); }; // Assuming COMP1_2_3 IRQ covers EXTI29
    (comp, COMP4) => { impl_comp_instance!(COMP4, COMP4, COMP4, 3, 30); };     // Assuming COMP4 IRQ covers EXTI30
    // Add COMP5, COMP6, COMP7 if they exist for the target, with correct IRQ and EXTI line
    // (comp, COMP5) => { impl_comp_instance!(COMP5, COMP5, COMP4_5_6_7, 4, 31); };
    // (comp, COMP6) => { impl_comp_instance!(COMP6, COMP6, COMP4_5_6_7, 5, 32); };
    // (comp, COMP7) => { impl_comp_instance!(COMP7, COMP7, COMP4_5_6_7, 6, 33); };
);

// Pin trait implementations for STM32G4
// TODO: These are example implementations for COMP1.
// A comprehensive mapping for all COMP instances (COMP1-COMP4 for typical G4)
// and their respective valid INP, INM, and OUT pins needs to be added,
// consulting the specific STM32G4 datasheet for correct pin assignments and alternate functions.

// Drop implementations are now on CompActive and CompActiveOut structs.
// The old Comp struct and its Drop implementation are removed.

#[must_use = "futures do nothing unless you `.await` or poll them"]
struct CompOutputFuture<T: Instance> {
    comp_idx: usize,
    exti_line: u8,
    _phantom: PhantomData<T>,
}

impl<T: Instance> CompOutputFuture<T> {
    fn new(comp_idx: usize, exti_line: u8, trigger_on_rising_exti: bool, trigger_on_falling_exti: bool) -> Self {
        critical_section::with(|_| {
            let bank = exti_line as usize / 32;
            let line_in_bank = exti_line as usize % 32;

            // Configure EXTI_RTSR and EXTI_FTSR
            pac::EXTI.rtsr(bank).modify(|r| {
                r.set_line(line_in_bank, trigger_on_rising_exti);
            });
            pac::EXTI.ftsr(bank).modify(|r| {
                r.set_line(line_in_bank, trigger_on_falling_exti);
            });

            // clear pending bit
            #[cfg(not(any(exti_c0, exti_g0, exti_u0, exti_l5, exti_u5, exti_h5, exti_h50)))]
            pac::EXTI.pr(0).write(|w| w.set_line(line_in_bank, true));
            #[cfg(any(exti_c0, exti_g0, exti_u0, exti_l5, exti_u5, exti_h5, exti_h50))]
            {
                EXTI.rpr(0).write(|w| w.set_line(line_in_bank, true));
                EXTI.fpr(0).write(|w| w.set_line(line_in_bank, true));
            }

            pac::EXTI.imr(0).modify(|w| w.set_line(line_in_bank, true));
        });

        Self {
            comp_idx,
            exti_line,
            _phantom: PhantomData,
        }
    }
}

impl<T: Instance> Drop for CompOutputFuture<T> {
    fn drop(&mut self) {
        critical_section::with(|_| {
            let bank = self.exti_line as usize / 32;
            let line_in_bank = self.exti_line as usize % 32;
            // Disable interrupt for the EXTI line when the future is dropped
            pac::EXTI.imr(bank).modify(|w| w.set_line(line_in_bank, false));
        });
    }
}

impl<T: Instance> Future for CompOutputFuture<T> {
    type Output = ();

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        COMP_WAKERS[self.comp_idx].register(cx.waker());

        critical_section::with(|_| {
            let bank = self.exti_line as usize / 32;
            let line_in_bank = self.exti_line as usize % 32;

            let imr = pac::EXTI.imr(bank).read();
            if !imr.line(line_in_bank as _) {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
    }
}

impl<'d, T: Instance> CompActive<'d, T> {
    /// Asynchronously wait until the comparator output is high.
    /// This returns immediately if the output is already high.
    pub async fn wait_for_high(&mut self) {
        if self.output_level() {
            return;
        }
        let exti_rising = !self.output_inverted;
        let exti_falling = self.output_inverted;
        CompOutputFuture::<T>::new(T::comp_index(), T::exti_line(), exti_rising, exti_falling).await
    }

    /// Asynchronously wait until the comparator output is low.
    /// This returns immediately if the output is already low.
    pub async fn wait_for_low(&mut self) {
        if !self.output_level() {
            return;
        }
        let exti_rising = self.output_inverted;
        let exti_falling = !self.output_inverted;
        CompOutputFuture::<T>::new(T::comp_index(), T::exti_line(), exti_rising, exti_falling).await
    }

    /// Asynchronously wait until the comparator output sees a rising edge (output goes Low -> High).
    /// If the output is already high, it will wait for it to go low then back high.
    pub async fn wait_for_rising_edge(&mut self) {
        CompOutputFuture::<T>::new(T::comp_index(), T::exti_line(), true, false).await
    }

    /// Asynchronously wait until the comparator output sees a falling edge (output goes High -> Low).
    /// If the output is already low, it will wait for it to go high then back low.
    pub async fn wait_for_falling_edge(&mut self) {
        CompOutputFuture::<T>::new(T::comp_index(), T::exti_line(), false, true).await;
    }

    /// Asynchronously wait until the comparator output sees any edge.
    /// This will wait for the next transition from the current level.
    pub async fn wait_for_any_edge(&mut self) {
        CompOutputFuture::<T>::new(T::comp_index(), T::exti_line(), true, true).await
    }
}

impl<'d, T: Instance, POut: OutputPin<T> + 'd> CompActiveOut<'d, T, POut> {
    /// Asynchronously wait until the comparator output is high.
    pub async fn wait_for_high(&mut self) {
        if self.output_level() {
            return;
        }
        let exti_rising = !self.output_inverted;
        let exti_falling = self.output_inverted;
        CompOutputFuture::<T>::new(T::comp_index(), T::exti_line(), exti_rising, exti_falling).await
    }

    /// Asynchronously wait until the comparator output is low.
    pub async fn wait_for_low(&mut self) {
        if !self.output_level() {
            return;
        }
        let exti_rising = self.output_inverted;
        let exti_falling = !self.output_inverted;
        CompOutputFuture::<T>::new(T::comp_index(), T::exti_line(), exti_rising, exti_falling).await
    }

    /// Asynchronously wait until the comparator output sees a rising edge.
    /// If the output is already high, it will wait for it to go low then back high.
    pub async fn wait_for_rising_edge(&mut self) {
        if self.output_level() {
            // Already high
            // Wait for it to go low first
            let fall_exti_rising = self.output_inverted;
            let fall_exti_falling = !self.output_inverted;
            CompOutputFuture::<T>::new(T::comp_index(), T::exti_line(), fall_exti_rising, fall_exti_falling).await;
        }
        // Now wait for it to go high (rising edge)
        let rise_exti_rising = !self.output_inverted;
        let rise_exti_falling = self.output_inverted;
        CompOutputFuture::<T>::new(T::comp_index(), T::exti_line(), rise_exti_rising, rise_exti_falling).await
    }

    /// Asynchronously wait until the comparator output sees a falling edge.
    /// If the output is already low, it will wait for it to go high then back low.
    pub async fn wait_for_falling_edge(&mut self) {
        if !self.output_level() {
            // Already low
            // Wait for it to go high first
            let rise_exti_rising = !self.output_inverted;
            let rise_exti_falling = self.output_inverted;
            CompOutputFuture::<T>::new(T::comp_index(), T::exti_line(), rise_exti_rising, rise_exti_falling).await;
        }
        // Now wait for it to go low (falling edge)
        let fall_exti_rising = self.output_inverted;
        let fall_exti_falling = !self.output_inverted;
        CompOutputFuture::<T>::new(T::comp_index(), T::exti_line(), fall_exti_rising, fall_exti_falling).await
    }

    /// Asynchronously wait until the comparator output sees any edge.
    /// This will wait for the next transition from the current level.
    pub async fn wait_for_any_edge(&mut self) {
        let current_level_is_high = self.output_level();

        let trigger_on_rising_exti;
        let trigger_on_falling_exti;

        if current_level_is_high {
            // Currently high, so we wait for a falling edge on the output.
            trigger_on_rising_exti = self.output_inverted;
            trigger_on_falling_exti = !self.output_inverted;
        } else {
            // Currently low, so we wait for a rising edge on the output.
            trigger_on_rising_exti = !self.output_inverted;
            trigger_on_falling_exti = self.output_inverted;
        }

        CompOutputFuture::<T>::new(
            T::comp_index(),
            T::exti_line(),
            trigger_on_rising_exti,
            trigger_on_falling_exti,
        )
        .await
    }
}
