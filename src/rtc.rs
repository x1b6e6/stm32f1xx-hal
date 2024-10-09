/*!
  Real time clock
*/
use crate::pac::{rcc::bdcr::RTCSEL, RCC, RTC};

use crate::backup_domain::BackupDomain;
use crate::time::{Hertz, Hz};

use core::convert::Infallible;
use core::marker::PhantomData;

/// RTC clock source HSE clock (type state)
///
/// NOTE: to prescaler registers will be written value of PRESCALER_HZ divided by 128
pub struct RtcClkHse<const PRESCALER_HZ: u32>;
/// RTC clock source LSE oscillator clock (type state)
pub struct RtcClkLse<const PRESCALER_HZ: u32 = 32_768>;
/// RTC clock source LSI oscillator clock (type state)
pub struct RtcClkLsi<const PRESCALER_HZ: u32 = 40_000>;

pub enum RestoredOrNewRtc<CS> {
    Restored(Rtc<CS>),
    New(Rtc<CS>),
}

/// RTC clock source with known pre-scaler and clock variant
///
/// Crate already have 3 predefined clock sources:
///
/// * [`RtcClkLse`] is LSE clock assumed to be 32'768HZ
/// * [`RtcClkLsi`] is LSI clock assumed to be 40'000HZ
/// * [`RtcClkHse`] is HSE clock with configured
pub trait RtcClockSource {
    const PRESCALER: Hertz;
    const VARIANT: RTCSEL;
}

impl<const PRESCALER_HZ: u32> RtcClockSource for RtcClkHse<PRESCALER_HZ> {
    const PRESCALER: Hertz = Hz(PRESCALER_HZ / 128);
    const VARIANT: RTCSEL = RTCSEL::Hse;
}

impl<const PRESCALER_HZ: u32> RtcClockSource for RtcClkLse<PRESCALER_HZ> {
    const PRESCALER: Hertz = Hz(PRESCALER_HZ);
    const VARIANT: RTCSEL = RTCSEL::Lse;
}

impl<const PRESCALER_HZ: u32> RtcClockSource for RtcClkLsi<PRESCALER_HZ> {
    const PRESCALER: Hertz = Hz(PRESCALER_HZ);
    const VARIANT: RTCSEL = RTCSEL::Lsi;
}

/**
  Real time clock

  A continuously running clock that counts seconds¹. It is part of the backup domain which means
  that the counter is not affected by system resets or standby mode. If Vbat is connected, it is
  not reset even if the rest of the device is powered off. This allows it to be used to wake the
  CPU when it is in low power mode.


  See [examples/rtc.rs] and [examples/blinky_rtc.rs] for usage examples.

  1: Unless configured to another frequency using [select_frequency](struct.Rtc.html#method.select_frequency)

  [examples/rtc.rs]: https://github.com/stm32-rs/stm32f1xx-hal/blob/v0.7.0/examples/rtc.rs
  [examples/blinky_rtc.rs]: https://github.com/stm32-rs/stm32f1xx-hal/blob/v0.7.0/examples/blinky_rtc.rs
*/

pub struct Rtc<CS = RtcClkLse> {
    regs: RTC,
    _clock_source: PhantomData<CS>,
}

impl<CS: RtcClockSource> Rtc<CS> {
    /**
      Initialises the RTC with chosen crystal source (CS parameter).
      The `BackupDomain` struct is created by `Rcc.bkp.constrain()`.

      The frequency is set to 1 Hz.

      Since the RTC is part of the backup domain, The RTC counter is not reset by normal resets or
      power cycles where (VBAT) still has power. Use [set_time](#method.set_time) if you want to
      reset the counter.

      In case application is running of a battery on VBAT,
      this method will reset the RTC every time, leading to lost time,
      you may want to use
      [`restore_or_new`](#method.restore_or_new) instead.
    */
    pub fn new(regs: RTC, bkp: &mut BackupDomain) -> Self {
        // fool protection
        assert!(CS::VARIANT != RTCSEL::NoClock);

        let mut result = Rtc {
            regs,
            _clock_source: PhantomData,
        };

        Self::enable_rtc(bkp);

        // Set the prescaler to make it count up once every second.
        let prl = CS::PRESCALER.raw() - 1;
        assert!(prl < 1 << 20);
        result.perform_write(|s| {
            s.regs.prlh().write(|w| unsafe { w.bits(prl >> 16) });
            s.regs
                .prll()
                .write(|w| unsafe { w.bits(prl as u16 as u32) });
        });

        result
    }

    /// Tries to obtain currently running RTC to prevent a reset in case it was running from VBAT.
    /// If the RTC is not running, or clock source is not equal to chosen, it will be reinitialized.
    ///
    /// # Examples
    /// ```
    /// let rtc = match Rtc::restore_or_new(p.RTC, &mut backup_domain) {
    ///    Restored(rtc) => rtc, // The rtc is restored from previous configuration. You may verify the frequency you want if needed.
    ///    New(rtc) => { // The rtc was just initialized, the clock source selected, frequency is 1.Hz()
    ///        // Initialize rtc with desired parameters
    ///        rtc.select_frequency(2u16.Hz()); // Set the frequency to 2 Hz. This will stay same after reset
    ///        rtc
    ///    }
    /// };
    /// // or
    /// let rtc = Rtc::restore_or_new(p.RTC, &mut backup_domain)
    ///         .or_configure(|rtc| rtc.select_frequency(2u16.Hz()));
    /// ```
    ///
    pub fn restore_or_new(regs: RTC, bkp: &mut BackupDomain) -> RestoredOrNewRtc<CS> {
        if !Self::is_enabled() {
            RestoredOrNewRtc::New(Rtc::new(regs, bkp))
        } else {
            RestoredOrNewRtc::Restored(Rtc {
                regs,
                _clock_source: PhantomData,
            })
        }
    }

    /// Returns whether the RTC is currently enabled and chosen clock source is selected
    fn is_enabled() -> bool {
        let rcc = unsafe { &*RCC::ptr() };
        let bdcr = rcc.bdcr().read();
        bdcr.rtcen().is_enabled() && bdcr.rtcsel().variant() == CS::VARIANT
    }

    /// Enables the RTC device with chosen clock source
    fn enable_rtc(_bkp: &mut BackupDomain) {
        // NOTE: Safe RCC access because we are only accessing bdcr
        // and we have a &mut on BackupDomain
        let rcc = unsafe { &*RCC::ptr() };

        if CS::VARIANT == RTCSEL::Hse {
            if rcc.cr().read().hserdy().bit_is_clear() {
                panic!("HSE oscillator not ready");
            }
        }

        rcc.csr().modify(|_, w| {
            // start the LSI oscillator
            w.lsion().bit(CS::VARIANT == RTCSEL::Lsi)
        });

        rcc.bdcr().modify(|_, w| {
            // start the LSE oscillator
            w.lseon().bit(CS::VARIANT == RTCSEL::Lse);
            // Enable the RTC
            w.rtcen().set_bit();
            // Set the source of the RTC to chosen type
            w.rtcsel().variant(CS::VARIANT)
        });
    }

    /// Selects the frequency of the RTC Timer
    /// NOTE: Maximum frequency of 16384 Hz using the internal LSE
    pub fn select_frequency(&mut self, frequency: Hertz) {
        // The manual says that the zero value for the prescaler is not recommended, thus the
        // minimum division factor is 2 (prescaler + 1)
        assert!(frequency <= CS::PRESCALER / 2);

        let prescaler = CS::PRESCALER / frequency - 1;
        let max_frequency = CS::PRESCALER * (1 << 20);
        assert!(
            prescaler < (1 << 20),
            "prescaler is exceed: prescaler={prescaler}, frequency={frequency}, maximum_frequency={max_frequency}"
        );
        self.perform_write(|s| {
            s.regs.prlh().write(|w| unsafe { w.bits(prescaler >> 16) });
            s.regs
                .prll()
                .write(|w| unsafe { w.bits(prescaler as u16 as u32) });
        });
    }

    /// Set the current RTC counter value to the specified amount
    pub fn set_time(&mut self, counter_value: u32) {
        self.perform_write(|s| {
            s.regs
                .cnth()
                .write(|w| unsafe { w.bits(counter_value >> 16) });
            s.regs
                .cntl()
                .write(|w| unsafe { w.bits(counter_value as u16 as u32) });
        });
    }

    /**
      Sets the time at which an alarm will be triggered

      This also clears the alarm flag if it is set
    */
    pub fn set_alarm(&mut self, counter_value: u32) {
        // Set alarm time
        // See section 18.3.5 for explanation
        let alarm_value = counter_value - 1;

        // TODO: Remove this `allow` once these fields are made safe for stm32f100
        #[allow(unused_unsafe)]
        self.perform_write(|s| {
            s.regs
                .alrh()
                .write(|w| unsafe { w.alrh().bits((alarm_value >> 16) as u16) });
            s.regs
                .alrl()
                .write(|w| unsafe { w.alrl().bits(alarm_value as u16) });
        });

        self.clear_alarm_flag();
    }

    /// Enables the RTC interrupt to trigger when the counter reaches the alarm value. In addition,
    /// if the EXTI controller has been set up correctly, this function also enables the RTCALARM
    /// interrupt.
    pub fn listen_alarm(&mut self) {
        // Enable alarm interrupt
        self.perform_write(|s| {
            s.regs.crh().modify(|_, w| w.alrie().set_bit());
        })
    }

    /// Stops the RTC alarm from triggering the RTC and RTCALARM interrupts
    pub fn unlisten_alarm(&mut self) {
        // Disable alarm interrupt
        self.perform_write(|s| {
            s.regs.crh().modify(|_, w| w.alrie().clear_bit());
        })
    }

    /// Reads the current counter
    pub fn current_time(&self) -> u32 {
        // Wait for the APB1 interface to be ready
        while !self.regs.crl().read().rsf().bit() {}

        self.regs.cnth().read().bits() << 16 | self.regs.cntl().read().bits()
    }

    /// Enables triggering the RTC interrupt every time the RTC counter is increased
    pub fn listen_seconds(&mut self) {
        self.perform_write(|s| s.regs.crh().modify(|_, w| w.secie().set_bit()))
    }

    /// Disables the RTC second interrupt
    pub fn unlisten_seconds(&mut self) {
        self.perform_write(|s| s.regs.crh().modify(|_, w| w.secie().clear_bit()))
    }

    /// Clears the RTC second interrupt flag
    pub fn clear_second_flag(&mut self) {
        self.perform_write(|s| s.regs.crl().modify(|_, w| w.secf().clear_bit()))
    }

    /// Clears the RTC alarm interrupt flag
    pub fn clear_alarm_flag(&mut self) {
        self.perform_write(|s| s.regs.crl().modify(|_, w| w.alrf().clear_bit()))
    }

    /**
      Return `Ok(())` if the alarm flag is set, `Err(nb::WouldBlock)` otherwise.

      ```rust
      use nb::block;

      rtc.set_alarm(rtc.read_counts() + 5);
      // NOTE: Safe unwrap because Infallible can't be returned
      block!(rtc.wait_alarm()).unwrap();
      ```
    */
    pub fn wait_alarm(&mut self) -> nb::Result<(), Infallible> {
        if self.regs.crl().read().alrf().bit() {
            self.regs.crl().modify(|_, w| w.alrf().clear_bit());
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    /**
      The RTC registers can not be written to at any time as documented on page
      485 of the manual. Performing writes using this function ensures that
      the writes are done correctly.
    */
    fn perform_write(&mut self, func: impl Fn(&mut Self)) {
        // Wait for the last write operation to be done
        while !self.regs.crl().read().rtoff().bit() {}
        // Put the clock into config mode
        self.regs.crl().modify(|_, w| w.cnf().set_bit());

        // Perform the write operation
        func(self);

        // Take the device out of config mode
        self.regs.crl().modify(|_, w| w.cnf().clear_bit());
        // Wait for the write to be done
        while !self.regs.crl().read().rtoff().bit() {}
    }
}

impl<CS> RestoredOrNewRtc<CS> {
    pub fn or_configure(self, confugurator: impl FnOnce(&mut Rtc<CS>)) -> Rtc<CS> {
        match self {
            Self::New(mut rtc) => {
                confugurator(&mut rtc);
                rtc
            }
            Self::Restored(rtc) => rtc,
        }
    }
}
