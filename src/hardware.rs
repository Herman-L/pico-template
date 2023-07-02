#![warn(unsafe_op_in_unsafe_fn)]

use cortex_m::delay::Delay;
use embedded_alloc::Heap;
use rp_pico::hal::clocks::init_clocks_and_plls;
use rp_pico::hal::{Clock, Sio, Timer, Watchdog};
use rp_pico::pac::{CorePeripherals, Peripherals};
use rp_pico::Pins;

// The heap allocator.
#[global_allocator]
static HEAP: Heap = Heap::empty();

/// The entry point. Sets up the hardware.
#[cortex_m_rt::entry]
fn entry() -> ! {
    unsafe { init_heap() };

    // Hardware setup.
    let mut pac = Peripherals::take().unwrap();
    let core = CorePeripherals::take().unwrap();

    let mut watchdog = Watchdog::new(pac.WATCHDOG);

    let clocks = init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    unsafe {
        serial::start(
            pac.USBCTRL_REGS,
            pac.USBCTRL_DPRAM,
            clocks.usb_clock,
            &mut pac.RESETS,
        );
    }

    let delay = Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let timer = Timer::new(pac.TIMER, &mut pac.RESETS);

    let sio = Sio::new(pac.SIO);

    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    super::start(delay, timer, pins);
}

/// Initializes the heap using the symbols provided by memory.x.
unsafe fn init_heap() {
    extern "C" {
        static mut _heap_start: u32;
        static mut _heap_end: u32;
    }

    unsafe {
        let start = &mut _heap_start as *mut u32 as usize;
        let end = &mut _heap_end as *mut u32 as usize;
        assert!(end > start);
        HEAP.init(start, end - start);
    }
}

/// Panic handler which prints the panic info to the serial device.
#[panic_handler]
fn panic_handler(info: &core::panic::PanicInfo) -> ! {
    serial::println!("{info}");
    loop {
        wait_for_interrupts();
    }
}

/// Safe wrapper around [`core::arch::arm::__wfi`].
fn wait_for_interrupts() {
    #[cfg(target_arch = "arm")]
    unsafe {
        core::arch::arm::__wfi()
    };
}

/// Serial communication.
pub mod serial {
    use core::fmt::{Arguments, Write};
    use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

    use alloc::string::String;
    use alloc::vec::Vec;
    use rp_pico::hal::clocks::UsbClock;
    use rp_pico::hal::usb::UsbBus;
    use rp_pico::pac::{interrupt, Interrupt, NVIC, RESETS, USBCTRL_DPRAM, USBCTRL_REGS};
    use usb_device::bus::UsbBusAllocator;
    use usb_device::device::{UsbDevice, UsbDeviceBuilder, UsbVidPid};
    use usbd_serial::SerialPort;

    static mut USB_DEVICE: Option<UsbDevice<UsbBus>> = None;
    static mut USB_BUS: Option<UsbBusAllocator<UsbBus>> = None;
    static mut USB_SERIAL: Option<SerialPort<UsbBus>> = None;

    /// The read buffer.
    static mut READ_BUFFER: [u8; 4096] = [0; 4096];

    /// The amount of bytes stored in the read buffer.
    static READ_AVAILABLE: AtomicUsize = AtomicUsize::new(0);

    /// Set to `true` when the writer should retry.
    static WRITE_AVAILABLE: AtomicBool = AtomicBool::new(true);

    /// Starts the serial communication.
    ///
    /// # Safety
    ///
    /// May only be called once, and must be called before any of the other funcitons in this module.
    pub(super) unsafe fn start(
        regs: USBCTRL_REGS,
        dpram: USBCTRL_DPRAM,
        clock: UsbClock,
        resets: &mut RESETS,
    ) {
        // Set up the USB driver.
        let bus = UsbBusAllocator::new(UsbBus::new(regs, dpram, clock, true, resets));
        let bus = unsafe { USB_BUS.insert(bus) };

        // Set up the serial port.
        let serial = SerialPort::new(bus);
        unsafe { USB_SERIAL = Some(serial) };

        // Create a USB device (with a fake ID and info).
        let device = UsbDeviceBuilder::new(bus, UsbVidPid(0x16c0, 0x27dd))
            .manufacturer("Raspberry Pi")
            .product("Pico")
            .serial_number("1234")
            .device_class(2)
            .build();
        unsafe { USB_DEVICE = Some(device) };

        // Enable the interrupt.
        unsafe { NVIC::unmask(Interrupt::USBCTRL_IRQ) };
    }

    /// USB interrupt.
    #[interrupt]
    #[allow(non_snake_case)]
    unsafe fn USBCTRL_IRQ() {
        cortex_m::interrupt::free(|_| {
            let usb_dev = unsafe { USB_DEVICE.as_mut().unwrap() };
            let serial = unsafe { USB_SERIAL.as_mut().unwrap() };

            // Poll the device, and return if nothing more needs to be done.
            if !usb_dev.poll(&mut [serial]) {
                return;
            }

            // Tell the writer that it may write again, in case it failed.
            WRITE_AVAILABLE.store(true, Ordering::Relaxed);

            // Discard reads if the buffer is full.
            let index = READ_AVAILABLE.load(Ordering::Acquire);
            if index >= unsafe { READ_BUFFER.len() } {
                _ = serial.read(&mut [0; 64]);
                return;
            }

            // Read into the remaining part of the buffer.
            let buffer = unsafe { &mut READ_BUFFER[index..] };

            match serial.read(buffer) {
                Ok(0) | Err(_) => {}
                Ok(count) => {
                    let index = index + count;
                    READ_AVAILABLE.store(index, Ordering::Release);
                }
            }
        });
    }

    /// Tests if any data is available to read.
    #[allow(unused)]
    pub fn available() -> bool {
        READ_AVAILABLE.load(Ordering::Relaxed) > 0
    }

    /// Waits until any data is available and then reads from the serial device.
    /// Consumes the amount of bytes returned from the handler.
    #[allow(unused)]
    pub fn read(handler: impl FnOnce(&[u8]) -> usize) {
        // Ensure it's not called recursively.
        static CALLED: AtomicBool = AtomicBool::new(false);
        assert!(!CALLED.load(Ordering::Relaxed), "recursive call to read");
        CALLED.store(true, Ordering::Relaxed);

        // Wait until data is available.
        while READ_AVAILABLE.load(Ordering::Relaxed) == 0 {
            super::wait_for_interrupts();
        }

        cortex_m::interrupt::free(|_| {
            let index = READ_AVAILABLE.load(Ordering::Acquire);
            let buffer = unsafe { &mut READ_BUFFER };

            let consumed = handler(&buffer[..index]).min(index);

            buffer.copy_within(consumed..index, 0);
            READ_AVAILABLE.store(index - consumed, Ordering::Release);
        });

        CALLED.store(false, Ordering::Relaxed);
    }

    /// Reads a single byte.
    #[allow(unused)]
    pub fn read_byte() -> u8 {
        let mut byte = None;

        loop {
            if let Some(byte) = byte {
                return byte;
            }

            read(|data| {
                byte = data.first().copied();
                data.len().min(1)
            });
        }
    }

    /// Reads until the specified byte is found. Returns the results as a `String`.
    /// The byte is included in the string. Invalid characters are replaced with the
    /// replacement character (U+FFFD).
    #[allow(unused)]
    pub fn read_until(byte: u8) -> String {
        let mut buf = Vec::new();
        let mut terminated = false;

        while !terminated {
            read(|data| {
                if let Some(i) = data.iter().position(|&b| b == byte) {
                    terminated = true;
                    buf.extend_from_slice(&data[..=i]);
                    i + 1
                } else {
                    buf.extend_from_slice(data);
                    data.len()
                }
            })
        }

        String::from_utf8(buf)
            .unwrap_or_else(|err| String::from_utf8_lossy(err.as_bytes()).into_owned())
    }

    /// Writes the data to the serial device.
    #[allow(unused)]
    pub fn write(mut data: &[u8]) {
        while !data.is_empty() {
            // Wait until writing is available.
            while !WRITE_AVAILABLE.load(Ordering::Relaxed) {
                super::wait_for_interrupts();
            }

            // Write as much as possible to the device.
            let count = cortex_m::interrupt::free(|_| {
                let serial = unsafe { USB_SERIAL.as_mut().unwrap() };
                match serial.write(data) {
                    Ok(0) | Err(_) => {
                        WRITE_AVAILABLE.store(false, Ordering::Relaxed);
                        0
                    }
                    Ok(len) => len,
                }
            });

            data = &data[count..];
        }
    }

    /// Writes the formatting arguments to the serial device.
    #[allow(unused)]
    pub fn write_fmt(args: Arguments<'_>) {
        struct Printer;
        impl Write for Printer {
            fn write_str(&mut self, s: &str) -> core::fmt::Result {
                write(s.as_bytes());
                Ok(())
            }
        }
        _ = Printer.write_fmt(args);
    }

    /// Prints to the serial device.
    #[allow(unused)]
    macro_rules! print {
        ($($args:tt)+) => {
            $crate::hardware::serial::write_fmt(::core::format_args!($($args)+))
        };
    }

    /// Prints to the serial device, followed by a carriage return and line break.
    #[allow(unused)]
    macro_rules! println {
        () => {
            $crate::hardware::serial::write(b"\r\n")
        };
        ($($args:tt)+) => {{
            $crate::hardware::serial::print!($($args)+);
            $crate::hardware::serial::println!();
        }};
    }

    #[allow(unused)]
    pub(crate) use {print, println};
}
