#![no_std]
#![allow(dead_code)]

const UART_TXDATA: *mut u32 = 0x00001000 as *mut u32;
const MTIME: *mut u32 = 0x00002000 as *mut u32;
const MTIMECMP: *mut u32 = 0x00002008 as *mut u32;
const MSIP: *mut u32 = 0x00002010 as *mut u32;
const EXTI: *mut u32 = 0x00002014 as *mut u32;
const INPUT: *mut u32 = 0x0003000 as *mut u32;
const FRAMEBUFFER: *mut u8 = 0x10000000 as *mut u8;
const FB_WIDTH: usize = 640;
const FB_HEIGHT: usize = 480;

fn int_to_string(_n: i32, s: &mut [u8], sign: bool, uppercase: bool, base: u8) -> usize {
    let neg;
    let mut i = 0;
    let mut n;
    let base = base as u32;
    if _n < 0 && sign {
        n = (-_n) as u32;
        neg = true;
    } else {
        n = _n as u32;
        neg = false;
    }
    loop {
        let d = (n % base) as u8;
        s[i] = (if d < 10 {'0'} else {if uppercase {'A'} else {'a'}}) as u8 + d;
        i += 1;
        n /= base;
        if n == 0 {
            break;
        }
    }
    if neg {
        s[i] = '-' as u8;
        i += 1;
    }
    for (j, k) in { 0..i }.zip({ 0..i }.rev()) {
        if j >= k {
            break;
        }
        let c = s[k];
        s[k] = s[j];
        s[j] = c;
    }
    i
}

pub fn itoa(n: i32, s: &mut [u8]) -> usize {
    int_to_string(n, s, true, false, 10)
}

pub fn putchar(c: u8) {
    unsafe {
        while (*UART_TXDATA) >> 31 != 0 {}
        UART_TXDATA.write_volatile(((*UART_TXDATA) & !0xff) | (c as u32));
    }
}

pub fn print(s: &str) {
    for c in s.as_bytes().iter() {
        putchar(*c);
    }
}

pub struct Serial;
impl core::fmt::Write for Serial {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        print(s);
        Ok(())
    }
}

pub fn set_timer(ncycles: u32) {
    unsafe {
        riscv::register::mie::clear_mtimer();
        MTIME.write_volatile(0);
        MTIMECMP.write_volatile(ncycles as u32);
        riscv::register::mie::set_mtimer();
    }
}

#[inline(always)]
pub fn get_framebuffer() -> &'static mut [u8] {
    unsafe {
        core::slice::from_raw_parts_mut(FRAMEBUFFER, FB_WIDTH * FB_HEIGHT)
    }
}

pub unsafe fn set_sw_interrupt() {
    MSIP.write_volatile(1);
}

pub unsafe fn clear_sw_interrupt() {
    MSIP.write_volatile(0);
}

pub unsafe fn clear_ext_interrupt() {
    EXTI.write_volatile(1);
}

pub fn get_input_key() -> u32 {
    unsafe {
        INPUT.read_volatile()
    }
}

#[macro_export]
macro_rules! uprint {
    ($($arg:tt)*) => {
        mriscv::Serial.write_fmt(format_args!($($arg)*)).ok()
    };
}

#[macro_export]
macro_rules! uprintln {
    ($fmt:expr) => {
        uprint!(concat!($fmt, "\n"))
    };
    ($fmt:expr, $($arg:tt)*) => {
        uprint!(concat!($fmt, "\n"), $($arg)*)
    };
}
