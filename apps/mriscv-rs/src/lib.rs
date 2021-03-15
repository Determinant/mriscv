#![no_std]
#![allow(dead_code)]

const UART_TXDATA: *mut u32 = 0x00001000 as *mut u32;

pub fn print(s: &str) {
    unsafe {
        for c in s.as_bytes().iter() {
            while (*UART_TXDATA) >> 31 != 0 {}
            UART_TXDATA.write_volatile(((*UART_TXDATA) & !0xff) | (*c as u32));
        }
    }
}
