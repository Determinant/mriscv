#![no_std]
#![allow(dead_code)]

const UART_TXDATA: *mut u32 = 0x00001000 as *mut u32;

pub fn itoa(mut n: i32, s: &mut [u8]) -> usize {
    let mut neg = false;
    let mut i = 0;
    if n < 0 {
        n = -n;
        neg = true;
    }
    loop {
        s[i] = '0' as u8 + (n % 10) as u8;
        i += 1;
        n /= 10;
        if n == 0 { break }
    }
    if neg {
        s[i] = '-' as u8;
        i += 1;
    }
    for (j, k) in {0..i}.zip({0..i}.rev()) {
        if j >= k { break }
        let c = s[k];
        s[k] = s[j];
        s[j] = c;
    }
    i
}

pub fn print(s: &str) {
    unsafe {
        for c in s.as_bytes().iter() {
            while (*UART_TXDATA) >> 31 != 0 {}
            UART_TXDATA.write_volatile(((*UART_TXDATA) & !0xff) | (*c as u32));
        }
    }
}
