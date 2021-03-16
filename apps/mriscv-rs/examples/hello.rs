#![no_std]
#![no_main]

extern crate panic_halt;
extern crate mriscv;
use riscv_rt::entry;
use mriscv::{uprint, uprintln};
use core::fmt::Write;

#[export_name="DefaultHandler"]
fn my_interrupt_handler() {
    uprintln!(mriscv::Serial, "interrupt received!");
}

#[entry]
fn main() -> ! {
    let mut s = mriscv::Serial;
    uprintln!(s, "hello, world! Count from 10:");
    for i in {0..10}.rev() {
        uprintln!(s, "now it is {}...", i);
    }
    loop {}
}
