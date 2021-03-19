#![no_std]
#![no_main]

extern crate mriscv;
extern crate panic_halt;
use core::fmt::Write;
use mriscv::{uprint, uprintln};
use riscv_rt::entry;

#[export_name = "ExceptionHandler"]
fn my_exception_handler(_trap_frame: &riscv_rt::TrapFrame) {
    uprintln!(
        mriscv::Serial,
        "got exception: {}",
        riscv::register::mcause::read().code()
    );
}

const INTERVAL: u32 = 0x200000;

#[export_name = "MachineTimer"]
fn timer_handler(_trap_frame: &riscv_rt::TrapFrame) {
    uprintln!(mriscv::Serial, "timer goes off! reset...");
    mriscv::set_timer(INTERVAL);
}

#[entry]
fn main() -> ! {
    let mut s = mriscv::Serial;
    uprintln!(s, "hello, world! Count from 10:");
    for i in { 0..10 }.rev() {
        uprintln!(s, "now it is {}...", i);
    }
    uprintln!(
        s,
        "this could be any value: {}",
        riscv::register::time::read()
    ); // triggers an exception
    uprintln!(s, "execution is resumed");
    unsafe {riscv::register::mstatus::set_mie();}
    mriscv::set_timer(INTERVAL);
    uprintln!(s, "timer set");
    loop {
        unsafe {
            riscv::asm::wfi();
        }
    }
}
