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
        "got exception: {}",
        riscv::register::mcause::read().code()
    );
}

const INTERVAL: u32 = 0x200000;

#[export_name = "MachineTimer"]
fn timer_handler(_trap_frame: &riscv_rt::TrapFrame) {
    uprintln!("timer goes off! resetting...");
    mriscv::set_timer(INTERVAL);
}

#[export_name = "MachineSoft"]
fn soft_handler(_trap_frame: &riscv_rt::TrapFrame) {
    uprintln!("software interrupt! clearing...");
    unsafe {mriscv::clear_interrupt();}
}

#[entry]
fn main() -> ! {
    uprintln!("hello, world! Count from 10:");
    for i in { 0..10 }.rev() {
        uprintln!("now it is {}...", i);
    }
    uprintln!(
        "this could be any value: {} (because time CSR is not supported)",
        riscv::register::time::read()
    ); // triggers an exception
    uprintln!("execution is resumed");
    unsafe {
        // enable interrupts
        riscv::register::mstatus::set_mie();
        // enable software interrupt
        riscv::register::mie::set_msoft();
        // trigger a software interrupt
        mriscv::set_interrupt();
    }
    mriscv::set_timer(INTERVAL);
    uprintln!("timer set");
    loop {
        unsafe {
            riscv::asm::wfi();
        }
    }
}
