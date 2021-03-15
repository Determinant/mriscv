#![no_std]
#![no_main]

extern crate panic_halt;
extern crate mriscv;
use riscv_rt::entry;
use mriscv::print;

#[export_name="DefaultHandler"]
fn my_interrupt_handler() {
    print("interrupt received!");
}

#[entry]
fn main() -> ! {
    print("hello, world!\n");
    loop {}
}
