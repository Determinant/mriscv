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
    print("hello, world! Count from 10:\n");
    let mut buff = [0u8; 10];
    for i in {0..10}.rev() {
        let n = mriscv::itoa(i, &mut buff);
        print(core::str::from_utf8(&buff[..n]).unwrap());
        print("\n");
    }
    loop {}
}
