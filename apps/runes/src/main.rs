#![no_std]
#![no_main]

extern crate panic_halt;
use riscv_rt::entry;

#[export_name="DefaultHandler"]
fn my_interrupt_handler() {
    panic!("hi");
}

#[entry]
fn main() -> ! {
    //let cpu_mem = CPUMemory::new();
    //let cpu = mos6502::CPU::new();

    let mut a = 1;
    for _ in 0..4 {
        let _c = a * 3;
        a += 1;
    }

    loop {}
}
