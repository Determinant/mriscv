#![no_std]
#![no_main]

use riscv_rt::entry;

extern crate panic_halt;
extern crate riscv_rt;

#[entry]
fn main() -> ! {
    //let cpu_mem = CPUMemory::new();
    //let cpu = mos6502::CPU::new();

    let mut a = 1;
    loop {
        let _c = a * 3;
        a += 1;
    }
}
