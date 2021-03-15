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
    let ptr = 0x400000 as *mut u32;
    loop {
        a += 3;
        unsafe {
            ptr.write_volatile(a);
        }
    }
}
