#![no_std]
#![no_main]

extern crate panic_halt;
use rand::Rng;
use rand::SeedableRng;
use riscv_rt::entry;

const INTERVAL: u32 = 0x1000000;
static mut COUNTER: usize = 0;
static mut RNG: Option<rand::rngs::SmallRng> = None;

#[export_name = "MachineTimer"]
fn timer_handler(_trap_frame: &riscv_rt::TrapFrame) {
    let fb = mriscv::get_framebuffer();
    unsafe {
        mriscv::set_timer(INTERVAL);
        for i in 0..fb.len() {
            COUNTER += 1;
            fb[i] = RNG.as_mut().unwrap().gen();
        }
    }
}

#[entry]
fn main() -> ! {
    unsafe {
        RNG = Some(rand::rngs::SmallRng::from_seed([0; 16]));
        riscv::register::mstatus::set_mie();
        mriscv::set_timer(INTERVAL);
    }
    mriscv::wfi_loop();
}
