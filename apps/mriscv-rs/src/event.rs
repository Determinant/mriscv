use core::mem::MaybeUninit;

#[derive(Clone, Debug)]
pub enum Event {
    Null,
    Timer,
    Keyboard(u16),
    Software,
}

type EventQueue = crate::Queue256<Event>;
pub struct Events(MaybeUninit<EventQueue>);

static mut EVENTS: Events = Events(MaybeUninit::uninit());

impl Events {
    pub fn init() {
        unsafe {
            EVENTS.0 = MaybeUninit::new(EventQueue::new());
        }
    }

    fn modify_events(&mut self, mut f: impl FnMut(&mut EventQueue) -> ()) {
        unsafe {
            f(&mut *self.0.as_mut_ptr());
        }
    }

    pub fn timer_handler() {
        unsafe {
            // disable timer
            riscv::register::mie::clear_mtimer();
            EVENTS.modify_events(|events| {
                events.push(Event::Timer);
            });
        };
    }

    pub fn keyboard_handler(key: u16) {
        unsafe {
            EVENTS.modify_events(|events| {
                events.push(Event::Keyboard(key));
            })
        }
    }

    pub fn sw_handler() {
        unsafe {
            EVENTS.modify_events(|events| {
                events.push(Event::Software);
            });
        };
    }

    pub fn dispatch(mut f: impl FnMut(Event) -> ()) -> ! {
        loop {
            unsafe {
                EVENTS.modify_events(|events| {
                    while let Some(e) = events.pop() {
                        f(e);
                    }
                });
                riscv::asm::wfi();
            }
        }
    }
}

#[macro_export]
macro_rules! evented_traps {
    () => {
        #[export_name = "MachineTimer"]
        fn timer_handler(_trap_frame: &riscv_rt::TrapFrame) {
            mriscv::event::Events::timer_handler();
        }

        #[export_name = "MachineSoft"]
        fn soft_handler(_trap_frame: &riscv_rt::TrapFrame) {
            mriscv::event::Events::sw_handler();
            unsafe {
                mriscv::clear_sw_interrupt();
            }
        }

        #[export_name = "MachineExternal"]
        fn ext_handler(_trap_frame: &riscv_rt::TrapFrame) {
            let code = mriscv::get_input_key() as u16;
            mriscv::event::Events::keyboard_handler(code);
            unsafe {
                mriscv::clear_ext_interrupt();
            }
        }
    };
}
