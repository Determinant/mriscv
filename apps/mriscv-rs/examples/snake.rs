#![no_std]
#![no_main]

extern crate mriscv;
extern crate panic_halt;
use core::fmt::Write;
use mriscv::{uprint, uprintln};
use rand::SeedableRng;
use riscv_rt::entry;

const INTERVAL: u32 = 0x300000;
const MAP_WIDTH: usize = 50;
const MAP_HEIGHT: usize = 50;
const MAP_SCALE: usize = 8;
const DELTAS: [(i8, i8); 4] = [(-1, 0), (1, 0), (0, -1), (0, 1)];
const COLOR_CELL_EMPTY: u8 = 0b010101;
const COLOR_CELL_SNAKEBODY: u8 = 0b000011;
const COLOR_CELL_FOOD: u8 = 0b001100;
const COLOR_CELL_POISON: u8 = 0b110000;
const KEY_UP: u8 = 0x52;
const KEY_DOWN: u8 = 0x51;
const KEY_LEFT: u8 = 0x50;
const KEY_RIGHT: u8 = 0x4f;

mriscv::evented_traps!();

#[derive(Clone, Copy)]
enum SnakeCell {
    Empty,
    SnakeBody,
    Food,
    Poison,
}

impl From<SnakeCell> for u8 {
    fn from(c: SnakeCell) -> Self {
        match c {
            SnakeCell::Empty => COLOR_CELL_EMPTY,
            SnakeCell::SnakeBody => COLOR_CELL_SNAKEBODY,
            SnakeCell::Food => COLOR_CELL_FOOD,
            SnakeCell::Poison => COLOR_CELL_POISON,
        }
    }
}

#[derive(Clone, Copy)]
enum Dir {
    Up = 0,
    Down,
    Left,
    Right,
}

#[derive(Clone, Copy)]
struct Pos {
    r: i8,
    c: i8,
}

impl Pos {
    fn is_valid(&self) -> bool {
        (0 <= self.r && self.r < MAP_HEIGHT as i8) && (0 <= self.c && self.c < MAP_WIDTH as i8)
    }

    fn to(&self, dir: &Dir, dis: i8) -> Pos {
        let d = DELTAS[(*dir) as usize];
        Pos {
            r: self.r + d.0 * dis,
            c: self.c + d.1 * dis,
        }
    }

    fn to_step(&self, dir: &Dir, dis: u8) -> impl core::iter::Iterator<Item = Pos> {
        let d = DELTAS[(*dir) as usize];
        (0..dis).scan((self.r, self.c), move |(r, c), _| {
            *r += d.0;
            *c += d.1;
            Some(Pos { r: *r, c: *c })
        })
    }

    fn from_rand(rng: &mut impl rand::Rng) -> Self {
        Pos {
            r: rng.gen_range(0..MAP_HEIGHT) as i8,
            c: rng.gen_range(0..MAP_WIDTH) as i8,
        }
    }
}

impl Dir {
    fn from_rand(rng: &mut impl rand::Rng) -> Self {
        match rng.gen_range(0..4) {
            0 => Dir::Up,
            1 => Dir::Down,
            2 => Dir::Left,
            _ => Dir::Right,
        }
    }
}

struct SnakeState {
    map: [[SnakeCell; MAP_WIDTH]; MAP_HEIGHT],
    body: mriscv::Queue256<Pos>,
    dir: Dir,
    gameover: bool,
}

impl SnakeState {
    fn new(nfood: u8, npoison: u8, rng: &mut impl rand::Rng) -> Self {
        let dir = Dir::from_rand(rng);
        let mut map = [[SnakeCell::Empty; MAP_WIDTH]; MAP_HEIGHT];
        let mut body = mriscv::Queue256::new();
        let tail = Pos {
            r: (MAP_HEIGHT / 2) as i8,
            c: (MAP_WIDTH / 2) as i8,
        };
        for h in tail.to_step(&dir, 5) {
            map[h.r as usize][h.c as usize] = SnakeCell::SnakeBody;
            body.push(h);
        }
        for _ in 0..nfood {
            let p = Pos::from_rand(rng);
            map[p.r as usize][p.c as usize] = SnakeCell::Food;
        }
        for _ in 0..npoison {
            let p = Pos::from_rand(rng);
            let r = p.r as usize;
            let c = p.c as usize;
            if let SnakeCell::SnakeBody = map[r][c] {
            } else {
                map[r][c] = SnakeCell::Poison;
            }
        }
        SnakeState {
            map,
            body,
            dir,
            gameover: false,
        }
    }

    fn set_dir(&mut self, dir: &Dir) {
        self.dir = *dir;
    }

    fn tick(&mut self, rng: &mut impl rand::Rng) -> bool {
        if self.gameover {
            return true;
        }
        let h = self.body.back().unwrap();
        let nh = h.to(&self.dir, 1);
        if !nh.is_valid() {
            self.gameover = true;
            return false;
        }
        let ncell = &mut self.map[nh.r as usize][nh.c as usize];
        match *ncell {
            SnakeCell::Empty => {
                *ncell = SnakeCell::SnakeBody;
                self.body.push(nh);
                let tail = self.body.pop().unwrap();
                self.map[tail.r as usize][tail.c as usize] = SnakeCell::Empty;
            }
            SnakeCell::Food => {
                *ncell = SnakeCell::SnakeBody;
                if !self.body.push(nh) {
                    self.body.pop();
                }
                // new food
                loop {
                    let p = Pos::from_rand(rng);
                    let r = p.r as usize;
                    let c = p.c as usize;
                    if let SnakeCell::SnakeBody = self.map[r][c] {
                        continue;
                    }
                    self.map[r][c] = SnakeCell::Food;
                    break;
                }
            }
            _ => {
                self.gameover = true;
                return false;
            }
        }
        true
    }
}

fn render(s: &SnakeState) {
    let fb = mriscv::get_framebuffer();
    let x_off = (mriscv::FB_WIDTH - MAP_WIDTH * MAP_SCALE) / 2;
    let x_remain = (mriscv::FB_WIDTH - MAP_WIDTH * MAP_SCALE) - x_off;
    let y_off = (mriscv::FB_HEIGHT - MAP_HEIGHT * MAP_SCALE) / 2 * mriscv::FB_WIDTH;
    let mut fb_pos = y_off;
    for r in 0..MAP_HEIGHT {
        let pos0 = fb_pos;
        fb_pos += x_off;
        for c in 0..MAP_WIDTH {
            let color = s.map[r][c].into();
            for f in &mut fb[fb_pos..fb_pos + MAP_SCALE] {
                *f = color;
            }
            fb_pos += MAP_SCALE
        }
        fb_pos += x_remain;
        let t = &mriscv::get_framebuffer()[pos0..fb_pos];
        let tlen = t.len();
        for _ in 1..MAP_SCALE {
            &mut fb[fb_pos..fb_pos + tlen].copy_from_slice(t);
            fb_pos += tlen;
        }
    }
}

#[entry]
fn main() -> ! {
    mriscv::event::Events::init();
    unsafe {
        // enable interrupts
        riscv::register::mstatus::set_mie();
        riscv::register::mie::set_mext();
        mriscv::set_timer(INTERVAL);
    }
    let mut rng = rand::rngs::SmallRng::from_seed([0; 16]);
    let mut state = SnakeState::new(10, 5, &mut rng);
    render(&state);
    mriscv::event::Events::dispatch(|e| match e {
        mriscv::event::Event::Timer => {
            if !state.tick(&mut rng) {
                uprintln!("gameover");
            }
            render(&state);
            unsafe { mriscv::set_timer(INTERVAL) }
        }
        mriscv::event::Event::Keyboard(k) => {
            if k >> 8 != 1 {
                return;
            }
            let dir = match k as u8 {
                KEY_UP => Dir::Up,
                KEY_DOWN => Dir::Down,
                KEY_LEFT => Dir::Left,
                KEY_RIGHT => Dir::Right,
                _ => return,
            };
            state.set_dir(&dir);
            if !state.tick(&mut rng) {
                uprintln!("gameover");
            }
            render(&state);
        }
        _ => (),
    });
}
