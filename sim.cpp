#include <cstdio>
#include <memory>
#include <vector>
#include <exception>
#include <queue>
#include <cstdlib>
#include <cassert>
#include <csignal>
#include <getopt.h>
#include "verilated.h"
#include "Vcpu.h"

#ifdef ENABLE_SDL
#include <SDL2/SDL.h>
#endif

#ifdef DEBUG
#define debug(f_, ...) printf((f_), ##__VA_ARGS__)
#else
#define debug(...) do {} while(0)
#endif

uint64_t main_time = 0;
uint32_t halt_addr = 0x00000000;

const uint32_t uart_txdata_addr = 0x00001000;
const uint32_t input_addr = 0x00003000;
const uint32_t framebuffer_addr = 0x10000000;

const uint32_t win_width = 640;
const uint32_t win_height = 480;
const size_t max_key_events = 4096;

#ifdef ENABLE_SDL
SDL_Window *window = nullptr;
SDL_Renderer *sdl_renderer;
SDL_Texture *frame;
uint32_t frame_cnt;
uint8_t frame_tmp[3 * win_width * win_height];
uint32_t cycles_per_frame = 100000;
struct timespec prev_time;
const double fps = 60;
std::queue<uint16_t> key_events;
#endif

double sc_time_stamp() {
    return main_time;
}

class SimulatedRAM {
    std::shared_ptr<Vcpu> cpu;
    std::vector<uint8_t> memory;
    std::vector<uint8_t> framebuffer;
    size_t capacity;
    uint64_t icache_next_rdy;
    uint64_t dcache_next_rdy;
    int icache_state; // 0 -> reset; 1 -> reading
    int dcache_state; // 0 -> reset; 1 -> reading/writing

    public:
    SimulatedRAM(std::shared_ptr<Vcpu> cpu, size_t capacity, size_t fb_capacity): \
            cpu(cpu), capacity(capacity),
            icache_next_rdy(0),
            dcache_next_rdy(0) {
        memory.resize(capacity);
        framebuffer.resize(fb_capacity);
        cpu->icache_rdy = 0;
        cpu->dcache_rdy = 0;
    }

    void load_image_from_file(FILE *img_file, size_t mem_off, size_t len = 0) {
        if (len == 0)
        {
            auto old = ftell(img_file);
            fseek(img_file, 0L, SEEK_END);
            len = ftell(img_file);
            fseek(img_file, old, SEEK_SET);
        }
        fread(&memory[0] + mem_off, 1, len, img_file);
    }

    void eval_posedge() {
        if (cpu->reset == 1)
        {
            debug("reset ram\n");
            icache_state = 0;
            dcache_state = 0;
            cpu->icache_rdy = 0;
            cpu->dcache_rdy = 0;
            return;
        }
        if (icache_state == 0)
        {
            cpu->icache_rdy = 0;
            if (cpu->icache_req)
                icache_state = 1;
        }
        if (icache_state == 1)
        {
            if (icache_next_rdy == 0)
            {
                assert(cpu->icache_addr + 4 < capacity);
                cpu->icache_data = \
                    memory[cpu->icache_addr] |
                    (memory[cpu->icache_addr + 1] << 8) |
                    (memory[cpu->icache_addr + 2] << 16) |
                    (memory[cpu->icache_addr + 3] << 24);
                cpu->icache_rdy = 1;
                icache_state = 0;
                debug("icache: read byte @ %08x = %08x\n",
                      cpu->icache_addr, cpu->icache_data);
                //schedule_next_icache_rdy(4);
            } else icache_next_rdy--;
        }

        if (dcache_state == 0)
        {
            cpu->dcache_rdy = 0;
            if (cpu->dcache_req)
                dcache_state = 1;
        }
        if (dcache_state == 1)
        {
            if (dcache_next_rdy == 0)
            {
                auto addr = cpu->dcache_addr;
                auto data = cpu->dcache_wdata;
                if (addr == uart_txdata_addr)
                {
                    if (cpu->dcache_wr)
                    {
                        debug("dcache: write uart = %02x\n", addr, data);
                        putchar((uint8_t)data);
                    }
                    else
                    {
                        cpu->dcache_rdata = 0;
                        debug("dcache: read uart = %02x\n", cpu->dcache_rdata);
                    }
                }
                else
                {
                    uint8_t *m = &memory[0];
                    if (addr >= framebuffer_addr &&
                        addr < framebuffer_addr + framebuffer.size())
                    {
                        m = &framebuffer[0];
                        addr -= framebuffer_addr;
                    }
                    else
                        assert(addr + 4 < capacity);

                    if (cpu->dcache_wr)
                    {
                        if (cpu->dcache_ws == 0)
                        {
                            debug("dcache: write byte @ %08x = %02x\n", addr, data);
                            m[addr] = data & 0xff;
                        }
                        else if (cpu->dcache_ws == 1)
                        {
                            debug("dcache: write halfword @ %08x = %04x\n", addr, data);
                            m[addr] = data & 0xff;
                            m[addr + 1] = (data >> 8) & 0xff;
                        }
                        else if (cpu->dcache_ws == 2)
                        {
                            debug("dcache: write word @ %08x = %08x\n", addr, data);
                            m[addr] = data & 0xff;
                            m[addr + 1] = (data >> 8) & 0xff;
                            m[addr + 2] = (data >> 16) & 0xff;
                            m[addr + 3] = (data >> 24) & 0xff;
                        }
                        else assert(0);
                    }
                    else
                    {
                        cpu->dcache_rdata = *(uint32_t *)(m + addr);
                        debug("dcache: read word @ %08x = %08x\n", addr, cpu->dcache_rdata);
                    }
                }
                cpu->dcache_rdy = 1;
                dcache_state = 0;
                //schedule_next_dcache_rdy(1);
            } else {
                debug("delayed dcache response: %lu\n", dcache_next_rdy);
                dcache_next_rdy--;
            }
        }
    }

    void schedule_next_icache_rdy(uint64_t nstep) {
        icache_next_rdy = nstep;
    }

    void schedule_next_dcache_rdy(uint64_t nstep) {
        dcache_next_rdy = nstep;
    }

    uint8_t *get_framebuffer() {
        return &framebuffer[0];
    }

    uint8_t *get_ram() {
        return &memory[0];
    }
};

struct SoC {
    std::shared_ptr<Vcpu> cpu;
    SimulatedRAM ram;

    SoC(std::shared_ptr<Vcpu> cpu, size_t mem_cap, size_t fb_cap): cpu(cpu), ram(cpu, mem_cap, fb_cap) {}

    void reset() {
        cpu->clock = 0;
        cpu->reset = 1;
        tick();

        cpu->clock = 1;
        tick();

        cpu->reset = 0;
    }

    void tick() {
        if (!cpu->clock)
        {
            main_time++;
            ram.eval_posedge();
        }
        cpu->eval();
    }

    void next_tick() {
        cpu->clock = !cpu->clock;
        tick();
    }

    void halt() {
        cpu->final();               // Done simulating
    }

    uint8_t *get_framebuffer() {
        return ram.get_framebuffer();
    }

    void set_key(uint16_t key) {
        *(uint16_t *)(ram.get_ram() + input_addr) = key;
    }
};

static struct option long_options[] = {
    {"load-image", required_argument, 0, 'l'},
    {"halt-addr", required_argument, 0, 'e'},
    {"video", no_argument, 0, 'v'},
};


void die(const char *s) {
    fprintf(stderr, "error: %s\n", s);
    exit(1);
}

void ok_or_die(int ret, const char *s) {
    if (ret) die(s);
}

void signal_handler(int) {
#ifdef ENABLE_SDL
    if (window)
    {
        SDL_DestroyWindow(window);
        SDL_Quit();
    }
#endif
    exit(0);
}

#ifdef ENABLE_SDL
void try_update_frame(uint8_t *fb) {
    if (window == nullptr || (++frame_cnt < cycles_per_frame))
        return;
    SDL_Event event;
    while (key_events.size() < max_key_events && SDL_PollEvent(&event))
    {
        if (event.type == SDL_KEYDOWN && event.key.repeat == 0)
        {
            if (event.key.keysym.scancode == SDL_SCANCODE_ESCAPE)
                signal_handler(0);
            key_events.push((1 << 8) | event.key.keysym.scancode);
        }
        else if (event.type == SDL_KEYUP && event.key.repeat == 0)
            key_events.push(event.key.keysym.scancode);
    }
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    double millisec = now.tv_nsec < prev_time.tv_nsec ?
        (prev_time.tv_nsec - now.tv_nsec) / 1e6 + (now.tv_sec - prev_time.tv_sec - 1) * 1e3 :
        (now.tv_nsec - prev_time.tv_nsec) / 1e6 + (now.tv_sec - prev_time.tv_sec) * 1e3;
    // correction of cycles_per_frame
    cycles_per_frame = (double)cycles_per_frame / millisec / (fps / 1e3);
    prev_time = now;
    frame_cnt = 0;
    uint32_t len = win_width * win_height;
    for (auto p = fb, v = frame_tmp; p < fb + len; p++, v += 3)
    {
        auto c = *p;
        v[0] = (c & 0x3) << 6;
        v[1] = (c & 0xc) << 4;
        v[2] = (c & 0x30) << 2;
    }
    uint8_t *frame_dst;
    int pitch;
    assert(SDL_LockTexture(frame, nullptr, (void **)&frame_dst, &pitch) == 0);
    memmove(frame_dst, frame_tmp, pitch * win_height);
    SDL_UnlockTexture(frame);
    assert(SDL_RenderCopy(sdl_renderer, frame, nullptr, nullptr) == 0);
    SDL_RenderPresent(sdl_renderer);
}
#endif

int main(int argc, char** argv) {
    std::signal(SIGINT, signal_handler);
    bool enable_video = false;
    int optidx = 0;
    auto soc = SoC(std::make_shared<Vcpu>(), 40 << 20, 320 << 10);
    for (;;)
    {
        int c = getopt_long(argc, argv, "l:e:v", long_options, &optidx);
        if (c == -1) break;
        switch (c)
        {
            case 'l':
                {
                    std::string arg{optarg};
                    auto pos = arg.find("=");
                    if (pos == std::string::npos)
                        die("invalid image spec, should be in the form of `<filename>=<hex location>`");
                    FILE *img = fopen(arg.substr(0, pos).c_str(), "r");
                    if (img)
                    {
                        size_t t;
                        try {
                            auto loc = std::stoul(arg.substr(pos + 1), &t, 16);
                            soc.ram.load_image_from_file(img, loc);
                        } catch (...) {
                            die("invalid image location");
                        }
                        fclose(img);
                    } else
                        die("failed to open file");
                    break;
                }
            case 'e':
                {
                    size_t t;
                    try {
                        halt_addr = std::stoul(optarg, &t, 16);
                    } catch (...) {
                        die("invalid addr");
                    }
                    break;
                }
            case 'v':
                {
                    enable_video = true;
                    break;
                }
        }
    }
    Verilated::commandArgs(argc, argv);
    soc.reset();
    debug("reset\n");

#ifdef ENABLE_SDL
    if (enable_video)
    {
        ok_or_die(SDL_Init(SDL_INIT_VIDEO), "failed to initialize SDL");
        window = SDL_CreateWindow("mriscv",
                SDL_WINDOWPOS_UNDEFINED,
                SDL_WINDOWPOS_UNDEFINED,
                win_width, win_height, 0);
        ok_or_die(window == nullptr, "failed to create SDL window");
        sdl_renderer = SDL_CreateRenderer(window, -1, 0);
        ok_or_die(sdl_renderer == nullptr, "failed to create SDL renderer");
        frame = SDL_CreateTexture(sdl_renderer,
                SDL_PIXELFORMAT_RGB24,
                SDL_TEXTUREACCESS_STREAMING,
                win_width, win_height);
        ok_or_die(frame == nullptr, "failed to create SDL texture");
        clock_gettime(CLOCK_MONOTONIC, &prev_time);
    }
#endif

    while (!Verilated::gotFinish()) {
        soc.next_tick();
        debug("===\n");
        if (soc.cpu->_debug_pc == halt_addr)
        {
            soc.halt();
            printf("halted the processor at 0x%x\n", halt_addr);
            break;
        }
#ifdef ENABLE_SDL
        if (!soc.cpu->clock)
        {
            if (soc.cpu->irq)
                soc.cpu->irq = false;
            try_update_frame(soc.get_framebuffer());
        }
        else
        {
            if (key_events.size())
            {
                soc.set_key(key_events.front());
                key_events.pop();
                soc.cpu->irq = true;
            }
        }
#endif
    }
}
