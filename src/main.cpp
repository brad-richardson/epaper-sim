#include <Arduino.h>
#include <esp_sleep.h>
#include <esp_random.h>

#include "sandpile.h"
#include "sources.h"
#include "territory.h"
#include "renderer.h"

// ---------------------------------------------------------------------------
// Display driver — include the Seeed reTerminal E10xx header.
// Adjust the include path to match the installed library version.
// ---------------------------------------------------------------------------
// #include <ReTerminal_E10xx.h>   // uncomment once library is installed

// ---------------------------------------------------------------------------
// Display geometry and framebuffer
// ---------------------------------------------------------------------------
#ifndef DISP_W
#define DISP_W 800
#endif
#ifndef DISP_H
#define DISP_H 480
#endif

#define FRAMEBUF_SIZE ((DISP_W * DISP_H + 1) / 2)

// Place the ~192 KB packed framebuffer in PSRAM when available.
// Without PSRAM this will eat nearly all of ESP32-S3's ~320 KB heap.
#ifdef BOARD_HAS_PSRAM
EXT_RAM_ATTR static uint8_t framebuf[FRAMEBUF_SIZE];
#else
static uint8_t framebuf[FRAMEBUF_SIZE];
#endif

// ---------------------------------------------------------------------------
// RTC memory — survives deep sleep.
//
// The full grid (uint16_t, 28.8 KB) is too large for RTC, but after toppling
// every cell is in [0, 3], so we pack 4 cells per byte (2 bits each).
// 120×120 / 4 = 3600 bytes — fits comfortably in ESP32-S3 RTC slow memory.
// ---------------------------------------------------------------------------
#define RTC_GRID_PACKED_SIZE ((GRID_W * GRID_H + 3) / 4)

RTC_DATA_ATTR static uint32_t rtc_seed      = 0;  // persistent RNG seed
RTC_DATA_ATTR static uint32_t rtc_frame     = 0;  // frames rendered this generation
RTC_DATA_ATTR static int      rtc_n_sources = 0;
RTC_DATA_ATTR static Source   rtc_sources[MAX_SOURCES];
RTC_DATA_ATTR static int      rtc_uf_parent[MAX_SOURCES];
RTC_DATA_ATTR static int      rtc_uf_rank[MAX_SOURCES];
// Palette persisted so colours are stable across every deep-sleep/wake cycle
RTC_DATA_ATTR static uint8_t  rtc_source_color[MAX_SOURCES];
RTC_DATA_ATTR static bool     rtc_valid = false; // set true once first frame is done
RTC_DATA_ATTR static uint8_t  rtc_grid_packed[RTC_GRID_PACKED_SIZE];

// ---------------------------------------------------------------------------
// Lifecycle tunables
// ---------------------------------------------------------------------------
#define FILL_THRESHOLD      0.82f         // new generation above this fill %
#define DROPS_PER_FRAME     2000          // grains to drop per refresh cycle
#define SLEEP_DURATION_US   30000000ULL   // 30 s deep sleep between refreshes
#define NUM_SOURCES_MIN     1
#define NUM_SOURCES_MAX     8
#define CIRCLE_RADIUS_FRAC  0.35f         // source circle radius as fraction of min(W,H)
// Start a new generation after this many frames (~100 min at 30 s/frame).
// Acts as a ceiling alongside the fill threshold so the display always
// cycles through different source configurations eventually.
#define MAX_FRAMES_PER_GENERATION 200

// ---------------------------------------------------------------------------
// Pack / unpack the grid into 2-bit-per-cell RTC storage.
// After toppling, all cell values are in [0, 3]; values >= 4 are clamped.
// ---------------------------------------------------------------------------
static void grid_pack()
{
    for (int i = 0; i < GRID_W * GRID_H; i++) {
        uint8_t v = (grid[i] < 4) ? (uint8_t)grid[i] : 3;
        int byte_idx = i / 4;
        int shift    = (i % 4) * 2;
        if (shift == 0)
            rtc_grid_packed[byte_idx] = v;
        else
            rtc_grid_packed[byte_idx] |= (v << shift);
    }
}

static void grid_unpack()
{
    for (int i = 0; i < GRID_W * GRID_H; i++) {
        int byte_idx = i / 4;
        int shift    = (i % 4) * 2;
        grid[i] = (rtc_grid_packed[byte_idx] >> shift) & 0x03;
    }
}

// ---------------------------------------------------------------------------
// Active generation state (mirrors the RTC copies for in-loop convenience)
// ---------------------------------------------------------------------------
static Source sources[MAX_SOURCES];
static int    n_sources = 0;
static int    uf_parent[MAX_SOURCES];
static int    uf_rank[MAX_SOURCES];

// ---------------------------------------------------------------------------
// Start a brand-new generation: new sources, palette, Voronoi map, clear grid
// ---------------------------------------------------------------------------
static void new_generation()
{
    sandpile_reset();

    n_sources = (int)random(NUM_SOURCES_MIN, NUM_SOURCES_MAX + 1);

    float cx     = GRID_W / 2.0f;
    float cy     = GRID_H / 2.0f;
    float radius = CIRCLE_RADIUS_FRAC * (float)(GRID_W < GRID_H ? GRID_W : GRID_H);

    sources_generate(n_sources, sources, cx, cy, radius);
    voronoi_compute(sources, n_sources);
    renderer_init(n_sources, sources);
    uf_init(uf_parent, uf_rank, n_sources);

    // Persist generation metadata in RTC so it survives deep sleep
    rtc_n_sources = n_sources;
    for (int i = 0; i < n_sources; i++) {
        rtc_sources[i]   = sources[i];
        rtc_uf_parent[i] = uf_parent[i];
        rtc_uf_rank[i]   = uf_rank[i];
    }
    // Save the palette so colours are stable every frame of this generation
    renderer_get_palette(rtc_source_color, n_sources);
    rtc_frame = 0;

    Serial.printf("[lifecycle] new generation started: %d sources\n", n_sources);
}

// ---------------------------------------------------------------------------
// Restore generation state from RTC memory after a deep sleep wake
// ---------------------------------------------------------------------------
static void generation_restore()
{
    n_sources = rtc_n_sources;
    for (int i = 0; i < n_sources; i++) {
        sources[i]   = rtc_sources[i];
        uf_parent[i] = rtc_uf_parent[i];
        uf_rank[i]   = rtc_uf_rank[i];
    }
    voronoi_compute(sources, n_sources);
    // Restore the saved palette -- do NOT reshuffle, so colours stay consistent
    renderer_set_palette(rtc_source_color, n_sources);
}

// ---------------------------------------------------------------------------
// Drop DROPS_PER_FRAME grains, then topple to stability.
// Use esp_random() (hardware TRNG) for full 32-bit randomness.
// ---------------------------------------------------------------------------
static void drop_and_topple()
{
    // Divisor: max value of a 31-bit right-shifted esp_random() result,
    // used to map the hardware TRNG output to the [0, 1) range.
    static const float RAND31_MAX = (float)0x7FFFFFFFu;

    for (int i = 0; i < DROPS_PER_FRAME; i++) {
        // Shift right by 1 to keep the value in [0, 0x7FFFFFFF], then
        // divide by RAND31_MAX to get a uniform float in [0, 1).
        float r = (float)(esp_random() >> 1) / RAND31_MAX;
        int src = sources_choose(sources, n_sources, r);
        sandpile_drop(sources[src].x, sources[src].y);
    }
    sandpile_topple();
}

// ---------------------------------------------------------------------------
// Render the framebuffer and push it to the display
// ---------------------------------------------------------------------------
static void push_display()
{
    renderer_render(framebuf, (int)FRAMEBUF_SIZE, uf_parent, n_sources);

    // Push to the Seeed E10xx display driver once the library is integrated:
    //
    //   epaper.writeBuffer(framebuf, FRAMEBUF_SIZE);
    //   epaper.refresh();   // starts the ~15-30 s waveform refresh
    //
    // IMPORTANT: wait for the display's BUSY pin to deassert (or for the
    // driver's completion callback) before calling esp_deep_sleep_start().
    // Sleeping while the waveform is still running will corrupt the image.
    // Example (adjust pin / API to match the actual driver):
    //
    //   while (digitalRead(EPAPER_BUSY_PIN) == HIGH) delay(10);
    //
    Serial.printf("[frame %lu] sources=%d fill=%.1f%%\n",
                  (unsigned long)rtc_frame,
                  n_sources,
                  sandpile_fill_percent() * 100.0f);
}

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------
void setup()
{
    Serial.begin(115200);

    // TODO: configure a GPIO wakeup source so a button press forces a reseed.
    // Example:
    //   esp_sleep_enable_ext0_wakeup((gpio_num_t)RESEED_BUTTON_PIN, 0);
    //   if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0) {
    //       rtc_valid = false;  // force new_generation() on next boot
    //   }

    // Seed the RNG.  On first power-on rtc_seed == 0, so use the hardware RNG.
    if (rtc_seed == 0) {
        rtc_seed = esp_random();
    }
    randomSeed(rtc_seed + rtc_frame);

    if (rtc_valid) {
        // Waking from deep sleep -- restore generation metadata and grid
        generation_restore();
        grid_unpack();
    } else {
        // True first boot -- start fresh
        new_generation();
        rtc_valid = true;
    }
}

void loop()
{
    // Start a new generation if the grid is saturated or we've been running long enough
    if (rtc_frame >= MAX_FRAMES_PER_GENERATION
            || sandpile_fill_percent() > FILL_THRESHOLD) {
        Serial.println("[lifecycle] generation reset triggered");
        // XOR-mix the existing seed with fresh hardware entropy so each
        // generation starts from a visually distinct random state.
        rtc_seed ^= esp_random();
        randomSeed(rtc_seed);
        new_generation();
    }

    // 1. Drop grains and topple to stability
    drop_and_topple();

    // 2. Detect territory collisions and merge
    territory_check_merge(uf_parent, uf_rank, n_sources);

    // 3. Mirror union-find state into RTC so merges survive deep sleep
    for (int i = 0; i < n_sources; i++) {
        rtc_uf_parent[i] = uf_parent[i];
        rtc_uf_rank[i]   = uf_rank[i];
    }

    // 4. Render and push to display
    push_display();
    rtc_frame++;

    // 5. Pack grid into RTC memory so it survives deep sleep (3600 bytes)
    grid_pack();

    // 6. Deep sleep until the next refresh cycle.
    //    The ePaper display holds its image with zero power draw during sleep.
    Serial.printf("[sleep] deep sleep for %llu ms\n", SLEEP_DURATION_US / 1000ULL);
    Serial.flush();
    esp_sleep_enable_timer_wakeup(SLEEP_DURATION_US);
    esp_deep_sleep_start();

    // esp_deep_sleep_start() does not return; the MCU wakes up in setup().
}
