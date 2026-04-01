#include <Arduino.h>
#include <esp_sleep.h>
#include <esp_random.h>

// ---------------------------------------------------------------------------
// Display driver — Seeed_GFX ePaper (Spectra 6 via ED2208)
// ---------------------------------------------------------------------------
#include "TFT_eSPI.h"

#ifdef EPAPER_ENABLE
EPaper *epaper = nullptr;
#endif

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

#ifdef BOARD_HAS_PSRAM
EXT_RAM_ATTR static uint8_t framebuf[FRAMEBUF_SIZE];
#else
static uint8_t framebuf[FRAMEBUF_SIZE];
#endif

// ===========================================================================
// SIM_GOL — Conway's Game of Life (bicolor)
// ===========================================================================
#ifdef SIM_GOL

#include "gol.h"
#include "renderer_bw.h"

// ---------------------------------------------------------------------------
// RTC memory for GoL
// ---------------------------------------------------------------------------
RTC_DATA_ATTR static bool     rtc_valid = false;
RTC_DATA_ATTR static uint32_t rtc_frame = 0;
RTC_DATA_ATTR static uint8_t  rtc_gol_grid[GOL_GRID_BYTES];

// Staleness detection: ring buffer of recent frame hashes
#define GOL_STALE_DEPTH 4
RTC_DATA_ATTR static uint32_t rtc_hashes[GOL_STALE_DEPTH];
RTC_DATA_ATTR static uint8_t  rtc_hash_idx = 0;
RTC_DATA_ATTR static uint8_t  rtc_hash_count = 0; // entries recorded so far

// ---------------------------------------------------------------------------
// GoL tunables
// ---------------------------------------------------------------------------
#define GOL_SEED_DENSITY      0.37f
#define GOL_SLEEP_DURATION_US 30000000ULL  // 30 s between frames
#define GOL_STEPS_PER_FRAME   20           // generations per display refresh
#define GOL_MAX_FRAMES        1000         // reseed ceiling (~8 hrs at 30s)

// ---------------------------------------------------------------------------
static void gol_new_generation()
{
    gol_seed(GOL_SEED_DENSITY);
    memcpy(rtc_gol_grid, gol_grid, GOL_GRID_BYTES);
    rtc_frame = 0;
    memset(rtc_hashes, 0, sizeof(rtc_hashes));
    rtc_hash_idx = 0;
    rtc_hash_count = 0;
    Serial0.printf("[lifecycle] new GoL generation seeded (%lu alive)\n",
                   (unsigned long)gol_population());
}

static void gol_restore()
{
    memcpy(gol_grid, rtc_gol_grid, GOL_GRID_BYTES);
}

static bool gol_is_stale()
{
    if (gol_population() == 0) return true;

    uint32_t h = gol_hash();
    int entries = (rtc_hash_count < GOL_STALE_DEPTH) ? rtc_hash_count : GOL_STALE_DEPTH;
    for (int i = 0; i < entries; i++) {
        if (rtc_hashes[i] == h) return true;
    }
    // Store current hash in ring buffer
    rtc_hashes[rtc_hash_idx] = h;
    rtc_hash_idx = (rtc_hash_idx + 1) % GOL_STALE_DEPTH;
    if (rtc_hash_count < GOL_STALE_DEPTH) rtc_hash_count++;
    return false;
}

static void gol_push_display()
{
    renderer_bw_render(framebuf, (int)FRAMEBUF_SIZE);

#ifdef EPAPER_ENABLE
    if (epaper) {
        for (int dy = 0; dy < DISP_H; dy++) {
            for (int dx = 0; dx < DISP_W; dx++) {
                int byte_idx = (dy * DISP_W + dx) / 2;
                uint8_t nibble;
                if ((dx & 1) == 0) {
                    nibble = (framebuf[byte_idx] >> 4) & 0x0F;
                } else {
                    nibble = framebuf[byte_idx] & 0x0F;
                }
                epaper->drawPixel(dx, dy, nibble == BW_BLACK ? TFT_BLACK : TFT_WHITE);
            }
        }
        epaper->update();
    }
#endif

    Serial0.printf("[frame %lu] pop=%lu\n",
                   (unsigned long)rtc_frame,
                   (unsigned long)gol_population());
}

// ---------------------------------------------------------------------------
// Arduino entry points — GoL
// ---------------------------------------------------------------------------
void setup()
{
    Serial0.begin(115200);
    delay(500);
    Serial0.println("[boot] Game of Life starting...");

#ifdef EPAPER_ENABLE
    Serial0.printf("[display] PSRAM: %s, free heap: %lu, free PSRAM: %lu\n",
                   psramFound() ? "yes" : "no",
                   (unsigned long)ESP.getFreeHeap(),
                   (unsigned long)ESP.getFreePsram());
    epaper = new EPaper();
    Serial0.printf("[display] sprite created: %s\n",
                   epaper->created() ? "yes" : "NO");
    epaper->begin();
    Serial0.println("[display] EPaper initialised");
#endif

    if (rtc_valid) {
        gol_restore();
    } else {
        gol_new_generation();
        rtc_valid = true;
    }
}

void loop()
{
    // Reseed if stale or hit frame ceiling
    if (rtc_frame >= GOL_MAX_FRAMES || gol_is_stale()) {
        Serial0.println("[lifecycle] GoL reseed triggered");
        gol_new_generation();
    }

    // Advance multiple generations per display refresh
    for (int s = 0; s < GOL_STEPS_PER_FRAME; s++) {
        gol_step();
    }

    // Persist grid to RTC
    memcpy(rtc_gol_grid, gol_grid, GOL_GRID_BYTES);

    // Render and push
    gol_push_display();
    rtc_frame++;

    // Deep sleep
    Serial0.printf("[sleep] deep sleep for %llu ms\n", GOL_SLEEP_DURATION_US / 1000ULL);
    Serial0.flush();
    esp_sleep_enable_timer_wakeup(GOL_SLEEP_DURATION_US);
    esp_deep_sleep_start();
}

// ===========================================================================
// SIM_SANDPILE — Abelian sandpile (Spectra 6 color)
// ===========================================================================
#else // default to sandpile

#include "sandpile.h"
#include "sources.h"
#include "territory.h"
#include "renderer.h"

// ---------------------------------------------------------------------------
// Spectra 6 colour mapping for display push
// ---------------------------------------------------------------------------
static uint16_t spectra6_to_gfx(uint8_t idx)
{
    switch (idx) {
        case SPECTRA6_BLACK:  return TFT_BLACK;
        case SPECTRA6_WHITE:  return TFT_WHITE;
        case SPECTRA6_GREEN:  return TFT_GREEN;
        case SPECTRA6_BLUE:   return TFT_BLUE;
        case SPECTRA6_RED:    return TFT_RED;
        case SPECTRA6_YELLOW: return TFT_YELLOW;
        default:              return TFT_WHITE;
    }
}

// ---------------------------------------------------------------------------
// RTC memory for sandpile
// ---------------------------------------------------------------------------
#define RTC_GRID_PACKED_SIZE ((GRID_W * GRID_H + 3) / 4)

RTC_DATA_ATTR static uint32_t rtc_seed      = 0;
RTC_DATA_ATTR static uint32_t rtc_frame     = 0;
RTC_DATA_ATTR static int      rtc_n_sources = 0;
RTC_DATA_ATTR static Source   rtc_sources[MAX_SOURCES];
RTC_DATA_ATTR static int      rtc_uf_parent[MAX_SOURCES];
RTC_DATA_ATTR static int      rtc_uf_rank[MAX_SOURCES];
RTC_DATA_ATTR static uint8_t  rtc_source_color[MAX_SOURCES];
RTC_DATA_ATTR static bool     rtc_valid = false;
RTC_DATA_ATTR static uint8_t  rtc_grid_packed[RTC_GRID_PACKED_SIZE];

// ---------------------------------------------------------------------------
// Sandpile tunables
// ---------------------------------------------------------------------------
#define FILL_THRESHOLD      0.82f
#define DROPS_PER_FRAME     2000
#define SLEEP_DURATION_US   30000000ULL
#define NUM_SOURCES_MIN     1
#define NUM_SOURCES_MAX     8
#define CIRCLE_RADIUS_FRAC  0.35f
#define MAX_FRAMES_PER_GENERATION 200

// ---------------------------------------------------------------------------
static void grid_pack()
{
    bool clamped = false;
    for (int i = 0; i < GRID_W * GRID_H; i++) {
        uint8_t v;
        if (grid[i] < 4) {
            v = (uint8_t)grid[i];
        } else {
            v = 3;
            clamped = true;
        }
        int byte_idx = i / 4;
        int shift    = (i % 4) * 2;
        if (shift == 0)
            rtc_grid_packed[byte_idx] = v;
        else
            rtc_grid_packed[byte_idx] |= (v << shift);
    }
    if (clamped) {
        Serial0.println("[pack] WARN: grid not fully stable, clamped cells >= 4");
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

static Source sources[MAX_SOURCES];
static int    n_sources = 0;
static int    uf_parent[MAX_SOURCES];
static int    uf_rank[MAX_SOURCES];

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
    rtc_n_sources = n_sources;
    for (int i = 0; i < n_sources; i++) {
        rtc_sources[i]   = sources[i];
        rtc_uf_parent[i] = uf_parent[i];
        rtc_uf_rank[i]   = uf_rank[i];
    }
    renderer_get_palette(rtc_source_color, n_sources);
    rtc_frame = 0;
    Serial0.printf("[lifecycle] new generation started: %d sources\n", n_sources);
}

static void generation_restore()
{
    n_sources = rtc_n_sources;
    for (int i = 0; i < n_sources; i++) {
        sources[i]   = rtc_sources[i];
        uf_parent[i] = rtc_uf_parent[i];
        uf_rank[i]   = rtc_uf_rank[i];
    }
    voronoi_compute(sources, n_sources);
    renderer_set_palette(rtc_source_color, n_sources);
}

static void drop_and_topple()
{
    static const float RAND31_MAX = (float)0x7FFFFFFFu;
    for (int i = 0; i < DROPS_PER_FRAME; i++) {
        float r = (float)(esp_random() >> 1) / RAND31_MAX;
        int src = sources_choose(sources, n_sources, r);
        sandpile_drop(sources[src].x, sources[src].y);
    }
    sandpile_topple();
}

static void push_display()
{
    renderer_render(framebuf, (int)FRAMEBUF_SIZE, uf_parent, n_sources);

#ifdef EPAPER_ENABLE
    if (epaper) {
        for (int dy = 0; dy < DISP_H; dy++) {
            for (int dx = 0; dx < DISP_W; dx++) {
                int byte_idx = (dy * DISP_W + dx) / 2;
                uint8_t nibble;
                if ((dx & 1) == 0) {
                    nibble = (framebuf[byte_idx] >> 4) & 0x0F;
                } else {
                    nibble = framebuf[byte_idx] & 0x0F;
                }
                epaper->drawPixel(dx, dy, spectra6_to_gfx(nibble));
            }
        }
        epaper->update();
    }
#endif

    Serial0.printf("[frame %lu] sources=%d fill=%.1f%%\n",
                  (unsigned long)rtc_frame,
                  n_sources,
                  sandpile_fill_percent() * 100.0f);
}

// ---------------------------------------------------------------------------
// Arduino entry points — Sandpile
// ---------------------------------------------------------------------------
void setup()
{
    Serial0.begin(115200);
    delay(500);
    Serial0.println("[boot] Sandpile starting...");

#ifdef EPAPER_ENABLE
    Serial0.printf("[display] PSRAM: %s, free heap: %lu, free PSRAM: %lu\n",
                   psramFound() ? "yes" : "no",
                   (unsigned long)ESP.getFreeHeap(),
                   (unsigned long)ESP.getFreePsram());
    epaper = new EPaper();
    Serial0.printf("[display] sprite created: %s\n",
                   epaper->created() ? "yes" : "NO");
    epaper->begin();
    Serial0.println("[display] EPaper initialised");
#endif

    if (rtc_seed == 0) {
        rtc_seed = esp_random();
    }
    randomSeed(rtc_seed + rtc_frame);

    if (rtc_valid) {
        generation_restore();
        grid_unpack();
    } else {
        new_generation();
        rtc_valid = true;
    }
}

void loop()
{
    if (rtc_frame >= MAX_FRAMES_PER_GENERATION
            || sandpile_fill_percent() > FILL_THRESHOLD) {
        Serial0.println("[lifecycle] generation reset triggered");
        rtc_seed ^= esp_random();
        randomSeed(rtc_seed);
        new_generation();
    }

    drop_and_topple();
    territory_check_merge(uf_parent, uf_rank, n_sources);
    for (int i = 0; i < n_sources; i++) {
        rtc_uf_parent[i] = uf_parent[i];
        rtc_uf_rank[i]   = uf_rank[i];
    }

    push_display();
    rtc_frame++;
    grid_pack();

    Serial0.printf("[sleep] deep sleep for %llu ms\n", SLEEP_DURATION_US / 1000ULL);
    Serial0.flush();
    esp_sleep_enable_timer_wakeup(SLEEP_DURATION_US);
    esp_deep_sleep_start();
}

#endif // SIM_GOL / sandpile
