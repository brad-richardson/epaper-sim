#include "sources.h"
#include "sandpile.h"
#include <math.h>
#include <Arduino.h>

// ---------------------------------------------------------------------------
void sources_generate(int n, Source *sources,
                      float cx, float cy, float radius)
{
    // For a single source, place it directly at the centre
    // (the angular formula would offset it by radius otherwise).
    // For multiple sources, use equidistant angular placement.
    float phase = (n > 1)
        ? (float)random(0, 1000) / 1000.0f * 2.0f * (float)M_PI
        : 0.0f;
    float angle_step = 2.0f * (float)M_PI / (float)n;
    float r = (n > 1) ? radius : 0.0f;

    // Generate raw random weights in [1, 10]
    float total = 0.0f;
    for (int i = 0; i < n; i++) {
        float angle = phase + i * angle_step;
        sources[i].x = (int)(cx + r * cosf(angle) + 0.5f);
        sources[i].y = (int)(cy + r * sinf(angle) + 0.5f);

        // Clamp to grid bounds
        if (sources[i].x < 0)        sources[i].x = 0;
        if (sources[i].x >= GRID_W)  sources[i].x = GRID_W - 1;
        if (sources[i].y < 0)        sources[i].y = 0;
        if (sources[i].y >= GRID_H)  sources[i].y = GRID_H - 1;

        sources[i].weight = (float)(random(1, 11)); // 1..10
        total += sources[i].weight;
    }

    // Normalise weights to sum to 1.0
    for (int i = 0; i < n; i++) {
        sources[i].weight /= total;
    }
}

// ---------------------------------------------------------------------------
int sources_choose(const Source *sources, int n, float r)
{
    float cumulative = 0.0f;
    for (int i = 0; i < n; i++) {
        cumulative += sources[i].weight;
        if (r < cumulative) {
            return i;
        }
    }
    // Floating-point rounding safety: return last source
    return n - 1;
}
