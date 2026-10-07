// ============================================================
// XIAO ESP32S3 + OV3660 -- ON-DEVICE VOWEL DETECTOR
// ============================================================
// Everything runs on the microcontroller: camera capture,
// image processing, and vowel classification. Results print to
// the Arduino IDE Serial Monitor. No PC/Python needed to use it.
//
// No AI / trained model of any kind is used. Every step below is
// classical, deterministic image processing:
//   - Otsu thresholding      (standard, non-ML, textbook algorithm)
//   - connected-component /
//     flood-fill labeling    (counts blobs and enclosed holes)
//   - geometric radial signatures (rotation-invariant pixel geometry)
//   - multi-frame label voting (stability, not learning)
//
// HOW IT WORKS
// ------------
// This version is designed for the ACTUAL custom vowel graphics used
// on the physical cylindrical blocks, not normal typed A/E/I/O/U.
//
// Tracking:
//   1) capture VGA 640x480 grayscale from OV3660
//   2) locate the bright circular face
//   3) temporally smooth/hold the marker bounding box
//   4) crop a square around the tracked face
//   5) resize to 128x128 without changing the symbol proportions
//
// Image cleanup:
//   - OV3660 sensor contrast/sharpness/denoise
//   - percentile black/white contrast stretch
//   - Otsu thresholding
//   - circular face mask
//   - tiny connected-component noise rejection
//
// Custom symbol rules:
//   I = separate dot + stem
//   U = one open curved stroke in the measured radial band
//   O = clean ring
//   A = ring + attached stroke
//   E = curved body + centre/cross stroke
//
// A/E/O use rotation-invariant radial ink signatures measured from
// the supplied close-up custom vowel masks. No AI, model, training,
// neural network, or learned parameters are used.
//
// Each frame is classified independently and the labels are voted.
// Unknown/poor frames are rejected instead of being forced into a vowel.
// If no valid marker/vowel is seen for several frames, Serial prints:
//   No vowel detected
//
// SERIAL MONITOR
// ---------------
// Open Tools > Serial Monitor at 115200 baud after uploading.
// Hold the vowel block steady, centered, filling most of the
// camera's view. Status messages guide you (MOVE CLOSER, MOVE
// LEFT, READY, ...). Once READY, give it a second to fill its
// frame buffer, then it prints the detected vowel.
// ============================================================

#include "esp_camera.h"
#include <string.h>
#include <math.h>

// Arduino auto-generates forward declarations for every function in this
// file and inserts them right after the includes, above. Any struct type
// used in a function signature (Comp/Hole/Blob/DiscInfo below) must
// already be declared by that point or the auto-generated prototype
// fails to compile -- so these live here instead of near the functions
// that use them.
struct Comp { int area; float cx, cy; };
struct Hole { int area; float cxNorm, cyNorm; };
struct Blob { int area; int x0, y0, x1, y1; };
struct DiscInfo {
    bool found;
    int x0, y0, x1, y1; // in FRAME coordinates
};

// ------------------------------------------------------------
// XIAO ESP32S3 Sense camera pin mapping
// ------------------------------------------------------------
#define PWDN_GPIO_NUM   -1
#define RESET_GPIO_NUM  -1
#define XCLK_GPIO_NUM   10
#define SIOD_GPIO_NUM   40
#define SIOC_GPIO_NUM   39

#define Y9_GPIO_NUM     48
#define Y8_GPIO_NUM     11  
#define Y7_GPIO_NUM     12
#define Y6_GPIO_NUM     14
#define Y5_GPIO_NUM     16
#define Y4_GPIO_NUM     18
#define Y3_GPIO_NUM     17
#define Y2_GPIO_NUM     15

#define VSYNC_GPIO_NUM  38
#define HREF_GPIO_NUM   47
#define PCLK_GPIO_NUM   13

// ------------------------------------------------------------
// UART link to PSoC 5LP
// XIAO D6 = GPIO43 = TX  -> PSoC P15[5] / Rx_2
// XIAO D7 = GPIO44 = RX  (not used for this one-way link)
// ------------------------------------------------------------
#define PSOC_UART_RX_PIN 44
#define PSOC_UART_TX_PIN 43
#define PSOC_UART_BAUD   9600

// ============================================================
// SETTINGS
// ============================================================

#define FRAME_WIDTH   640
#define FRAME_HEIGHT  480

// Small downsampled grid used only to locate the white marker.
#define WORK_W 160
#define WORK_H 120
#define WORK_SCALE_X (FRAME_WIDTH / WORK_W)   // 4
#define WORK_SCALE_Y (FRAME_HEIGHT / WORK_H)  // 4

// Final normalized square the vowel shape gets resized into.
#define ANALYSIS_SIZE 128
#define ANALYSIS_N (ANALYSIS_SIZE * ANALYSIS_SIZE)

// Dynamic white-marker threshold. Otsu finds the scene split, then
// we bias toward only the brighter pixels. The clamp prevents bad
// thresholds in unusually dark or bright scenes.
#define WHITE_THRESHOLD_MIN 95
#define WHITE_THRESHOLD_MAX 225
#define WHITE_THRESHOLD_OFFSET 0
#define MIN_BLOB_AREA_RATIO 0.018f

// Marker positioning is intentionally generous so the detector is less fussy.
#define MIN_MARKER_WIDTH_RATIO 0.10f
#define MAX_MARKER_WIDTH_RATIO 0.95f
#define CENTER_TOLERANCE_X 0.38f
#define CENTER_TOLERANCE_Y 0.38f

// Ignore tiny noise blobs / tiny enclosed areas.
#define MIN_BLACK_COMPONENT_RATIO 0.0012f
#define MIN_HOLE_RATIO 0.0020f

// Reject crops that contain almost no ink or are mostly black.
#define MIN_INK_RATIO 0.0030f
#define MAX_INK_RATIO 0.55f

// ============================================================
// LIGHTING TEST MODE
// ============================================================
// This used to be a single compile-time choice (BRIGHT_LIGHT_MODE
// true/false) between "room light" and "one specific bright-light test".
// That breaks as soon as the actual lighting sits somewhere else on the
// scale: extra light beyond whatever level true/false was tuned for
// re-washes out the ink contrast, and the detector gets stuck on
// "No vowels detected" or misreads the shape.
//
// Instead, EXPOSURE_TIERS below are picked per frame from the crop's
// own measured mean brightness, so it keeps adapting as lighting
// changes instead of topping out at one assumed level. BRIGHT_LIGHT_MODE
// still controls the one-time sensor init bias (SENSOR_BRIGHTNESS/
// SENSOR_AE_LEVEL below) since that can't be changed per frame.
#define BRIGHT_LIGHT_MODE true

// Ordered brightest-first: the first tier whose meanBrightnessAbove the
// frame's mean brightness meets or exceeds is used.
//   { meanBrightnessAbove, contrastLowPercent, contrastHighPercent, otsuBias }
struct ExposureTier { float meanBrightnessAbove; int lowPercent; int highPercent; int otsuBias; };
static const ExposureTier EXPOSURE_TIERS[] = {
    { 175.0f, 10, 90, 18 }, // extra-bright / direct glare
    { 140.0f,  8, 92, 14 }, // original "BRIGHT_LIGHT_MODE true" tuning
    {   0.0f,  3, 97,  6 }, // original room-light tuning
};

// Measures the crop's mean brightness and selects the matching tier.
// ponytail: brightness tiers are a coarse 3-step curve, not a fitted
// exposure model. If lighting keeps drifting past the top tier, add
// another { meanBrightnessAbove, ... } row above rather than a new flag.
static void pickExposureTier(uint8_t* gray, int n, int* lowPercent, int* highPercent, int* otsuBias) {
    long sum = 0;
    for (int i = 0; i < n; i++) sum += gray[i];
    float mean = (float)sum / (float)n;

    for (unsigned i = 0; i < sizeof(EXPOSURE_TIERS) / sizeof(EXPOSURE_TIERS[0]); i++) {
        if (mean >= EXPOSURE_TIERS[i].meanBrightnessAbove) {
            *lowPercent = EXPOSURE_TIERS[i].lowPercent;
            *highPercent = EXPOSURE_TIERS[i].highPercent;
            *otsuBias = EXPOSURE_TIERS[i].otsuBias;
            return;
        }
    }
}

// ============================================================
// CUSTOM PHYSICAL VOWEL RULES (from the actual close-up designs)
// ============================================================
// I = separate dot + stem
// U = one open curved stroke
// O = clean ring
// A = ring + attached stroke
// E = curved body + centre/cross stroke
//
// The A/E/O reference signatures below were measured from the
// supplied custom vowel masks. This is deterministic template
// geometry, NOT ML/training.
#define I_SECOND_RATIO_MIN 0.030f
#define I_SECOND_RATIO_MAX 0.60f

#define U_RADIAL_MIN 0.100f
#define U_RADIAL_MAX 0.360f

#define RADIAL_BINS 12
#define RADIAL_MAX_RADIUS_BIN 192
#define AEO_MATCH_MAX 0.85f
#define AEO_MATCH_MIN_MARGIN 0.025f

// Temporal marker tracking. A single bad frame will not immediately
// lose the white circular face.
#define TRACK_HOLD_FRAMES 5
#define TRACK_SMOOTH_ALPHA 0.35f
#define TRACK_MAX_CENTER_JUMP 0.24f
#define TRACK_MAX_SIZE_RATIO 1.75f

// Continuous sliding-window confirmation:
// Always evaluate the latest 5 classification checks.
// Accept a vowel only when it appears at least 4 times out of those 5.
// Continue checking after a result is confirmed.
#define VOTE_FRAMES 5
#define VOTE_REQUIRED 4
#define NO_VOWEL_FRAMES 18

// OV3660 sensor-side image controls.
// Keep contrast/sharpness/denoise unchanged so the decision geometry
// is not disturbed.  For strong light, bias the sensor slightly darker
// so highlights are less likely to wash out the black ink.
#define SENSOR_CONTRAST 3
#define SENSOR_SHARPNESS 1
#if BRIGHT_LIGHT_MODE
    #define SENSOR_BRIGHTNESS -1
    #define SENSOR_AE_LEVEL   -1
#else
    #define SENSOR_BRIGHTNESS  0
    #define SENSOR_AE_LEVEL    0
#endif
#define SENSOR_DENOISE 1
#define CAMERA_HMIRROR 0
#define CAMERA_VFLIP 0

// Turn on while tuning lighting. It prints the live threshold statistics.
#define SHOW_DEBUG false

// V4 tracking: if the bright white face cannot be isolated reliably,
// analyse a generous centre crop and remove all dark pixels connected
// to the crop border. This leaves the isolated black vowel on the
// white face and prevents a bright/dark background from blocking detection.
#define FALLBACK_CROP_RATIO 0.80f
#define NORMALIZED_SYMBOL_MAX 100
#define UNCLEAR_FRAMES 12

// Bright-light tuning diagnostics.
// IMPORTANT: these are declared here because percentileContrastStretch()
// uses them later in the file.
int debugStretchLow = 0;
int debugStretchHigh = 255;
int debugBinaryThreshold = 0;

// ============================================================
// SMALL IMAGE-PROCESSING TOOLKIT (classical CV, no ML)
// ============================================================
// (struct Comp/Hole/Blob/DiscInfo moved to the top of the file, right
// after the includes -- see note up there.)

// ---- 4-connected flood-fill labeling ----
static int labelComponents(uint8_t* bin, int w, int h, Comp* out, int maxComps,
                            float minAreaRatio, int* stack, uint8_t* visited) {
    int n = w * h;
    memset(visited, 0, n);
    int found = 0;
    float minArea = minAreaRatio * n;

    for (int start = 0; start < n; start++) {
        if (bin[start] == 0 || visited[start]) continue;

        int sp = 0;
        stack[sp++] = start;
        visited[start] = 1;

        long area = 0;
        double sumx = 0, sumy = 0;

        while (sp > 0) {
            int idx = stack[--sp];
            int x = idx % w, y = idx / w;
            area++;
            sumx += x;
            sumy += y;

            int nb[4] = {idx - 1, idx + 1, idx - w, idx + w};
            for (int k = 0; k < 4; k++) {
                int nidx = nb[k];
                if (k == 0 && x == 0) continue;
                if (k == 1 && x == w - 1) continue;
                if (k == 2 && y == 0) continue;
                if (k == 3 && y == h - 1) continue;
                if (nidx < 0 || nidx >= n) continue;
                if (bin[nidx] && !visited[nidx]) {
                    visited[nidx] = 1;
                    stack[sp++] = nidx;
                }
            }
        }

        if (area >= minArea && found < maxComps) {
            out[found].area = (int)area;
            out[found].cx = (float)(sumx / area);
            out[found].cy = (float)(sumy / area);
            found++;
        }
    }

    for (int i = 1; i < found; i++) {
        Comp key = out[i];
        int j = i - 1;
        while (j >= 0 && out[j].area < key.area) {
            out[j + 1] = out[j];
            j--;
        }
        out[j + 1] = key;
    }

    return found;
}

// ---- Enclosed holes: background not reachable from the border ----
static int findHoles(uint8_t* bin, int w, int h, Hole* out, int maxHoles,
                      float minAreaRatio, int* stack, uint8_t* outsideBuf,
                      uint8_t* holeBinBuf, int32_t* labelScratch) {
    int n = w * h;
    memset(outsideBuf, 0, n);

    int sp = 0;
    for (int x = 0; x < w; x++) {
        int top = x, bot = (h - 1) * w + x;
        if (bin[top] == 0 && !outsideBuf[top]) { outsideBuf[top] = 1; stack[sp++] = top; }
        if (bin[bot] == 0 && !outsideBuf[bot]) { outsideBuf[bot] = 1; stack[sp++] = bot; }
    }
    for (int y = 0; y < h; y++) {
        int left = y * w, right = y * w + (w - 1);
        if (bin[left] == 0 && !outsideBuf[left]) { outsideBuf[left] = 1; stack[sp++] = left; }
        if (bin[right] == 0 && !outsideBuf[right]) { outsideBuf[right] = 1; stack[sp++] = right; }
    }

    while (sp > 0) {
        int idx = stack[--sp];
        int x = idx % w, y = idx / w;
        int nb[4] = {idx - 1, idx + 1, idx - w, idx + w};
        for (int k = 0; k < 4; k++) {
            int nidx = nb[k];
            if (k == 0 && x == 0) continue;
            if (k == 1 && x == w - 1) continue;
            if (k == 2 && y == 0) continue;
            if (k == 3 && y == h - 1) continue;
            if (nidx < 0 || nidx >= n) continue;
            if (bin[nidx] == 0 && !outsideBuf[nidx]) {
                outsideBuf[nidx] = 1;
                stack[sp++] = nidx;
            }
        }
    }

    for (int i = 0; i < n; i++) {
        holeBinBuf[i] = (bin[i] == 0 && !outsideBuf[i]) ? 1 : 0;
    }

    Comp comps[16];
    // reuse outsideBuf as a plain visited scratch buffer here (safe: no longer needed)
    int count = labelComponents(holeBinBuf, w, h, comps, 16, minAreaRatio, stack, outsideBuf);

    for (int i = 0; i < count && i < maxHoles; i++) {
        out[i].area = comps[i].area;
        out[i].cxNorm = comps[i].cx / w;
        out[i].cyNorm = comps[i].cy / h;
    }

    return count < maxHoles ? count : maxHoles;
}

// ---- Otsu threshold ----
static int otsuThreshold(uint8_t* gray, int n) {
    int hist[256] = {0};
    for (int i = 0; i < n; i++) hist[gray[i]]++;

    double total = n;
    double sumAll = 0;
    for (int t = 0; t < 256; t++) sumAll += (double)t * hist[t];

    double sumB = 0, wB = 0, maxVar = 0;
    int threshold = 127;

    for (int t = 0; t < 256; t++) {
        wB += hist[t];
        if (wB == 0) continue;
        double wF = total - wB;
        if (wF == 0) break;

        sumB += (double)t * hist[t];
        double mB = sumB / wB;
        double mF = (sumAll - sumB) / wF;

        double varBetween = wB * wF * (mB - mF) * (mB - mF);
        if (varBetween > maxVar) {
            maxVar = varBetween;
            threshold = t;
        }
    }
    return threshold;
}

// ---- Percentile contrast stretch ----
// Maps a dark percentile to 0 and a bright percentile to 255.
// This is much more stable than a fixed brightness/contrast multiplier
// when room lighting or automatic exposure changes.
static void percentileContrastStretch(uint8_t* gray, int n, int lowPercent, int highPercent) {
    int hist[256] = {0};
    for (int i = 0; i < n; i++) hist[gray[i]]++;

    int lowTarget = (n * lowPercent) / 100;
    int highTarget = (n * highPercent) / 100;

    int cumulative = 0;
    int lo = 0, hi = 255;

    for (int v = 0; v < 256; v++) {
        cumulative += hist[v];
        if (cumulative >= lowTarget) {
            lo = v;
            break;
        }
    }

    cumulative = 0;
    for (int v = 0; v < 256; v++) {
        cumulative += hist[v];
        if (cumulative >= highTarget) {
            hi = v;
            break;
        }
    }

    // Save these values for Serial debug so lighting can be tuned.
    debugStretchLow = lo;
    debugStretchHigh = hi;

    // If the image is already almost flat, do not amplify noise.
    if (hi - lo < 24) return;

    int range = hi - lo;
    for (int i = 0; i < n; i++) {
        int v = gray[i];
        if (v <= lo) gray[i] = 0;
        else if (v >= hi) gray[i] = 255;
        else gray[i] = (uint8_t)(((v - lo) * 255) / range);
    }
}


static float iouSameSize(uint8_t* a, uint8_t* b, int n) {
    long inter = 0, uni = 0;
    for (int i = 0; i < n; i++) {
        bool av = a[i] != 0, bv = b[i] != 0;
        if (av || bv) uni++;
        if (av && bv) inter++;
    }
    if (uni == 0) return 0.0f;
    return (float)inter / (float)uni;
}

static void rotate180(uint8_t* src, uint8_t* dst, int w, int h) {
    int n = w * h;
    for (int i = 0; i < n; i++) dst[i] = src[n - 1 - i];
}
static void flipVertical(uint8_t* src, uint8_t* dst, int w, int h) {
    for (int y = 0; y < h; y++) memcpy(dst + y * w, src + (h - 1 - y) * w, w);
}
static void flipHorizontal(uint8_t* src, uint8_t* dst, int w, int h) {
    for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++)
            dst[y * w + x] = src[y * w + (w - 1 - x)];
}

// ---- Zero out connected blobs smaller than minAreaRatio ----
static void removeSmallComponents(uint8_t* bin, int w, int h, float minAreaRatio,
                                   int32_t* labelBuf, int* stack) {
    int n = w * h;
    for (int i = 0; i < n; i++) labelBuf[i] = -1;

    int nextLabel = 0;
    int areas[64];

    for (int start = 0; start < n; start++) {
        if (!bin[start] || labelBuf[start] != -1) continue;

        int label = nextLabel < 64 ? nextLabel : 63;
        nextLabel++;

        int sp = 0;
        stack[sp++] = start;
        labelBuf[start] = label;
        int area = 0;

        while (sp > 0) {
            int idx = stack[--sp];
            int x = idx % w, y = idx / w;
            area++;

            int nb[4] = {idx - 1, idx + 1, idx - w, idx + w};
            for (int k = 0; k < 4; k++) {
                int nidx = nb[k];
                if (k == 0 && x == 0) continue;
                if (k == 1 && x == w - 1) continue;
                if (k == 2 && y == 0) continue;
                if (k == 3 && y == h - 1) continue;
                if (nidx < 0 || nidx >= n) continue;
                if (bin[nidx] && labelBuf[nidx] == -1) {
                    labelBuf[nidx] = label;
                    stack[sp++] = nidx;
                }
            }
        }

        if (label < 64) areas[label] = area;
    }

    float minArea = minAreaRatio * n;
    for (int i = 0; i < n; i++) {
        if (bin[i]) {
            int lbl = labelBuf[i];
            if (lbl < 64 && areas[lbl] < minArea) bin[i] = 0;
        }
    }
}

static void dilate3x3(uint8_t* src, uint8_t* dst, int w, int h) {
    for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++) {
            uint8_t v = 0;
            for (int dy = -1; dy <= 1 && !v; dy++)
                for (int dx = -1; dx <= 1 && !v; dx++) {
                    int nx = x + dx, ny = y + dy;
                    if (nx >= 0 && nx < w && ny >= 0 && ny < h && src[ny * w + nx]) v = 1;
                }
            dst[y * w + x] = v;
        }
}
static void erode3x3(uint8_t* src, uint8_t* dst, int w, int h) {
    for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++) {
            uint8_t v = 1;
            for (int dy = -1; dy <= 1 && v; dy++)
                for (int dx = -1; dx <= 1 && v; dx++) {
                    int nx = x + dx, ny = y + dy;
                    if (nx < 0 || nx >= w || ny < 0 || ny >= h) { v = 0; break; }
                    if (!src[ny * w + nx]) v = 0;
                }
            dst[y * w + x] = v;
        }
}

static bool findLargestBlob(uint8_t* bin, int w, int h, float minAreaRatio,
                             Blob* out, int* stack, uint8_t* visited) {
    int n = w * h;
    memset(visited, 0, n);
    float minArea = minAreaRatio * n;

    Blob best; best.area = 0;
    bool found = false;

    for (int start = 0; start < n; start++) {
        if (!bin[start] || visited[start]) continue;

        int sp = 0;
        stack[sp++] = start;
        visited[start] = 1;

        int area = 0, x0 = w, y0 = h, x1 = -1, y1 = -1;

        while (sp > 0) {
            int idx = stack[--sp];
            int x = idx % w, y = idx / w;
            area++;
            if (x < x0) x0 = x;
            if (x > x1) x1 = x;
            if (y < y0) y0 = y;
            if (y > y1) y1 = y;

            int nb[4] = {idx - 1, idx + 1, idx - w, idx + w};
            for (int k = 0; k < 4; k++) {
                int nidx = nb[k];
                if (k == 0 && x == 0) continue;
                if (k == 1 && x == w - 1) continue;
                if (k == 2 && y == 0) continue;
                if (k == 3 && y == h - 1) continue;
                if (nidx < 0 || nidx >= n) continue;
                if (bin[nidx] && !visited[nidx]) {
                    visited[nidx] = 1;
                    stack[sp++] = nidx;
                }
            }
        }

        if (area >= minArea && area > best.area) {
            best.area = area; best.x0 = x0; best.y0 = y0; best.x1 = x1; best.y1 = y1;
            found = true;
        }
    }

    if (found) *out = best;
    return found;
}

// ============================================================
// GLOBAL BUFFERS (allocated once in PSRAM)
// ============================================================

uint8_t* workBuf;            // WORK_W x WORK_H downsampled brightness
uint8_t* workBin;            // WORK_W x WORK_H bright mask
int* workStack;
uint8_t* workVisited;

uint8_t* analysisGray;       // ANALYSIS_SIZE^2 cropped/resized grayscale
uint8_t* analysisBin;        // ANALYSIS_SIZE^2 ink mask (this frame)
uint8_t* tmpA;                // scratch, ANALYSIS_SIZE^2
uint8_t* tmpB;                // scratch, ANALYSIS_SIZE^2
int* stackA;                  // ANALYSIS_SIZE^2
uint8_t* visitedA;
uint8_t* outsideA;
uint8_t* holeBinA;
int32_t* labelA;

// Marker lock and label voting. We classify each frame first and
// vote the labels; this avoids smearing a rotating cylindrical block.
DiscInfo trackedDisc = {false, 0, 0, 0, 0};
bool trackerHasLock = false;
int trackerMisses = 0;

char voteLabels[VOTE_FRAMES] = {0};
int voteIndex = 0;
int voteCount = 0;

char lastPrinted = 0;
bool noVowelPrinted = false;

// Continuous confirmation behaviour:
// Keep checking forever.
// Send a confirmed vowel only when it is different from the last confirmed one.
// If the block disappears, lastConfirmedVowel is cleared so the same vowel
// can be detected again later.
char lastConfirmedVowel = 0;
int noMarkerStreak = 0;
int unknownStreak = 0;

/*
 * UART behaviour for the PSoC link:
 * - repeat a confirmed vowel every 500 ms while it remains stable
 * - only send 'N' after the marker/pattern has really been absent for 2000 ms
 */
#define PSOC_NO_VOWEL_DELAY_MS 2000UL

unsigned long lastPatternSeenMs = 0;

char lastStatus[24] = "";
float debugInkRatio = 0.0f;

// ============================================================
// CAMERA SETUP
// ============================================================

bool initializeCamera() {
    camera_config_t config = {};

    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer   = LEDC_TIMER_0;

    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;

    config.pin_xclk  = XCLK_GPIO_NUM;
    config.pin_pclk  = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href  = HREF_GPIO_NUM;

    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;

    config.pin_pwdn  = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;

    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_GRAYSCALE;

    if (!psramFound()) {
        Serial.println("ERROR: PSRAM is required for this detector.");
        return false;
    }

    Serial.println("PSRAM detected.");
    config.frame_size = FRAMESIZE_VGA;    // 640x480: more pixels on the custom vowel
    config.fb_count = 2;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = CAMERA_GRAB_LATEST;

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed: 0x%x\n", err);
        return false;
    }

    // OV3660 hardware image tuning. These are sensor controls, so they
    // improve separation before the pixels even reach our thresholding.
    sensor_t* s = esp_camera_sensor_get();
    if (s) {
        if (s->set_contrast)   s->set_contrast(s, SENSOR_CONTRAST);
        if (s->set_brightness) s->set_brightness(s, SENSOR_BRIGHTNESS);
        if (s->set_sharpness)  s->set_sharpness(s, SENSOR_SHARPNESS);
        if (s->set_denoise)    s->set_denoise(s, SENSOR_DENOISE);
        if (s->set_hmirror)    s->set_hmirror(s, CAMERA_HMIRROR);
        if (s->set_vflip)      s->set_vflip(s, CAMERA_VFLIP);

        // Keep automatic exposure/gain on. The software contrast
        // normalization below handles the remaining illumination change.
        if (s->set_exposure_ctrl) s->set_exposure_ctrl(s, 1);
        if (s->set_gain_ctrl)     s->set_gain_ctrl(s, 1);
        if (s->set_ae_level)      s->set_ae_level(s, SENSOR_AE_LEVEL);
    }

    Serial.printf("Camera tuning: VGA grayscale contrast=%d sharpness=%d denoise=%d\n",
                  SENSOR_CONTRAST, SENSOR_SHARPNESS, SENSOR_DENOISE);

    // Let auto exposure settle, then discard unstable warm-up frames.
    delay(400);
    for (int i = 0; i < 6; i++) {
        camera_fb_t* warm = esp_camera_fb_get();
        if (warm) esp_camera_fb_return(warm);
        delay(40);
    }

    return true;
}

// ============================================================
// SETUP
// ============================================================

void setup() {
    // USB serial monitor for Arduino IDE debug output
    Serial.begin(115200);

    // UART to PSoC 5LP:
    // ESP32 TX GPIO43/D6 -> PSoC P15[5]/Rx_2
    Serial1.begin(PSOC_UART_BAUD, SERIAL_8N1,
                  PSOC_UART_RX_PIN, PSOC_UART_TX_PIN);

    delay(300);

    Serial.println();
    Serial.println("XIAO ESP32S3 custom-vowel tracker v4");
    Serial.println("No PC or Python needed -- results print right here.");
    Serial.println();

    if (!initializeCamera()) {
        Serial.println("Camera failed to start. Check wiring and reset the board.");
        while (true) delay(1000);
    }

    Serial.println("Camera ready.");
    lastPatternSeenMs = millis();

    Serial.println("Hold the custom white vowel block in front of the camera,");
    Serial.println("centered and filling most of the frame.");
    Serial.println();

    // PSRAM allocations
    workBuf     = (uint8_t*)ps_malloc(WORK_W * WORK_H);
    workBin     = (uint8_t*)ps_malloc(WORK_W * WORK_H);
    workStack   = (int*)ps_malloc(WORK_W * WORK_H * sizeof(int));
    workVisited = (uint8_t*)ps_malloc(WORK_W * WORK_H);

    analysisGray = (uint8_t*)ps_malloc(ANALYSIS_N);
    analysisBin  = (uint8_t*)ps_malloc(ANALYSIS_N);
    tmpA         = (uint8_t*)ps_malloc(ANALYSIS_N);
    tmpB         = (uint8_t*)ps_malloc(ANALYSIS_N);
    stackA       = (int*)ps_malloc(ANALYSIS_N * sizeof(int));
    visitedA     = (uint8_t*)ps_malloc(ANALYSIS_N);
    outsideA     = (uint8_t*)ps_malloc(ANALYSIS_N);
    holeBinA     = (uint8_t*)ps_malloc(ANALYSIS_N);
    labelA       = (int32_t*)ps_malloc(ANALYSIS_N * sizeof(int32_t));

    bool allocOK =
        workBuf && workBin && workStack && workVisited &&
        analysisGray && analysisBin && tmpA && tmpB &&
        stackA && visitedA && outsideA && holeBinA && labelA;

    if (!allocOK) {
        Serial.println("ERROR: PSRAM allocation failed.");
        while (true) delay(1000);
    }
}

// ============================================================
// MARKER (WHITE DISC) DETECTION
// ============================================================

DiscInfo detectDisc(camera_fb_t* fb) {
    DiscInfo info; info.found = false;

    for (int y = 0; y < WORK_H; y++) {
        for (int x = 0; x < WORK_W; x++) {
            int fx = x * WORK_SCALE_X;
            int fy = y * WORK_SCALE_Y;
            uint8_t v = fb->buf[fy * FRAME_WIDTH + fx];
            workBuf[y * WORK_W + x] = v;
        }
    }

    // Instead of assuming "white" is always >= 150, estimate the
    // scene threshold every frame and bias it toward the bright side.
    int whiteThreshold = otsuThreshold(workBuf, WORK_W * WORK_H) + WHITE_THRESHOLD_OFFSET;
    if (whiteThreshold < WHITE_THRESHOLD_MIN) whiteThreshold = WHITE_THRESHOLD_MIN;
    if (whiteThreshold > WHITE_THRESHOLD_MAX) whiteThreshold = WHITE_THRESHOLD_MAX;

    for (int i = 0; i < WORK_W * WORK_H; i++) {
        workBin[i] = (workBuf[i] >= whiteThreshold) ? 1 : 0;
    }

    Blob blob;
    bool ok = findLargestBlob(workBin, WORK_W, WORK_H, MIN_BLOB_AREA_RATIO,
                               &blob, workStack, workVisited);

    if (!ok) return info;

    info.found = true;
    info.x0 = blob.x0 * WORK_SCALE_X;
    info.y0 = blob.y0 * WORK_SCALE_Y;
    info.x1 = (blob.x1 + 1) * WORK_SCALE_X;
    info.y1 = (blob.y1 + 1) * WORK_SCALE_Y;

    return info;
}

// Smooth/hold the marker bounding box so one noisy frame does not make
// the crop jump or disappear. Sudden implausible jumps are rejected.
DiscInfo stabilizeDisc(DiscInfo candidate) {
    if (!candidate.found) {
        if (trackerHasLock && trackerMisses < TRACK_HOLD_FRAMES) {
            trackerMisses++;
            return trackedDisc;
        }

        trackerHasLock = false;
        trackerMisses = 0;
        DiscInfo none = {false, 0, 0, 0, 0};
        return none;
    }

    if (!trackerHasLock) {
        trackedDisc = candidate;
        trackerHasLock = true;
        trackerMisses = 0;
        return trackedDisc;
    }

    float oldCx = (trackedDisc.x0 + trackedDisc.x1) * 0.5f;
    float oldCy = (trackedDisc.y0 + trackedDisc.y1) * 0.5f;
    float newCx = (candidate.x0 + candidate.x1) * 0.5f;
    float newCy = (candidate.y0 + candidate.y1) * 0.5f;

    float dx = (newCx - oldCx) / FRAME_WIDTH;
    float dy = (newCy - oldCy) / FRAME_HEIGHT;
    float centerJump = sqrtf(dx * dx + dy * dy);

    float oldW = (float)(trackedDisc.x1 - trackedDisc.x0);
    float newW = (float)(candidate.x1 - candidate.x0);
    float sizeRatio = (oldW > 1.0f && newW > 1.0f)
                    ? fmaxf(oldW, newW) / fminf(oldW, newW)
                    : 99.0f;

    if (centerJump > TRACK_MAX_CENTER_JUMP ||
        sizeRatio > TRACK_MAX_SIZE_RATIO) {
        if (trackerMisses < TRACK_HOLD_FRAMES) {
            trackerMisses++;
            return trackedDisc;
        }
        trackedDisc = candidate;
        trackerMisses = 0;
        return trackedDisc;
    }

    trackerMisses = 0;
    float a = TRACK_SMOOTH_ALPHA;

    trackedDisc.x0 = (int)lroundf((1.0f - a) * trackedDisc.x0 + a * candidate.x0);
    trackedDisc.y0 = (int)lroundf((1.0f - a) * trackedDisc.y0 + a * candidate.y0);
    trackedDisc.x1 = (int)lroundf((1.0f - a) * trackedDisc.x1 + a * candidate.x1);
    trackedDisc.y1 = (int)lroundf((1.0f - a) * trackedDisc.y1 + a * candidate.y1);
    trackedDisc.found = true;

    return trackedDisc;
}

// Returns true if ready, and fills statusOut with a short message.
bool positionStatus(DiscInfo info, char* statusOut) {
    if (!info.found) { strcpy(statusOut, "NO MARKER"); return false; }

    int markerW = info.x1 - info.x0;
    float markerCx = (info.x0 + info.x1) / 2.0f;
    float markerCy = (info.y0 + info.y1) / 2.0f;

    float targetCx = FRAME_WIDTH / 2.0f;
    float targetCy = FRAME_HEIGHT / 2.0f;

    float markerRatio = (float)markerW / FRAME_WIDTH;

    if (markerRatio < MIN_MARKER_WIDTH_RATIO) { strcpy(statusOut, "MOVE CLOSER"); return false; }
    if (markerRatio > MAX_MARKER_WIDTH_RATIO) { strcpy(statusOut, "MOVE FURTHER"); return false; }

    float dx = markerCx - targetCx;
    float dy = markerCy - targetCy;

    if (fabsf(dx) > FRAME_WIDTH * CENTER_TOLERANCE_X) {
        strcpy(statusOut, dx < 0 ? "MOVE RIGHT" : "MOVE LEFT");
        return false;
    }
    if (fabsf(dy) > FRAME_HEIGHT * CENTER_TOLERANCE_Y) {
        strcpy(statusOut, dy < 0 ? "MOVE DOWN" : "MOVE UP");
        return false;
    }

    strcpy(statusOut, "READY");
    return true;
}

// ============================================================
// EXTRACT + NORMALIZE THE VOWEL PATTERN FROM ONE FRAME
// ============================================================

// Remove black regions that touch the crop border. In fallback mode this
// gets rid of dark table/background areas while keeping the printed vowel,
// which should be surrounded by the white block face.
static void removeBorderConnectedInk(uint8_t* bin, int w, int h) {
    int n = w * h;
    memset(visitedA, 0, n);
    int sp = 0;

    for (int x = 0; x < w; x++) {
        int a = x;
        int b = (h - 1) * w + x;
        if (bin[a] && !visitedA[a]) { visitedA[a] = 1; stackA[sp++] = a; }
        if (bin[b] && !visitedA[b]) { visitedA[b] = 1; stackA[sp++] = b; }
    }
    for (int y = 0; y < h; y++) {
        int a = y * w;
        int b = y * w + (w - 1);
        if (bin[a] && !visitedA[a]) { visitedA[a] = 1; stackA[sp++] = a; }
        if (bin[b] && !visitedA[b]) { visitedA[b] = 1; stackA[sp++] = b; }
    }

    while (sp > 0) {
        int idx = stackA[--sp];
        int x = idx % w;
        int y = idx / w;
        bin[idx] = 0;

        const int nb[4] = {idx - 1, idx + 1, idx - w, idx + w};
        for (int k = 0; k < 4; k++) {
            int ni = nb[k];
            if (k == 0 && x == 0) continue;
            if (k == 1 && x == w - 1) continue;
            if (k == 2 && y == 0) continue;
            if (k == 3 && y == h - 1) continue;
            if (ni < 0 || ni >= n) continue;
            if (bin[ni] && !visitedA[ni]) {
                visitedA[ni] = 1;
                stackA[sp++] = ni;
            }
        }
    }
}

// Scale the detected ink itself to a fixed-size centred canvas while
// preserving its aspect ratio. Recognition therefore depends much less on
// camera distance and on how large the white face appears in the frame.
static bool normalizeInkToCanvas(uint8_t* bin) {
    int minX = ANALYSIS_SIZE, minY = ANALYSIS_SIZE;
    int maxX = -1, maxY = -1;
    int ink = 0;

    for (int y = 0; y < ANALYSIS_SIZE; y++) {
        for (int x = 0; x < ANALYSIS_SIZE; x++) {
            if (!bin[y * ANALYSIS_SIZE + x]) continue;
            ink++;
            if (x < minX) minX = x;
            if (x > maxX) maxX = x;
            if (y < minY) minY = y;
            if (y > maxY) maxY = y;
        }
    }

    if (ink < 20 || maxX < minX || maxY < minY) return false;

    int bw = maxX - minX + 1;
    int bh = maxY - minY + 1;
    if (bw < 3 || bh < 3) return false;

    // A symbol covering almost the whole analysis image is normally a
    // background object rather than the printed vowel.
    if (bw > ANALYSIS_SIZE * 0.92f && bh > ANALYSIS_SIZE * 0.92f) return false;

    memcpy(tmpA, bin, ANALYSIS_N);
    memset(bin, 0, ANALYSIS_N);

    float scale = (float)NORMALIZED_SYMBOL_MAX / (float)((bw > bh) ? bw : bh);
    int dw = (int)lroundf(bw * scale);
    int dh = (int)lroundf(bh * scale);
    if (dw < 1) dw = 1;
    if (dh < 1) dh = 1;
    if (dw > NORMALIZED_SYMBOL_MAX) dw = NORMALIZED_SYMBOL_MAX;
    if (dh > NORMALIZED_SYMBOL_MAX) dh = NORMALIZED_SYMBOL_MAX;

    int dx0 = (ANALYSIS_SIZE - dw) / 2;
    int dy0 = (ANALYSIS_SIZE - dh) / 2;

    for (int dy = 0; dy < dh; dy++) {
        int sy = minY + (dy * bh) / dh;
        if (sy > maxY) sy = maxY;
        for (int dx = 0; dx < dw; dx++) {
            int sx = minX + (dx * bw) / dw;
            if (sx > maxX) sx = maxX;
            bin[(dy0 + dy) * ANALYSIS_SIZE + (dx0 + dx)] =
                tmpA[sy * ANALYSIS_SIZE + sx];
        }
    }

    return true;
}

static bool processSquareCrop(camera_fb_t* fb, int x0, int y0, int side,
                              bool removeBorderNoise) {
    if (side < 24) return false;
    if (x0 < 0 || y0 < 0 ||
        x0 + side > (int)fb->width || y0 + side > (int)fb->height) {
        return false;
    }

    for (int y = 0; y < ANALYSIS_SIZE; y++) {
        int sy = y0 + (y * side) / ANALYSIS_SIZE;
        for (int x = 0; x < ANALYSIS_SIZE; x++) {
            int sx = x0 + (x * side) / ANALYSIS_SIZE;
            analysisGray[y * ANALYSIS_SIZE + x] =
                fb->buf[sy * fb->width + sx];
        }
    }

    int lowPercent, highPercent, otsuBias;
    pickExposureTier(analysisGray, ANALYSIS_N, &lowPercent, &highPercent, &otsuBias);

    percentileContrastStretch(analysisGray, ANALYSIS_N, lowPercent, highPercent);

    int t = otsuThreshold(analysisGray, ANALYSIS_N) + otsuBias;
    if (t < 35) t = 35;
    if (t > 225) t = 225;
    debugBinaryThreshold = t;

    for (int i = 0; i < ANALYSIS_N; i++) {
        analysisBin[i] = (analysisGray[i] < t) ? 1 : 0;
    }

    // The centre-crop fallback may contain dark background around a white
    // cylindrical face. Remove any dark region connected to the crop edge.
    if (removeBorderNoise) {
        removeBorderConnectedInk(analysisBin, ANALYSIS_SIZE, ANALYSIS_SIZE);
    }

    removeSmallComponents(analysisBin,
                          ANALYSIS_SIZE,
                          ANALYSIS_SIZE,
                          MIN_BLACK_COMPONENT_RATIO,
                          labelA,
                          stackA);

    if (!normalizeInkToCanvas(analysisBin)) return false;

    // Run tiny-component rejection again after scale normalization.
    removeSmallComponents(analysisBin,
                          ANALYSIS_SIZE,
                          ANALYSIS_SIZE,
                          MIN_BLACK_COMPONENT_RATIO,
                          labelA,
                          stackA);

    int ink = 0;
    for (int i = 0; i < ANALYSIS_N; i++) {
        if (analysisBin[i]) ink++;
    }

    debugInkRatio = (float)ink / (float)ANALYSIS_N;
    return debugInkRatio >= MIN_INK_RATIO && debugInkRatio <= MAX_INK_RATIO;
}

// Primary extraction: use the tracked bright face if it is sane.
bool extractPattern(camera_fb_t* fb, DiscInfo info) {
    if (!info.found) return false;

    int markerW = info.x1 - info.x0;
    int markerH = info.y1 - info.y0;
    if (markerW <= 0 || markerH <= 0) return false;

    float markerCx = (info.x0 + info.x1) * 0.5f;
    float markerCy = (info.y0 + info.y1) * 0.5f;
    int side = (markerW > markerH) ? markerW : markerH;

    // Reject a "white blob" that is basically the whole scene; that is
    // usually a white wall/table, not the physical vowel face.
    if (side > (int)(fminf((float)fb->width, (float)fb->height) * 0.96f)) {
        return false;
    }

    side = (int)lroundf(side * 1.12f);
    int x0 = (int)lroundf(markerCx - side * 0.5f);
    int y0 = (int)lroundf(markerCy - side * 0.5f);

    // Clamp instead of rejecting a slightly off-centre marker.
    if (side > (int)fb->width) side = fb->width;
    if (side > (int)fb->height) side = fb->height;
    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (x0 + side > (int)fb->width) x0 = fb->width - side;
    if (y0 + side > (int)fb->height) y0 = fb->height - side;

    return processSquareCrop(fb, x0, y0, side, true);
}

// Fallback extraction: do NOT require the white face tracker. It analyses a
// large central area, removes dark background connected to its border, then
// automatically finds and enlarges the isolated black vowel. This is the
// main V4 change that makes the camera much easier to use.
bool extractFallbackPattern(camera_fb_t* fb) {
    int minDim = ((int)fb->width < (int)fb->height) ? fb->width : fb->height;
    int side = (int)lroundf(minDim * FALLBACK_CROP_RATIO);
    int x0 = ((int)fb->width - side) / 2;
    int y0 = ((int)fb->height - side) / 2;
    return processSquareCrop(fb, x0, y0, side, true);
}

// ============================================================
// PER-FRAME LABEL VOTING
// ============================================================

static void clearVotes() {
    memset(voteLabels, 0, sizeof(voteLabels));
    voteIndex = 0;
    voteCount = 0;
}

static void pushVote(char label) {
    // label may be 0 for an unclear/unknown classification.
    // It still counts as one of the 5 checks, but gives no vowel a vote.
    voteLabels[voteIndex] = label;
    voteIndex = (voteIndex + 1) % VOTE_FRAMES;

    if (voteCount < VOTE_FRAMES) voteCount++;
}

static char getVotedLabel() {
    // Do not make any final decision until all 5 checks are available.
    if (voteCount < VOTE_FRAMES) return 0;

    const char letters[5] = {'A', 'E', 'I', 'O', 'U'};
    int counts[5] = {0, 0, 0, 0, 0};

    for (int i = 0; i < voteCount; i++) {
        for (int j = 0; j < 5; j++) {
            if (voteLabels[i] == letters[j]) {
                counts[j]++;
                break;
            }
        }
    }

    int best = -1;
    int bestCount = 0;

    for (int j = 0; j < 5; j++) {
        if (counts[j] > bestCount) {
            bestCount = counts[j];
            best = j;
        }
    }

    if (best >= 0 && bestCount >= VOTE_REQUIRED) {
        return letters[best];
    }

    return 0;
}

// ============================================================
// CUSTOM VOWEL SHAPE DESCRIPTORS
// ============================================================

static const float REF_A[RADIAL_BINS] = {
    0.000000f, 0.000000f, 0.000000f, 0.000000f,
    0.000000f, 0.000000f, 0.037418f, 0.141254f,
    0.229186f, 0.255379f, 0.197381f, 0.139383f
};

static const float REF_E[RADIAL_BINS] = {
    0.010980f, 0.026182f, 0.032095f, 0.026182f,
    0.029561f, 0.027027f, 0.024493f, 0.025338f,
    0.098818f, 0.221284f, 0.273649f, 0.204392f
};

static const float REF_O[RADIAL_BINS] = {
    0.000000f, 0.000000f, 0.000000f, 0.000000f,
    0.000000f, 0.000000f, 0.000000f, 0.000000f,
    0.073350f, 0.283619f, 0.343521f, 0.299511f
};

static float computeRadialCV(uint8_t* bin) {
    long count = 0;
    double sx = 0.0;
    double sy = 0.0;

    for (int y = 0; y < ANALYSIS_SIZE; y++) {
        for (int x = 0; x < ANALYSIS_SIZE; x++) {
            if (!bin[y * ANALYSIS_SIZE + x]) continue;
            count++;
            sx += x;
            sy += y;
        }
    }

    if (count < 10) return 99.0f;

    float cx = (float)(sx / count);
    float cy = (float)(sy / count);

    double sr = 0.0;
    double sr2 = 0.0;

    for (int y = 0; y < ANALYSIS_SIZE; y++) {
        for (int x = 0; x < ANALYSIS_SIZE; x++) {
            if (!bin[y * ANALYSIS_SIZE + x]) continue;
            float dx = x - cx;
            float dy = y - cy;
            float r = sqrtf(dx * dx + dy * dy);
            sr += r;
            sr2 += r * r;
        }
    }

    float mean = (float)(sr / count);
    float var = (float)(sr2 / count) - mean * mean;
    if (var < 0.0f) var = 0.0f;

    return mean > 0.001f ? sqrtf(var) / mean : 99.0f;
}

static void computeRadialSignature(uint8_t* bin,
                                   float out[RADIAL_BINS]) {
    for (int i = 0; i < RADIAL_BINS; i++) out[i] = 0.0f;

    long count = 0;
    double sx = 0.0;
    double sy = 0.0;

    for (int y = 0; y < ANALYSIS_SIZE; y++) {
        for (int x = 0; x < ANALYSIS_SIZE; x++) {
            if (!bin[y * ANALYSIS_SIZE + x]) continue;
            count++;
            sx += x;
            sy += y;
        }
    }

    if (count <= 0) return;

    float cx = (float)(sx / count);
    float cy = (float)(sy / count);

    int radiusHist[RADIAL_MAX_RADIUS_BIN] = {0};

    for (int y = 0; y < ANALYSIS_SIZE; y++) {
        for (int x = 0; x < ANALYSIS_SIZE; x++) {
            if (!bin[y * ANALYSIS_SIZE + x]) continue;

            float dx = x - cx;
            float dy = y - cy;
            int ri = (int)lroundf(sqrtf(dx * dx + dy * dy));

            if (ri < 0) ri = 0;
            if (ri >= RADIAL_MAX_RADIUS_BIN) {
                ri = RADIAL_MAX_RADIUS_BIN - 1;
            }
            radiusHist[ri]++;
        }
    }

    long target = (long)ceilf(count * 0.98f);
    long cumulative = 0;
    int r98 = 2;

    for (int r = 0; r < RADIAL_MAX_RADIUS_BIN; r++) {
        cumulative += radiusHist[r];
        if (cumulative >= target) {
            r98 = r;
            break;
        }
    }

    if (r98 < 2) r98 = 2;

    int binCounts[RADIAL_BINS] = {0};

    for (int y = 0; y < ANALYSIS_SIZE; y++) {
        for (int x = 0; x < ANALYSIS_SIZE; x++) {
            if (!bin[y * ANALYSIS_SIZE + x]) continue;

            float dx = x - cx;
            float dy = y - cy;
            float r = sqrtf(dx * dx + dy * dy);
            float normalized = r / (float)r98;

            int b = (normalized >= 1.0f)
                  ? RADIAL_BINS - 1
                  : (int)(normalized * RADIAL_BINS);

            if (b < 0) b = 0;
            if (b >= RADIAL_BINS) b = RADIAL_BINS - 1;
            binCounts[b]++;
        }
    }

    for (int i = 0; i < RADIAL_BINS; i++) {
        out[i] = (float)binCounts[i] / (float)count;
    }
}

static float signatureDistance(const float a[RADIAL_BINS],
                               const float b[RADIAL_BINS]) {
    float d = 0.0f;
    for (int i = 0; i < RADIAL_BINS; i++) {
        d += fabsf(a[i] - b[i]);
    }
    return d;
}

float debugRadialCV = 99.0f;
float debugDA = 99.0f;
float debugDE = 99.0f;
float debugDO = 99.0f;
int debugRawComponents = 0;
int debugHoles = 0;

static char classifyAEO(uint8_t* bin) {
    float sig[RADIAL_BINS];
    computeRadialSignature(bin, sig);

    debugDA = signatureDistance(sig, REF_A);
    debugDE = signatureDistance(sig, REF_E);
    debugDO = signatureDistance(sig, REF_O);

    char bestLabel = 'A';
    float best = debugDA;
    float second = 99.0f;

    if (debugDE < best) {
        second = best;
        best = debugDE;
        bestLabel = 'E';
    } else {
        second = debugDE;
    }

    if (debugDO < best) {
        second = best;
        best = debugDO;
        bestLabel = 'O';
    } else if (debugDO < second) {
        second = debugDO;
    }

    if (best > AEO_MATCH_MAX) return 0;
    if ((second - best) < AEO_MATCH_MIN_MARGIN) return 0;

    return bestLabel;
}

char classifyFrame(uint8_t* rawBin) {
    debugRadialCV = 99.0f;
    debugDA = debugDE = debugDO = 99.0f;
    debugRawComponents = 0;
    debugHoles = 0;

    Comp rawComps[16];
    int rawNC = labelComponents(rawBin,
                                ANALYSIS_SIZE,
                                ANALYSIS_SIZE,
                                rawComps,
                                16,
                                MIN_BLACK_COMPONENT_RATIO,
                                stackA,
                                visitedA);

    debugRawComponents = rawNC;

    // I: actual custom symbol has a separate dot and stem.
    if (rawNC == 2 && rawComps[0].area > 0) {
        float ratio =
            (float)rawComps[1].area / (float)rawComps[0].area;

        float radial = computeRadialCV(rawBin);
        debugRadialCV = radial;

        if (ratio >= I_SECOND_RATIO_MIN &&
            ratio <= I_SECOND_RATIO_MAX &&
            radial > 0.26f) {
            return 'I';
        }
    }

    // Repair tiny camera/focus gaps, and small specular-highlight gaps
    // (added lighting can blow out a patch of the black ink to white),
    // only in a temporary copy. Two closing passes bridge gaps up to
    // ~2px; a real letter hole (O/A/E) is far larger and survives this
    // untouched.
    // ponytail: closing only fixes small/medium glare specks. A wide
    // diffuse glare patch still needs the light physically diffused or
    // angled off-axis, or the exposure tiers above widened further.
    memcpy(tmpB, rawBin, ANALYSIS_N);
    for (int pass = 0; pass < 2; pass++) {
        dilate3x3(tmpB, tmpA, ANALYSIS_SIZE, ANALYSIS_SIZE);
        erode3x3(tmpA, tmpB, ANALYSIS_SIZE, ANALYSIS_SIZE);
    }

    removeSmallComponents(tmpB,
                          ANALYSIS_SIZE,
                          ANALYSIS_SIZE,
                          MIN_BLACK_COMPONENT_RATIO,
                          labelA,
                          stackA);

    Comp closedComps[16];
    int closedNC = labelComponents(tmpB,
                                   ANALYSIS_SIZE,
                                   ANALYSIS_SIZE,
                                   closedComps,
                                   16,
                                   MIN_BLACK_COMPONENT_RATIO,
                                   stackA,
                                   visitedA);

    Hole holes[16];
    int nh = findHoles(tmpB,
                       ANALYSIS_SIZE,
                       ANALYSIS_SIZE,
                       holes,
                       16,
                       MIN_HOLE_RATIO,
                       stackA,
                       outsideA,
                       holeBinA,
                       labelA);

    debugHoles = nh;
    debugRadialCV = computeRadialCV(tmpB);

    // U: one open curved stroke. Require its measured radial band so
    // a random dark blob is not automatically called U.
    if (closedNC == 1 && nh == 0) {
        if (debugRadialCV >= U_RADIAL_MIN &&
            debugRadialCV <= U_RADIAL_MAX) {
            return 'U';
        }
        return 0;
    }

    // A/E/O: closed custom shapes, compared against the actual supplied
    // radial signatures. Poor matches and near ties become UNKNOWN.
    if (closedNC == 1 && nh >= 1) {
        return classifyAEO(tmpB);
    }

    return 0;
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop() {
    camera_fb_t* fb = esp_camera_fb_get();

    if (!fb) {
        Serial.println("Camera capture failed.");
        delay(30);
        return;
    }

    if (fb->format != PIXFORMAT_GRAYSCALE) {
        Serial.println("Unexpected pixel format from camera.");
        esp_camera_fb_return(fb);
        delay(30);
        return;
    }

    // Primary white-face tracking is still useful when it works, but V4 no
    // longer depends on it. If the white face is lost/merged with a bright
    // background, we immediately fall back to direct black-vowel tracking.
    DiscInfo candidate = detectDisc(fb);
    DiscInfo disc = stabilizeDisc(candidate);

    char status[24];
    bool ready = positionStatus(disc, status);

    bool patternOK = false;
    bool usedFallback = false;

    if (disc.found) {
        // Try the face crop regardless of READY; READY is guidance only now.
        patternOK = extractPattern(fb, disc);
    }

    if (!patternOK) {
        patternOK = extractFallbackPattern(fb);
        usedFallback = patternOK;
    }


    // No isolated dark symbol exists in either tracking path.
    // Ignore short tracking drop-outs and only send 'N' after 2 seconds.
    if (!patternOK) {
        noMarkerStreak++;
        unknownStreak = 0;
        clearVotes();

        unsigned long now = millis();

        if (!noVowelPrinted &&
            (now - lastPatternSeenMs >= PSOC_NO_VOWEL_DELAY_MS)) {

            Serial.println("No vowels detected");
            Serial1.write('N');

            lastConfirmedVowel = 0;
            lastPrinted = 0;
            noVowelPrinted = true;
            clearVotes();
        }

        esp_camera_fb_return(fb);
        delay(20);
        return;
    }

    // A usable marker/pattern is present.
    lastPatternSeenMs = millis();
    noMarkerStreak = 0;
    noVowelPrinted = false;

    char detected = classifyFrame(analysisBin);

    if (SHOW_DEBUG) {
        static unsigned long lastDebug = 0;
        if (millis() - lastDebug > 350) {
            Serial.printf(
                "  [live] source=%s guess=%c ink=%.3f rawComp=%d holes=%d radial=%.3f "
                "A=%.3f E=%.3f O=%.3f stretch=%d..%d thr=%d votes=%d/%d\n",
                usedFallback ? "VOWEL" : "FACE",
                detected ? detected : '?',
                debugInkRatio,
                debugRawComponents,
                debugHoles,
                debugRadialCV,
                debugDA,
                debugDE,
                debugDO,
                debugStretchLow,
                debugStretchHigh,
                debugBinaryThreshold,
                voteCount,
                VOTE_FRAMES
            );
            lastDebug = millis();
        }
    }

    // Every classification attempt counts as one check.
    // A valid vowel contributes a vote; an unclear result contributes 0.
    if (detected == 0) {
        unknownStreak++;
    } else {
        unknownStreak = 0;
    }

    pushVote(detected);

    // getVotedLabel() only returns a letter after 5 checks have been
    // collected AND at least 4 of those 5 checks agree on the same vowel.
    char finalLabel = getVotedLabel();

    if (finalLabel != 0) {
        /*
         * Keep checking continuously, but only print/send ONCE
         * when a new vowel has been finally confirmed.
         */
        if (finalLabel != lastConfirmedVowel) {
            Serial.print("Final detected vowel: ");
            Serial.println(finalLabel);

            // Send exactly one ASCII byte to the PSoC.
            Serial1.write(finalLabel);

            lastConfirmedVowel = finalLabel;
            lastPrinted = finalLabel;
        }
    }

    esp_camera_fb_return(fb);
    delay(20);
}

