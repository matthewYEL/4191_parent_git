#include "esp_camera.h"
#include <string.h>
#include <math.h>
#include <NimBLEDevice.h>

#define DEBUG_ENABLE 1

#if DEBUG_ENABLE
#define DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
#define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
#define DEBUG_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
#define DEBUG_PRINT(...)
#define DEBUG_PRINTLN(...)
#define DEBUG_PRINTF(...)
#endif

// ---- 新增: 失败调试开关 ----
#define FAIL_DEBUG 1            // 预处理失败时打印原因
#define RAW_DUMP 1              // 墨迹过多时打印"删边缘之前"的二值图和灰度图
#define RAW_DUMP_INK_MIN 6000   // 墨迹超过这个数才打印
#define RAW_DUMP_INTERVAL_MS 2000UL

// ------------------------------------------------------------ //
// CONFIGURATION & CONSTANTS
// ------------------------------------------------------------ //
#define SERVICE_UUID "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define CHARACTERISTIC_UUID_RX "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
#define CHARACTERISTIC_UUID_TX "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
#define BLE_DEVICE_NAME "Turtleeeee"

#define PSOC_UART_RX_PIN 44
#define PSOC_UART_TX_PIN 43
#define PSOC_UART_BAUD 9600

#define FRAME_WIDTH 640
#define FRAME_HEIGHT 480

// Downsampled work grid for marker detection
#define WORK_W 160
#define WORK_H 120
#define WORK_SCALE_X (FRAME_WIDTH / WORK_W)   // 4
#define WORK_SCALE_Y (FRAME_HEIGHT / WORK_H)  // 4

// Normalized symbol classification grid
#define ANALYSIS_SIZE 128
#define ANALYSIS_N (ANALYSIS_SIZE * ANALYSIS_SIZE)

#define WHITE_THRESHOLD_MIN 95
#define WHITE_THRESHOLD_MAX 225
#define WHITE_THRESHOLD_OFFSET 0
#define MIN_BLOB_AREA_RATIO 0.018f

#define MIN_MARKER_WIDTH_RATIO 0.10f
#define MAX_MARKER_WIDTH_RATIO 0.95f
#define CENTER_TOLERANCE_X 0.38f
#define CENTER_TOLERANCE_Y 0.38f

#define MIN_BLACK_COMPONENT_RATIO 0.0012f
#define MIN_HOLE_RATIO 0.0020f
#define MIN_INK_RATIO 0.0030f
#define MAX_INK_RATIO 0.55f

#define BRIGHT_LIGHT_MODE true

// Custom Vowel Geometry Constants
#define I_SECOND_RATIO_MIN 0.030f
#define I_SECOND_RATIO_MAX 0.60f
#define U_RADIAL_MIN 0.100f
#define U_RADIAL_MAX 0.360f
#define RADIAL_BINS 12
#define RADIAL_MAX_RADIUS_BIN 192
#define AEO_MATCH_MAX 0.85f
#define AEO_MATCH_MIN_MARGIN 0.025f

#define TRACK_HOLD_FRAMES 5
#define TRACK_SMOOTH_ALPHA 0.35f
#define TRACK_MAX_CENTER_JUMP 0.24f
#define TRACK_MAX_SIZE_RATIO 1.75f

#define NO_VOWEL_FRAMES 18

#define SENSOR_CONTRAST 3
#define SENSOR_SHARPNESS 1
#if BRIGHT_LIGHT_MODE
#define SENSOR_BRIGHTNESS -1
#define SENSOR_AE_LEVEL -1
#else
#define SENSOR_BRIGHTNESS 0
#define SENSOR_AE_LEVEL 0
#endif
#define SENSOR_DENOISE 1
#define CAMERA_HMIRROR 0
#define CAMERA_VFLIP 0

#define SHOW_DEBUG false
#define FALLBACK_CROP_RATIO 0.60f
#define NORMALIZED_SYMBOL_MAX 100
#define UNCLEAR_FRAMES 12
#define PSOC_NO_VOWEL_DELAY_MS 2000UL

// ------------------------------------------------------------ //
// CAMERA PIN MAPPING (XIAO ESP32-S3 Sense)
// ------------------------------------------------------------ //
#define PWDN_GPIO_NUM -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 10
#define SIOD_GPIO_NUM 40
#define SIOC_GPIO_NUM 39
#define Y9_GPIO_NUM 48
#define Y8_GPIO_NUM 11
#define Y7_GPIO_NUM 12
#define Y6_GPIO_NUM 14
#define Y5_GPIO_NUM 16
#define Y4_GPIO_NUM 18
#define Y3_GPIO_NUM 17
#define Y2_GPIO_NUM 15
#define VSYNC_GPIO_NUM 38
#define HREF_GPIO_NUM 47
#define PCLK_GPIO_NUM 13

// (已删除前面重复的 VOTE_FRAMES 5 / VOTE_REQUIRED 4)
#define VOTE_FRAMES 10
#define VOTE_REQUIRED 8

// ------------------------------------------------------------ //
// STRUCTS & TYPES
// ------------------------------------------------------------ //
struct Comp {
  int area;
  float cx, cy;
  int x0, y0, x1, y1;
};
struct Hole {
  int area;
  int cx, cy;
};
struct Blob {
  int area;
  int x0, y0, x1, y1;
};
struct DiscInfo {
  bool found;
  int x0, y0, x1, y1;
};

struct ExposureTier {
  float meanBrightnessAbove;
  int lowPercent;
  int highPercent;
  int otsuBias;
};

static const ExposureTier EXPOSURE_TIERS[] = {
  { 175.0f, 10, 90, 18 },
  { 140.0f, 8, 92, 14 },
  { 0.0f, 3, 97, 6 }
};

// ------------------------------------------------------------ //
// GLOBAL VARIABLES & BUFFERS
// ------------------------------------------------------------ //
NimBLEServer* pServer = NULL;
NimBLECharacteristic* pTxCharacteristic = NULL;
NimBLECharacteristic* pRxCharacteristic = NULL;
bool deviceConnected = false;
String logoBufferFromPSoC = "";

uint8_t* workBuf;
uint8_t* workBin;
int* workStack;
uint8_t* workVisited;

uint8_t* analysisGray;
uint8_t* analysisBin;
uint8_t* tmpA;
uint8_t* tmpB;
int* stackA;
uint8_t* visitedA;
uint8_t* outsideA;
uint8_t* holeBinA;
int32_t* labelA;

DiscInfo trackedDisc = { false, 0, 0, 0, 0 };
bool trackerHasLock = false;
int trackerMisses = 0;
char lastPrinted = 0;
bool noVowelPrinted = false;

char lastConfirmedVowel = 0;
int noMarkerStreak = 0;
int unknownStreak = 0;
unsigned long lastPatternSeenMs = 0;
char lastStatus[24] = "";
float debugInkRatio = 0.0f;

int debugStretchLow = 0;
int debugStretchHigh = 255;
int debugBinaryThreshold = 0;

// ---- 新增: 失败调试变量 (必须放在全局, 且在 processSquareCrop 之前) ----
static const char* debugFailReason = "";
static const char* debugSrc = "";
static int debugInkBeforeBorder = 0;
static int debugInkAfterBorder = 0;

static int countInk(const uint8_t* b) {
  int c = 0;
  for (int i = 0; i < ANALYSIS_N; i++)
    if (b[i]) c++;
  return c;
}

static void dumpBin(const uint8_t* b) {
  Serial.println("---- BIN DUMP ----");
  for (int y = 0; y < ANALYSIS_SIZE; y += 2) {
    for (int x = 0; x < ANALYSIS_SIZE; x += 2) {
      Serial.print(b[y * ANALYSIS_SIZE + x] ? '#' : '.');
    }
    Serial.println();
  }
}

static void dumpGray(const uint8_t* g) {
  const char* ramp = " .:-=+*#%@";  // 越暗字符越"重"
  Serial.println("---- GRAY DUMP (stretched) ----");
  for (int y = 0; y < ANALYSIS_SIZE; y += 4) {
    for (int x = 0; x < ANALYSIS_SIZE; x += 2) {
      int v = g[y * ANALYSIS_SIZE + x];
      Serial.print(ramp[(255 - v) * 9 / 255]);
    }
    Serial.println();
  }
}

static int voteIndex = 0;
static int voteCount = 0;
static char voteLabels[VOTE_FRAMES];

// 前向声明 (getVotedLabel 里用到 clearVotes)
static void clearVotes();

// ------------------------------------------------------------ //
// BLE CALLBACKS
// ------------------------------------------------------------ //
class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {
    deviceConnected = true;
  }

  void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
    deviceConnected = false;
  }
};

class CharacteristicCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override {
    std::string rxValue = pCharacteristic->getValue();
    if (rxValue.length() > 0) {
      String webCommand = String(rxValue.c_str());
      webCommand.trim();
      Serial.print("📥 [Web UI -> ESP32]: ");
      Serial.println(webCommand);
      Serial1.println(webCommand);
    }
  }
};

// ------------------------------------------------------------ //
// IMAGE PROCESSING UTILITIES
// ------------------------------------------------------------ //
static void pickExposureTier(uint8_t* gray, int n, int* lowPercent, int* highPercent, int* otsuBias) {
  long sum = 0;
  for (int i = 0; i < n; i++) sum += gray[i];
  float mean = (float)sum / (float)n;

  for (size_t i = 0; i < sizeof(EXPOSURE_TIERS) / sizeof(EXPOSURE_TIERS[0]); i++) {
    if (mean >= EXPOSURE_TIERS[i].meanBrightnessAbove) {
      *lowPercent = EXPOSURE_TIERS[i].lowPercent;
      *highPercent = EXPOSURE_TIERS[i].highPercent;
      *otsuBias = EXPOSURE_TIERS[i].otsuBias;
      return;
    }
  }
}

static int otsuThreshold(uint8_t* gray, int n) {
  int hist[256] = { 0 };
  for (int i = 0; i < n; i++) hist[gray[i]]++;

  double total = n, sumAll = 0;
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

static void percentileContrastStretch(uint8_t* gray, int n, int lowPercent, int highPercent) {
  int hist[256] = { 0 };
  for (int i = 0; i < n; i++) hist[gray[i]]++;

  int lowTarget = (n * lowPercent) / 100;
  int highTarget = (n * highPercent) / 100;
  int cumulative = 0, lo = 0, hi = 255;

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

  debugStretchLow = lo;
  debugStretchHigh = hi;

  if (hi - lo < 24) return;
  int range = hi - lo;

  for (int i = 0; i < n; i++) {
    int v = gray[i];
    if (v <= lo) gray[i] = 0;
    else if (v >= hi) gray[i] = 255;
    else gray[i] = (uint8_t)(((v - lo) * 255) / range);
  }
}

static bool findLargestBlob(uint8_t* bin, int w, int h, float minAreaRatio, Blob* out, int* stack, uint8_t* visited) {
  int n = w * h;
  memset(visited, 0, n);
  float minArea = minAreaRatio * n;
  Blob best = { 0, 0, 0, 0, 0 };
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

      int nb[4] = { idx - 1, idx + 1, idx - w, idx + w };
      for (int k = 0; k < 4; k++) {
        int nidx = nb[k];
        if ((k == 0 && x == 0) || (k == 1 && x == w - 1) || (k == 2 && y == 0) || (k == 3 && y == h - 1)) continue;
        if (nidx < 0 || nidx >= n) continue;
        if (bin[nidx] && !visited[nidx]) {
          visited[nidx] = 1;
          stack[sp++] = nidx;
        }
      }
    }

    if (area >= minArea && area > best.area) {
      best = { area, x0, y0, x1, y1 };
      found = true;
    }
  }

  if (found) *out = best;
  return found;
}

// ------------------------------------------------------------ //
// SETUP & CAMERA INITIALIZATION
// ------------------------------------------------------------ //
bool initializeCamera() {
  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_GRAYSCALE;

  if (!psramFound()) {
    Serial.println("ERROR: PSRAM is required.");
    return false;
  }

  config.frame_size = FRAMESIZE_VGA;
  config.fb_count = 2;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.grab_mode = CAMERA_GRAB_LATEST;

  if (esp_camera_init(&config) != ESP_OK) return false;

  sensor_t* s = esp_camera_sensor_get();
  if (s) {
    if (s->set_contrast) s->set_contrast(s, SENSOR_CONTRAST);
    if (s->set_brightness) s->set_brightness(s, SENSOR_BRIGHTNESS);
    if (s->set_sharpness) s->set_sharpness(s, SENSOR_SHARPNESS);
    if (s->set_denoise) s->set_denoise(s, SENSOR_DENOISE);
    if (s->set_hmirror) s->set_hmirror(s, CAMERA_HMIRROR);
    if (s->set_vflip) s->set_vflip(s, CAMERA_VFLIP);
    if (s->set_exposure_ctrl) s->set_exposure_ctrl(s, 1);
    if (s->set_gain_ctrl) s->set_gain_ctrl(s, 1);
    if (s->set_ae_level) s->set_ae_level(s, SENSOR_AE_LEVEL);
  }

  delay(400);
  for (int i = 0; i < 6; i++) {
    camera_fb_t* warm = esp_camera_fb_get();
    if (warm) esp_camera_fb_return(warm);
    delay(40);
  }
  return true;
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(PSOC_UART_BAUD, SERIAL_8N1, PSOC_UART_RX_PIN, PSOC_UART_TX_PIN);
  delay(300);

  Serial.println("\n--- XIAO ESP32-S3 Vowel Detector ---");

  NimBLEDevice::init(BLE_DEVICE_NAME);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  NimBLEService* pService = pServer->createService(SERVICE_UUID);
  pTxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_TX, NIMBLE_PROPERTY::NOTIFY);
  pRxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_RX, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  pRxCharacteristic->setCallbacks(new CharacteristicCallbacks());
  pService->start();

  NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setName("Turtleeeee");
  pAdvertising->start();

  if (!initializeCamera()) {
    Serial.println("Camera initialization failed!");
    while (true) delay(1000);
  }

  // Allocate PSRAM buffers
  workBuf = (uint8_t*)ps_malloc(WORK_W * WORK_H);
  workBin = (uint8_t*)ps_malloc(WORK_W * WORK_H);
  workStack = (int*)ps_malloc(WORK_W * WORK_H * sizeof(int));
  workVisited = (uint8_t*)ps_malloc(WORK_W * WORK_H);

  analysisGray = (uint8_t*)ps_malloc(ANALYSIS_N);
  analysisBin = (uint8_t*)ps_malloc(ANALYSIS_N);
  tmpA = (uint8_t*)ps_malloc(ANALYSIS_N);
  tmpB = (uint8_t*)ps_malloc(ANALYSIS_N);
  stackA = (int*)ps_malloc(ANALYSIS_N * sizeof(int));
  visitedA = (uint8_t*)ps_malloc(ANALYSIS_N);
  outsideA = (uint8_t*)ps_malloc(ANALYSIS_N);
  holeBinA = (uint8_t*)ps_malloc(ANALYSIS_N);
  labelA = (int32_t*)ps_malloc(ANALYSIS_N * sizeof(int32_t));

  if (!workBuf || !workBin || !workStack || !workVisited || !analysisGray || !analysisBin || !tmpA || !tmpB || !stackA || !visitedA || !outsideA || !holeBinA || !labelA) {
    Serial.println("ERROR: PSRAM allocation failed.");
    while (true) delay(1000);
  }

  lastPatternSeenMs = millis();
  Serial.println("Initialization complete. Awaiting input frames...\n");
}

DiscInfo detectDisc(camera_fb_t* fb) {
  DiscInfo info = { false, 0, 0, 0, 0 };

  for (int y = 0; y < WORK_H; y++) {
    for (int x = 0; x < WORK_W; x++) {
      workBuf[y * WORK_W + x] = fb->buf[(y * WORK_SCALE_Y) * FRAME_WIDTH + (x * WORK_SCALE_X)];
    }
  }

  int whiteThreshold = otsuThreshold(workBuf, WORK_W * WORK_H) + WHITE_THRESHOLD_OFFSET;
  if (whiteThreshold < WHITE_THRESHOLD_MIN) whiteThreshold = WHITE_THRESHOLD_MIN;
  if (whiteThreshold > WHITE_THRESHOLD_MAX) whiteThreshold = WHITE_THRESHOLD_MAX;

  for (int i = 0; i < WORK_W * WORK_H; i++) {
    workBin[i] = (workBuf[i] >= whiteThreshold) ? 1 : 0;
  }

  Blob blob;
  if (!findLargestBlob(workBin, WORK_W, WORK_H, MIN_BLOB_AREA_RATIO, &blob, workStack, workVisited)) {
    return info;
  }

  info.found = true;
  info.x0 = blob.x0 * WORK_SCALE_X;
  info.y0 = blob.y0 * WORK_SCALE_Y;
  info.x1 = (blob.x1 + 1) * WORK_SCALE_X;
  info.y1 = (blob.y1 + 1) * WORK_SCALE_Y;
  return info;
}

// ------------------------------------------------------------ //
// MORPHOLOGY & COMPONENT ANALYSIS HELPERS
// ------------------------------------------------------------ //
static void dilate3x3(const uint8_t* in, uint8_t* out, int w, int h) {
  for (int y = 0; y < h; y++) {
    for (int x = 0; x < w; x++) {
      int idx = y * w + x;
      if (in[idx]) {
        out[idx] = 1;
        continue;
      }
      uint8_t val = 0;
      for (int dy = -1; dy <= 1 && !val; dy++) {
        int ny = y + dy;
        if (ny < 0 || ny >= h) continue;
        for (int dx = -1; dx <= 1; dx++) {
          int nx = x + dx;
          if (nx < 0 || nx >= w) continue;
          if (in[ny * w + nx]) {
            val = 1;
            break;
          }
        }
      }
      out[idx] = val;
    }
  }
}

static void erode3x3(const uint8_t* in, uint8_t* out, int w, int h) {
  for (int y = 0; y < h; y++) {
    for (int x = 0; x < w; x++) {
      int idx = y * w + x;
      if (!in[idx]) {
        out[idx] = 0;
        continue;
      }
      uint8_t val = 1;
      for (int dy = -1; dy <= 1 && val; dy++) {
        int ny = y + dy;
        if (ny < 0 || ny >= h) {
          val = 0;
          break;
        }
        for (int dx = -1; dx <= 1; dx++) {
          int nx = x + dx;
          if (nx < 0 || nx >= w) {
            val = 0;
            break;
          }
          if (!in[ny * w + nx]) {
            val = 0;
            break;
          }
        }
      }
      out[idx] = val;
    }
  }
}

static int labelComponents(const uint8_t* bin, int w, int h, Comp* comps, int maxComps, float minAreaRatio, int* stack, uint8_t* visited) {
  int n = w * h;
  memset(visited, 0, n);
  float minArea = minAreaRatio * n;
  int count = 0;

  for (int start = 0; start < n; start++) {
    if (!bin[start] || visited[start]) continue;

    int sp = 0;
    stack[sp++] = start;
    visited[start] = 1;
    int area = 0, x0 = w, y0 = h, x1 = -1, y1 = -1;
    double sx = 0, sy = 0;

    while (sp > 0) {
      int idx = stack[--sp];
      int x = idx % w, y = idx / w;
      area++;
      sx += x;
      sy += y;
      if (x < x0) x0 = x;
      if (x > x1) x1 = x;
      if (y < y0) y0 = y;
      if (y > y1) y1 = y;

      const int nb[4] = { idx - 1, idx + 1, idx - w, idx + w };
      for (int k = 0; k < 4; k++) {
        int ni = nb[k];
        if ((k == 0 && x == 0) || (k == 1 && x == w - 1) || (k == 2 && y == 0) || (k == 3 && y == h - 1)) continue;
        if (ni < 0 || ni >= n) continue;
        if (bin[ni] && !visited[ni]) {
          visited[ni] = 1;
          stack[sp++] = ni;
        }
      }
    }

    if (area >= minArea && count < maxComps) {
      comps[count].area = area;
      comps[count].cx = (float)(sx / area);
      comps[count].cy = (float)(sy / area);
      comps[count].x0 = x0;
      comps[count].y0 = y0;
      comps[count].x1 = x1;
      comps[count].y1 = y1;
      count++;
    }
  }
  return count;
}

static int findHoles(const uint8_t* bin, int w, int h, Hole* holes, int maxHoles, float minHoleRatio, int* stack, uint8_t* outside, uint8_t* holeBin, int32_t* labelBuf) {
  int n = w * h;
  // 必须确保每次调用的临时数组被彻底清空
  memset(outside, 0, n);
  memset(holeBin, 0, n);

  int sp = 0;

  // 顶部和底部边缘 Flood Fill
  for (int x = 0; x < w; x++) {
    int topIdx = x;
    int botIdx = (h - 1) * w + x;
    if (!bin[topIdx] && !outside[topIdx]) {
      outside[topIdx] = 1;
      stack[sp++] = topIdx;
    }
    if (!bin[botIdx] && !outside[botIdx]) {
      outside[botIdx] = 1;
      stack[sp++] = botIdx;
    }
  }
  // 左侧和右侧边缘 Flood Fill
  for (int y = 0; y < h; y++) {
    int leftIdx = y * w;
    int rightIdx = y * w + (w - 1);
    if (!bin[leftIdx] && !outside[leftIdx]) {
      outside[leftIdx] = 1;
      stack[sp++] = leftIdx;
    }
    if (!bin[rightIdx] && !outside[rightIdx]) {
      outside[rightIdx] = 1;
      stack[sp++] = rightIdx;
    }
  }

  // Flood fill 寻找真正的外部背景
  while (sp > 0) {
    int idx = stack[--sp];
    int x = idx % w;
    int y = idx / w;

    const int nb[4] = { idx - 1, idx + 1, idx - w, idx + w };
    for (int k = 0; k < 4; k++) {
      if ((k == 0 && x == 0) || (k == 1 && x == w - 1) || (k == 2 && y == 0) || (k == 3 && y == h - 1)) continue;

      int ni = nb[k];
      if (ni >= 0 && ni < n) {
        if (!bin[ni] && !outside[ni]) {
          outside[ni] = 1;
          stack[sp++] = ni;
        }
      }
    }
  }

  // 标记内部封闭孔洞
  for (int i = 0; i < n; i++) {
    holeBin[i] = (!bin[i] && !outside[i]) ? 1 : 0;
  }

  // 连通域提取
  Comp compHoles[16];
  int count = labelComponents(holeBin, w, h, compHoles, maxHoles, 0.0f, stack, outside);

  int validHoles = 0;
  for (int i = 0; i < count; i++) {
    if (compHoles[i].area >= 1) {
      holes[validHoles].area = compHoles[i].area;
      holes[validHoles].cx = (int)lroundf(compHoles[i].cx);
      holes[validHoles].cy = (int)lroundf(compHoles[i].cy);
      validHoles++;
      if (validHoles >= maxHoles) break;
    }
  }

  return validHoles;
}

// 修复: 原来的 int componentPixels[ANALYSIS_N] 是 64KB 栈数组, 容易栈溢出
// 改用 labelBuf (ANALYSIS_N 个 int32, 大小刚好)
static void removeSmallComponents(uint8_t* bin, int w, int h, float minAreaRatio, int32_t* labelBuf, int* stack) {
  int n = w * h;
  float minArea = minAreaRatio * n;
  memset(visitedA, 0, n);
  int* componentPixels = (int*)labelBuf;

  for (int start = 0; start < n; start++) {
    if (!bin[start] || visitedA[start]) continue;

    int sp = 0;
    stack[sp++] = start;
    visitedA[start] = 1;
    int compSize = 0;

    while (sp > 0) {
      int idx = stack[--sp];
      componentPixels[compSize++] = idx;
      int x = idx % w, y = idx / w;

      const int nb[4] = { idx - 1, idx + 1, idx - w, idx + w };
      for (int k = 0; k < 4; k++) {
        int ni = nb[k];
        if ((k == 0 && x == 0) || (k == 1 && x == w - 1) || (k == 2 && y == 0) || (k == 3 && y == h - 1)) continue;
        if (ni < 0 || ni >= n) continue;
        if (bin[ni] && !visitedA[ni]) {
          visitedA[ni] = 1;
          stack[sp++] = ni;
        }
      }
    }

    if (compSize < minArea) {
      for (int i = 0; i < compSize; i++) {
        bin[componentPixels[i]] = 0;
      }
    }
  }
}

// ------------------------------------------------------------ //
// TRACKING & POSITIONING
// ------------------------------------------------------------ //
DiscInfo stabilizeDisc(DiscInfo candidate) {
  if (!candidate.found) {
    if (trackerHasLock && trackerMisses < TRACK_HOLD_FRAMES) {
      trackerMisses++;
      return trackedDisc;
    }
    trackerHasLock = false;
    trackerMisses = 0;
    DiscInfo none = { false, 0, 0, 0, 0 };
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
  float sizeRatio = (oldW > 1.0f && newW > 1.0f) ? fmaxf(oldW, newW) / fminf(oldW, newW) : 99.0f;

  if (centerJump > TRACK_MAX_CENTER_JUMP || sizeRatio > TRACK_MAX_SIZE_RATIO) {
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

bool positionStatus(DiscInfo info, char* statusOut) {
  if (!info.found) {
    strcpy(statusOut, "NO MARKER");
    return false;
  }

  int markerW = info.x1 - info.x0;
  float markerCx = (info.x0 + info.x1) / 2.0f;
  float markerCy = (info.y0 + info.y1) / 2.0f;
  float targetCx = FRAME_WIDTH / 2.0f;
  float targetCy = FRAME_HEIGHT / 2.0f;
  float markerRatio = (float)markerW / FRAME_WIDTH;

  if (markerRatio < MIN_MARKER_WIDTH_RATIO) {
    strcpy(statusOut, "MOVE CLOSER");
    return false;
  }
  if (markerRatio > MAX_MARKER_WIDTH_RATIO) {
    strcpy(statusOut, "MOVE FURTHER");
    return false;
  }

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

// ------------------------------------------------------------ //
// EXTRACT & NORMALIZE VOWEL PATTERN
// ------------------------------------------------------------ //
static void removeBorderConnectedInk(uint8_t* bin, int w, int h) {
  int n = w * h;
  memset(visitedA, 0, n);
  int sp = 0;

  for (int x = 0; x < w; x++) {
    int a = x;
    int b = (h - 1) * w + x;
    if (bin[a] && !visitedA[a]) {
      visitedA[a] = 1;
      stackA[sp++] = a;
    }
    if (bin[b] && !visitedA[b]) {
      visitedA[b] = 1;
      stackA[sp++] = b;
    }
  }
  for (int y = 0; y < h; y++) {
    int a = y * w;
    int b = y * w + (w - 1);
    if (bin[a] && !visitedA[a]) {
      visitedA[a] = 1;
      stackA[sp++] = a;
    }
    if (bin[b] && !visitedA[b]) {
      visitedA[b] = 1;
      stackA[sp++] = b;
    }
  }

  while (sp > 0) {
    int idx = stackA[--sp];
    int x = idx % w;
    int y = idx / w;
    bin[idx] = 0;

    const int nb[4] = { idx - 1, idx + 1, idx - w, idx + w };
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
      bin[(dy0 + dy) * ANALYSIS_SIZE + (dx0 + dx)] = tmpA[sy * ANALYSIS_SIZE + sx];
    }
  }
  return true;
}

static bool processSquareCrop(camera_fb_t* fb, int x0, int y0, int side, bool removeBorderNoise) {
  debugFailReason = "";  // [DEBUG] 每次进入先清空
  debugInkBeforeBorder = 0;
  debugInkAfterBorder = 0;

  if (side < 24) {
    debugFailReason = "side<24";
    return false;
  }
  if (x0 < 0 || y0 < 0 || x0 + side > (int)fb->width || y0 + side > (int)fb->height) {
    debugFailReason = "out of bounds";
    return false;
  }

  for (int y = 0; y < ANALYSIS_SIZE; y++) {
    int sy = y0 + (y * side) / ANALYSIS_SIZE;
    for (int x = 0; x < ANALYSIS_SIZE; x++) {
      int sx = x0 + (x * side) / ANALYSIS_SIZE;
      analysisGray[y * ANALYSIS_SIZE + x] = fb->buf[sy * fb->width + sx];
    }
  }

  int lowPercent, highPercent, otsuBias;
  pickExposureTier(analysisGray, ANALYSIS_N, &lowPercent, &highPercent, &otsuBias);
  percentileContrastStretch(analysisGray, ANALYSIS_N, lowPercent, highPercent);

  int t = otsuThreshold(analysisGray, ANALYSIS_N) + otsuBias;
  if (t < 35) t = 35;
  if (t > 225) t = 225;
  debugBinaryThreshold = t;

  static const int T_OFFSETS[4] = { 0, -35, -60, -80 };
  for (int attempt = 0; attempt < 4; attempt++) {
    int tt = t + T_OFFSETS[attempt];
    if (tt < 20) tt = 20;
    debugBinaryThreshold = tt;

    for (int i = 0; i < ANALYSIS_N; i++) {
      analysisBin[i] = (analysisGray[i] < tt) ? 1 : 0;
    }
    debugInkBeforeBorder = countInk(analysisBin);

    if (removeBorderNoise) {
      removeBorderConnectedInk(analysisBin, ANALYSIS_SIZE, ANALYSIS_SIZE);
    }
    debugInkAfterBorder = countInk(analysisBin);

    if (debugInkAfterBorder >= 20) break;   // 有东西留下来了，不用再降阈值
  }

  removeSmallComponents(analysisBin, ANALYSIS_SIZE, ANALYSIS_SIZE, MIN_BLACK_COMPONENT_RATIO, labelA, stackA);
  if (!normalizeInkToCanvas(analysisBin)) {
    debugFailReason = "normalize";
    return false;
  }

  removeSmallComponents(analysisBin, ANALYSIS_SIZE, ANALYSIS_SIZE, MIN_BLACK_COMPONENT_RATIO, labelA, stackA);

  int ink = 0;
  for (int i = 0; i < ANALYSIS_N; i++) {
    if (analysisBin[i]) ink++;
  }
  debugInkRatio = (float)ink / (float)ANALYSIS_N;

  if (debugInkRatio < MIN_INK_RATIO) {
    debugFailReason = "ink too low";
    return false;
  }
  if (debugInkRatio > MAX_INK_RATIO) {
    debugFailReason = "ink too high";
    return false;
  }
  return true;
}

bool extractPattern(camera_fb_t* fb, DiscInfo info) {
  if (!info.found) return false;
  int markerW = info.x1 - info.x0;
  int markerH = info.y1 - info.y0;
  if (markerW <= 0 || markerH <= 0) return false;

  float markerCx = (info.x0 + info.x1) * 0.5f;
  float markerCy = (info.y0 + info.y1) * 0.5f;
  int side = (markerW > markerH) ? markerW : markerH;

  if (side > (int)(fminf((float)fb->width, (float)fb->height) * 0.96f)) {
    debugFailReason = "disc too large";
    return false;
  }

  side = (int)lroundf(side * 0.95f);
  int x0 = (int)lroundf(markerCx - side * 0.5f);
  int y0 = (int)lroundf(markerCy - side * 0.5f);

  if (side > (int)fb->width) side = fb->width;
  if (side > (int)fb->height) side = fb->height;
  if (x0 < 0) x0 = 0;
  if (y0 < 0) y0 = 0;
  if (x0 + side > (int)fb->width) x0 = fb->width - side;
  if (y0 + side > (int)fb->height) y0 = fb->height - side;

  debugSrc = "FACE";
  return processSquareCrop(fb, x0, y0, side, true);
}

bool extractFallbackPattern(camera_fb_t* fb) {
  int minDim = ((int)fb->width < (int)fb->height) ? fb->width : fb->height;
  int side = (int)lroundf(minDim * FALLBACK_CROP_RATIO);
  int x0 = ((int)fb->width - side) / 2;
  int y0 = ((int)fb->height - side) / 2;
  debugSrc = "FALLBACK";
  return processSquareCrop(fb, x0, y0, side, true);
}

// ------------------------------------------------------------ //
// PER-FRAME LABEL VOTING
// ------------------------------------------------------------ //
static char getVotedLabel() {
  static char activeVowel = 0;  // Remembers the currently active vowel state

  const char letters[5] = { 'a', 'e', 'i', 'o', 'u' };
  int counts[5] = { 0, 0, 0, 0, 0 };

  // 1. Tally up all vote counts currently in the buffer
  for (int i = 0; i < voteCount; i++) {
    for (int j = 0; j < 5; j++) {
      if (voteLabels[i] == letters[j]) {
        counts[j]++;
        break;
      }
    }
  }

  // 2. Find which vowel got the most votes
  int best = -1;
  int bestCount = 0;
  for (int j = 0; j < 5; j++) {
    if (counts[j] > bestCount) {
      bestCount = counts[j];
      best = j;
    }
  }

  char currentBestChar = (best >= 0) ? letters[best] : 'N';
  Serial.printf("[VOTE METRICS] Frames: %d/%d | Active: '%c' | Highest: '%c' (%d votes) | [a:%d, e:%d, i:%d, o:%d, u:%d]\n",
                voteCount, VOTE_FRAMES,
                activeVowel ? activeVowel : '-',
                currentBestChar, bestCount,
                counts[0], counts[1], counts[2], counts[3], counts[4]);

  // Wait until buffer is full before attempting state change
  if (voteCount < VOTE_FRAMES) return activeVowel;

  // 3. Check if the top vote-getter meets VOTE_REQUIRED
  if (best >= 0 && bestCount >= VOTE_REQUIRED) {
    if (activeVowel != letters[best]) {
      activeVowel = letters[best];  // Update state
      clearVotes();                 // Clear buffer for a fresh start
    }
  }

  return activeVowel;
}

// ------------------------------------------------------------ //
// SHAPE DESCRIPTORS & CLASSIFICATION
// ------------------------------------------------------------ //
static const float REF_A[RADIAL_BINS] = { 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.037418f, 0.141254f, 0.229186f, 0.255379f, 0.197381f, 0.139383f };
static const float REF_E[RADIAL_BINS] = { 0.010980f, 0.026182f, 0.032095f, 0.026182f, 0.029561f, 0.027027f, 0.024493f, 0.025338f, 0.098818f, 0.221284f, 0.273649f, 0.204392f };
static const float REF_O[RADIAL_BINS] = { 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.073350f, 0.283619f, 0.343521f, 0.299511f };

static float computeRadialCV(uint8_t* bin) {
  long count = 0;
  double sx = 0.0, sy = 0.0;
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
  double sr = 0.0, sr2 = 0.0;

  for (int y = 0; y < ANALYSIS_SIZE; y++) {
    for (int x = 0; x < ANALYSIS_SIZE; x++) {
      if (!bin[y * ANALYSIS_SIZE + x]) continue;
      float dx = x - cx, dy = y - cy;
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

static void computeRadialSignature(uint8_t* bin, float out[RADIAL_BINS]) {
  for (int i = 0; i < RADIAL_BINS; i++) out[i] = 0.0f;
  long count = 0;
  double sx = 0.0, sy = 0.0;

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
  int radiusHist[RADIAL_MAX_RADIUS_BIN] = { 0 };

  for (int y = 0; y < ANALYSIS_SIZE; y++) {
    for (int x = 0; x < ANALYSIS_SIZE; x++) {
      if (!bin[y * ANALYSIS_SIZE + x]) continue;
      float dx = x - cx, dy = y - cy;
      int ri = (int)lroundf(sqrtf(dx * dx + dy * dy));
      if (ri < 0) ri = 0;
      if (ri >= RADIAL_MAX_RADIUS_BIN) ri = RADIAL_MAX_RADIUS_BIN - 1;
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

  int binCounts[RADIAL_BINS] = { 0 };
  for (int y = 0; y < ANALYSIS_SIZE; y++) {
    for (int x = 0; x < ANALYSIS_SIZE; x++) {
      if (!bin[y * ANALYSIS_SIZE + x]) continue;
      float dx = x - cx, dy = y - cy;
      float r = sqrtf(dx * dx + dy * dy);
      float normalized = r / (float)r98;
      int b = (normalized >= 1.0f) ? RADIAL_BINS - 1 : (int)(normalized * RADIAL_BINS);
      if (b < 0) b = 0;
      if (b >= RADIAL_BINS) b = RADIAL_BINS - 1;
      binCounts[b]++;
    }
  }

  for (int i = 0; i < RADIAL_BINS; i++) {
    out[i] = (float)binCounts[i] / (float)count;
  }
}

static float signatureDistance(const float a[RADIAL_BINS], const float b[RADIAL_BINS]) {
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

int debugHoleCx = 0;
int debugHoleCy = 0;
float debugRelHoleX = 0.0f;
float debugRelHoleY = 0.0f;
float debugRightToLeftRatio = 0.0f;
float debugHoleAreaRatio = 0.0f;

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

static void clearVotes() {
  memset(voteLabels, 0, sizeof(voteLabels));
  voteIndex = 0;
  voteCount = 0;
}

static void pushVote(char label) {
  voteLabels[voteIndex] = label;
  voteIndex = (voteIndex + 1) % VOTE_FRAMES;
  if (voteCount < VOTE_FRAMES) voteCount++;
}

char classifyFrame(uint8_t* rawBin) {
  debugRadialCV = 99.0f;
  debugRawComponents = 0;
  debugHoles = 0;

  // ---------------------------------------------------------
  // 1. Lowercase 'i': Two raw components (Dot + Stem)
  // ---------------------------------------------------------
  Comp rawComps[16];
  int rawNC = labelComponents(rawBin, ANALYSIS_SIZE, ANALYSIS_SIZE, rawComps, 16, MIN_BLACK_COMPONENT_RATIO, stackA, visitedA);
  debugRawComponents = rawNC;

  if (rawNC >= 2) {
    int dotIdx = -1, stemIdx = -1;
    int minTop = 999, maxArea = -1;

    for (int i = 0; i < rawNC; i++) {
      if (rawComps[i].y0 < minTop) {
        minTop = rawComps[i].y0;
        dotIdx = i;
      }
      if (rawComps[i].area > maxArea) {
        maxArea = rawComps[i].area;
        stemIdx = i;
      }
    }

    if (dotIdx != -1 && stemIdx != -1 && dotIdx != stemIdx) {
      float areaRatio = (float)rawComps[dotIdx].area / (float)rawComps[stemIdx].area;
      bool posOk = (rawComps[dotIdx].y1 <= rawComps[stemIdx].y0 + 8);
      bool ratioOk = (areaRatio > 0.03f && areaRatio < 0.60f);
      if (posOk && ratioOk) {
        DEBUG_PRINTLN("  -> MATCHED: 'i' (Dot + Stem)");
        return 'i';
      }
    }
  }

  // Morphological Closing to clean up image gaps
  memcpy(tmpB, rawBin, ANALYSIS_N);
  dilate3x3(tmpB, tmpA, ANALYSIS_SIZE, ANALYSIS_SIZE);
  erode3x3(tmpA, tmpB, ANALYSIS_SIZE, ANALYSIS_SIZE);

  removeSmallComponents(tmpB, ANALYSIS_SIZE, ANALYSIS_SIZE, MIN_BLACK_COMPONENT_RATIO, labelA, stackA);

  Comp closedComps[16];
  int closedNC = labelComponents(tmpB, ANALYSIS_SIZE, ANALYSIS_SIZE, closedComps, 16, MIN_BLACK_COMPONENT_RATIO, stackA, visitedA);

  Hole holes[16];
  int nh = findHoles(tmpB, ANALYSIS_SIZE, ANALYSIS_SIZE, holes, 16, MIN_HOLE_RATIO, stackA, outsideA, holeBinA, labelA);
  debugHoles = nh;
  debugRadialCV = computeRadialCV(tmpB);

  // [DEBUG] 防止 closedNC==0 时读取未初始化的 closedComps[0]
  if (closedNC == 0) {
    Serial.printf("[UNKNOWN] no component after closing rawNC=%d\n", rawNC);
    return 0;
  }

  if (closedNC > 3) {
    DEBUG_PRINTLN("  [REJECT] Too many components (noise)");
    return 0;  // Ignore multi-blob noise
  }

  int bw = closedComps[0].x1 - closedComps[0].x0 + 1;
  int bh = closedComps[0].y1 - closedComps[0].y0 + 1;
  float aspectRatio = (float)bw / (float)bh;

  // ---------------------------------------------------------
  // 2. CASE A: HOLE PRESENT (nh >= 1) -> 'e', 'o', 'a'
  // ---------------------------------------------------------
  if (nh >= 1) {
    int holeCy = holes[0].cy;
    int holeCx = holes[0].cx;
    int rightInk = 0, leftInk = 0;

    int charY0 = closedComps[0].y0;
    int charH = closedComps[0].y1 - closedComps[0].y0 + 1;

    float relHoleY = (float)(holeCy - charY0) / (float)charH;
    float relHoleX = (float)(holeCx - closedComps[0].x0) / (float)bw;

    float charArea = (float)closedComps[0].area;
    float holeArea = (float)holes[0].area;
    float holeAreaRatio = holeArea / (charArea + 1e-5f);

    for (int y = 0; y < ANALYSIS_SIZE; y++) {
      for (int x = 0; x < ANALYSIS_SIZE; x++) {
        if (!tmpB[y * ANALYSIS_SIZE + x]) continue;
        if (x > holeCx) rightInk++;
        else leftInk++;
      }
    }
    float rightToLeftRatio = (float)rightInk / (float)(leftInk + 1e-5f);

    debugHoleCx = holeCx;
    debugHoleCy = holeCy;
    debugRelHoleX = relHoleX;
    debugRelHoleY = relHoleY;
    debugRightToLeftRatio = rightToLeftRatio;
    debugHoleAreaRatio = holeAreaRatio;

    DEBUG_PRINTF("[1+ HOLE METRICS] AbsPos: (%d, %d) | RelPos: (X:%.2f, Y:%.2f) | R/L Ink Ratio: %.2f\n HoleRatio: %.3f | RadCV: %.3f\n",
                 debugHoleCx, debugHoleCy, debugRelHoleX, debugRelHoleY, debugRightToLeftRatio, debugHoleAreaRatio, debugRadialCV);

    // Rule 1: 'e' has a right-heavy loop/tail
    if (holeAreaRatio < 0.5 && 0.25 <= debugRadialCV) {
      DEBUG_PRINTLN("   -> MATCHED: 'e' (Right-heavy loop)");
      return 'e';
    }

    // Rule 2: 'o'（先判断）
    if (holeAreaRatio >= 0.7 && debugRadialCV < 0.12) {
      DEBUG_PRINTLN("   -> MATCHED: 'o' (Centered hole)");
      return 'o';
    }

    // Rule 3: 'a'
    if (holeAreaRatio >= 0.8 && debugRadialCV >= 0.12 && debugRadialCV <= 0.25) {
      DEBUG_PRINTLN("   -> MATCHED: 'a'");
      return 'a';
    }

    DEBUG_PRINTLN("   -> [REJECTED] GOT HOLE BUT NO MATCH");
    Serial.printf("[UNKNOWN] nh=%d holeR=%.3f radCV=%.3f relY=%.2f R/L=%.2f holeCy=%d rawNC=%d closedNC=%d\n",
                  nh, holeAreaRatio, debugRadialCV, relHoleY, rightToLeftRatio, holeCy, rawNC, closedNC);
  }

  // ---------------------------------------------------------
  // 3. CASE B: NO HOLES (nh == 0) -> 'u', 'i', or AEO Fallback
  // ---------------------------------------------------------
  int topHalfInk = 0, bottomHalfInk = 0;
  int midY = closedComps[0].y0 + bh / 2;

  for (int y = closedComps[0].y0; y <= closedComps[0].y1; y++) {
    for (int x = closedComps[0].x0; x <= closedComps[0].x1; x++) {
      if (tmpB[y * ANALYSIS_SIZE + x]) {
        if (y < midY) topHalfInk++;
        else bottomHalfInk++;
      }
    }
  }

  float bottomToTopRatio = (float)bottomHalfInk / (float)(topHalfInk + 1e-5f);
  DEBUG_PRINTF("[0 HOLE METRICS] B/T Ratio: %.2f | AspectRatio: %.2f | RadCV: %.3f | NC: %d\n", bottomToTopRatio, aspectRatio, debugRadialCV, rawNC);

  // 1. Check for 'u'
  if (debugRadialCV <= 0.3) {
    DEBUG_PRINTF("   MATCHED: 'u' (Open top)\n");
    return 'u';
  }

  // 2. Check for 'i'
  if (debugRadialCV > 0.3) {
    DEBUG_PRINTF("   MATCHED: 'i' (Narrow stroke)\n");
    return 'i';
  }

  // 3. Fallback for non-looping 'a', 'e', 'o'
  return classifyAEO(tmpB);
}

// ------------------------------------------------------------ //
// MAIN LOOP
// ------------------------------------------------------------ //
unsigned long lastCameraFrameMs = 0;
const unsigned long CAMERA_INTERVAL_MS = 50;

void loop() {
  // 1. Read incoming strings from PSoC via UART
  while (Serial1.available()) {
    char c = (char)Serial1.read();
    if (c == '\n' || c == '\r') {
      if (logoBufferFromPSoC.length() > 0) {
        logoBufferFromPSoC.trim();
        Serial.print("📥 [PSoC -> ESP32]: ");
        Serial.println(logoBufferFromPSoC);

        if (deviceConnected && pTxCharacteristic != nullptr) {
          pTxCharacteristic->setValue(logoBufferFromPSoC.c_str());
          pTxCharacteristic->notify();
          Serial.print("📤 [ESP32 -> Web UI]: ");
          Serial.println(logoBufferFromPSoC);
        } else {
          Serial.println("⚠️ [BLE]: Web Dashboard not connected!");
        }
        logoBufferFromPSoC = "";
      }
    } else {
      logoBufferFromPSoC += c;
    }
  }

  // 2. Camera Processing Rate Limiter
  if (millis() - lastCameraFrameMs < CAMERA_INTERVAL_MS) {
    return;
  }
  lastCameraFrameMs = millis();

  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed.");
    return;
  }
  if (fb->format != PIXFORMAT_GRAYSCALE) {
    Serial.println("Unexpected pixel format from camera.");
    esp_camera_fb_return(fb);
    return;
  }

  // 3. Pattern Extraction & Detection
  DiscInfo candidate = detectDisc(fb);
  DiscInfo disc = stabilizeDisc(candidate);
  char status[24];
  bool ready = positionStatus(disc, status);
  (void)ready;

  bool patternOK = false;
  bool usedFallback = false;

  if (disc.found) {
    patternOK = extractPattern(fb, disc);
  }
  if (!patternOK) {
    patternOK = extractFallbackPattern(fb);
    usedFallback = patternOK;
  }

#if FAIL_DEBUG
  if (!patternOK) {
    static unsigned long lastFailPrint = 0;
    unsigned long nowDbg = millis();
    if (nowDbg - lastFailPrint > 300) {
      lastFailPrint = nowDbg;
      Serial.printf("[FAIL] src=%s reason=%s inkBefore=%d inkAfter=%d ratio=%.4f thr=%d disc=%d stretch=%d..%d\n",
                    debugSrc, debugFailReason, debugInkBeforeBorder, debugInkAfterBorder,
                    debugInkRatio, debugBinaryThreshold, disc.found ? 1 : 0,
                    debugStretchLow, debugStretchHigh);
    }
  }
#endif

  if (!patternOK) {
    noMarkerStreak++;
    unknownStreak = 0;
    clearVotes();

    unsigned long now = millis();
    if (!noVowelPrinted && (now - lastPatternSeenMs >= PSOC_NO_VOWEL_DELAY_MS)) {
      // 注意: 旧的 [NO VOWEL] AbsPos... 已删除, 因为它打印的是上一次分类留下的旧值, 会误导
      Serial.printf("[NO-VOWEL reason] preprocess=%s inkBefore=%d inkAfter=%d thr=%d\n",
                    debugFailReason, debugInkBeforeBorder, debugInkAfterBorder, debugBinaryThreshold);
      Serial.println("No vowels detected");
      Serial1.write('N');
      lastConfirmedVowel = 0;
      lastPrinted = 0;
      noVowelPrinted = true;
      clearVotes();
    }
    esp_camera_fb_return(fb);
    return;
  }

  lastPatternSeenMs = millis();
  noMarkerStreak = 0;
  noVowelPrinted = false;

  char detected = classifyFrame(analysisBin);
  Serial.printf("[FRAME] src=%s det=%c votes=%d/%d\n",
                usedFallback ? "FALLBACK" : "FACE", detected ? detected : '?', voteCount, VOTE_FRAMES);

  if (detected == 'u' || detected == 'e') {
    static unsigned long lastU = 0;
    if (millis() - lastU > 2000) {
      lastU = millis();
      Serial.printf("[U-DUMP] thr=%d nh=%d closedNC? radCV=%.3f\n", debugBinaryThreshold, debugHoles, debugRadialCV);
      dumpBin(analysisBin);
    }
  }
  if (SHOW_DEBUG) {
    static unsigned long lastDebug = 0;
    if (millis() - lastDebug > 350) {
      Serial.printf(
        "  [live] source=%s guess=%c ink=%.3f rawComp=%d holes=%d radial=%.3f "
        "A=%.3f E=%.3f O=%.3f stretch=%d..%d thr=%d votes=%d/%d\n",
        usedFallback ? "VOWEL" : "FACE", detected ? detected : '?', debugInkRatio,
        debugRawComponents, debugHoles, debugRadialCV, debugDA, debugDE, debugDO,
        debugStretchLow, debugStretchHigh, debugBinaryThreshold, voteCount, VOTE_FRAMES);
      lastDebug = millis();
    }
  }

  if (detected == 0) {
    unknownStreak++;
  } else {
    unknownStreak = 0;
  }
  pushVote(detected);

  char finalLabel = getVotedLabel();
  if (finalLabel != 0) {
    if (finalLabel != lastConfirmedVowel) {
      Serial.print("Final detected vowel: ");
      Serial.println(finalLabel);
      Serial1.write(finalLabel);
      lastConfirmedVowel = finalLabel;
      lastPrinted = finalLabel;
    }
  }

  esp_camera_fb_return(fb);
}
