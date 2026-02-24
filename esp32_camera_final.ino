#include <sample_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "esp_camera.h"

#define CAMERA_MODEL_AI_THINKER

// -------------------- FLASH LED PIN --------------------
#define FLASH_PIN 4   // ESP32-CAM AI Thinker flash LED

#if defined(CAMERA_MODEL_AI_THINKER)
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22
#endif

#define EI_CAMERA_RAW_FRAME_BUFFER_COLS 320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS 240
#define EI_CAMERA_FRAME_BYTE_SIZE 3
#define CONFIDENCE_THRESHOLD 0.80  

// -------------------- SEND CONTROL --------------------
bool detectionLocked = false;
unsigned long detectionLockTime = 0;
const unsigned long DETECTION_COOLDOWN = 5000; // 5 seconds


static bool debug_nn = false;
static bool is_initialised = false;
uint8_t *snapshot_buf;

static camera_config_t camera_config = {
    .pin_pwdn = PWDN_GPIO_NUM,
    .pin_reset = RESET_GPIO_NUM,
    .pin_xclk = XCLK_GPIO_NUM,
    .pin_sscb_sda = SIOD_GPIO_NUM,
    .pin_sscb_scl = SIOC_GPIO_NUM,

    .pin_d7 = Y9_GPIO_NUM,
    .pin_d6 = Y8_GPIO_NUM,
    .pin_d5 = Y7_GPIO_NUM,
    .pin_d4 = Y6_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM,
    .pin_d2 = Y4_GPIO_NUM,
    .pin_d1 = Y3_GPIO_NUM,
    .pin_d0 = Y2_GPIO_NUM,
    .pin_vsync = VSYNC_GPIO_NUM,
    .pin_href = HREF_GPIO_NUM,
    .pin_pclk = PCLK_GPIO_NUM,

    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG,
    .frame_size = FRAMESIZE_QVGA,
    .jpeg_quality = 12,
    .fb_count = 1,
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

bool ei_camera_init(void);
void ei_camera_deinit(void);
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf);

void setup()
{
    Serial.begin(115200);
    Serial1.begin(115200, SERIAL_8N1, -1, 14);

    // FLASH LED INIT
    pinMode(FLASH_PIN, OUTPUT);
    digitalWrite(FLASH_PIN, LOW);

    while (!Serial);
    Serial.println("Edge Impulse Inferencing Demo");

    if (!ei_camera_init()) {
        ei_printf("Failed to initialize Camera!\r\n");
    } else {
        ei_printf("Camera initialized\r\n");
    }

    ei_printf("\nStarting continious inference in 2 seconds...\n");
    ei_sleep(2000);
}

void loop() {

    if (ei_sleep(5) != EI_IMPULSE_OK) return;

    snapshot_buf = (uint8_t*)malloc(EI_CAMERA_RAW_FRAME_BUFFER_COLS *
                                    EI_CAMERA_RAW_FRAME_BUFFER_ROWS *
                                    EI_CAMERA_FRAME_BYTE_SIZE);

    if (!snapshot_buf) {
        ei_printf("ERR: Failed to allocate snapshot buffer!\n");
        return;
    }

    ei::signal_t signal;
    signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    signal.get_data = &ei_camera_get_data;

    if (!ei_camera_capture(EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT, snapshot_buf)) {
        ei_printf("Failed to capture image\r\n");
        free(snapshot_buf);
        return;
    }

    // Run classifier
    ei_impulse_result_t result = {0};
    EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);
    if (err != EI_IMPULSE_OK) {
        ei_printf("ERR: Failed to run classifier (%d)\n", err);
        return;
    }

    ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
                result.timing.dsp, result.timing.classification, result.timing.anomaly);

    bool objectDetected = false;

#if EI_CLASSIFIER_OBJECT_DETECTION == 1
    ei_printf("Object detection bounding boxes:\r\n");
    for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {

        auto bb = result.bounding_boxes[i];

        if (bb.value == 0) continue;

        // KEEP YOUR EXACT PRINTS
        ei_printf("  %s (%f) [ x: %u, y: %u, width: %u, height: %u ]\r\n",
                    bb.label, bb.value, bb.x, bb.y, bb.width, bb.height);

        Serial1.print(bb.label);
        Serial1.print(",");
        Serial1.println(bb.value);

        int min_width = 5;
        int min_height = 5;

       if (bb.value >= CONFIDENCE_THRESHOLD &&
    bb.width >= min_width &&
    bb.height >= min_height &&
    !detectionLocked)
{
    detectionLocked = true;
    detectionLockTime = millis();
    objectDetected = true;

    // TURN FLASH ON
    digitalWrite(FLASH_PIN, HIGH);

    // SEND ONLY ONE RESULT
    if (strcmp(bb.label, "bio") == 0)
        Serial1.println("BIO");
    else if (strcmp(bb.label, "nobio") == 0)
        Serial1.println("NON");
    else if (strcmp(bb.label, "recyclable") == 0)
        Serial1.println("REC");

    break; //                                                        
}
    }
#endif

    // Turn flash OFF if no object detected
    if (!objectDetected) {
        digitalWrite(FLASH_PIN, LOW);
    }

#if EI_CLASSIFIER_HAS_ANOMALY
    ei_printf("Anomaly prediction: %.3f\r\n", result.anomaly);
#endif

    free(snapshot_buf);

    // -------------------- COOLDOWN RESET --------------------
if (detectionLocked && (millis() - detectionLockTime >= DETECTION_COOLDOWN)) {
    detectionLocked = false;
    digitalWrite(FLASH_PIN, LOW);
}
}

bool ei_camera_init(void) {
    if (is_initialised) return true;

    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) return false;

    sensor_t * s = esp_camera_sensor_get();
    if (s->id.PID == OV3660_PID) {
        s->set_vflip(s, 1);
        s->set_brightness(s, 1);
        s->set_saturation(s, 0);
    }

    is_initialised = true;
    return true;
}

bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) {

    if (!is_initialised) return false;

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) return false;

    bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, snapshot_buf);
    esp_camera_fb_return(fb);

    if (!converted) return false;

    if (img_width != EI_CAMERA_RAW_FRAME_BUFFER_COLS ||
        img_height != EI_CAMERA_RAW_FRAME_BUFFER_ROWS)
    {
        ei::image::processing::crop_and_interpolate_rgb888(
            out_buf,
            EI_CAMERA_RAW_FRAME_BUFFER_COLS,
            EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
            out_buf,
            img_width,
            img_height);
    }

    return true;
}

static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr)
{
    size_t pixel_ix = offset * 3;
    size_t out_ptr_ix = 0;

    while (length--) {
        out_ptr[out_ptr_ix] =
            (snapshot_buf[pixel_ix + 2] << 16) +
            (snapshot_buf[pixel_ix + 1] << 8) +
             snapshot_buf[pixel_ix];

        pixel_ix += 3;
        out_ptr_ix++;
    }
    return 0;
}
