#include "esp_camera.h"
#include <WiFi.h>
#include <vector>
#include <math.h>

// WiFi credentials
const char* ssid = "Hello";
const char* password = "12345678";

// Camera Model
#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"

// Reference Feature Vectors
const std::vector<float> catFeature = {114.81, 131.31, 109.92, 96.79};
const std::vector<float> dogFeature = {129.13, 135.78, 127.94, 117.92};

// Adjusted magnitudes to give more weight to dog features
const float catMagnitude = 231.47;
const float dogMagnitude = 229.83;

// Declare calculateDistance function first
float calculateDistance(const std::vector<float>& a, const std::vector<float>& b) {
  if (a.size() != b.size()) return 999999.0f;
  
  float sum = 0;
  for (size_t i = 0; i < a.size(); ++i) {
    float diff = a[i] - b[i];
    sum += diff * diff;
  }
  return sqrt(sum);
}

String predictLabel(const std::vector<float>& inputFeature) {
  float distCat = calculateDistance(inputFeature, catFeature);
  float distDog = calculateDistance(inputFeature, dogFeature) * 0.9;
  
  // Normalize distances
  float normalizedDistCat = distCat / catMagnitude;
  float normalizedDistDog = distDog / dogMagnitude;
  
  Serial.printf("Raw distances - Cat: %.2f, Dog: %.2f\n", distCat, distDog);
  Serial.printf("Normalized distances - Cat: %.2f, Dog: %.2f\n", 
                normalizedDistCat, normalizedDistDog);

  // Calculate scaled confidence scores with bias adjustment for dogs
  float catConf = max(20.0f, (1.0f - (normalizedDistCat * 1.5f)) * 100.0f);
  float dogConf = max(20.0f, (1.0f - (normalizedDistDog * 1.3f)) * 100.0f);
  
  // Adjust confidence scores to favor dogs slightly
  dogConf *= 1.1f;
  
  // Decision making with adjusted threshold
  if (normalizedDistDog <= normalizedDistCat * 1.1f) {
    return "Dog (" + String(min(dogConf, 100.0f), 1) + "%)";
  } else {
    return "Cat (" + String(min(catConf, 100.0f), 1) + "%)";
  }
}

std::vector<float> extractFeatures(camera_fb_t *fb) {
  if (!fb || !fb->buf) return {0, 0, 0, 0};
  
  uint32_t sumR = 0, sumG = 0, sumB = 0, sumGray = 0;
  uint32_t pixelsCount = fb->len;
  
  // Process image data with weights optimized for dog detection
  for (size_t i = 0; i < fb->len && fb->buf; i++) {
    uint8_t val = fb->buf[i];
    sumGray += val;
    sumR += val * 0.45;
    sumG += val * 0.35;
    sumB += val * 0.20;
  }

  float avgGray = pixelsCount > 0 ? sumGray / (float)pixelsCount : 0;
  float avgR = pixelsCount > 0 ? (sumR / (float)pixelsCount) * 1.1f : 0;
  float avgG = pixelsCount > 0 ? (sumG / (float)pixelsCount) * 1.2f : 0;
  float avgB = pixelsCount > 0 ? (sumB / (float)pixelsCount) * 1.3f : 0;

  Serial.printf("Features: Gray=%.2f R=%.2f G=%.2f B=%.2f\n", 
                avgGray, avgR, avgG, avgB);
  return {avgGray, avgR, avgG, avgB};
}

bool initCamera() {
  camera_config_t config;
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
  config.frame_size = FRAMESIZE_QVGA;
  config.jpeg_quality = 12;
  config.fb_count = 1;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.grab_mode = CAMERA_GRAB_LATEST;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return false;
  }

  sensor_t * s = esp_camera_sensor_get();
  if (s) {
    s->set_brightness(s, 2);
    s->set_contrast(s, 2);
    s->set_saturation(s, 3);
    s->set_whitebal(s, 1);
    s->set_awb_gain(s, 1);
    s->set_exposure_ctrl(s, 1);
    s->set_gain_ctrl(s, 1);
    s->set_aec2(s, 1);
    s->set_gainceiling(s, GAINCEILING_8X);
  }

  return true;
}

void setup() {
  Serial.begin(115200);
  while(!Serial) {
    delay(100);
  }
  
  Serial.println("\nInitializing ESP32-CAM Cat/Dog Classifier...");

  // Initialize WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  Serial.println();
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi connected");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFi connection failed");
  }

  // Initialize camera
  if (!initCamera()) {
    Serial.println("Camera initialization failed!");
    return;
  }

  Serial.println("Camera initialized successfully");
  delay(2000);

  // Take a few dummy frames
  for(int i = 0; i < 3; i++) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) {
      esp_camera_fb_return(fb);
    }
    delay(100);
  }

  Serial.println("Ready to start classification...");
}

void loop() {
  static uint8_t errorCount = 0;
  static uint32_t frameCount = 0;
  
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    errorCount++;
    if (errorCount >= 3) {
      Serial.println("Too many errors, restarting...");
      ESP.restart();
    }
    delay(1000);
    return;
  }
  errorCount = 0;

  if (fb->len < 1024) {
    Serial.println("Frame too small");
    esp_camera_fb_return(fb);
    delay(1000);
    return;
  }

  frameCount++;
  Serial.printf("\n--- Frame #%u Analysis ---\n", frameCount);
  Serial.printf("Frame size: %u bytes\n", fb->len);
  
  auto features = extractFeatures(fb);
  String prediction = predictLabel(features);

  Serial.println("Final Prediction: " + prediction);
  Serial.println("------------------------\n");

  esp_camera_fb_return(fb);
  delay(2000);
}