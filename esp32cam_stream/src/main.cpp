#include "esp_camera.h"
#include <WiFi.h>
#include "esp_http_server.h"

//================ WIFI =================
const char* ssid = "Q3 Pro";
const char* password = "12345678909";

//============ AI THINKER ESP32-CAM ============
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

httpd_handle_t camera_httpd = NULL;

//================ STREAM =================

static const char* STREAM_CONTENT_TYPE =
"multipart/x-mixed-replace;boundary=frame";

static const char* STREAM_BOUNDARY =
"--frame\r\n";

static const char* STREAM_PART =
"Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static esp_err_t stream_handler(httpd_req_t *req)
{
    camera_fb_t *fb = NULL;
    char part_buf[64];
    esp_err_t res = ESP_OK;
    int frame_skip = 0;

    httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");

    while(true)
    {
        fb = esp_camera_fb_get();

        if(!fb)
        {
            delay(5);
            continue;
        }

        // Bỏ qua 2 frame, gửi frame thứ 3 để giảm tải
        if(frame_skip++ < 2) {
            esp_camera_fb_return(fb);
            continue;
        }
        frame_skip = 0;

        size_t hlen = snprintf(
            part_buf,
            sizeof(part_buf),
            STREAM_PART,
            fb->len
        );

        res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
        if(res != ESP_OK) {
            esp_camera_fb_return(fb);
            break;
        }

        res = httpd_resp_send_chunk(req, part_buf, hlen);
        if(res != ESP_OK) {
            esp_camera_fb_return(fb);
            break;
        }

        res = httpd_resp_send_chunk(req, (const char*)fb->buf, fb->len);
        esp_camera_fb_return(fb);
        if(res != ESP_OK) {
            break;
        }

        delay(30);  // Tăng từ 10ms lên 30ms để CPU nghỉ ngơi
    }

    return res;
}

//================ WEB PAGE =================

static esp_err_t index_handler(httpd_req_t *req)
{
    const char* html = R"rawliteral(

<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">

<title>ESP32-CAM LIVE</title>

<style>

body{
    margin:0;
    background:#101010;
    color:white;
    font-family:Arial;
    text-align:center;
}

.header{
    background:#202020;
    padding:15px;
    font-size:28px;
    font-weight:bold;
}

.info{
    margin-top:10px;
    font-size:20px;
}

img{
    width:95vw;
    max-width:1200px;
    border:4px solid #00ff99;
    border-radius:12px;
}

.footer{
    margin-top:10px;
    font-size:18px;
}

</style>

</head>

<body>

<div class="header">
ESP32-CAM LIVE STREAM
</div>

<img src="/stream">

<div class="footer">
Real-Time Video
</div>

</body>
</html>

)rawliteral";

    httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);

    return ESP_OK;
}

//================ SERVER =================

void startCameraServer()
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    httpd_uri_t index_uri =
    {
        .uri = "/",
        .method = HTTP_GET,
        .handler = index_handler,
        .user_ctx = NULL
    };

    httpd_uri_t stream_uri =
    {
        .uri = "/stream",
        .method = HTTP_GET,
        .handler = stream_handler,
        .user_ctx = NULL
    };

    if(httpd_start(&camera_httpd, &config) == ESP_OK)
    {
        httpd_register_uri_handler(camera_httpd, &index_uri);
        httpd_register_uri_handler(camera_httpd, &stream_uri);
    }
}

//================ SETUP =================

void setup()
{
    Serial.begin(115200);

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

    config.xclk_freq_hz = 10000000;  // Giảm từ 20MHz xuống 10MHz để tiết kiệm điện
    config.pixel_format = PIXFORMAT_JPEG;

    if(psramFound())
    {
        // Tối ưu tài nguyên: giảm độ phân giải, tăng nén, giảm số buffer
        config.frame_size = FRAMESIZE_QQVGA;
        config.jpeg_quality = 30;  // Tăng nén (số cao hơn = chất lượng thấp hơn, CPU ít hơn)
        config.fb_count = 1;  // Giảm từ 2 xuống 1 buffer
    }
    else
    {
        config.frame_size = FRAMESIZE_QQVGA;
        config.jpeg_quality = 35;  // Nén cao hơn
        config.fb_count = 1;
    }

    esp_err_t err = esp_camera_init(&config);

    if(err != ESP_OK)
    {
        Serial.printf("Camera Init Failed: 0x%x\n", err);
        return;
    }

    Serial.println("Camera OK");

    WiFi.begin(ssid, password);

    Serial.print("Connecting");

    while(WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }

    Serial.println();
    Serial.println("==========================");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    Serial.println("==========================");

    Serial.print("PSRAM: ");
    Serial.println(psramFound() ? "YES" : "NO");

    startCameraServer();

    Serial.println("Camera Server Started");
}

void loop()
{
    delay(100);
}
