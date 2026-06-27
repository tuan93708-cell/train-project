#include "esp_camera.h"
#include <WiFi.h>
#include "esp_http_server.h"

//================ WIFI =================
const char* ssid     = "Q3 Pro";
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
"\r\n--frame\r\n";

static const char* STREAM_PART =
"Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static esp_err_t stream_handler(httpd_req_t *req)
{
    camera_fb_t *fb  = NULL;
    char        part_buf[64];
    esp_err_t   res  = ESP_OK;

    res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
    if(res != ESP_OK) return res;

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache, must-revalidate");
    httpd_resp_set_hdr(req, "X-Framerate", "25");

    // Bỏ qua phần warm-up vì có thể gây lỗi trên một số camera clone

    while(true)
    {
        fb = esp_camera_fb_get();
        if(!fb)
        {
            // Camera bận: chờ 10ms rồi thử lại, không spin-loop gây nóng CPU
            vTaskDelay(10 / portTICK_PERIOD_MS);
            continue;
        }

        size_t hlen = snprintf(part_buf, sizeof(part_buf), STREAM_PART, fb->len);

        res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
        if(res == ESP_OK)
            res = httpd_resp_send_chunk(req, part_buf, hlen);
        if(res == ESP_OK)
            res = httpd_resp_send_chunk(req, (const char*)fb->buf, fb->len);

        esp_camera_fb_return(fb);
        fb = NULL;

        if(res != ESP_OK)
        {
            // Client ngắt kết nối: dừng stream, giải phóng task -> giảm tải CPU, giảm nhiệt
            break;
        }

        // === GIẢM NHIỆT: delay 2ms thay vì 1ms ===
        // 2ms = ~20-25 FPS thực tế, CPU có thời gian idle -> mát hơn đáng kể
        // 1ms trước đó khiến CPU chạy gần 100% liên tục
        vTaskDelay(2 / portTICK_PERIOD_MS);
    }

    return res;
}

//================ WEB PAGE =================

static esp_err_t index_handler(httpd_req_t *req)
{
    const char* html = R"rawliteral(
<!DOCTYPE html>
<html lang="vi">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Hệ Thống Giám Sát Đường Ngang</title>
<style>
    * { margin: 0; padding: 0; box-sizing: border-box; }
    body {
        background: #0d1117;
        color: #c9d1d9;
        font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
        display: flex;
        flex-direction: column;
        height: 100vh;
        overflow: hidden;
    }
    .top-bar {
        background: #161b22;
        padding: 10px 20px;
        display: flex;
        justify-content: space-between;
        align-items: center;
        border-bottom: 1px solid #30363d;
        flex-shrink: 0;
    }
    .top-bar h1 {
        font-size: 17px;
        color: #58a6ff;
        text-transform: uppercase;
        letter-spacing: 1px;
    }
    .status-badge {
        background: #238636;
        color: white;
        padding: 4px 10px;
        border-radius: 20px;
        font-size: 12px;
        font-weight: bold;
        display: flex;
        align-items: center;
        gap: 6px;
    }
    .status-badge.offline { background: #da3633; }
    .dot {
        width: 7px; height: 7px;
        background: #fff;
        border-radius: 50%;
        animation: blink 1.5s infinite;
    }
    @keyframes blink { 50% { opacity: 0.2; } }

    .container {
        flex: 1;
        display: flex;
        justify-content: center;
        align-items: center;
        padding: 12px;
    }
    .video-panel {
        width: 100%;
        max-width: 900px;
        background: #000;
        border: 1px solid #30363d;
        border-radius: 8px;
        overflow: hidden;
        position: relative;
        box-shadow: 0 8px 32px rgba(0,0,0,0.7);
    }
    .panel-header {
        background: #21262d;
        padding: 8px 14px;
        font-size: 12px;
        border-bottom: 1px solid #30363d;
        display: flex;
        justify-content: space-between;
        align-items: center;
    }
    /* Dùng thẻ img trực tiếp với src=/stream để giữ kết nối MJPEG liên tục,
       tránh re-request gây chớp hình */
    .video-panel img {
        width: 100%;
        height: auto;
        display: block;
        aspect-ratio: 4/3;
        object-fit: contain;
        background: #000;
    }
    .overlay {
        position: absolute;
        top: 40px; left: 12px;
        color: #00ff41;
        font-family: monospace;
        font-size: 12px;
        text-shadow: 0 0 4px rgba(0,0,0,0.9);
        pointer-events: none;
        display: flex;
        flex-direction: column;
        gap: 3px;
    }
    .rec {
        display: flex; align-items: center; gap: 5px; font-weight: bold;
    }
    .rec-dot {
        width: 8px; height: 8px;
        background: #ff0000;
        border-radius: 50%;
        animation: blink 1s infinite;
    }
</style>
</head>
<body>

<div class="top-bar">
    <h1>Giám Sát Đường Ngang</h1>
    <div class="status-badge" id="badge">
        <div class="dot"></div> LIVE
    </div>
</div>

<div class="container">
    <div class="video-panel">
        <div class="panel-header">
            <span>CAM-01 | Đường Ngang Km 12+345</span>
            <span id="dt">--:--:--</span>
        </div>
        <div class="overlay">
            <div class="rec"><span class="rec-dot"></span>REC</div>
        </div>
        <img id="stream" src="/stream"
             onerror="onStreamError()">
    </div>
</div>

<script>
    setInterval(() => {
        document.getElementById('dt').innerText = new Date().toLocaleTimeString('vi-VN');
    }, 1000);

    // --- Reconnect thông minh: debounce 3s để tránh spam request gây nóng ESP ---
    let reconnectTimer = null;
    function onStreamError() {
        if(reconnectTimer) return;
        const badge = document.getElementById('badge');
        badge.className = 'status-badge offline';
        badge.innerHTML = 'OFFLINE';
        reconnectTimer = setTimeout(() => {
            const img = document.getElementById('stream');
            img.src = '/stream?' + Date.now();
            badge.className = 'status-badge';
            badge.innerHTML = '<div class="dot"></div> LIVE';
            reconnectTimer = null;
        }, 3000);
    }
</script>

</body>
</html>
)rawliteral";

    httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

//================ SERVER =================

void startCameraServer()
{
    httpd_config_t config   = HTTPD_DEFAULT_CONFIG();
    config.server_port      = 80;
    config.max_open_sockets = 4;  // Tối đa 4 kết nối đồng thời, tránh chiếm RAM

    httpd_uri_t index_uri = {
        .uri      = "/",
        .method   = HTTP_GET,
        .handler  = index_handler,
        .user_ctx = NULL
    };

    httpd_uri_t stream_uri = {
        .uri      = "/stream",
        .method   = HTTP_GET,
        .handler  = stream_handler,
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

    // 10MHz - 16MHz: giúp camera chạy ổn định hơn, ít nóng và đơ hơn so với 20MHz
    config.xclk_freq_hz = 16000000;
    config.pixel_format = PIXFORMAT_JPEG;

    if(psramFound())
    {
        config.frame_size   = FRAMESIZE_QVGA;
        config.jpeg_quality = 30; // Nén ảnh mạnh hơn (số càng cao size càng nhỏ), giảm băng thông WiFi
        config.fb_count     = 1;  // 1 buffer giúp gửi ảnh đi ngay lập tức, giảm hẳn độ trễ (delay/lag) so với 2 buffer
    }
    else
    {
        config.frame_size   = FRAMESIZE_QQVGA;
        config.jpeg_quality = 30;
        config.fb_count     = 1;
    }

    esp_err_t err = esp_camera_init(&config);
    if(err != ESP_OK)
    {
        Serial.printf("Camera Init Failed: 0x%x\n", err);
        return;
    }

    // =============== TỐI ƯU CẢM BIẾN OV2640 ===============
    sensor_t *s = esp_camera_sensor_get();
    if(s)
    {
        // --- Phơi sáng & màu sắc ---
        s->set_whitebal(s, 1);          // Cân bằng trắng tự động (AWB)
        s->set_awb_gain(s, 1);          // Bật AWB Gain
        s->set_wb_mode(s, 0);           // Chế độ AWB: 0=Auto (tốt nhất cho ngoài trời)
        s->set_exposure_ctrl(s, 1);     // Phơi sáng tự động (AEC)
        s->set_aec2(s, 1);              // AEC DSP: cải thiện phơi sáng ở vùng tối
        s->set_ae_level(s, 0);          // Mức phơi sáng: 0 = chuẩn (không over/under)
        s->set_gain_ctrl(s, 1);         // Tự động điều chỉnh gain (AGC)
        s->set_gainceiling(s, GAINCEILING_4X); // Giới hạn gain = 4X: đủ sáng, KHÔNG gây chớp sáng đột ngột
        s->set_brightness(s, 0);        // Độ sáng = 0 (trung tính, tự động xử lý qua AEC)
        s->set_contrast(s, 1);          // Tương phản +1: ảnh rõ nét, không bị xỉn màu
        // --- Chất lượng ảnh ---
        s->set_sharpness(s, 1);         // Sắc nét +1: chống nhòe nhẹ
        s->set_denoise(s, 1);           // Lọc nhiễu ảnh: giảm rỗ hạt khi thiếu sáng
        // --- Tắt các hiệu ứng thừa làm nặng CPU ---
        s->set_colorbar(s, 0);          // Tắt color bar test
        s->set_special_effect(s, 0);    // Không dùng hiệu ứng màu
        s->set_hmirror(s, 0);           // Không lật ngang
        s->set_vflip(s, 0);             // Không lật dọc
    }

    Serial.println("Camera OK");

    // =============== TỐI ƯU WIFI ===============
    WiFi.begin(ssid, password);
    WiFi.setSleep(false);                      // Tắt Power Save: WiFi luôn sẵn sàng, giảm lag

    // Bỏ giới hạn công suất phát WiFi (ESP32 sẽ tự dùng công suất tối đa) để sóng mạnh nhất
    // WiFi.setTxPower(WIFI_POWER_15dBm);

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
    // Không làm gì trong loop, mọi xử lý stream chạy trong FreeRTOS task của HTTP server
    vTaskDelay(1000 / portTICK_PERIOD_MS);
}