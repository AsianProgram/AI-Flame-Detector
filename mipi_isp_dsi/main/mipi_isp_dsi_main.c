#include <stdio.h> 
#include <stdlib.h> 
#include <string.h> 
#include "sdkconfig.h" 
#include "esp_attr.h" 
#include "esp_log.h" 
#include "freertos/FreeRTOS.h" 
#include "esp_lcd_mipi_dsi.h" 
#include "esp_lcd_panel_ops.h" 
#include "esp_ldo_regulator.h" 
#include "esp_cache.h" 
#include "driver/i2c_master.h" 
#include "driver/isp.h" 
#include "esp_cam_ctlr_csi.h" 
#include "esp_cam_ctlr.h" 
#include "example_dsi_init.h" 
#include "example_dsi_init_config.h" 
#include "example_sensor_init.h" 
#include "example_config.h" 
#include "ai_wrapper.h" 
#include "freertos/task.h" 
#include "freertos/semphr.h" 

#define ENABLE_DRAW_DISPLAY 1

static const char *TAG = "mipi_isp_dsi"; 
static uint32_t finished_trans_counter = 0; 
static bool s_camera_get_new_vb(esp_cam_ctlr_handle_t handle, esp_cam_ctlr_trans_t *trans, void *user_data); 
static bool s_camera_get_finished_trans(esp_cam_ctlr_handle_t handle, esp_cam_ctlr_trans_t *trans, void *user_data); 

void timer_init(); 
void timer_callback(TimerHandle_t xTimer); 
void camera_task(void *pvParameters); 
void ai_task(void *pvParameters); 

//void draw_box_rgb565(uint16_t *fb, int width, int height, int x1, int y1, int x2, int y2, uint16_t color); 
void draw_box_rgb565(uint16_t *fb, int width, int height, int x1, int y1, int x2, int y2, uint16_t color, int thickness); 
void draw_dot_rgb565(uint16_t *fb, int width, int height, int x, int y, uint16_t color, int size); 

TimerHandle_t timer1_handle; 
StaticTimer_t timer1_buffer; 
esp_cam_ctlr_handle_t cam_handle = NULL; 

esp_cam_ctlr_trans_t new_trans = { 
    .buffer = NULL, 
    .buflen = 0, 
}; 

static volatile int grab = 0; 
static volatile int infer = 0; 
void *last_frame = NULL; 
void *frame_buffer = NULL; 
size_t frame_buffer_size = 0; 

TaskHandle_t app_main_handle = NULL; 
TaskHandle_t cam_task_handle = NULL; 
TaskHandle_t ai_task_handle = NULL; 

void *display_fb = NULL; 
detection_t detectRes = {}; 

void app_main(void) { 
    app_main_handle = xTaskGetCurrentTaskHandle(); 
    esp_err_t ret = ESP_FAIL; 
    esp_lcd_dsi_bus_handle_t mipi_dsi_bus = NULL; 
    esp_lcd_panel_io_handle_t mipi_dbi_io = NULL; 
    esp_lcd_panel_handle_t mipi_dpi_panel = NULL; 
    
    //mipi ldo 
    esp_ldo_channel_handle_t ldo_mipi_phy = NULL; 
    esp_ldo_channel_config_t ldo_mipi_phy_config = { 
        .chan_id = CONFIG_EXAMPLE_USED_LDO_CHAN_ID, 
        .voltage_mv = CONFIG_EXAMPLE_USED_LDO_VOLTAGE_MV, 
    }; 
    ESP_ERROR_CHECK(esp_ldo_acquire_channel(&ldo_mipi_phy_config, &ldo_mipi_phy)); 
    
    /** * @background * Sensor use RAW8 * ISP convert to RGB565 */ 
    //---------------DSI Init------------------// 
    //example_dsi_resource_alloc(&mipi_dsi_bus, &mipi_dbi_io, &mipi_dpi_panel, &frame_buffer); 
    example_dsi_resource_alloc(&mipi_dsi_bus, &mipi_dbi_io, &mipi_dpi_panel); 
    
    //---------------Necessary variable config------------------// 
    frame_buffer_size = CONFIG_EXAMPLE_MIPI_CSI_DISP_HRES * CONFIG_EXAMPLE_MIPI_DSI_DISP_VRES * EXAMPLE_RGB565_BITS_PER_PIXEL / 8; 
    
    //ESP_LOGD(TAG, "CONFIG_EXAMPLE_MIPI_CSI_DISP_HRES: %d, CONFIG_EXAMPLE_MIPI_DSI_DISP_VRES: %d, bits per pixel: %d", CONFIG_EXAMPLE_MIPI_CSI_DISP_HRES, CONFIG_EXAMPLE_MIPI_DSI_DISP_VRES, EXAMPLE_RGB565_BITS_PER_PIXEL); 
    //ESP_LOGD(TAG, "frame_buffer_size: %zu", frame_buffer_size); 
    //ESP_LOGD(TAG, "frame_buffer: %p", frame_buffer); 

    // frame_buffer = heap_caps_malloc(frame_buffer_size, MALLOC_CAP_DMA | MALLOC_CAP_SPIRAM); 
    // if (!frame_buffer) { 
    // ESP_LOGE(TAG, "Failed to allocate DMA-capable framebuffer"); 
    // return; 
    // } 
    
    
    //ESP_ERROR_CHECK(esp_lcd_dpi_panel_get_frame_buffer(mipi_dpi_panel, 1, &frame_buffer)); 
    
    //frame_buffer = display_fb; 
    //ESP_LOGI(TAG, "Display framebuffer: %p", frame_buffer); 
    frame_buffer = heap_caps_malloc(frame_buffer_size, MALLOC_CAP_DMA | MALLOC_CAP_SPIRAM);
    assert(frame_buffer);
    last_frame = heap_caps_malloc(frame_buffer_size, MALLOC_CAP_DMA | MALLOC_CAP_SPIRAM); 
    assert(last_frame); 

    memset(frame_buffer, 0xFF, frame_buffer_size); 
    esp_cache_msync(frame_buffer, frame_buffer_size, ESP_CACHE_MSYNC_FLAG_DIR_C2M); 

    ESP_ERROR_CHECK(esp_lcd_dpi_panel_get_frame_buffer(mipi_dpi_panel, 1, &frame_buffer)); 
    ESP_LOGI(TAG, "Display framebuffer: %p", frame_buffer); 
    
    //---------------DPI Reset------------------// 
    example_dpi_panel_reset(mipi_dpi_panel); 
    example_dpi_panel_init(mipi_dpi_panel); 
    
    //--------Camera Sensor and SCCB Init-----------// 
    example_sensor_handle_t sensor_handle = { 
        .sccb_handle = NULL, 
        .i2c_bus_handle = NULL, 
    }; 
    
    example_sensor_config_t cam_sensor_config = { 
        .i2c_port_num = I2C_NUM_0, 
        .i2c_sda_io_num = EXAMPLE_MIPI_CSI_CAM_SCCB_SDA_IO, 
        .i2c_scl_io_num = EXAMPLE_MIPI_CSI_CAM_SCCB_SCL_IO, 
        .port = ESP_CAM_SENSOR_MIPI_CSI, 
        .format_name = EXAMPLE_CAM_FORMAT, 
    }; 
    
    example_sensor_init(&cam_sensor_config, &sensor_handle); 
    
    //---------------CSI Init------------------// 
    new_trans.buffer = frame_buffer; 
    new_trans.buflen = frame_buffer_size; 
    
    esp_cam_ctlr_csi_config_t csi_config = { 
        .ctlr_id = 0, 
        .h_res = CONFIG_EXAMPLE_MIPI_CSI_DISP_HRES, 
        .v_res = CONFIG_EXAMPLE_MIPI_CSI_DISP_VRES, 
        .lane_bit_rate_mbps = EXAMPLE_MIPI_CSI_LANE_BITRATE_MBPS, 
        .input_data_color_type = CAM_CTLR_COLOR_RAW10, //CAM_CTLR_COLOR_RAW8 
        .output_data_color_type = CAM_CTLR_COLOR_RGB565, 
        .data_lane_num = 2, 
        .byte_swap_en = false, 
        .queue_items = 1, 
    }; 

    cam_handle = NULL; 
    ret = esp_cam_new_csi_ctlr(&csi_config, &cam_handle); 
    
    if (ret != ESP_OK) { 
        ESP_LOGE(TAG, "csi init fail[%d]", ret); 
        return; 
    } 
    
    esp_cam_ctlr_evt_cbs_t cbs = { 
        .on_get_new_trans = s_camera_get_new_vb, 
        .on_trans_finished = s_camera_get_finished_trans, 
    }; 
    
    ESP_ERROR_CHECK(esp_cam_ctlr_register_event_callbacks(cam_handle, &cbs, &new_trans)); 
    ESP_ERROR_CHECK(esp_cam_ctlr_enable(cam_handle)); 
    
    //---------------ISP Init------------------// 
    isp_proc_handle_t isp_proc = NULL; 
    esp_isp_processor_cfg_t isp_config = { 
        .clk_hz = 80 * 1000 * 1000, 
        .input_data_source = ISP_INPUT_DATA_SOURCE_CSI, 
        .input_data_color_type = ISP_COLOR_RAW8, 
        .output_data_color_type = ISP_COLOR_RGB565, 
        .has_line_start_packet = false, 
        .has_line_end_packet = false, 
        .h_res = CONFIG_EXAMPLE_MIPI_CSI_DISP_HRES, 
        .v_res = CONFIG_EXAMPLE_MIPI_CSI_DISP_VRES, 
        .bayer_order = COLOR_RAW_ELEMENT_ORDER_GBRG, 
        .flags = { 
            .bypass_isp = 0, 
            .byte_swap_en = 0, 
        } 
    }; 
    
    ESP_ERROR_CHECK(esp_isp_new_processor(&isp_config, &isp_proc)); 
    ESP_ERROR_CHECK(esp_isp_enable(isp_proc)); 
    ESP_ERROR_CHECK(esp_cam_ctlr_start(cam_handle)); 
    
    xTaskCreatePinnedToCore(camera_task, "camera_task", 32768, NULL, 5, &cam_task_handle, 0); 
    xTaskCreatePinnedToCore(ai_task, "ai_task", 32768, NULL, 5, &ai_task_handle, 0); 

    ai_init(); 
    timer_init(); 
    xTimerStart(timer1_handle, portMAX_DELAY); 
    
    ESP_LOGI(TAG, "frame_buffer ptr: %p", frame_buffer); 
    ESP_LOGI(TAG, "last_frame ptr: %p", last_frame); 
    
    if (cam_task_handle) { 
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); 
    } 
    
    if (ENABLE_DRAW_DISPLAY) { 
        if (detectRes.score > 0.1) { 
            int cx = (detectRes.x1 + detectRes.x2) / 2; 
            int cy = (detectRes.y1 + detectRes.y2) / 2; 
            
            draw_box_rgb565(
                (uint16_t *)last_frame, 
                CONFIG_EXAMPLE_MIPI_DSI_DISP_HRES, 
                CONFIG_EXAMPLE_MIPI_DSI_DISP_VRES, 
                detectRes.x1, 
                detectRes.y1, 
                detectRes.x2, 
                detectRes.y2, 
                0x07E0, 
                10
            ); 

            draw_dot_rgb565(
                (uint16_t *)last_frame, 
                CONFIG_EXAMPLE_MIPI_DSI_DISP_HRES, 
                CONFIG_EXAMPLE_MIPI_DSI_DISP_VRES, 
                cx, 
                cy, 
                0x001F, 
                5
            ); 

            memcpy(frame_buffer, last_frame, frame_buffer_size); 
            esp_cache_msync(frame_buffer, frame_buffer_size, ESP_CACHE_MSYNC_FLAG_DIR_C2M); 
        
        } 
    } 
} 

static bool s_camera_get_new_vb(esp_cam_ctlr_handle_t handle, esp_cam_ctlr_trans_t *trans, void *user_data) { 
    esp_cam_ctlr_trans_t new_trans = *(esp_cam_ctlr_trans_t *)user_data; 
    trans->buffer = new_trans.buffer; 
    trans->buflen = new_trans.buflen; 
    //ESP_LOGI(TAG, "CSI writing to buffer: %p", trans->buffer); 
    return false; 
} 

static bool s_camera_get_finished_trans(esp_cam_ctlr_handle_t handle, esp_cam_ctlr_trans_t *trans, void *user_data) { 
    finished_trans_counter++; 
    return false; 
} 

//start timer for capture 
void timer_init() { 
    timer1_handle = xTimerCreateStatic("Timer1", pdMS_TO_TICKS(5000), pdFALSE, NULL, timer_callback, &timer1_buffer); 
} 

void timer_callback(TimerHandle_t xTimer) { 
    ESP_LOGI(TAG, "Timer callback executed"); 
    grab = 1; 
} 

//task to capture frame and stop camera     
void camera_task(void *pvParameters) { 
    while (!grab) { 
        esp_err_t err = esp_cam_ctlr_receive(cam_handle, &new_trans, 100); 
        if (err == ESP_OK) { 
            // frame arrived 
            esp_cache_msync(frame_buffer, frame_buffer_size, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
        } 
        else if (err != ESP_ERR_TIMEOUT) { 
            ESP_LOGE(TAG, "Camera receive error: %s", esp_err_to_name(err)); 
        } 
    } 
    
    ESP_LOGI(TAG, "Stopping camera"); 
    ESP_ERROR_CHECK(esp_cam_ctlr_stop(cam_handle)); 
    vTaskDelay(pdMS_TO_TICKS(10)); 
    ESP_LOGI(TAG, "Freezing last frame"); 
    
    // Freeze frame safely 
    memcpy(last_frame, frame_buffer, frame_buffer_size); 
    esp_cache_msync(last_frame, frame_buffer_size, ESP_CACHE_MSYNC_FLAG_DIR_C2M); 

    infer = 1; 
    xTaskNotifyGive(ai_task_handle); 

    vTaskDelete(NULL); 
} 

//task where inference is done and result is stored 
void ai_task(void *pvParameters) { 
    while (1) { 
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); 
        if (infer == 1) { 
            ESP_LOGI(TAG, "Starting inference"); 
            ai_run(last_frame, CONFIG_EXAMPLE_MIPI_CSI_DISP_HRES, CONFIG_EXAMPLE_MIPI_CSI_DISP_VRES, &detectRes); 
            ESP_LOGI(TAG, "Inference completed."); 
            ESP_LOGI("AI", "cat=%d score=%.2f box=[%d,%d,%d,%d]", detectRes.category, detectRes.score, detectRes.x1, detectRes.y1, detectRes.x2, detectRes.y2); 
            
            xTaskNotifyGive(app_main_handle); 
            vTaskDelete(NULL); 
        } 
    } 
} 
    
void draw_box_rgb565(uint16_t *fb, int width, int height, int x1, int y1, int x2, int y2, uint16_t color, int thickness) { 
    if (x1 < 0) x1 = 0; 
    if (y1 < 0) y1 = 0; 
    if (x2 >= width) x2 = width - 1; 
    if (y2 >= height) y2 = height - 1; 
    
    for (int t = 0; t < thickness; t++) { 
        // Top + Bottom 
        for (int x = x1; x <= x2; x++) { 
            fb[(y1 + t) * width + x] = color; 
            fb[(y2 - t) * width + x] = color; 
        } 
        
        // Left + Right 
        for (int y = y1; y <= y2; y++) { 
            fb[y * width + (x1 + t)] = color; 
            fb[y * width + (x2 - t)] = color; 
        } 
    } 
} 

void draw_dot_rgb565(uint16_t *fb, int width, int height, int x, int y, uint16_t color, int size) { 
    for (int dy = -size; dy <= size; dy++) { 
        for (int dx = -size; dx <= size; dx++) { 
            int px = x + dx; 
            int py = y + dy; 
            
            if (px >= 0 && px < width && py >= 0 && py < height) { 
                fb[py * width + px] = color; 
            } 
        } 
    } 
}