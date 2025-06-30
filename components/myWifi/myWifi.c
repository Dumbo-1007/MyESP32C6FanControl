#include "myWifi.h"
#include "esp_http_client.h"
#include <esp_log.h>
#include <cJSON.h>

static const char *TAG = "HTTP_CLIENT";
void wificonfig()
{
    //----------------准备阶段-------------------
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    //----------------初始化阶段-------------------
    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    // 注册事件(wifi启动成功)
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        WIFI_EVENT_STA_START, WIFI_CallBack, NULL, NULL));
    // 注册事件(wifi连接失败)
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        WIFI_EVENT_STA_DISCONNECTED, WIFI_CallBack, NULL, NULL));
    // 注册事件(wifi连接失败)
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP, WIFI_CallBack, NULL, NULL));

    // 初始化STA设备
    esp_netif_create_default_wifi_sta();

    /*Initialize WiFi */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    // WIFI_INIT_CONFIG_DEFAULT 是一个默认配置的宏

    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    //----------------配置阶段-------------------
    // 初始化WIFI设备( 为 WiFi 驱动初始化 WiFi 分配资源，如 WiFi 控制结构、RX/TX 缓冲区、WiFi NVS 结构等，这个 WiFi 也启动 WiFi 任务。必须先调用此API，然后才能调用所有其他WiFi API)
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    // STA详细配置
    wifi_config_t sta_config = {
        .sta = {
            .ssid = ESP_WIFI_STA_SSID,
            .password = ESP_WIFI_STA_PASSWD,
            .bssid_set = false,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_config));

    //----------------启动阶段-------------------
    ESP_ERROR_CHECK(esp_wifi_start());

    //----------------配置省电模式-------------------
    // 不省电(数据传输会更快)
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
}

void send_data_to_server(float temperature, float humidity, float pwm_duty, float lux)
{
    // 1. 创建JSON对象
    cJSON *root = cJSON_CreateObject();
    if (root == NULL)
    {
        ESP_LOGE(TAG, "Failed to create JSON object");
        return;
    }

    // 2. 添加数据到JSON
    cJSON_AddNumberToObject(root, "temp", temperature);
    cJSON_AddNumberToObject(root, "hum", humidity);
    cJSON_AddNumberToObject(root, "pwm", pwm_duty);
    cJSON_AddNumberToObject(root, "lux", lux);
    cJSON_AddStringToObject(root, "device_id", "ESP32C6");

    // 3. 生成JSON字符串
    char *post_data = cJSON_PrintUnformatted(root);
    if (post_data == NULL)
    {
        ESP_LOGE(TAG, "Failed to print JSON");
        cJSON_Delete(root);
        return;
    }

    // 4. 配置HTTP客户端
    esp_http_client_config_t config = {
        .url = "http://192.168.124.113:8080/api/data",
        .method = HTTP_METHOD_POST,
        .timeout_ms = 5000, // 添加超时
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, post_data, strlen(post_data));

    // 5. 执行请求
    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK)
    {
        int status_code = esp_http_client_get_status_code(client);
        ESP_LOGI(TAG, "HTTP Status = %d", status_code);

        if (status_code >= 200 && status_code < 300)
        {
            ESP_LOGI(TAG, "Data sent successfully");
        }
        else
        {
            ESP_LOGW(TAG, "Server returned error status");
        }
    }
    else
    {
        ESP_LOGE(TAG, "HTTP request failed: %s", esp_err_to_name(err));
    }

    // 6. 清理资源
    free(post_data);
    cJSON_Delete(root);
    esp_http_client_cleanup(client);
    vTaskDelay(pdMS_TO_TICKS(2000));
}
void WIFI_CallBack(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    // 连接失败的次数
    static uint8_t connect_count = 0;
    // WIFI 启动成功
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        ESP_LOGI("WIFI_EVENT", "WIFI_EVENT_STA_START");
        ESP_ERROR_CHECK(esp_wifi_connect());
    }
    // WIFI 连接失败
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        ESP_LOGI("WIFI_EVENT", "WIFI_EVENT_STA_DISCONNECTED");
        connect_count++;
        if (connect_count < 6)
        {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            ESP_ERROR_CHECK(esp_wifi_connect());
        }
        else
        {
            ESP_LOGI("WIFI_EVENT", "WIFI_EVENT_STA_DISCONNECTED 10 times");
        }
    }
    // WIFI 连接成功(获取到了IP)
    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ESP_LOGI("WIFI_EVENT", "WIFI_EVENT_STA_GOT_IP");
        ip_event_got_ip_t *info = (ip_event_got_ip_t *)event_data;
        ESP_LOGI("WIFI_EVENT", "got ip:" IPSTR "", IP2STR(&info->ip_info.ip));
    }
}