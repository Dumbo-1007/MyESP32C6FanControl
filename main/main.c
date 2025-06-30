#include "OLED.h"
#include "dht.h"
#include <driver/gpio.h>
#include <driver/ledc.h>
#include "TMF.h"
#include "mydht11.h"
#include <pid_ctrl.h>
#include "myWifi.h"
#include "lux.h"
#include "cJSON.h"
#include "esp_log.h"
#include <esp_http_client.h>
#include "mymqtt.h"
#include "esp_log.h"
#include "freertos/event_groups.h"

#define DELAY_TIME 3000
#define FAN_MODE_CHANGED_BIT BIT0
static EventGroupHandle_t fan_control_event_group;
// 全局变量保存当前PWM
static float pwm = 0.0f;
#define TAG1 "PID_TEST"
static float temperature = 0.0f;
static float humidity = 0.0f;
float target_temperature = 25.0; // 目标温度
float voltage = 0.0f;
const char *fan_state_to_str(fan_state_t state)
{
  switch (state)
  {
  case FAN_MANUAL_OFF:
    return "MANUAL_OFF";
  case FAN_MANUAL_ON:
    return "MANUAL_ON";
  case FAN_AUTO:
    return "AUTO";
  case FAN_UNKNOWN:
    return "UNKNOWN";
  default:
    return "INVALID";
  }
}
void fuzzy_pid_control(float target_temp);
fan_state_t parse_fan_state(const char *state_str);

fan_state_t parse_fan_state(const char *state_str)
{
  if (strcmp(state_str, "manual_off") == 0)
  {
    return FAN_MANUAL_OFF;
  }
  else if (strcmp(state_str, "manual_on") == 0)
  {
    return FAN_MANUAL_ON;
  }
  else if (strcmp(state_str, "auto") == 0)
  {
    return FAN_AUTO;
  }
  return FAN_UNKNOWN;
}
void control_fan(fan_state_t state)
{
  fan_state1 = state;
  xEventGroupSetBits(fan_control_event_group, FAN_MODE_CHANGED_BIT);
  ESP_LOGI("EVENT", "Set event group bit: FAN_MODE_CHANGED_BIT");
  switch (state)
  {
  case FAN_MANUAL_OFF:
    pwm = 0;
    set_motor_speed(pwm);
    ESP_LOGI("fan", "Fan manually turned OFF");
    break;
  case FAN_MANUAL_ON:
    pwm = 255;
    set_motor_speed(pwm);
    ESP_LOGI("fan", "Fan manually turned ON");
    break;
  case FAN_AUTO:
    // 实现自动控制逻辑，例如基于温度传感器
    fuzzy_pid_control(target_temperature);
    ESP_LOGI("fan", "Fan in AUTO mode");
    break;
  default:
    pwm = 128;
    set_motor_speed(pwm);
    ESP_LOGW("fan", "Unknown fan state");
    break;
  }
}
// 显示温湿度数据
void display_temp_hum(float temperature, float humidity, float output)
{
  OLED_Clear();
  OLED_ShowString(0, 0, "Fuzzy PID", OLED_8X16);
  OLED_ShowString(0, 16, "hum:", OLED_8X16);
  OLED_ShowString(0, 32, "tmp:", OLED_8X16);
  OLED_ShowString(0, 48, "pwm:", OLED_8X16);
  OLED_ShowFloatNum(32, 16, humidity, 2, 2, OLED_8X16);
  OLED_ShowFloatNum(32, 32, temperature, 2, 2, OLED_8X16);
  OLED_ShowFloatNum(35, 48, output, 3, 2, OLED_8X16);
  OLED_Update();
}

void fuzzy_control_task(void *pvParameter)
{
  float current_temperature = 0.0;
  static float last_error = 0.0;

  while (1)
  {
    // 读取温湿度数据
    esp_err_t result = dht_read_float_data(DHT_SENSOR_TYPE,
                                           DHT_SENSOR_PIN, &humidity, &current_temperature);
    if (result == ESP_OK)
    {
      // 计算误差和误差变化
      float error = target_temperature - current_temperature;
      float delta_error = error - last_error;

      // 模糊化
      FuzzyMembership error_membership = fuzzify_error(error);
      FuzzyMembership delta_error_membership = fuzzify_delta_error(delta_error);

      // 模糊推理
      FuzzyMembership control_membership = infer_control(error_membership,
                                                         delta_error_membership);
      // 去模糊化
      float control_output = defuzzify(control_membership);

      // 更新上一次误差
      last_error = error;

      // 显示温湿度和控制输出
      display_temp_hum(current_temperature, humidity, control_output);
      ESP_LOGI("fuzzycontrol",
               "Temp: %.1f, Hum: %.1f, Error: %.1f,Delta Error: %.1f, Control Output: %.1f",
               current_temperature, humidity, error, delta_error, control_output);
      pwm = control_output;
      set_motor_speed((int)control_output);
    }
    else
    {
      ESP_LOGE("dht", "Failed to read data from DHT sensor: %s",
               esp_err_to_name(result));
      OLED_Clear();
      OLED_ShowString(0, 0, "Sensor Error", OLED_8X16);
      OLED_Update();
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void fuzzy_pid_control(float target_temp)
{
  // 静态变量用于保存PID控制块和上一次误差
  static pid_ctrl_block_handle_t pid = NULL;
  static float last_error = 0.0;

  // 初始化PID参数配置
  const pid_ctrl_config_t pid_config = {
      .init_param = {
          .kp = 15.8,
          .ki = 0.1,
          .kd = 0.06,
          .max_output = 255.0,
          .min_output = 0.0,
          .max_integral = 2.0,
          .min_integral = -2.0,
          .cal_type = PID_CAL_TYPE_POSITIONAL}};
  // 如果PID控制块未初始化，则创建新的控制块
  fan_state_t prev_fan_state = fan_state1;
  if (pid_new_control_block(&pid_config, &pid) != ESP_OK)
  {
    ESP_LOGE("PID_CTRL", "Failed to create PID control block");
  }
  while (fan_state1 == FAN_AUTO)
  {
    ESP_LOGE("fid", "currentTemp:%f,target:%f", temperature, target_temp);
    if (pid == NULL)
    {
      pid_new_control_block(&pid_config, &pid);
    }
    // 检测模式是否变化
    EventBits_t bits = xEventGroupWaitBits(
        fan_control_event_group,
        FAN_MODE_CHANGED_BIT,
        pdTRUE,
        pdFALSE,
        pdMS_TO_TICKS(1000));
    ESP_LOGI("EVENT", "WaitBits returns: %u, fan_state1=%d", (unsigned int)bits, fan_state1);
    if (bits & FAN_MODE_CHANGED_BIT && prev_fan_state == FAN_AUTO && fan_state1 != FAN_AUTO)
    {
      ESP_LOGW("EVENT", "Exit AUTO mode");
      break;
    }
    float error = temperature - target_temperature;
    float delta_error = error - last_error;
    // 模糊化
    FuzzyMembership error_memb = fuzzify_error(error);
    FuzzyMembership delta_error_memb = fuzzify_delta_error(delta_error);

    // 模糊推理
    float delta_kp, delta_ki, delta_kd;
    infer_pid(error_memb, delta_error_memb, &delta_kp, &delta_ki, &delta_kd);
    // 更新PID参数
    pid_ctrl_parameter_t new_params;
    new_params.kp = pid_config.init_param.kp + delta_kp;
    new_params.ki = pid_config.init_param.ki + delta_ki;
    new_params.kd = pid_config.init_param.kd + delta_kd;
    new_params.max_output = pid_config.init_param.max_output;
    new_params.min_output = pid_config.init_param.min_output;
    new_params.max_integral = pid_config.init_param.max_integral;
    new_params.min_integral = pid_config.init_param.min_integral;
    new_params.cal_type = pid_config.init_param.cal_type;
    pid_update_parameters(pid, &new_params);
    ESP_LOGI(TAG1, "Delta Kp: %.2f, Delta Ki: %.2f, Delta Kd: %.2f", delta_kp, delta_ki, delta_kd);

    // 计算控制输出
    float control_output;
    if (pid_compute(pid, error, &control_output) == ESP_OK)
    {
      if (control_output > 255.0)
        control_output = 255.0;
      ESP_LOGI(TAG1, "Temp: %.2f, Hum: %.2f, Error: %.2f, Control Output: %.2f", temperature, humidity, error, control_output);
      pwm = control_output;
      set_motor_speed(pwm);
    }
    else
      ESP_LOGE(TAG1, "PID computation failed");
    last_error = error;

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
  pid_del_control_block(pid);
}

void fuzzy_pid_task(void *pvParameter)
{
  float current_temp = 0.0;
  static float last_error = 0.0;
  pid_ctrl_block_handle_t pid;

  // 初始化PID控制器
  pid_ctrl_config_t pid_config = {
      .init_param = {
          .kp = 10.8,          // 比例增益
          .ki = 0.1,           // 积分增益（适当增大以消除稳态误差）
          .kd = 0.06,          // 微分增益（适当增大以减少振荡）
          .max_output = 255.0, // 限制最大输出为255
          .min_output = 0.0,
          .max_integral = 2.0,
          .min_integral = -2.0,
          .cal_type = PID_CAL_TYPE_POSITIONAL}};
  if (pid_new_control_block(&pid_config, &pid) != ESP_OK)
  {
    ESP_LOGE(TAG1, "Failed to create PID control block");
    vTaskDelete(NULL);
  }

  while (1)
  {
    esp_err_t result = dht_read_float_data(DHT_SENSOR_TYPE, DHT_SENSOR_PIN, &humidity, &current_temp);

    if (result == ESP_OK)
    {
      float error = current_temp - target_temperature;
      float delta_error = error - last_error;
      // 模糊化
      FuzzyMembership error_memb = fuzzify_error(error);
      FuzzyMembership delta_error_memb = fuzzify_delta_error(delta_error);

      // 模糊推理
      float delta_kp, delta_ki, delta_kd;
      infer_pid(error_memb, delta_error_memb, &delta_kp, &delta_ki, &delta_kd);
      // 更新PID参数
      pid_ctrl_parameter_t new_params;
      new_params.kp = pid_config.init_param.kp + delta_kp;
      new_params.ki = pid_config.init_param.ki + delta_ki;
      new_params.kd = pid_config.init_param.kd + delta_kd;
      new_params.max_output = pid_config.init_param.max_output;
      new_params.min_output = pid_config.init_param.min_output;
      new_params.max_integral = pid_config.init_param.max_integral;
      new_params.min_integral = pid_config.init_param.min_integral;
      new_params.cal_type = pid_config.init_param.cal_type;
      pid_update_parameters(pid, &new_params);
      ESP_LOGI(TAG1, "Delta Kp: %.2f, Delta Ki: %.2f, Delta Kd: %.2f", delta_kp, delta_ki, delta_kd);

      // 计算控制输出
      float control_output;
      if (pid_compute(pid, error, &control_output) == ESP_OK)
      {
        if (control_output > 255.0)
          control_output = 255.0;
        display_temp_hum(current_temp, humidity, control_output);
        ESP_LOGI(TAG1, "Temp: %.2f, Hum: %.2f, Error: %.2f, Control Output: %.2f", current_temp, humidity, error, control_output);
        if (delta_error != 0)
          pwm = control_output;
        set_motor_speed(control_output);
      }
      else
        ESP_LOGE(TAG1, "PID computation failed");
      last_error = error;
    }
    else
    {
      ESP_LOGE(TAG1, "Failed to read data from DHT sensor: %s",
               esp_err_to_name(result));
      OLED_Clear();
      OLED_ShowString(0, 0, "Sensor Error", OLED_8X16);
      OLED_Update();
    }
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
  pid_del_control_block(pid);
}
// PID控制任务
void pid_control_task(void *pvParameter)
{
  float current_temperature = 0.0;
  pid_ctrl_block_handle_t pid;

  // 初始化PID控制器
  pid_ctrl_config_t pid_config = {
      .init_param = {
          .kp = 15.8,          // 比例增益
          .ki = 0.1,           // 积分增益（适当增大以消除稳态误差）
          .kd = 0.02,          // 微分增益（适当增大以减少振荡）
          .max_output = 255.0, // 限制最大输出为255
          .min_output = 0.0,
          .max_integral = 2.0,
          .min_integral = -2.0,
          .cal_type = PID_CAL_TYPE_POSITIONAL}};
  if (pid_new_control_block(&pid_config, &pid) != ESP_OK)
  {
    ESP_LOGE(TAG1, "Failed to create PID control block");
    vTaskDelete(NULL);
  }

  while (1)
  {
    // 读取温湿度数据
    esp_err_t result = dht_read_float_data(DHT_SENSOR_TYPE,
                                           DHT_SENSOR_PIN, &humidity, &current_temperature);
    if (result == ESP_OK)
    {
      // 确保目标温度不超过45℃
      if (target_temperature > 45.0)
        target_temperature = 45.0;

      // 计算控制输出
      float error = current_temperature - target_temperature;
      float control_output;
      if (pid_compute(pid, error, &control_output) == ESP_OK)
      {
        // 限制电机速度不超过255
        if (control_output > 255.0)
          control_output = 255.0;
        // 显示温湿度和控制输出
        display_temp_hum(current_temperature, humidity, control_output);
        ESP_LOGI(TAG1,
                 "Temp: %.1f, Hum: %.1f, Error: %.1f, Control Output: %.1f",
                 current_temperature, humidity, error, control_output);
        set_motor_speed(control_output);
      }
      else
        ESP_LOGE(TAG1, "PID computation failed");
    }
    else
    {
      ESP_LOGE(TAG1, "Failed to read data from DHT sensor: %s",
               esp_err_to_name(result));
      OLED_Clear();
      OLED_ShowString(0, 0, "Sensor Error", OLED_8X16);
      OLED_Update();
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
  pid_del_control_block(pid);
}

// Ziegler-Nichols调参任务
void ziegler_nichols_tuning_task(void *pvParameter)
{
  float current_temperature = 0.0;
  float humidity = 0.0;
  float Ku = 0.0; // 临界增益
  float Tu = 0.0; // 振荡周期
  float last_error = 0.0;
  float error = 0.0;
  float control_output = 0.0;
  TickType_t last_tick = xTaskGetTickCount();

  pid_ctrl_block_handle_t pid;
  pid_ctrl_config_t pid_config = {
      .init_param = {
          .kp = 0.0, // 初始增益为0
          .ki = 0.0,
          .kd = 0.0,
          .max_output = 255.0,
          .min_output = 0.0,
          .max_integral = 50.0,
          .min_integral = -50.0,
          .cal_type = PID_CAL_TYPE_POSITIONAL}};

  if (pid_new_control_block(&pid_config, &pid) != ESP_OK)
  {
    ESP_LOGE(TAG1, "Failed to create PID control block");
    vTaskDelete(NULL);
  }

  while (Ku == 0.0 || Tu == 0.0)
  {
    esp_err_t result = dht_read_float_data(DHT_SENSOR_TYPE, DHT_SENSOR_PIN, &humidity, &current_temperature);
    if (result == ESP_OK)
    {
      display_temp_hum(current_temperature, humidity, control_output);
      ESP_LOGI(TAG1,
               "Temp: %.1f, Hum: %.1f, Error: %.1f, Control Output: %.1f",
               current_temperature, humidity, error, control_output);
      error = target_temperature - current_temperature;
      pid_update_parameters(pid, &(pid_ctrl_parameter_t){.kp = pid_config.init_param.kp, .ki = 0.0, .kd = 0.0});
      pid_compute(pid, error, &control_output);

      set_motor_speed((int)control_output);

      if (fabs(error - last_error) > 0.01) // 检测振荡
      {
        TickType_t current_tick = xTaskGetTickCount();
        Tu = (current_tick - last_tick) * portTICK_PERIOD_MS / 1000.0; // 计算周期
        last_tick = current_tick;
        Ku = pid_config.init_param.kp;   // 记录临界增益
        pid_config.init_param.kp += 0.1; // 增大增益
      }

      last_error = error;
    }
    else
    {
      ESP_LOGE(TAG1, "Failed to read data from DHT sensor: %s", esp_err_to_name(result));
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }

  ESP_LOGI(TAG1, "Ku: %.2f, Tu: %.2f", Ku, Tu);

  // 根据Ziegler-Nichols公式计算PID参数
  pid_update_parameters(pid, &(pid_ctrl_parameter_t){
                                 .kp = 0.6 * Ku,
                                 .ki = 2 * (0.6 * Ku) / Tu,
                                 .kd = (0.6 * Ku) * Tu / 8,
                                 .max_output = 100.0,
                                 .min_output = 0.0,
                                 .max_integral = 50.0,
                                 .min_integral = -50.0,
                                 .cal_type = PID_CAL_TYPE_POSITIONAL});

  ESP_LOGI(TAG1, "PID Parameters: Kp: %.2f, Ki: %.2f, Kd: %.2f", 0.6 * Ku, 2 * (0.6 * Ku) / Tu, (0.6 * Ku) * Tu / 8);

  pid_del_control_block(pid);
  vTaskDelete(NULL);
}

// 数据检测任务
void sensor_task(void *pvParameters)
{
  while (1)
  {
    voltage = lux_read();
    voltage = lux_read();
    dht_read_float_data(DHT_SENSOR_TYPE, DHT_SENSOR_PIN, &humidity, &temperature);

    display_temp_hum(temperature, humidity, pwm);
    
    ESP_LOGI("TASK", "Temp: %.2f, Hum: %.2f, Lux: %.2f", temperature, humidity, voltage);
    if (voltage != 0)
      send_data_to_server(temperature, humidity, pwm, voltage); // 发送到服务器
    ESP_LOGI("SERVER_TASK", "数据已发送: Temp=%.2f, Hum=%.2f, PWM=%.2f, Voltage=%.2f,fanstate:%s",
             temperature, humidity, pwm, voltage, fan_state_to_str(fan_state1));

    vTaskDelay(pdMS_TO_TICKS(3500)); // 每秒检测一次
  }
}

// 获取服务器上的控制指令
void fetch_control_status_task(void *pvParameters)
{
  const esp_http_client_config_t config = {
      .url = "http://192.168.124.113/api/control",
      .method = HTTP_METHOD_GET,
      .timeout_ms = 15000, // 增加超时时间至15秒
      .event_handler = NULL,
      .buffer_size = 1024,
      .skip_cert_common_name_check = true,
      .keep_alive_enable = true,
      .transport_type = HTTP_TRANSPORT_OVER_TCP,
      .is_async = false,
  };

  while (1)
  {
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client)
    {
      ESP_LOGE("http", "Failed to initialize HTTP client");
      vTaskDelay(pdMS_TO_TICKS(2000));
      continue;
    }

    // 添加自定义请求头
    esp_http_client_set_header(client, "Host", "192.168.219.113:8080");
    esp_http_client_set_header(client, "Accept", "*/*");
    esp_http_client_set_header(client, "User-Agent", "ESP32-HTTP-Client");

    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK)
    {
      int status_code = esp_http_client_get_status_code(client);
      ESP_LOGI("http", "HTTP Status Code: %d", status_code);

      if (status_code == 200)
      {
        int content_length = esp_http_client_get_content_length(client);
        char *response = NULL;

        if (content_length > 0)
        {
          response = malloc(content_length + 1);
          if (response != NULL)
          {
            int total_read = 0;
            int read_bytes;

            while (total_read < content_length &&
                   (read_bytes = esp_http_client_read_response(client, response + total_read, content_length - total_read)) > 0)
            {
              total_read += read_bytes;
            }

            response[total_read] = '\0';
            ESP_LOGI("http", "Raw Response: %s", response);

            cJSON *root = cJSON_Parse(response);
            if (root != NULL)
            {
              cJSON *fan = cJSON_GetObjectItem(root, "fan");
              if (fan && cJSON_IsString(fan))
              {
                ESP_LOGI("http", "New fan state: %s", fan->valuestring);
                control_fan(parse_fan_state(fan->valuestring));
              }
              else
              {
                ESP_LOGW("http", "Missing or invalid 'fan' field in JSON");
              }
              cJSON_Delete(root);
            }
            else
            {
              ESP_LOGE("http", "JSON parse failed");
            }

            free(response);
          }
          else
          {
            ESP_LOGE("http", "Failed to allocate memory for response buffer");
          }
        }
        else
        {
          ESP_LOGW("http", "Content length is 0 or chunked encoding used, trying chunked read...");

#define CHUNK_BUFFER_SIZE 128
          char chunk_buffer[CHUNK_BUFFER_SIZE];
          int total_read = 0;
          int read_bytes;

          while ((read_bytes = esp_http_client_read_response(client, chunk_buffer, CHUNK_BUFFER_SIZE - 1)) > 0)
          {
            chunk_buffer[read_bytes] = '\0';
            ESP_LOGI("http", "Chunked data (%d bytes): %s", read_bytes, chunk_buffer);
            total_read += read_bytes;
          }

          if (total_read == 0)
          {
            ESP_LOGW("http", "No data received from server (chunked)");
          }
        }
      }
      else
      {
        ESP_LOGE("http", "Unexpected HTTP status code: %d", status_code);
      }
    }
    else
    {
      ESP_LOGE("http", "HTTP request failed: %s", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
    vTaskDelay(pdMS_TO_TICKS(5000)); // 每 5 秒请求一次
  }
}

void mqtt_task(void *pvParameter)
{
  ESP_LOGI("MQTT_TASK", "Starting MQTT task");
  mqtt_app_start();

  while (1)
  {
    control_fan(fan_state1);
    ESP_LOGI("MQTT_TASK", "MQTT Task is alive");
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}
void app_main(void)
{
  l9110_pwm_init();
  OLED_Init(OLED_I2C, OLED_ADD, OLED_SCL, OLED_SDA, OLED_SPEED);
  // 设置 DHT11 引脚为上拉模式
  // gpio_set_pull_mode(DHT_SENSOR_PIN, GPIO_PULLUP_ONLY);
  wificonfig();
  lux_init();
  // 初始化事件组
  fan_control_event_group = xEventGroupCreate();

  xTaskCreate(sensor_task, "server_task", 4096, NULL, 5, NULL);
  xTaskCreate(mqtt_task, "ms", 4096, NULL, 5, NULL);
  // xTaskCreate(fuzzy_pid_task, "FuzzyPIDTask", 4096, NULL, 5, NULL);
}