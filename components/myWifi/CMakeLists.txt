idf_component_register(SRCS "myWifi.c"
                    REQUIRES esp_netif
                    REQUIRES esp_wifi
                    REQUIRES esp_http_client
                    INCLUDE_DIRS "include"
                    REQUIRES nvs_flash)
set(COMPONENT_REQUIRES esp_wifi)

