idf_component_register(SRCS "mymqtt.c"
                    REQUIRES esp_netif
                    REQUIRES mqtt
                    REQUIRES esp_wifi
                    REQUIRES esp_http_client
                    REQUIRES nvs_flash
                    INCLUDE_DIRS "include")
