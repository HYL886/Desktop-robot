idf_component_register(SRCS "desktop_robot_main.c"
                            "robot_display.c"
                            "robot_motor.c"
                            "robot_web.c"
                       PRIV_REQUIRES driver esp_event esp_http_server esp_lcd esp_netif esp_timer esp_wifi lwip nvs_flash
                       INCLUDE_DIRS ".")
