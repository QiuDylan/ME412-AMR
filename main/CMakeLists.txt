idf_component_register(SRCS "main.c" "drive.c" "avoid.c" "map.c" "ir_tracker.c" "align.c" "shoot.c" "return_home.c" "yahboom.c" "ir_demo.c" "mpu6050.c" "netlog.c" "vl53l0x.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES driver esp_adc esp_timer esp_wifi esp_netif esp_event nvs_flash lwip)
