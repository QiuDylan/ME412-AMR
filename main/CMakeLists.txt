idf_component_register(SRCS "main.c" "drive.c" "avoid.c" "ir_tracker.c" "align.c" "shoot.c" "return_home.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES driver esp_adc esp_timer)
