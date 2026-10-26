idf_component_register(SRCS "main.c" "drive.c" "obstacle.c" "ir_track.c" "shooter.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES driver esp_adc esp_timer)

