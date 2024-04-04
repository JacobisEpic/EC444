# idf_component_register(SRCS "i2c_oled_example_main.c" "lvgl_demo_ui.c"
#                        INCLUDE_DIRS ".")
idf_component_register(SRCS "oled.c" "bits.c"
                       INCLUDE_DIRS ".")

