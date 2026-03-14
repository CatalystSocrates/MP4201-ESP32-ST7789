# #all c file in this directory
# file(GLOB SRCS "*.c")

# #register component
# idf_component_register(
#     SRCS ${SRCS}
#     INCLUDE_DIRS "."
#     REQUIRES driver
#              esp_lcd
# )
idf_component_register(
    SRCS "st7789_driver.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_lcd
)