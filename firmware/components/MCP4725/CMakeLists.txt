# MP4201/CMakeLists.txt

idf_component_register(SRCS "MCP4725.c" 
                       INCLUDE_DIRS "include"
                       REQUIRES driver) # 依赖 ESP-IDF 的 driver 组件以使用 I2C