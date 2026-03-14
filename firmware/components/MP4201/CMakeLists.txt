# MP4201/CMakeLists.txt

idf_component_register(SRCS "MP4201.c" 
                            "MP4201Driver.c" 
                            "MP4201Middleware.c"
                            "IIC_driver.c"
                       INCLUDE_DIRS "include"
                       REQUIRES driver) # 依赖 ESP-IDF 的 driver 组件以使用 I2C