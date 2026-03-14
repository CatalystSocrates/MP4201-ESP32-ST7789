#all c file in this directory
file(GLOB SRCS "ble_subs.c" "lv_port_disp.c" "main.c" "*.c")

#register component
idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS ".")
