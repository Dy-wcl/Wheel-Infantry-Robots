set(_bsp_sources
    ${CMAKE_CURRENT_LIST_DIR}/bsp_gpio/bsp_gpio.c
    ${CMAKE_CURRENT_LIST_DIR}/bsp_can/bsp_can.c
    ${CMAKE_CURRENT_LIST_DIR}/bsp_pwm/bsp_pwm.c
    ${CMAKE_CURRENT_LIST_DIR}/bsp_uart/bsp_uart.c
    ${CMAKE_CURRENT_LIST_DIR}/dwt/bsp_dwt.c
    ${CMAKE_CURRENT_LIST_DIR}/log/bsp_log.c
    ${CMAKE_CURRENT_LIST_DIR}/../SEGGER/RTT/SEGGER_RTT.c
    ${CMAKE_CURRENT_LIST_DIR}/../SEGGER/RTT/SEGGER_RTT_printf.c
    # ${CMAKE_CURRENT_LIST_DIR}/bsp_iic/bsp_iic.c
    # ${CMAKE_CURRENT_LIST_DIR}/bsp_usb/bsp_usb.c
)

target_sources(${PROJECT_NAME} PRIVATE ${_bsp_sources})

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/bsp_gpio
    ${CMAKE_CURRENT_LIST_DIR}/bsp_can
    ${CMAKE_CURRENT_LIST_DIR}/bsp_pwm
    ${CMAKE_CURRENT_LIST_DIR}/bsp_uart
    ${CMAKE_CURRENT_LIST_DIR}/dwt
    ${CMAKE_CURRENT_LIST_DIR}/log

    # ${CMAKE_CURRENT_LIST_DIR}/bsp_iic
    # ${CMAKE_CURRENT_LIST_DIR}/bsp_usb
)
