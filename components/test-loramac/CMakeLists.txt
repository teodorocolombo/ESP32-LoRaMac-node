idf_component_register(SRCS "test-ping.c" "test-rtc.c" "test-uart.c" "test-spi.c" "test-sx1276"
                       INCLUDE_DIRS "include"
                       REQUIRES Loramac 
                       )