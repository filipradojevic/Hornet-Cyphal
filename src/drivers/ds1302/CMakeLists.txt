idf_component_register(
    SRCS 
        "ds1302.c"
    
    REQUIRES 
        driver 
        freertos
        esp_timer
        
    INCLUDE_DIRS 
        "."
)