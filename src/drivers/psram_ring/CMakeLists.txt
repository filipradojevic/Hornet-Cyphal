idf_component_register(
    SRCS 
        "psram_ring.c"
    
    PRIV_REQUIRES 
        # API za standardne drajvere i osnovne funkcije
        driver
        freertos    # Za semafore i mutex-e (xSemaphoreCreateMutex, xSemaphoreTake, itd.)
        heap        # Za alokaciju PSRAM-a (heap_caps_malloc)
        esp_psram 
        
    INCLUDE_DIRS 
        "."
)