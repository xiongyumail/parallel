if(IDF_TARGET STREQUAL "esp32s2")
    list(APPEND COMPONENT_SRCS "esp32s2/parallel.c")
    list(APPEND COMPONENT_PRIV_INCLUDEDIRS "esp32s2/include/soc")
    list(APPEND COMPONENT_ADD_INCLUDEDIRS "esp32s2/include")
endif()

if(IDF_TARGET STREQUAL "esp32s3")
    list(APPEND COMPONENT_SRCS "esp32s3/parallel.c")
    list(APPEND COMPONENT_PRIV_INCLUDEDIRS "esp32s3/include/soc")
    list(APPEND COMPONENT_ADD_INCLUDEDIRS "esp32s3/include")
endif()

register_component()