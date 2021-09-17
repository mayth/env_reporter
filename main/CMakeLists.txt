idf_component_register(SRCS "env_reporter.cpp" "sht3x.cpp" "i2c_wrapper.cpp"
                    INCLUDE_DIRS ".")

# TODO: ESP-IDF needs GNU extension now
idf_build_set_property(CXX_COMPILE_OPTIONS "-std=gnu++14" APPEND)

target_compile_definitions(${COMPONENT_LIB} PUBLIC "-DLOG_LOCAL_LEVEL=ESP_LOG_INFO")
