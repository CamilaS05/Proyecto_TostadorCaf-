idf_component_register(SRCS "src/max6675_driver.cpp" "src/heater_driver.cpp" "src/motor_driver.cpp" "src/fan_driver.cpp"
                       INCLUDE_DIRS "include"
                       REQUIRES driver app_core)
