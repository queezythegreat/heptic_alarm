set(ARDUINO_DEFAULT_BOARD ethernet)    # Default Board ID, when not specified
set(ARDUINO_DEFAULT_PORT /dev/ttyUSB0) # Default Port, when not specified


generate_arduino_firmware(heptic_alarm
    SRCS main.cpp)

