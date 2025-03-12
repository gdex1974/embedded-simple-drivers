cmake_minimum_required(VERSION 3.15)

if ("${COMPONENT_NAME}" STREQUAL "")
# TODO
else ()
    set(srcsSPS30
            SPS30/ShdlcTransport.cpp
            SPS30/Sps30i2c.cpp
            SPS30/Sps30Uart.cpp
            )
    set(srcsBME280
            BME280/BME280.cpp
            BME280/SpiHelper.cpp
            )
    set(srceInk
            display/Epd3in7Display.cpp
            display/Epd4in26Display.cpp
            display/EpdInterface.cpp
            display/SpiDisplayInterface.cpp
            display/ST7302SpiDisplay.cpp
        )
    set(includedirs
            ${CMAKE_CURRENT_LIST_DIR}
            )
    idf_component_register(INCLUDE_DIRS ${includedirs}
            SRCS ${srcsSPS30} ${srcsBME280} ${srceInk}
            REQUIRES general-support-library
            )

endif ()

