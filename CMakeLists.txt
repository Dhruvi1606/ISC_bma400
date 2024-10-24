cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(BMA400_nrf5340)

target_sources(app PRIVATE src/main.c
                                    BMA400-API/bma400.c)
# Custom files and folders

target_sources(app PRIVATE
    src/my_lbs.c
)

zephyr_library_include_directories(.)

target_include_directories(app PRIVATE BMA400-API)