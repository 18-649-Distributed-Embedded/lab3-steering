cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(lab3_steering)

target_sources(app PRIVATE
    src/main.c

    # Pure logic (also built by tests/host)
    src/core/conn_state.c

    # Hardware drivers
    src/hw/servo.c
    src/hw/adc.c
    src/hw/blinkers.c
    src/hw/can.c

    # System tasks
    src/sys/can_rx.c
    src/sys/relay.c
    src/sys/steering.c
    src/sys/blinkers_control.c
    src/sys/hb_tx.c
    src/sys/force_tx.c
    src/sys/hb_mon.c
    src/sys/health.c
    src/sys/debug_status.c
)


target_include_directories(app PRIVATE src)
