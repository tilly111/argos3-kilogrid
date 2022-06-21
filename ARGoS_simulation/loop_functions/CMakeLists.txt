include_directories(argos3/plugins/robot/kilobot/control_interface argos3/plugins/robot/kilobot/simulator)

add_library(kilogrid_template MODULE kilogrid_template.h kilogrid_template.cpp)
target_link_libraries(kilogrid_template
        argos3core_simulator
        argos3plugin_simulator_dynamics2d
        argos3plugin_simulator_entities
        argos3plugin_simulator_media
        argos3plugin_simulator_kilobot
        argos3plugin_simulator_kilolib
        )
