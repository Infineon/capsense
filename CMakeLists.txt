
add_library(capsense INTERFACE)


target_sources(capsense
    INTERFACE
        cy_capsense_control.c
        cy_capsense_tuner.c
        cy_capsense_centroid.c
        cy_capsense_csx_v2.c
        cy_capsense_csd_v2.c
        cy_capsense_filter.c
        cy_capsense_generator_v3.c
        cy_capsense_processing.c
        cy_capsense_selftest.c
        cy_capsense_sensing_v2.c
        cy_capsense_sensing_v3.c
        cy_capsense_structure.c
       
)
