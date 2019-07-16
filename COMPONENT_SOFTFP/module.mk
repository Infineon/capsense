################################################################################
# \file module.mk
# \version 2.0
#
# \brief
# CapSense software component with soft FP pre-built library constructed to work
# on top of the CSD driver. This software component is compatible with all 
# CPU cores.
#
################################################################################
# \copyright
# Copyright 2018-2019, Cypress Semiconductor Corporation.  All rights reserved.
# You may use this file only in accordance with the license, terms, conditions,
# disclaimers, and limitations in the end user license agreement accompanying
# the software package with which this file was provided.
################################################################################

ifeq ($(WHICHFILE),true)
$(info Processing $(lastword $(MAKEFILE_LIST)) file from directory $(PWD))
$(info Path: $(MAKEFILE_LIST))
endif

#
# Needed by describe goal processing
#
ifeq ($(MAKECMDGOALS),describe)
ifndef PLATFORMS_VERSION
PLATFORMS_VERSION=1.0
endif
include $(CYSDK)/libraries/platforms-$(PLATFORMS_VERSION)/common/swcomps.mk
endif

#
# The internal tag name of the software component
#
MIDDLEWARE_CAPSENSE_SOFTFP_NAME=MIDDLEWARE_CAPSENSE_SOFTFP

#
# If defined, the list of legal PLATFORM values for this component.
# If not defined, this component is valid for all values of PLATFORM
#
CY_SUPPORTED_PLATFORM_LIST=PSOC6_DUAL_CORE PSoC6_cm4_dual PSoC6_cm0p PSOC6_SINGLE_CORE PSoC6_cm4_single $(PSOC_EXTRA_PLATFORMS)

#
# A list of UDD devices that this component can be used with.
#
CY_SUPPORTED_DEVICE_LIST=$(CY_DEVICES_WITH_CAPSENSE)

#
# If defined, the list of legal TOOLCHAIN values for this component.  If not
# defined, this component is valid for all values of TOOLCHAIN
#
#CY_SUPPORTED_TOOLCHAIN_LIST=

#
# Used by the IDE to group and categorize components.
#
CY_COMP_CATEGORY=Middleware

#
# The displayed human readable name of the component
#
CY_COMP_NAME_DISPLAY=CapSense, Soft FP prebuilt library

#
# The name in the form of an identifier ([a-z_][a-z0-9_]*).
# Used to generate directories in the IDE.
# 
CY_COMP_NAME_ID=middlewareCapsenseSoftFp

#
# The human readable description of the component
#
CY_COMP_DESCR=CapSense software component with soft FP pre-built library constructed to work\
	on top of the CSD driver. This software component is compatible with all CPU cores.

#
# The type of component ...
#   link - means link the source code from the IDE project to the SDK
#   copy - means copy the source code into the IDE project
#
CY_COMP_TYPE=link

#
# Defines if the component can change based on which artifact is being used
#
CY_COMP_PER_ARTIFACT=false

#
# The list of components this component is dependent on. Path is relative the
# the SDK's libraries folder.
#
CY_COMP_DEPS=

#
# Used by the build recipe for an ELF file to add this component
# to the list of components that must be included
#
$(CYARTIFACT)_OBJ_COMP_TAG_LIST += $(MIDDLEWARE_CAPSENSE_SOFTFP_NAME)

#
# Defines the series of needed make variables that include this component in the
# build process.  Also defines the describe target if we are describing a component
#
$(MIDDLEWARE_CAPSENSE_SOFTFP_NAME)_INCLUDES=\
	-I$(CY_PSOC_LIB_COMP_MIDDLEWARE)/capsense

ifeq ($(TOOLCHAIN),GCC)
CY_CAPSENSE_LIB = \
	TOOLCHAIN_GCC_ARM/libcy_capsense.a
else
ifeq ($(TOOLCHAIN),IAR)
CY_CAPSENSE_LIB = \
	TOOLCHAIN_IAR/libcy_capsense.a
else
ifeq ($(TOOLCHAIN),ARMCC)
CY_CAPSENSE_LIB = \
	TOOLCHAIN_ARM/libcy_capsense.ar
endif
endif
endif

$(eval $(call \
	CY_DECLARE_SWCOMP_OBJECT,$(MIDDLEWARE_CAPSENSE_SOFTFP_NAME),\
	$(lastword $(MAKEFILE_LIST)),\
	$(CY_CAPSENSE_LIB)\
	../cy_capsense.h\
	../cy_capsense_centroid.c\
	../cy_capsense_centroid.h\
	../cy_capsense_common.h\
	../cy_capsense_control.c\
	../cy_capsense_control.h\
	../cy_capsense_csd.c\
	../cy_capsense_csd.h\
	../cy_capsense_csx.c\
	../cy_capsense_csx.h\
	../cy_capsense_filter.c\
	../cy_capsense_filter.h\
	../cy_capsense_lib.h\
	../cy_capsense_gesture_lib.h\
	../cy_capsense_processing.c\
	../cy_capsense_processing.h\
	../cy_capsense_sensing.c\
	../cy_capsense_sensing.h\
	../cy_capsense_structure.c\
	../cy_capsense_structure.h\
	../cy_capsense_tuner.c\
	../cy_capsense_tuner.h))

