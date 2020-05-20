TOP=../..

include $(TOP)/configure/CONFIG


# Build the IOC application
PROD_IOC = cathodeSupportSim

# cathodeSupportSim.dbd will be created and installed
DBD += cathodeSupportSim.dbd

# cathodeSupportSim.dbd will include these files:
cathodeSupportSim_DBD += base.dbd
cathodeSupportSim_DBD += iocAdmin.dbd
cathodeSupportSim_SRCS += cathodeSupportSim_registerRecordDeviceDriver.cpp

# Build the main IOC entry point where needed
cathodeSupportSim_SRCS_DEFAULT += cathodeSupportSimMain.cpp
cathodeSupportSim_SRCS_vxWorks += -nil-

# Required library support
cathodeSupportSim_LIBS += devIocStats

# Finally link IOC to the EPICS Base libraries
cathodeSupportSim_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES


