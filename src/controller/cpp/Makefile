# Copyright 1996-2024 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# OMNISIM_HOME is the canonical install-root variable; WEBOTS_HOME is kept as a
# legacy alias so existing setups keep working. OMNISIM_HOME_PATH is the contract
# variable owned and consumed by resources/Makefile.include -- it was renamed
# from WEBOTS_HOME_PATH in the same atomic change as every other caller.
# Adopt the legacy name ONLY when it names a directory that EXISTS: a machine that
# once had Webots keeps WEBOTS_HOME after the install is removed, and adopting that
# phantom path resolves every include and library under a directory that is not there.
# `test -d`, not $(wildcard): wildcard splits on spaces and would reject a real
# "C:\Program Files\..." install.
OM_LEGACY_HOME := $(strip $(if $(strip $(WEBOTS_HOME)),$(if $(strip $(shell test -d "$(WEBOTS_HOME)" && echo yes)),$(WEBOTS_HOME))))
OMNISIM_ROOT := $(if $(OMNISIM_HOME),$(OMNISIM_HOME),$(OM_LEGACY_HOME))
ifeq ($(OMNISIM_ROOT),)
OMNISIM_HOME_PATH = ../../..
else
null :=
space := $(null) $(null)
OMNISIM_HOME_PATH?=$(subst $(space),\ ,$(strip $(subst \,/,$(OMNISIM_ROOT))))
endif

default:
	@echo "Type one of the following (OSTYPE is \""$(OSTYPE)"\"):"
	@echo "  make debug                 for debug (with gdb symbols)"
	@echo "  make profile               for profiling (with gprof information)"
	@echo "  make release               for finale release"
	@echo "  make clean                 remove build directory"
	@echo "  make count-lines           count the lines of code of C++ controller library"

count-lines:
	wc -l *.c* *.h*

include $(OMNISIM_HOME_PATH)/resources/Makefile.include

OBJDIR = build/$(MAKECMDGOALS)
ifneq (,$(findstring $(MAKECMDGOALS),debug profile release))
BUILD_PATH_SETUP := $(shell mkdir -p $(OBJDIR))
endif

vpath %.o $(OBJDIR)
vpath %.cpp .

CPPSRC = Accelerometer.cpp \
         Altimeter.cpp \
         Brake.cpp \
         Camera.cpp \
         Compass.cpp \
         Connector.cpp \
         Device.cpp \
         Display.cpp \
         DistanceSensor.cpp \
         Emitter.cpp \
         Field.cpp \
         GPS.cpp \
         Gyro.cpp \
         InertialUnit.cpp \
         Joystick.cpp \
         Keyboard.cpp \
         LED.cpp \
         Lidar.cpp \
         LightSensor.cpp \
         Motion.cpp \
         Motor.cpp \
         Mouse.cpp \
         Node.cpp \
         Pen.cpp \
         PositionSensor.cpp \
         Proto.cpp \
         Radar.cpp \
         RangeFinder.cpp \
         Receiver.cpp \
         Robot.cpp \
         Skin.cpp \
         Speaker.cpp \
         Supervisor.cpp \
         TouchSensor.cpp \
         VacuumGripper.cpp

OBJECTS = $(CPPSRC:.cpp=.o)
CPPOBJS = $(addprefix $(OBJDIR)/, $(OBJECTS))
DEPENDENCIES = $(addprefix $(OBJDIR)/,$(notdir $(CPPSRC:.cpp=.d)))

ifneq (,$(filter $(MAKECMDGOALS),debug release profile)) # if $(MAKECMDGOALS) is either of those
-include $(DEPENDENCIES)
endif

ifeq ($(OSTYPE),windows)
LDFLAGS       = -frandom-seed=0 -shared -Wl,--out-implib,$(OMNISIM_CONTROLLER_LIB_PATH)/CppController.lib
LIBCONTROLLER = $(OMNISIM_CONTROLLER_LIB_PATH)/Controller.dll
SHAREDLIBS    = $(LIBCONTROLLER)
CPPFLAGS      = -c -Wall -mwindows
CPPINCLUDES   = -I$(OMNISIM_HOME_PATH)/include/controller/c -I$(OMNISIM_HOME_PATH)/include/controller/cpp
TARGET        = $(OMNISIM_CONTROLLER_LIB_PATH)/CppController.dll
endif

ifeq ($(OSTYPE),darwin)
LDFLAGS       = -dynamiclib -compatibility_version 1.0 -current_version 1.0.0 -mmacosx-version-min=$(MACOSX_MIN_SDK_VERSION)
SHAREDLIBS    = -L$(OMNISIM_CONTROLLER_LIB_PATH) -lController
CPPFLAGS      = -c -fPIC -Wall -mmacosx-version-min=$(MACOSX_MIN_SDK_VERSION)
CPPINCLUDES   = -I$(OMNISIM_HOME_PATH)/include/controller/c -I$(OMNISIM_HOME_PATH)/include/controller/cpp
LIBCONTROLLER = $(OMNISIM_CONTROLLER_LIB_PATH)/libController.dylib
LDFLAGS      += -install_name @rpath/Contents/lib/controller/libCppController.dylib
TARGET        = $(OMNISIM_CONTROLLER_LIB_PATH)/libCppController.dylib
endif

ifeq ($(OSTYPE),linux)
LDFLAGS       = -shared -Wl,-rpath,'$$ORIGIN'
SHAREDLIBS    = -L$(OMNISIM_CONTROLLER_LIB_PATH) -lController
CPPFLAGS      = -c -fpic -Wall
CPPINCLUDES   = -I$(OMNISIM_HOME_PATH)/include/controller/c -I$(OMNISIM_HOME_PATH)/include/controller/cpp
LIBCONTROLLER = $(OMNISIM_CONTROLLER_LIB_PATH)/libController.so
TARGET        = $(OMNISIM_CONTROLLER_LIB_PATH)/libCppController.so
endif

ifeq ($(TREAT_WARNINGS_AS_ERRORS),1)
CPPFLAGS += -Werror
endif

all debug profile release: $(TARGET)

ifeq ($(OSTYPE),darwin)

X86_64_OBJECTS = $(addprefix $(BUILD_GOAL_DIR)/x86_64/,$(OBJECTS))
ARM64_OBJECTS = $(addprefix $(BUILD_GOAL_DIR)/arm64/,$(OBJECTS))

$(TARGET): $(BUILD_GOAL_DIR)/x86_64/libCppController.dylib $(BUILD_GOAL_DIR)/arm64/libCppController.dylib
	@echo "# creating fat" $(notdir $@)
	@lipo -create -output $@ $(BUILD_GOAL_DIR)/x86_64/libCppController.dylib $(BUILD_GOAL_DIR)/arm64/libCppController.dylib
	@codesign -s - $@

$(BUILD_GOAL_DIR)/x86_64/libCppController.dylib: $(X86_64_OBJECTS)
	@echo "# linking" $@ "(x86_64)"
	@$(CXX) -target x86_64-apple-macos12 $(LDFLAGS) $(X86_64_OBJECTS) $(SHAREDLIBS) -o "$@"

$(BUILD_GOAL_DIR)/arm64/libCppController.dylib: $(ARM64_OBJECTS)
	@echo "# linking" $@ "(arm64)"
	@$(CXX) -target arm64-apple-macos12 $(LDFLAGS) $(ARM64_OBJECTS) $(SHAREDLIBS) -o "$@"

else

$(TARGET):$(CPPOBJS) $(LIBCONTROLLER)
	@echo "# linking" $@
	@$(CXX) $(LDFLAGS) $(CPPOBJS) $(SHAREDLIBS) -o "$@"

endif

$(LIBCONTROLLER):
	@echo "Warning: $(LIBCONTROLLER) doesn't exist, rebuild it first."

# cpp compilation rule
$(OBJDIR)/%.o:%.cpp
	@echo "# compiling" $<
	@$(CXX) $(CPPFLAGS) $(CPPINCLUDES) -o $@ $<

$(OBJDIR)/arm64/%.o:%.cpp
	@echo "# compiling" $< "(arm64)"
	@$(CXX) -target arm64-apple-macos12 $(CPPFLAGS) $(CPPINCLUDES) -o $@ $<

$(OBJDIR)/x86_64/%.o:%.cpp
	@echo "# compiling" $< "(x86_64)"
	@$(CXX) -target x86_64-apple-macos12 $(CPPFLAGS) $(CPPINCLUDES) -o $@ $<

# dependency rule
$(OBJDIR)/%.d:%.cpp
	@echo "# updating" $(notdir $@)
	@printf $(OBJDIR)/ > $@
	@$(CXX) $(CPPINCLUDES) -MM $< >> $@

ifeq ($(OSTYPE),windows)
FILES_TO_REMOVE = $(OMNISIM_CONTROLLER_LIB_PATH)/CppController.dll $(OMNISIM_CONTROLLER_LIB_PATH)/CppController.lib *.exp
else
FILES_TO_REMOVE = $(OMNISIM_CONTROLLER_LIB_PATH)/*Cpp*
endif
FILES_TO_REMOVE += $(CPPOBJS) $(DEPENDENCIES)
