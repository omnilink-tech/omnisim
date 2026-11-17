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
#
# Modifications copyright 2026 OmniLink, licensed under the Apache License, Version 2.0.

# OmniSim Makefile system
#
# You may add some variable definitions hereafter to customize the build process
# See documentation in $(OMNISIM_HOME_PATH)/resources/Makefile.include


CXX_SOURCES = entry_points.cpp

USE_C_API = true

# Legacy alias: seed the install root from the pre-rebrand WEBOTS_HOME when only
# that name is exported. It has to be resolved HERE, before the first use below
# and before OMNISIM_HOME_PATH locates resources/Makefile.include -- leave it
# empty and make fails with "No rule to make target
# '/resources/Makefile.include'" instead of an explanation. WEBOTS_HOME is
# deprecated; export OMNISIM_HOME.
# Adopt the legacy name ONLY when it names a directory that EXISTS: a machine that
# once had Webots keeps WEBOTS_HOME after the install is removed, and adopting that
# phantom path resolves every include and library under a directory that is not there.
# `test -d`, not $(wildcard): wildcard splits on spaces and would reject a real
# "C:\Program Files\..." install.
OM_LEGACY_HOME := $(strip $(if $(strip $(WEBOTS_HOME)),$(if $(strip $(shell test -d "$(WEBOTS_HOME)" && echo yes)),$(WEBOTS_HOME))))
ifeq ($(strip $(OMNISIM_HOME)),)
 OMNISIM_HOME := $(OM_LEGACY_HOME)
endif

QT_UTILS = $(OMNISIM_HOME)/resources/projects/libraries/qt_utils
INCLUDE = -I"$(QT_UTILS)"
LIBRARIES = -L"$(QT_UTILS)" -lqt_utils

null :=
space := $(null) $(null)
OMNISIM_HOME_PATH?=$(subst $(space),\ ,$(strip $(subst \,/,$(OMNISIM_HOME))))
include $(OMNISIM_HOME_PATH)/resources/Makefile.include


# Qt linking

CFLAGS += -std=c++17
MOC_SOURCES = $(addprefix $(BUILD_DIR)/,$(notdir $(HPP_FILES_TO_MOC:.hpp=.moc.cpp)))
MOC_DIRECTORIES = $(sort $(dir $(HPP_FILES_TO_MOC)))
OBJECTS += $(addprefix $(BUILD_GOAL_DIR)/,$(notdir $(MOC_SOURCES:.cpp=.o)))

ifeq ($(OSTYPE),windows)
 CFLAGS += -D_USE_MATH_DEFINES
 QT_INCLUDE_DIR = $(OMNISIM_HOME)/include/qt
 MOC = /mingw64/share/qt6/bin/moc
 MOC_PLATFORM_FLAGS = -D_WIN32
 DYNAMIC_LIBRARIES += -L"$(OMNISIM_HOME)/msys64/mingw64/bin" -lQt6Core -lQt6Gui -lQt6Widgets
 INCLUDE += -isystem "$(QT_INCLUDE_DIR)/QtCore" -isystem "$(QT_INCLUDE_DIR)/QtGui" -isystem "$(QT_INCLUDE_DIR)/QtWidgets"
endif

ifeq ($(OSTYPE),linux)
 QT_INCLUDE_DIR = $(OMNISIM_HOME)/include/qt
 QT_LIB_DIR = $(OMNISIM_HOME)/lib/webots
 MOC = "$(OMNISIM_HOME)/bin/qt/moc"
 MOC_PLATFORM_FLAGS = -D__linux__
 DYNAMIC_LIBRARIES += -L"$(QT_LIB_DIR)" -lQt6Core -lQt6Gui -lQt6Widgets
 INCLUDE += -isystem "$(QT_INCLUDE_DIR)/QtCore" -isystem "$(QT_INCLUDE_DIR)/QtGui" -isystem "$(QT_INCLUDE_DIR)/QtWidgets"
endif

ifeq ($(OSTYPE),darwin)
 WBCFLAGS += -Wno-unknown-pragmas
 FRAMEWORKS_DIR = $(OMNISIM_HOME)/Contents/Frameworks
 INCLUDE += -F"$(FRAMEWORKS_DIR)"
 FRAMEWORKS += -F"$(FRAMEWORKS_DIR)"
 DYNAMIC_LIBRARIES += -bind_at_load $(FRAMEWORKS)
 MOC = "$(OMNISIM_HOME)/bin/qt/moc"
 MOC_PLATFORM_FLAGS = -D__APPLE__
 FRAMEWORKS += -framework QtCore -framework QtGui -framework QtWidgets
endif

vpath %.cpp $(BUILD_DIR)
vpath %.hpp $(MOC_DIRECTORIES)

.PRECIOUS: $(MOC_SOURCES)

$(BUILD_DIR)/%.moc.cpp: %.hpp
	@echo "# moc-generating" $(notdir $<)
	$(SILENT)$(MOC) $(MOC_PLATFORM_FLAGS) $(INCLUDE:-isystem:-I) $< -o $@


$(BUILD_GOAL_DIR)/$(MAIN_TARGET): $(OBJECTS)
