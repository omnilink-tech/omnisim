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

# OMNISIM_HOME is the canonical install-root variable (same pattern as ../c/Makefile and
# ../cpp/Makefile, which this file previously did not follow -- it read only WEBOTS_HOME, so a
# standalone build with OMNISIM_HOME alone silently fell back to the relative path). WEBOTS_HOME
# is still accepted as a deprecated legacy alias so a pre-rebrand shell profile keeps building;
# the build-variable contract it used to name (WEBOTS_HOME_PATH, WEBOTS_DEPENDENCY_PATH and the
# rest) was renamed to OMNISIM_* across every Makefile in one atomic change.
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
	@echo "  make release               for final release"
	@echo "  make clean                 remove build directory"

include $(OMNISIM_HOME_PATH)/resources/Makefile.include

ifeq ($(OSTYPE),windows)
CONTROLLER_LAUNCHER = $(OMNISIM_HOME_PATH)/msys64/mingw64/bin/omnisim-controller.exe
else ifeq ($(OSTYPE),darwin)
CONTROLLER_LAUNCHER = $(OMNISIM_HOME_PATH)/Contents/MacOS/omnisim-controller
else
CONTROLLER_LAUNCHER = $(OMNISIM_HOME_PATH)/omnisim-controller
endif

.PHONY: release debug profile

release debug profile: $(CONTROLLER_LAUNCHER)

ifeq ($(OSTYPE),darwin)

OBJDIR = build/$(MAKECMDGOALS)
BUILD_PATH_SETUP := $(shell mkdir -p $(OBJDIR)/x86_64 $(OBJDIR)/arm64)

$(CONTROLLER_LAUNCHER): $(OBJDIR)/x86_64/omnisim-controller $(OBJDIR)/arm64/omnisim-controller
	@echo "# creating fat " $(notdir $@)
	@lipo -create -output $@ $(OBJDIR)/x86_64/omnisim-controller $(OBJDIR)/arm64/omnisim-controller
	@codesign -s - $@

$(OBJDIR)/x86_64/omnisim-controller: omnisim_controller.c
	@echo "# compiling " $@ "(x64_64)"
	@gcc $(CFLAGS) -target x86_64-apple-macos12 omnisim_controller.c -o $@

$(OBJDIR)/arm64/omnisim-controller: omnisim_controller.c
	@echo "# compiling " $@ "(arm64)"
	@gcc $(CFLAGS) -target arm64-apple-macos12 omnisim_controller.c -o $@

else

$(CONTROLLER_LAUNCHER): omnisim_controller.c
	@echo "# compiling" $<
	@gcc $(CFLAGS) omnisim_controller.c -o $@

endif

FILES_TO_REMOVE += $(CONTROLLER_LAUNCHER)
