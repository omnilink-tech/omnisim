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

# Install root: OMNISIM_HOME, then the deprecated pre-rebrand WEBOTS_HOME, then
# this Makefile's own location. Resolved in one := so a set-but-empty variable
# cannot leave OMNISIM_HOME defined-and-empty, which would make ?= skip the
# fallback and silently produce "/resources/Makefile.os.include".
# Adopt the legacy name ONLY when it names a directory that EXISTS: a machine that
# once had Webots keeps WEBOTS_HOME after the install is removed, and adopting that
# phantom path resolves every include and library under a directory that is not there.
# `test -d`, not $(wildcard): wildcard splits on spaces and would reject a real
# "C:\Program Files\..." install.
OM_LEGACY_HOME := $(strip $(if $(strip $(WEBOTS_HOME)),$(if $(strip $(shell test -d "$(WEBOTS_HOME)" && echo yes)),$(WEBOTS_HOME))))
ifeq ($(strip $(OMNISIM_HOME)),)
 OMNISIM_HOME := $(if $(OM_LEGACY_HOME),$(OM_LEGACY_HOME),../..)
endif

null :=
space := $(null) $(null)
OMNISIM_HOME_PATH?=$(subst $(space),\ ,$(strip $(subst \,/,$(OMNISIM_HOME))))
include $(OMNISIM_HOME_PATH)/resources/Makefile.os.include

STATIC_LIBRARY = true
INCLUDE = -I$(OMNISIM_HOME_PATH)/include
C_SOURCES = glad.c

include $(OMNISIM_HOME_PATH)/resources/Makefile.include
