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


# Legacy alias: seed the install root from the pre-rebrand WEBOTS_HOME when only
# that name is exported. It has to be resolved HERE, because OMNISIM_HOME_PATH is
# what locates resources/Makefile.include below -- leave it empty and make fails
# with "No rule to make target '/resources/Makefile.include'" instead of an
# explanation. WEBOTS_HOME is deprecated; export OMNISIM_HOME.
# Adopt the legacy name ONLY when it names a directory that EXISTS: a machine that
# once had Webots keeps WEBOTS_HOME after the install is removed, and adopting that
# phantom path resolves every include and library under a directory that is not there.
# `test -d`, not $(wildcard): wildcard splits on spaces and would reject a real
# "C:\Program Files\..." install.
OM_LEGACY_HOME := $(strip $(if $(strip $(WEBOTS_HOME)),$(if $(strip $(shell test -d "$(WEBOTS_HOME)" && echo yes)),$(WEBOTS_HOME))))
ifeq ($(strip $(OMNISIM_HOME)),)
 OMNISIM_HOME := $(OM_LEGACY_HOME)
endif

# Do not modify the following: this includes OmniSim global Makefile.include
null :=
space := $(null) $(null)
OMNISIM_HOME_PATH?=$(subst $(space),\ ,$(strip $(subst \,/,$(OMNISIM_HOME))))
include $(OMNISIM_HOME_PATH)/resources/Makefile.include
