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

# OMNISIM_HOME is a sine qua non condition to run this Makefile
ifndef OMNISIM_HOME
 $(error The OMNISIM_HOME environment variable is not defined.)
endif

ifndef OMNISIM_HOME_PATH
 null :=
 space := $(null) $(null)
 OMNISIM_HOME_PATH=$(subst $(space),\ ,$(strip $(subst \,/,$(OMNISIM_HOME))))
endif

include $(OMNISIM_HOME_PATH)/resources/Makefile.os.include

ifneq ($(SNAPCRAFT_PROJECT_NAME),)
ifeq ($(DISPLAY),)
export DISPLAY = :99
XVFB = xvfb
endif
endif

all: $(OMNISIM_DISTRO_EXE) $(XVFB)
	+@echo "# removing files and folders not in git repository"
	+@cd $(OMNISIM_HOME_PATH) ; git clean -fdf | sed -e 's/^/# - /' ; cd scripts/packaging
	+@echo "# checking submodules update"
	+@./check_submodules_update.sh
	+@echo "# generating package"
	+@python3 omnisim_distro.py
	+@echo "# done for" $(OSTYPE)

# The distro scripts derive these names from the application name ("OmniSim"), see
# generic_distro.WebotsPackage.application_name_lowercase_and_dashes -- they generate
# omnisim.snap / omnisim.iss, not the pre-rebrand webots.* names.
install: all
	./omnisim.snap

xvfb:
	+@echo "# starting Xvfb: X Virtual Frame Buffer"
	+@Xvfb :99 -screen 0 1024x768x16 &

clean:
	+@rm -rf $(OMNISIM_DISTRO_EXE) omnisim.snap omnisim.iss webots.snap webots.iss
	+@find $(OMNISIM_HOME_PATH)/projects $(OMNISIM_HOME_PATH)/resources -name .*.cache | xargs rm -f
	+@rm -rf __pycache__ node_modules package-lock.json
