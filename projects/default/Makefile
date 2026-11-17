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
null :=
space := $(null) $(null)
OMNISIM_HOME_PATH?=$(subst $(space),\ ,$(strip $(subst \,/,$(OMNISIM_HOME))))
include $(OMNISIM_HOME_PATH)/resources/Makefile.os.include

.PHONY: release debug profile clean

# Descend into every controller here that is COMPILED, i.e. ships its own Makefile;
# the Python controllers have none and must not be descended into. Globbed, not
# hard-coded: the previous hard-coded `controllers/braitenberg.Makefile` outlived the
# controller itself and broke the Linux build on a fresh checkout ("No such file or
# directory"), while the local Windows engine build, which never descends here, stayed
# green. A glob cannot go stale when a controller is added or removed.
TARGETS = $(patsubst %/Makefile,%.Makefile,$(wildcard controllers/*/Makefile))

release debug profile: $(TARGETS)
ifneq ($(MESSAGE),)
	+@echo -e $(MESSAGE)
endif

clean: $(TARGETS)

controllers/%.Makefile:
	+@echo "# make" $(MAKECMDGOALS) $(@:.Makefile=)
	+@make -s -C $(@:.Makefile=) $(MAKECMDGOALS)
