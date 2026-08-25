# Copyright 2026 OmniLink
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
null :=
space := $(null) $(null)
OMNISIM_HOME_PATH?=$(subst $(space),\ ,$(strip $(subst \,/,$(OMNISIM_HOME))))
include $(OMNISIM_HOME_PATH)/resources/Makefile.os.include

.PHONY: release debug profile clean

# Python controllers — their Makefiles are no-ops, but recursing keeps the
# project consistent with the rest of the tree and ready for any future
# compiled (C/C++) controllers.
TARGETS = controllers/omni_quest_nav.Makefile \
          controllers/omni_quest_eye.Makefile \
          controllers/omni_quest_wander.Makefile

release debug profile: $(TARGETS)
ifneq ($(MESSAGE),)
	+@echo -e $(MESSAGE)
endif

clean: $(TARGETS)

controllers/%.Makefile:
	+@echo "# make" $(MAKECMDGOALS) $(@:.Makefile=)
	+@make -s -C $(@:.Makefile=) $(MAKECMDGOALS)
