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

ifeq ($(OMNISIM_HOME),)
OMNISIM_HOME_ARG=OMNISIM_HOME=$(shell cd .. ; pwd)
endif

release profile debug clean:
	@echo "#"; echo "# !!! languages !!!"
	@+make -s -C languages $@ $(OMNISIM_HOME_ARG)
	@echo "#"; echo "# !!! default !!!"
	@+make -s -C default $@ $(OMNISIM_HOME_ARG)
	@echo "#"; echo "# !!! objects !!!"
	@+make -s -C objects $@ $(OMNISIM_HOME_ARG)
	@echo "#"; echo "# !!! robots !!!"
	@+make -s -C robots $@ $(OMNISIM_HOME_ARG)
	@echo "#"; echo "# !!! samples !!!"
	@+make -s -C samples $@ $(OMNISIM_HOME_ARG)

cleanse: clean
