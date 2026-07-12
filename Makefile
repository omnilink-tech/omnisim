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

START := $(shell date +%s)

USE_CCACHE ?= 1
CCACHE := $(shell command -v ccache 2>/dev/null)

ifneq ($(CCACHE),)
ifneq ($(USE_CCACHE),0)
CC := $(CCACHE) $(CC)
CXX := $(CCACHE) $(CXX)
export CC
export CXX
endif
endif

# OMNISIM_HOME is the canonical install-root variable post-rebrand.
# OMNISIM_HOME is kept as a legacy alias for unmigrated controllers / CI;
# both are exported so either name picks up the same path.
ifeq ($(OMNISIM_HOME),)
ifneq ($(OMNISIM_HOME),)
export OMNISIM_HOME := $(OMNISIM_HOME)
else
ifneq ($(findstring MINGW,$(shell uname)),) # under MINGW, native Windows format is required
export OMNISIM_HOME := $(shell pwd -W | tr -s / '\\')
else
export OMNISIM_HOME := $(PWD)
endif
endif
endif
ifeq ($(OMNISIM_HOME),)
export OMNISIM_HOME := $(OMNISIM_HOME)
endif

# Legacy alias: sub-makefiles (src/ode/Makefile, dependencies/Makefile.*,
# resources/Makefile.os.include) still reference WEBOTS_HOME / WEBOTS_PATH
# from before the rebrand. Export both names so either resolves to the
# same path until those sub-makes get migrated.
export WEBOTS_HOME := $(OMNISIM_HOME)
export WEBOTS_PATH := $(OMNISIM_HOME)

include resources/Makefile.os.include

WEBOTS_DISTRIBUTION_PATH ?= $(OMNISIM_HOME)/distribution

ifeq ($(MAKECMDGOALS),)
MAKECMDGOALS = release
else
ifeq ($(MAKECMDGOALS),webots_target)
MAKECMDGOALS = release
endif
endif

ifeq ($(MAKECMDGOALS),distrib)
TARGET = release
export TREAT_WARNINGS_AS_ERRORS=1
else
ifeq ($(MAKECMDGOALS),cleanse)
TARGET = clean
else ifneq ($(filter sim-core sim-gui renderer controller-libs all-controllers tests-smoke benchmarks compile-commands package dev-help,$(MAKECMDGOALS)),)
TARGET = release
else
TARGET = $(MAKECMDGOALS)
endif
endif

null :=
space := $(null) $(null)
OMNISIM_HOME_PATH?=$(subst $(space),\ ,$(strip $(subst \,/,$(OMNISIM_HOME))))
include $(OMNISIM_HOME_PATH)/resources/Makefile.os.include

.PHONY: clean cleanse debug distrib release webots_dependencies webots_target webots_projects clean-docs docs clean-urls sim-core sim-gui renderer controller-libs all-controllers tests-smoke benchmarks compile-commands package dev-help

release debug profile: docs webots_projects

sim-core: docs webots_target

sim-gui:
	@+make --silent -C src/omnisim $(TARGET) OMNISIM_HOME="$(OMNISIM_HOME)"

renderer:
	@+make --silent -C src/wren $(TARGET) OMNISIM_HOME="$(OMNISIM_HOME)"

controller-libs:
	@+make --silent -C src/controller $(TARGET) OMNISIM_HOME="$(OMNISIM_HOME)"

all-controllers: controller-libs
	@+echo "#"; echo "# * project controllers *"; echo "#"
	@+make --silent -C projects $(TARGET) OMNISIM_HOME="$(OMNISIM_HOME)"

tests-smoke:
	@+python3 scripts/dev/omnisim_dev.py test-smoke --nomake

benchmarks:
	@+python3 scripts/dev/omnisim_dev.py benchmarks --nomake

compile-commands:
	@+python3 scripts/dev/omnisim_dev.py compile-commands

package: distrib

dev-help:
	@+echo
	@+$(ECHO) "\033[32;1mOmniSim developer fast-path targets:\033[0m"
	@+echo
	@+$(ECHO) "\033[33;1mmake sim-core\033[0m\t# build the current runtime-oriented core path (legacy webots_target wrapper)"
	@+$(ECHO) "\033[33;1mmake sim-gui\033[0m\t# build the desktop shell target in src/omnisim"
	@+$(ECHO) "\033[33;1mmake renderer\033[0m\t# build the Wren renderer target"
	@+$(ECHO) "\033[33;1mmake controller-libs\033[0m\t# build controller libraries only"
	@+$(ECHO) "\033[33;1mmake all-controllers\033[0m\t# build controller libs + all project controllers/demos"
	@+$(ECHO) "\033[33;1mmake tests-smoke\033[0m\t# run the fast smoke suite"
	@+$(ECHO) "\033[33;1mmake benchmarks\033[0m\t# run the benchmark world set"
	@+$(ECHO) "\033[33;1mmake compile-commands\033[0m\t# generate compile_commands.json using bear"
	@+$(ECHO) "\033[33;1mpython scripts/dev/omnisim_dev.py --help\033[0m\t# see the full developer CLI"

distrib: release
	@+echo "#"; echo "# packaging"; echo "#"
	@+make --silent -C scripts/packaging OMNISIM_HOME="$(OMNISIM_HOME)"
	$(eval DT := `expr \`date +%s\` - $(START)`)
	@printf "# distribution compiled in %d:%02d:%02d\n" $$(($(DT) / 3600)) $$(($(DT) % 3600 / 60)) $$(($(DT) % 60))

ifeq ($(OSTYPE),windows)
CLEAN_IGNORE += -e lib/webots/qt -e include/qt
endif

# we should make clean before building a release
clean: webots_projects clean-docs clean-urls
	@+echo "#"; echo "# * packaging *"; echo "#"
	@+make --silent -C scripts/packaging clean OMNISIM_HOME="$(OMNISIM_HOME)"
	@+echo "#"; echo "# remove OS generated files and text editor backup files"
	@+find . -type f \( -name "*~" -o -name "*.bak" -o -name ".DS_Store" -o -name ".DS_Store?" -o -name ".Spotlight-V100" -o -name ".Trashes" -o -name "__pycache__" -o -name "Thumbs.db" -o -name "ehthumbs.db" \) -exec /bin/rm -f -- {} + -exec echo "# removed" {} +
	@+find . -type d \( -name "__pycache__" \) -exec /bin/rm -rf -- {} + -exec echo "# removed" {} +
ifeq ($(MAKECMDGOALS),clean)
	@+echo "#"; echo "# testing if everything was cleaned..."
	@+git clean -fdfxn -e tests $(CLEAN_IGNORE)
	@+echo "# done"
endif

# cleanse is the ultimate cleansing (agressive cleaning)
cleanse: clean
	@rm -fr docs/index.html docs/dependencies
	@rm -rf $(WEBOTS_DISTRIBUTION_PATH)/*
ifeq ($(OSTYPE),windows)
	@rm -rf msys64
endif
ifeq ($(OSTYPE),darwin)
	@+make --silent -C dependencies -f Makefile.mac $(MAKECMDGOALS) OMNISIM_HOME="$(OMNISIM_HOME)"
endif
	@+echo "#"; echo "# * tests *"; echo "#"
	@find tests -name .*.cache | xargs rm -f
	@+make --silent -C tests clean OMNISIM_HOME="$(OMNISIM_HOME)"
	@+echo "#"; echo "# testing if everything was cleansed..."
	@+git clean -fdfxn $(CLEAN_IGNORE)
	@+echo "# done"

webots_target: webots_dependencies
	@+echo "#"; echo "# * ode *"; echo "#"
	@+make --silent -C src/ode $(TARGET) OMNISIM_HOME="$(OMNISIM_HOME)"
ifeq ($(TARGET),profile)  # a shared version of the library is required for physics-plugins
	@+make --silent -C src/ode release OMNISIM_HOME="$(OMNISIM_HOME)"
endif
	@+echo "#"; echo "# * glad *"; echo "#"
	@+make --silent -C src/glad $(TARGET) OMNISIM_HOME="$(OMNISIM_HOME)"
	@+echo "#"; echo "# * wren *"; echo "#"
	@+make --silent -C src/wren $(TARGET) OMNISIM_HOME="$(OMNISIM_HOME)"
	@+echo "#"; echo "# * webots (core) *"; echo "#"
	@+make --silent -C src/omnisim $(TARGET) OMNISIM_HOME="$(OMNISIM_HOME)"

webots_projects: webots_target
	@+echo "#"; echo "# * controller library *"
	@+make --silent -C src/controller $(TARGET) OMNISIM_HOME="$(OMNISIM_HOME)"
	@+echo "#"; echo "# * resources *"
	@+make --silent -C resources $(MAKECMDGOALS) OMNISIM_HOME="$(OMNISIM_HOME)"
	@+echo "#"; echo "# * projects *"
	@+make --silent -C projects $(TARGET) OMNISIM_HOME="$(OMNISIM_HOME)"

DEPS_GOAL := $(if $(filter sim-core sim-gui renderer controller-libs all-controllers tests-smoke benchmarks compile-commands package dev-help,$(MAKECMDGOALS)),release,$(MAKECMDGOALS))

webots_dependencies:
	@+echo "#"; echo "# * dependencies *"; echo "#"
ifeq ($(OSTYPE),darwin)
	@+make --silent -C dependencies -f Makefile.mac $(DEPS_GOAL) OMNISIM_HOME="$(OMNISIM_HOME)"
endif
ifeq ($(OSTYPE),linux)
	@+make --silent -C dependencies -f Makefile.linux $(DEPS_GOAL) OMNISIM_HOME="$(OMNISIM_HOME)"
endif
ifeq ($(OSTYPE),windows)
	@+make --silent -C dependencies -f Makefile.windows $(DEPS_GOAL) OMNISIM_HOME="$(OMNISIM_HOME)"
endif
ifneq ($(TARGET),clean)
	@+python3 scripts/packaging/generate_proto_list.py
else
	@+rm -f resources/proto-list.xml
endif

ifeq ($(OSTYPE),darwin)
NUMBER_OF_PROCESSORS = `sysctl -n hw.ncpu`
else
NUMBER_OF_PROCESSORS ?= `grep -c ^processor /proc/cpuinfo`
endif
THREADS = $$(($(NUMBER_OF_PROCESSORS) * 3 / 2))

docs:
	@$(OMNISIM_HOME_PATH)/scripts/get_git_info/get_git_info.sh
	@$(shell find $(OMNISIM_HOME_PATH)/docs -name '*.md' | sed 's/.*docs[/]//' > $(OMNISIM_HOME_PATH)/docs/list.txt)

clean-docs:
	@+echo "#"; echo "# * documentation *"
	@-rm -f docs/list.txt

clean-urls:
	@+echo "#"; echo "# * clean URLs *"
	@+python3 scripts/packaging/update_urls.py webots

install:
	@+echo "#"; echo "# * installing (snap) *"
	@+make --silent -C scripts/packaging -f Makefile install OMNISIM_HOME="$(OMNISIM_HOME)"

help:
	@+echo
	@+$(ECHO) "\033[32;1mOmniSim Makefile targets:\033[0m"
	@+echo
	@+$(ECHO) "\033[33;1mmake -j$(THREADS) release\033[0m\t# compile with maximum optimization (default)"
	@+$(ECHO) "\033[33;1mmake -j$(THREADS) debug\033[0m  \t# compile with gdb debugging symbols"
	@+$(ECHO) "\033[33;1mmake -j$(THREADS) profile\033[0m\t# compile with gprof profiling information"
	@+$(ECHO) "\033[33;1mmake -j$(THREADS) distrib\033[0m\t# compile in release mode & create distribution package"
	@+$(ECHO) "\033[33;1mmake -j$(THREADS) clean\033[0m  \t# clean-up the compilation output"
	@+$(ECHO) "\033[33;1mmake -j$(THREADS) cleanse\033[0m\t# deep clean-up (dependencies are also removed)"
	@+$(ECHO) "\033[33;1mmake sim-core\033[0m\t\t# build the current runtime-oriented core path"
	@+$(ECHO) "\033[33;1mmake sim-gui\033[0m\t\t# build the main desktop simulator target"
	@+$(ECHO) "\033[33;1mmake renderer\033[0m\t\t# build the renderer target"
	@+$(ECHO) "\033[33;1mmake controller-libs\033[0m\t# build controller libraries only"
	@+$(ECHO) "\033[33;1mmake tests-smoke\033[0m\t# run the fast smoke suite"
	@+$(ECHO) "\033[33;1mmake help\033[0m\t\t# display this message and exit"
	@+echo
	@+$(ECHO) "\033[32;1mNote:\033[0m You seem to have a processor with $(NUMBER_OF_PROCESSORS) virtual cores,"
	@+$(ECHO) "      hence the \033[33;1m-j$(THREADS)\033[0m option to speed-up the compilation."
