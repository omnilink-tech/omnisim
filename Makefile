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

# Interpreter for the pure-python test lanes. `python3` is NOT a safe hardcode:
# under the MSYS2 build shell it resolves to msys64/mingw64/bin/python3.exe
# (3.14.3), which carries neither pytest nor Pillow. Measured 2026-08-16 on a
# working clone, `make tests-docs` failed there with "No module named pytest"
# -- the target ran the wrong python, not a broken suite. Probe for one that
# can import both; PYTHON=<interpreter> overrides.
PYTHON ?= $(shell for p in python3 python py; do \
	command -v $$p >/dev/null 2>&1 && $$p -c 'import pytest, PIL' >/dev/null 2>&1 && \
	{ echo $$p; break; }; done)

ifneq ($(CCACHE),)
ifneq ($(USE_CCACHE),0)
CC := $(CCACHE) $(CC)
CXX := $(CCACHE) $(CXX)
export CC
export CXX
endif
endif

# OMNISIM_HOME is the canonical install-root variable post-rebrand.
# When unset, derive it from the current directory (this Makefile's location).
#
# WEBOTS_HOME is the pre-rebrand name and is the one legacy variable a human may
# still have exported in a shell profile. A top-level build has never actually
# read it -- OMNISIM_HOME is derived from this Makefile's own directory, which is
# the tree you asked to build -- so honouring WEBOTS_HOME here could silently
# build a DIFFERENT clone. Warn instead, and name the replacement. (Standalone
# sub-makes are the case where the legacy name genuinely still resolves the root;
# resources/Makefile.include and resources/Makefile.os.include adopt it there.)
ifeq ($(OMNISIM_HOME),)
ifneq ($(WEBOTS_HOME),)
$(warning WEBOTS_HOME is set but OMNISIM_HOME is not. WEBOTS_HOME is the pre-rebrand name and is not read by this Makefile; OMNISIM_HOME is being derived from this directory instead. Export OMNISIM_HOME=<clone root> to silence this warning.)
endif
ifneq ($(findstring MINGW,$(shell uname)),) # under MINGW, native Windows format is required
export OMNISIM_HOME := $(shell pwd -W | tr -s / '\\')
else
# $(PWD) is the SHELL's variable and is only present when the shell exports it.
# Fall back to $(CURDIR), which make always defines, so this branch can never
# leave OMNISIM_HOME empty -- an empty root silently becomes "-I/include" and a
# path rooted at "/", and it would also let the resources/*.include legacy-alias
# adoption below override the derivation this warning just promised.
export OMNISIM_HOME := $(if $(PWD),$(PWD),$(CURDIR))
endif
endif

# Normalize absolute source/include paths relative to the clone root before
# hashing them.  This lets release-build entries be reused from another checkout
# with the same tree layout while preserving ccache's normal compiler, flags and
# dependency validation.  A caller-provided CCACHE_BASEDIR still wins.
ifneq ($(CCACHE),)
ifneq ($(USE_CCACHE),0)
CCACHE_BASEDIR ?= $(OMNISIM_HOME)
CCACHE_DIR ?= $(OMNISIM_HOME)/.build_tmp/ccache
export CCACHE_BASEDIR
export CCACHE_DIR
endif
endif

# Hand the install root to every sub-make (src/omnisim/Makefile,
# dependencies/Makefile.*, resources/Makefile.os.include and ~15 others) and to
# every script the recipes spawn. The derivation above already exports
# OMNISIM_HOME; this covers the case where it arrived on the make command line.
# Both names are build-internal and never user-visible.
#
# Note OMNISIM_PATH is currently inert: its only consumer, src/omnisim/Makefile,
# assigns it itself ("OMNISIM_PATH = ../.."), and a makefile assignment beats the
# environment. It is exported for the benefit of any future sub-make that does
# not.
export OMNISIM_HOME
export OMNISIM_PATH := $(OMNISIM_HOME)

include resources/Makefile.os.include

# ---- ODE is GONE ------------------------------------------------------------
# src/ode and include/ode were deleted (106,283 lines). Newton is the only
# physics backend. OMNISIM_WITH_ODE survives for exactly one job: refusing a
# stale `OMNISIM_WITH_ODE=ON` invocation with a sentence instead of a thousand
# missing-header errors. Scripts and CI that still pass =OFF keep working.
OMNISIM_WITH_ODE ?= OFF
ifeq ($(OMNISIM_WITH_ODE),ON)
$(error OMNISIM_WITH_ODE=ON is no longer buildable: src/ode was deleted and Newton is the only physics backend. Drop the flag.)
endif

# Same precedence scripts/packaging/generic_distro.py applies at runtime:
# OMNISIM_DISTRIBUTION_PATH wins, WEBOTS_DISTRIBUTION_PATH is the legacy alias
# still set by some external scripts and CI. `cleanse` deletes this directory's
# contents, so it must resolve to the same place the distro scripts write to.
OMNISIM_DISTRIBUTION_PATH ?= $(if $(WEBOTS_DISTRIBUTION_PATH),$(WEBOTS_DISTRIBUTION_PATH),$(OMNISIM_HOME)/distribution)

ifeq ($(MAKECMDGOALS),)
MAKECMDGOALS = release
else
ifneq ($(filter omnisim_target webots_target,$(MAKECMDGOALS)),)
MAKECMDGOALS = release
endif
endif

ifeq ($(MAKECMDGOALS),distrib)
TARGET = release
export TREAT_WARNINGS_AS_ERRORS=1
else
ifeq ($(MAKECMDGOALS),cleanse)
TARGET = clean
else ifneq ($(filter sim-core sim-gui sim-check-objects sim-gui-staged renderer controller-libs all-controllers tests-smoke benchmarks compile-commands package dev-help,$(MAKECMDGOALS)),)
TARGET = release
else
TARGET = $(MAKECMDGOALS)
endif
endif

null :=
space := $(null) $(null)
OMNISIM_HOME_PATH?=$(subst $(space),\ ,$(strip $(subst \,/,$(OMNISIM_HOME))))
include $(OMNISIM_HOME_PATH)/resources/Makefile.os.include

.PHONY: clean cleanse debug distrib release omnisim_dependencies omnisim_target omnisim_projects clean-docs docs clean-urls sim-core sim-gui sim-check-objects sim-gui-staged renderer controller-libs all-controllers tests-smoke tests-docs tests-sources benchmarks compile-commands package dev-help ccache-stats

release debug profile: docs omnisim_projects

sim-core: docs omnisim_target

sim-gui:
	@+make --silent -C src/omnisim $(TARGET) OMNISIM_HOME="$(OMNISIM_HOME)"

sim-check-objects:
	@+make --silent -C src/omnisim check-objects CHECK_OBJECTS="$(CHECK_OBJECTS)" OMNISIM_HOME="$(OMNISIM_HOME)"

# Link to a sibling executable so a running GUI never blocks the agent build.
# Activation is deliberately a separate explicit CLI operation.
sim-gui-staged:
	@+make --silent -C src/omnisim release \
		TARGET="$(OMNISIM_HOME_PATH)/msys64/mingw64/bin/omnisim-bin.next.exe" \
		OMNISIM_HOME="$(OMNISIM_HOME)"

controller-libs:
	@+make --silent -C src/controller $(TARGET) OMNISIM_HOME="$(OMNISIM_HOME)"

all-controllers: controller-libs
	@+echo "#"; echo "# * project controllers *"; echo "#"
	@+make --silent -C projects $(TARGET) OMNISIM_HOME="$(OMNISIM_HOME)"

tests-smoke:
	@+python3 scripts/dev/omnisim_dev.py test-smoke --nomake

# Documentation structure gate: links resolve, anchors resolve, menus complete,
# images used. ~2 s, no engine, no GPU. It is EXCLUDED from a bare pytest by
# pytest.ini's norecursedirs, so without this target nothing runs it -- which is
# how it sat dead for months while accumulating three independent breakages.
tests-docs:
ifeq ($(PYTHON),)
	@echo "tests-docs: no python with pytest+Pillow on PATH. Set PYTHON=<interpreter>." && false
else
	@+$(PYTHON) -m pytest docs/tests -q
endif

# Source lint gate: licences, header versions, the naming ratchet, PROTO /
# texture / world-layout conventions, URDF import, the Windows SxS manifest.
# ~8.5 min, no engine, no GPU -- dominated by test_header_version, which runs
# one `git check-ignore` per candidate file across three full tree walks.
# EXCLUDED from a bare pytest by pytest.ini's norecursedirs (for cost), so
# without this target nothing runs it -- which is how 28 lint tests sat
# un-executed for months while two dev scripts carried comments assuming they
# were green.
#
# test_pep8 and test_line_ending are deliberately NOT in this lane:
#   test_pep8        would lint 12,222 files under the vendored msys64/ tree.
#   test_line_ending is structurally impossible under core.autocrlf=true, which
#                    makes 3,786 tracked files CRLF in the worktree. CI-only.
# Do NOT swap this for `python tests/test_sources.py`: that runner does a
# unittest discover over sources/*.py and would pull both back in.
tests-sources:
ifeq ($(PYTHON),)
	@echo "tests-sources: no python with pytest+Pillow on PATH. Set PYTHON=<interpreter>." && false
else
	@+$(PYTHON) -m pytest tests/sources -q \
		--ignore=tests/sources/test_pep8.py \
		--ignore=tests/sources/test_line_ending.py
endif

benchmarks:
	@+python3 scripts/dev/omnisim_dev.py benchmarks --nomake

compile-commands:
	@+python3 scripts/dev/omnisim_dev.py compile-commands

ccache-stats:
ifeq ($(CCACHE),)
	@echo "ccache is not installed or is not on PATH"
else ifeq ($(USE_CCACHE),0)
	@echo "ccache is installed at $(CCACHE), but USE_CCACHE=0 disables it for this build"
else
	@echo "# ccache configuration (CCACHE_BASEDIR=$(CCACHE_BASEDIR), CCACHE_DIR=$(CCACHE_DIR))"
	@$(CCACHE) -p
	@echo "# ccache statistics"
	@$(CCACHE) -s
endif

package: distrib

dev-help:
	@+echo
	@+$(ECHO) "\033[32;1mOmniSim developer fast-path targets:\033[0m"
	@+echo
	@+$(ECHO) "\033[33;1mmake sim-core\033[0m\t# build the current runtime-oriented core path (wraps omnisim_target)"
	@+$(ECHO) "\033[33;1mmake sim-gui\033[0m\t# build the desktop shell target in src/omnisim"
	@+$(ECHO) "\033[33;1mmake sim-check-objects CHECK_OBJECTS='Foo.o'\033[0m\t# compile selected simulator objects only"
	@+$(ECHO) "\033[33;1mmake sim-gui-staged\033[0m\t# link omnisim-bin.next without touching a running binary"
	@+$(ECHO) "\033[33;1mmake controller-libs\033[0m\t# build controller libraries only"
	@+$(ECHO) "\033[33;1mmake all-controllers\033[0m\t# build controller libs + all project controllers/demos"
	@+$(ECHO) "\033[33;1mmake tests-smoke\033[0m\t# run the fast smoke suite"
	@+$(ECHO) "\033[33;1mmake benchmarks\033[0m\t# run the benchmark world set"
	@+$(ECHO) "\033[33;1mmake compile-commands\033[0m\t# generate compile_commands.json using bear"
	@+$(ECHO) "\033[33;1mmake ccache-stats\033[0m\t# show cache configuration, hits, misses, and uncacheable reasons"
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
clean: omnisim_projects clean-docs clean-urls
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
	@rm -rf $(OMNISIM_DISTRIBUTION_PATH)/*
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

omnisim_target: omnisim_dependencies
	@+echo "#"; echo "# * glad *"; echo "#"
	@+make --silent -C src/glad $(TARGET) OMNISIM_HOME="$(OMNISIM_HOME)"
	@+echo "#"; echo "# * omnisim (core) *"; echo "#"
	@+make --silent -C src/omnisim $(TARGET) OMNISIM_HOME="$(OMNISIM_HOME)"

omnisim_projects: omnisim_target
	@+echo "#"; echo "# * controller library *"
	@+make --silent -C src/controller $(TARGET) OMNISIM_HOME="$(OMNISIM_HOME)"
	@+echo "#"; echo "# * resources *"
	@+make --silent -C resources $(MAKECMDGOALS) OMNISIM_HOME="$(OMNISIM_HOME)"
	@+echo "#"; echo "# * projects *"
	@+make --silent -C projects $(TARGET) OMNISIM_HOME="$(OMNISIM_HOME)"

DEPS_GOAL := $(if $(filter sim-core sim-gui sim-check-objects sim-gui-staged renderer controller-libs all-controllers tests-smoke benchmarks compile-commands package dev-help,$(MAKECMDGOALS)),release,$(MAKECMDGOALS))

omnisim_dependencies:
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

# legacy aliases
.PHONY: webots_target webots_projects webots_dependencies
webots_target: omnisim_target
webots_projects: omnisim_projects
webots_dependencies: omnisim_dependencies

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
	@+$(ECHO) "\033[33;1mmake controller-libs\033[0m\t# build controller libraries only"
	@+$(ECHO) "\033[33;1mmake tests-smoke\033[0m\t# run the fast smoke suite"
	@+$(ECHO) "\033[33;1mmake help\033[0m\t\t# display this message and exit"
	@+echo
	@+$(ECHO) "\033[32;1mNote:\033[0m You seem to have a processor with $(NUMBER_OF_PROCESSORS) virtual cores,"
	@+$(ECHO) "      hence the \033[33;1m-j$(THREADS)\033[0m option to speed-up the compilation."
