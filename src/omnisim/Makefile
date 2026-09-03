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

OMNISIM_PATH = ../..

include $(OMNISIM_PATH)/resources/Makefile.os.include

# `check-objects` is a release-mode compile-only goal.  MAKECMDGOALS itself is
# immutable, so use this logical goal anywhere mode controls flags or paths.
BUILD_GOAL := $(if $(filter check-objects,$(MAKECMDGOALS)),release,$(MAKECMDGOALS))

# Force-export TMP/TEMP to recipe sub-shells (msys make otherwise drops them,
# leaving g++/ld trying C:\Windows for temp files -> Permission denied builds).
export TMP ?= C:/tmp/build_tmp
export TEMP ?= C:/tmp/build_tmp
export TMPDIR ?= C:/tmp/build_tmp

# WEBOTS_DEPENDENCY_PATH is the pre-rebrand name and is still honoured when set.
OMNISIM_DEPENDENCY_PATH ?= $(if $(WEBOTS_DEPENDENCY_PATH),$(WEBOTS_DEPENDENCY_PATH),$(OMNISIM_PATH)/dependencies)

# compiler
ifeq ($(CC),cc)
CC = gcc
endif

# ---- ccache, for the STANDALONE sub-make too --------------------------------
# The top-level Makefile prefixes CC/CXX with ccache and exports them, so a
# top-level `make` is cached.  But the documented fast iteration path is a
# standalone `make -C src/omnisim` (and `make release` run from this directory),
# which never reads the top-level Makefile -- it resolved a bare `g++` and every
# recompile paid full price.  MEASURED 2026-08-14 on machine 9722d23d12a3:
# the repo's ccache had THREE cacheable calls in its entire lifetime, because
# essentially every real build takes this path.
#
# A hit is worth far more than the miss penalty: nodes/OmCloth.cpp 2330 -> 135 ms
# and physics/OmNewtonBackend.cpp 3481 -> 151 ms (17-23x), against a miss that
# costs ~1.4x a plain compile.  That trade pays on every branch switch, rebase,
# revert and A/B -- constant traffic in a tree several lanes share -- and is
# neutral-to-negative only on a from-scratch build of never-before-seen code.
#
# Only wire it when this Makefile is the entry point: if CC already contains
# ccache the top level did it, and prefixing twice would run ccache on ccache.
# USE_CCACHE=0 opts out, matching the top-level Makefile's own switch.
#
# ⚠ TWO WINDOWS FOOTGUNS, both learned by tripping over them:
#   1. ccache REQUIRES USERPROFILE and exits 1 without it -- and msys make
#      drops it from recipe sub-shells, exactly like TMP/TEMP above.  A
#      recipe that shells out to ccache then dies with "The USERPROFILE
#      environment variable must be set", i.e. wiring in a cache would BREAK
#      the build rather than speed it up.  CCACHE_CONFIGPATH does not
#      substitute (measured: still rc=1).  Force-export it, same remedy this
#      Makefile already applies to TMP/TEMP/TMPDIR.
#   2. Never let this optimisation be able to fail the build.  Probe the real
#      thing -- with the env it will actually run under -- and wire it in only
#      when it genuinely works.  A missing, broken or misconfigured ccache
#      silently leaves a plain compiler behind instead of a red build.
#      MEASURED: msys make launched from a non-cmd shell (git-bash, a CI
#      runner, an agent harness) inherits almost NO Windows environment --
#      USERPROFILE, APPDATA, LOCALAPPDATA and TEMP all arrive EMPTY and HOME
#      is the msys /home/<user>, not the Windows profile.  A cmd.exe build via
#      build_omni.bat does have USERPROFILE, so this would have worked for
#      humans and silently not for everything else.  Supply one when it is
#      missing: ccache only needs the variable to name a writable directory,
#      and CCACHE_DIR below already decides where the cache really lives.
#      ⚠ It needs BOTH USERPROFILE *and* LOCALAPPDATA, and it reports them one
#      at a time -- fix USERPROFILE and the next run fails with "could not find
#      configuration file and the LOCALAPPDATA environment variable is not
#      set".  Supplying only the one named in the error looks like a fix and
#      is not.  CCACHE_CONFIGPATH does NOT substitute for either (measured).
OM_CCACHE_HOME := $(abspath $(OMNISIM_PATH))/.build_tmp
ifeq ($(strip $(USERPROFILE)),)
  USERPROFILE := $(OM_CCACHE_HOME)
endif
ifeq ($(strip $(LOCALAPPDATA)),)
  LOCALAPPDATA := $(OM_CCACHE_HOME)
endif
export USERPROFILE
export LOCALAPPDATA
# ---- cached toolchain probes ------------------------------------------------
# One `sh` spawn answers every environment probe below (ccache, uname, nvcc +
# cygpath, ld.lld, the gcc-version expr, the Windows python prefixes) from a
# fragment build_probes.sh regenerates only when PATH, CXX, OSTYPE, the
# script, this Makefile or a resolved toolchain binary's mtime changes. They
# used to be ~10 parse-time $(shell) calls costing ~1.1 s of every make.
# OMNISIM_PROBE_CACHE=0 runs the probes inline as before.
OMNISIM_PROBE_CACHE ?= 1
ifneq ($(OMNISIM_PROBE_CACHE),0)
OM_PROBES_MK := $(shell sh ./build_probes.sh "$(OMNISIM_PATH)/.build_tmp" "$(CXX)" "$(OSTYPE)")
-include $(OM_PROBES_MK)
endif

USE_CCACHE ?= 1
ifneq ($(USE_CCACHE),0)
ifeq ($(findstring ccache,$(CC)),)
ifdef OM_PROBES_CACHED
OM_CCACHE := $(OM_PROBE_CCACHE)
else
OM_CCACHE := $(shell command -v ccache >/dev/null 2>&1 && ccache $(CXX) --version >/dev/null 2>&1 && command -v ccache 2>/dev/null)
endif
ifneq ($(OM_CCACHE),)
CC  := $(OM_CCACHE) $(CC)
CXX := $(OM_CCACHE) $(CXX)
# Normalize absolute paths so entries stay valid across checkouts, and keep the
# cache inside the tree next to the top level's (same CCACHE_DIR, one cache).
CCACHE_BASEDIR ?= $(abspath $(OMNISIM_PATH))
CCACHE_DIR ?= $(abspath $(OMNISIM_PATH))/.build_tmp/ccache
export CCACHE_BASEDIR
export CCACHE_DIR
endif
endif
endif

# module dependencies: please keep these lists of dependencies up-to-date so that we don't break dependencies in the code
# listed from the module with the less dependencies to the module with the more dependencies

WB_MATHS_INCLUDE         = $(QT_CORE_INCLUDE)
WB_CORE_INCLUDE          = $(QT_CORE_INCLUDE) $(QT_NETWORK_INCLUDE) $(QT_GUI_INCLUDE) $(QT_QML_INCLUDE) $(CONTROLLER_INCLUDE) -Iqml -Iexternal/compilation_timestamp
WB_UTIL_INCLUDE          = $(QT_CORE_INCLUDE) $(CONTROLLER_INCLUDE) -Isound -Icore
WB_VRML_INCLUDE          = $(QT_CORE_INCLUDE) $(QT_XML_INCLUDE) $(CONTROLLER_INCLUDE) -Icore -Imaths -Ivrml
WB_USER_COMMANDS_INCLUDE = $(QT_CORE_INCLUDE) $(QT_GUI_INCLUDE) $(QT_WIDGETS_INCLUDE) $(QT_NETWORK_INCLUDE) $(CONTROLLER_INCLUDE) -Icore -isystem external/siphash -Imaths -Inodes -Inodes/utils -Iphysics -Iuser_commands -Ivrml
WB_WIDGETS_INCLUDE       = $(QT_CORE_INCLUDE) $(QT_GUI_INCLUDE) $(QT_WIDGETS_INCLUDE) $(CONTROLLER_INCLUDE) -Icore -Imaths -Iuser_commands -Ivrml
WB_EDITOR_INCLUDE        = $(QT_CORE_INCLUDE) $(QT_GUI_INCLUDE) $(QT_WIDGETS_INCLUDE) $(QT_PRINT_SUPPORT_INCLUDE) $(CONTROLLER_INCLUDE) -Icore -Iwidgets -Imaths -Inodes -Inodes/utils -Iphysics -Iuser_commands -Ivrml
WB_ENGINE_INCLUDE        = $(QT_CORE_INCLUDE) $(QT_NETWORK_INCLUDE) $(INSTALL_INCLUDE) $(CONTROLLER_INCLUDE) -Icore -Imaths -Inodes -Inodes/utils -Iphysics -Irender -Isound -Iuser_commands -Iutil -Ivrml
WB_CONTROL_INCLUDE       = $(QT_CORE_INCLUDE) $(QT_NETWORK_INCLUDE) $(CONTROLLER_INCLUDE) -Icore -Iengine -Imaths -Inodes -Inodes/utils -Iphysics -Ivrml -Iutil
# dependency of NODES on QT_GUI is necessary in OmImageTexture to retrieve the size of a texture before loading it in Wren
WB_NODES_INCLUDE         = $(QT_CORE_INCLUDE) $(QT_NETWORK_INCLUDE) $(QT_GUI_INCLUDE) $(CONTROLLER_INCLUDE) $(OIS_INCLUDE) $(FREETYPE_INCLUDE) $(INSTALL_INCLUDE) $(STB_INCLUDE) $(ASSIMP_INCLUDE) $(CUDA_INCLUDE) -Iapp -Icompute/cuda -Icore -Iengine -isystem external/siphash -Imaths -Inodes -Inodes/utils -Iphysics -Irender -Isound -Iutil -Ivrml
WB_APP_INCLUDE           = $(QT_CORE_INCLUDE) $(QT_NETWORK_INCLUDE) $(INSTALL_INCLUDE) $(CONTROLLER_INCLUDE) -Icompute/cuda -Icontrol -Icore -Ieditor -Iengine -Imaths -Inodes -Inodes/utils -Iphysics -Iscene_tree -Ivrml -Irender
WB_SCENE_TREE_INCLUDE    = $(QT_CORE_INCLUDE) $(QT_GUI_INCLUDE) $(QT_WIDGETS_INCLUDE) $(ASSIMP_INCLUDE) $(CONTROLLER_INCLUDE) -isystem external/siphash -Iapp -Icompute/cuda -Icore -Ieditor -Imaths -Inodes -Inodes/utils -Iphysics -Iuser_commands -Ivrml -Iwidgets
WB_SOUND_INCLUDE         = $(QT_CORE_INCLUDE) $(QT_XML_INCLUDE) $(CONTROLLER_INCLUDE) -Icore -Imaths -Inodes -Inodes/utils -Iphysics -Isound -Ivrml
WB_GUI_INCLUDE           = $(QT_CORE_INCLUDE) $(QT_NETWORK_INCLUDE) $(QT_WEBSOCKETS_INCLUDE) $(QT_GUI_INCLUDE) $(QT_WIDGETS_INCLUDE) $(QT_OPENGL_INCLUDE) $(QT_OPENGL_WIDGETS_INCLUDE) $(INSTALL_INCLUDE) $(CONTROLLER_INCLUDE) -Iapp -Icontrol -Icore -Iwidgets -Ieditor -Iengine -isystem external/siphash -Imaths -Inodes -Inodes/utils -Iphysics -Irender -Iscene_tree -Isound -Iuser_commands -Iutil -Ivrml
WB_RENDER_INCLUDE        = $(QT_CORE_INCLUDE) $(QT_GUI_INCLUDE) $(QT_OPENGL_INCLUDE) $(QT_WIDGETS_INCLUDE) $(INSTALL_INCLUDE) -isystem external/siphash

ifdef OM_PROBES_CACHED
HOSTTYPE ?= $(OM_PROBE_HOSTTYPE)
else
HOSTTYPE ?= $(shell uname -m)
endif

# clear out all suffixes for implicit rules to speed up Makefile
.SUFFIXES:
%:: %,v
%:: RCS/%,v
%:: RCS/%
%:: s.%
%:: SCCS/s.%

OIS_INCLUDE = -isystem $(OMNISIM_PATH)/include/libOIS
# D1.0 (WREN-deletion runbook): the install-root include, under its OWN name. This -I is the
# ONLY path reaching include/glad/glad.h (the GL present fallback L1 depends on), and it used
# to be smuggled in as WREN_INCLUDE -- so deleting the WREN variable would have silently
# stopped glad compiling. INSTALL_INCLUDE survives the deletion; WREN_INCLUDE now carries only
# the -Iwren shim-dir path (appended near the INCLUDE definition below) and dies with D1.4.
INSTALL_INCLUDE = -I$(OMNISIM_PATH)/include
STB_INCLUDE  = -isystem $(OMNISIM_PATH)/src/stb
OPEN_VR_INCLUDE = -isystem $(OMNISIM_PATH)/include/openvr

ifeq ($(OSTYPE),darwin)
EXECUTABLE_PATH    = $(OMNISIM_PATH)/Contents/MacOS
ifeq ($(wildcard $(EXECUTABLE_PATH)),)
EXECUTABLE_PATH_SETUP := $(shell mkdir -p $(EXECUTABLE_PATH))
endif
FRAMEWORKS         = $(OMNISIM_PATH)/Contents/Frameworks
TARGET             = $(EXECUTABLE_PATH)/webots
# D1.0: glad linked under its OWN variable. LIB_WREN used to expand to BOTH archives, so
# deleting it (D1.4) would silently unlink glad and break the GL present fallback.
# Order matters: wren references glad symbols, so LIB_GLAD stays after LIB_WREN in LIBS.
LIB_GLAD           = ../glad/libglad.a
QT_CORE_INCLUDE    = -isystem $(FRAMEWORKS)/QtCore.framework/Headers
QT_GUI_INCLUDE     = -isystem $(FRAMEWORKS)/QtGui.framework/Headers
QT_WIDGETS_INCLUDE = -isystem $(FRAMEWORKS)/QtWidgets.framework/Headers
QT_OPENGL_INCLUDE  = -isystem $(FRAMEWORKS)/QtOpenGL.framework/Headers
QT_OPENGL_WIDGETS_INCLUDE = -isystem $(FRAMEWORKS)/QtOpenGLWidgets.framework/Headers
QT_PRINT_SUPPORT_INCLUDE = -isystem $(FRAMEWORKS)/QtPrintSupport.framework/Headers
QT_QML_INCLUDE     = -isystem $(FRAMEWORKS)/QtQml.framework/Headers
QT_NETWORK_INCLUDE = -isystem $(FRAMEWORKS)/QtNetwork.framework/Headers
QT_WEBSOCKETS_INCLUDE = -isystem $(FRAMEWORKS)/QtWebSockets.framework/Headers
QT_XML_INCLUDE     = -isystem $(FRAMEWORKS)/QtXml.framework/Headers
FREETYPE_INCLUDE   = -isystem $(OMNISIM_DEPENDENCY_PATH)/freetype2
ASSIMP_INCLUDE     = -I$(OMNISIM_DEPENDENCY_PATH)/assimp/include
LD_FLAGS           = -mmacosx-version-min=$(MACOSX_MIN_SDK_VERSION) -L$(OMNISIM_LIB_PATH) -lOIS -lassimp -Xlinker -rpath -Xlinker @loader_path/../..
MOC                =  $(OMNISIM_PATH)/bin/qt/moc
LIBS_FRAMEWORKS    = -F$(FRAMEWORKS) -framework OpenAL -framework OpenGL -framework QtCore -framework QtGui -framework QtPrintSupport -framework QtQml -framework QtNetwork -framework QtOpenGL -framework QtOpenGLWidgets -framework QtWebSockets -framework QtWidgets -framework QtXml -framework IOKit -framework CoreFoundation -framework Cocoa
LIBS              += -bind_at_load $(LIBS_FRAMEWORKS) $(LIB_GLAD)
CFLAGS             = -isysroot $(MACOSX_SDK_PATH) -mmacosx-version-min=$(MACOSX_MIN_SDK_VERSION)
CXXFLAGS           = -std=c++11 -F$(FRAMEWORKS) -x objective-c++
# macOS is an untested platform (see AGENTS.md) and its TARGET is literally
# Contents/MacOS/webots, so the install layout is deliberately left alone here:
# only the source script's filename changed. See the launcher/omnisim-mac.sh
# header for the remaining gap.
EXTRA_CMD          = cp launcher/omnisim-mac.sh $(OMNISIM_PATH)/webots && chmod 755 $(OMNISIM_PATH)/webots
FILES_TO_REMOVE    = $(OMNISIM_PATH)/webots
ifeq ($(BUILD_GOAL), profile)
LD_FLAGS += -lpthread
endif
else ifeq ($(OSTYPE),linux)
# D1.0: glad linked under its OWN variable. LIB_WREN used to expand to BOTH archives, so
# deleting it (D1.4) would silently unlink glad and break the GL present fallback.
# Order matters: wren references glad symbols, so LIB_GLAD stays after LIB_WREN in LIBS.
LIB_GLAD           = ../glad/libglad.a
ASSIMP_INCLUDE     = -I$(OMNISIM_PATH)/include/libassimp/include
QT_CORE_INCLUDE    = -isystem $(OMNISIM_PATH)/include/qt/QtCore
QT_GUI_INCLUDE     = -isystem $(OMNISIM_PATH)/include/qt/QtGui
# Wayland surface creation (gui/OmWgpuView.cpp): Qt publishes wl_display through
# its PUBLIC native-interface header (QtGui/qguiapplication_platform.h) but NOT
# wl_surface -- only the QPA native-resource accessor returns that, and it lives
# in Qt's PRIVATE headers, which the mirror ships under a VERSION directory that
# is not on the default search path. Discovered by wildcard so no Qt version is
# hard-coded, and appended lazily (recursive +=) so it does not depend on where
# OMNISIM_PATH is defined. When the private headers are absent the wildcard is
# empty, this adds NOTHING, and OmWgpuView.cpp's __has_include guard compiles
# the Wayland branch out -- a Wayland session then just keeps the readback +
# GL-blit presentation path. X11/xcb needs none of this (public headers only),
# and this is inside the linux branch so the Windows/macOS builds are untouched.
QT_GUI_INCLUDE    += $(addprefix -isystem ,$(patsubst %/qpa/qplatformnativeinterface.h,%,$(firstword $(wildcard $(OMNISIM_PATH)/include/qt/QtGui/QtGui/*/QtGui/qpa/qplatformnativeinterface.h))))
QT_WIDGETS_INCLUDE = -isystem $(OMNISIM_PATH)/include/qt/QtWidgets
QT_PRINT_SUPPORT_INCLUDE = -isystem $(OMNISIM_PATH)/include/qt/QtPrintSupport
QT_QML_INCLUDE     = -isystem $(OMNISIM_PATH)/include/qt/QtQml
QT_OPENGL_INCLUDE  = -isystem $(OMNISIM_PATH)/include/qt/QtOpenGL
QT_OPENGL_WIDGETS_INCLUDE = -isystem $(OMNISIM_PATH)/include/qt/QtOpenGLWidgets
QT_NETWORK_INCLUDE = -isystem $(OMNISIM_PATH)/include/qt/QtNetwork
QT_WEBSOCKETS_INCLUDE = -isystem $(OMNISIM_PATH)/include/qt/QtWebSockets
QT_XML_INCLUDE     = -isystem $(OMNISIM_PATH)/include/qt/QtXml
FREETYPE_INCLUDE   = -I/usr/include/freetype2
TARGET             = $(OMNISIM_PATH)/bin/omnisim-bin
CFLAGS             = -fPIC
CXXFLAGS           = -std=c++11
MOC                = $(OMNISIM_PATH)/bin/qt/moc
LIBS              += $(LIB_GLAD) -Wl,-rpath $(OMNISIM_LIB_PATH) -L$(OMNISIM_LIB_PATH) -lQt6Core -lQt6Network -lQt6Gui -lQt6OpenGL -lQt6OpenGLWidgets -lQt6WebSockets -lQt6Widgets -lQt6PrintSupport -lQt6Qml -lQt6Xml -lopenal -lGL -lGLU -ldl -lOIS -lfreetype -lassimp -lrt
LD_FLAGS           = -rdynamic
# Install the shell launcher under BOTH names, mirroring the Windows pattern
# (omnisim.exe primary + webots.exe byte-identical legacy copy, see $(LAUNCHER)
# / $(LAUNCHER_LEGACY) below):
#   bin/omnisim  the primary entry point.  It goes in bin/ rather than the repo
#                root because a tracked Python package directory `omnisim/`
#                already occupies the root (the `python -m omnisim` CLI), and it
#                is also where Windows puts omnisim.exe.
#   ./webots     the legacy alias, unchanged, so existing scripts keep working.
# The script resolves its install root by locating bin/omnisim-bin, so one file
# serves both paths.
EXTRA_CMD          = cp launcher/omnisim-linux.sh $(OMNISIM_PATH)/bin/omnisim && chmod 755 $(OMNISIM_PATH)/bin/omnisim && cp launcher/omnisim-linux.sh $(OMNISIM_PATH)/webots && chmod 755 $(OMNISIM_PATH)/webots
FILES_TO_REMOVE    = $(OMNISIM_PATH)/bin/omnisim $(OMNISIM_PATH)/webots
LIBS += -lpthread

else ifeq ($(OSTYPE),windows)
vpath %.c ../controller/c
WB_NODES_INCLUDE         += $(OPEN_VR_INCLUDE)
C_SOURCES                += system.c
# split from LIB_WREN (D1.0), see the linux branch note
LIB_GLAD                  = ../glad/glad.a
QT_CORE_INCLUDE           = -isystem $(OMNISIM_PATH)/include/qt/QtCore
QT_GUI_INCLUDE            = -isystem $(OMNISIM_PATH)/include/qt/QtGui
QT_WIDGETS_INCLUDE        = -isystem $(OMNISIM_PATH)/include/qt/QtWidgets
QT_PRINT_SUPPORT_INCLUDE  = -isystem $(OMNISIM_PATH)/include/qt/QtPrintSupport
QT_QML_INCLUDE            = -isystem $(OMNISIM_PATH)/include/qt/QtQml
QT_OPENGL_INCLUDE         = -isystem $(OMNISIM_PATH)/include/qt/QtOpenGL
QT_OPENGL_WIDGETS_INCLUDE = -isystem $(OMNISIM_PATH)/include/qt/QtOpenGLWidgets
QT_NETWORK_INCLUDE        = -isystem $(OMNISIM_PATH)/include/qt/QtNetwork
QT_WEBSOCKETS_INCLUDE     = -isystem $(OMNISIM_PATH)/include/qt/QtWebSockets
QT_XML_INCLUDE            = -isystem $(OMNISIM_PATH)/include/qt/QtXml
ASSIMP_INCLUDE            = -I$(OMNISIM_PATH)/dependencies
FREETYPE_INCLUDE          = -I/mingw64/include/freetype2
TARGET_PATH               = $(OMNISIM_PATH)/msys64/mingw64/bin
TARGET                    = $(TARGET_PATH)/omnisim-bin.exe
CFLAGS                    = -isystem /mingw64/lib/libzip/include
CXXFLAGS                  = -std=c++11 -D_USE_MATH_DEFINES
MOC                       = /mingw64/share/qt6/bin/moc
MOC_FLAGS                += -D_WIN32
LIBS                     += $(LIB_GLAD) -L/mingw64/lib -L/mingw64/bin -L$(TARGET_PATH) -lQt6Core -lQt6Network -lQt6Gui -lQt6OpenGL -lQt6OpenGLWidgets -lQt6WebSockets -lQt6Widgets -lQt6PrintSupport -lQt6Qml -lQt6Xml -lopenal -lopengl32 -liphlpapi -ld3d9 -lgdi32 -lglu32 -lOIS -ldinput8 -ldxguid -lole32 -lsapi -loleaut32 -luuid -lfreetype-6 -lopenvr_api -lassimp-5
LD_FLAGS                  = -Wl,--enable-auto-import
EXTRA_CMD                 = cp launcher/omnisim-bin.exe.config $(TARGET_PATH)/
FILES_TO_REMOVE           = $(TARGET_PATH)/omnisim-bin.exe.config
ifeq ($(BUILD_GOAL),release)
# -fipa-cp-clone included in -O3 seems to cause some stack overflow on Windows,
# in particular in OmController::setProcessEnvironment(), however increasing the
# stack size doesn't seems to help, so we need to remove this optimization
CFLAGS                   += -fno-ipa-cp-clone
LD_FLAGS                 += -Wl,-subsystem,windows
else
EXTRA_CMD                += ; cp launcher/omnisim_debug.bat $(TARGET_PATH)/omnisim_debug.bat
endif
ifeq ($(BUILD_GOAL), profile)
LIBS += -lpthread
endif
endif

ALL_INCLUDE        = -Iapp -Icontrol -Icore -Iwidgets -Ieditor -Iengine -isystem external/siphash -Igui -Imaths -Inodes -Inodes/utils -Iphysics -Irender -Iscene_tree -Isound -Iuser_commands -Ivrml -Iutil -Iexternal/compilation_timestamp -Icompute/cuda

# ---- CUDA compute layer (M0 of cuda-compute-infrastructure-plan.md) ---------
# OMNISIM_WITH_CUDA defaults to ON if `nvcc` is on PATH, OFF otherwise. Users
# can force either with `make OMNISIM_WITH_CUDA=ON` / `=OFF`.
ifdef OM_PROBES_CACHED
OM_NVCC_ON_PATH := $(OM_PROBE_NVCC)
else
OM_NVCC_ON_PATH := $(shell command -v nvcc 2>/dev/null)
endif
ifeq ($(origin OMNISIM_WITH_CUDA), undefined)
  ifneq ($(OM_NVCC_ON_PATH),)
    OMNISIM_WITH_CUDA = ON
  else
    OMNISIM_WITH_CUDA = OFF
  endif
endif

ifeq ($(OMNISIM_WITH_CUDA),ON)
  CUDA_NVCC      ?= nvcc
  # Resolve CUDA_HOME inside a single shell subprocess. We can't use Make's
  # $(dir ...) because the canonical install path contains spaces
  # ("C:\Program Files\NVIDIA GPU Computing Toolkit\...") and $(dir) splits
  # on whitespace. On Windows we further pipe through `cygpath -m -d` to get
  # a short, forward-slashed form (e.g. "C:/PROGRA~1/NVIDIA~2/CUDA/v12.1")
  # that GCC's `-isystem` and the linker accept without quoting.
  ifeq ($(CUDA_NVCC)$(OM_PROBES_CACHED),nvcc1)
    CUDA_HOME    := $(OM_PROBE_CUDA_HOME)
    CUDA_LIB_DIR := $(if $(filter windows,$(OSTYPE)),$(CUDA_HOME)/lib/x64,$(CUDA_HOME)/lib64)
  else
  ifeq ($(OSTYPE),windows)
    CUDA_HOME    := $(shell p="$$(command -v $(CUDA_NVCC) 2>/dev/null)"; [ -n "$$p" ] && cygpath -m -d "$$(dirname "$$(dirname "$$p")")" 2>/dev/null)
    CUDA_LIB_DIR := $(CUDA_HOME)/lib/x64
  else
    CUDA_HOME    := $(shell p="$$(command -v $(CUDA_NVCC) 2>/dev/null)"; [ -n "$$p" ] && dirname "$$(dirname "$$p")")
    CUDA_LIB_DIR := $(CUDA_HOME)/lib64
  endif
  endif
  CUDA_INCLUDE    = -isystem $(CUDA_HOME)/include
  CFLAGS += -DOMNISIM_WITH_CUDA
  CXXFLAGS += -DOMNISIM_WITH_CUDA
  # cudart    — runtime API (cudaMalloc, cudaMemcpy, ...) used by OmCudaBuffer
  # cuda      — driver API (cuLaunchKernel, cuModuleLoadDataEx, ...) used by
  #             OmGranularGroup so we can dispatch NVRTC-compiled kernels
  #             without dragging nvcc-compiled .obj files into the mingw link.
  # nvrtc     — runtime kernel compilation, sidesteps the MSVC/mingw ABI gap.
  LIBS += -L$(CUDA_LIB_DIR) -lcudart -lcuda -lnvrtc
  ALL_INCLUDE += $(CUDA_INCLUDE)
endif

# ---- NVIDIA Newton physics backend (P0 of cuda-newton-physics-plan.md) ------
# DEFAULT BACKEND as of the Stage 3 flip (see the OMNISIM_WITH_NEWTON block
# below). There is no other backend.
# OMNISIM_WITH_NEWTON=ON compiles the Newton backend in; the runtime field
# `physicsBackend "newton"` (or the flipped `"auto"` default) opts a Solid into
# the GPU solver. With the flag OFF, every Newton-related .cpp file is a no-op
# stub and the binary has NO physics at all.
#
# Newton itself is a Python package (newton + warp-lang on PyPI; the former
# was renamed from newton-physics in late 2025). Driven from C++ via the
# stable CPython API in OmNewtonBackend.cpp -- no pybind11 dep at P2 because
# the call surface is small. The flag now defaults to ON (Stage 3 flip);
# OMNISIM_WITH_NEWTON=OFF strips the Newton backend and leaves the binary with
# NO physics at all -- ODE was deleted in bdc02139 (2026-08-08), so there is no
# "pure-ODE legacy build" to restore (comment corrected 2026-09-01).
# ---- Vulkan/wgpu render backend (R1 of engine-migration-plan.md) ------------
# DEFAULT ON as of the architectural-baseline C1 flip (2026-06-07) -- symmetric
# with OMNISIM_WITH_NEWTON: a no-flags `make` now COMPILES the wgpu backend in,
# so the new render architecture is "live by default." HISTORICAL, corrected
# 2026-09-01: the next sentence described the C1-era runtime default ("WREN stays
# canonical, renderBackend defaults to wren"); the runtime default flipped to
# wgpu on 2026-08-19 and WREN was DELETED on 2026-08-23 (976b9449d), so wgpu is
# now the ONLY renderer and `renderBackend "wren"` is a warned no-op.
# The wgpu code is DOUBLE-GUARDED:
#   * OMNISIM_WITH_VULKAN       -- compiles the backend TUs in (this flag), and
#   * WB_WGPU_NATIVE_AVAILABLE  -- set only when WGPU_NATIVE_HOME points at a real
#     wgpu-native lib (below); every wgpu* API call lives behind it.
# So with the flag ON but WGPU_NATIVE_HOME unset (the plain-`make` case on a box
# without wgpu-native), every wgpu* call compiles out and isAvailable() reports
# false. HISTORICAL: that used to leave a WREN-only binary; WREN is deleted, so
# it now leaves a binary with NO renderer, which the guard below REFUSES to
# build unless OMNISIM_RENDERERLESS=ON names it. OMNISIM_WITH_VULKAN=OFF is the
# same rendererless build under the same guard. (The "vulkan" name is historical; the backend is wgpu-native --
# the rename is a Phase-ζ cleanup, engine-migration-plan.md §14.3.)
OMNISIM_WITH_VULKAN ?= ON
ifeq ($(OMNISIM_WITH_VULKAN),ON)
  CFLAGS   += -DOMNISIM_WITH_VULKAN
  CXXFLAGS += -DOMNISIM_WITH_VULKAN
  # R3.1 (2026-05-28): try to locate wgpu-native — the actual backend
  # the Vulkan flag enables (kept name for historical reasons; rename is
  # a Phase ζ cleanup per engine-migration-plan.md §14.3).
  #
  # Discovery is opt-in via WGPU_NATIVE_HOME. Set it to the directory
  # that contains include/webgpu.h + lib/libwgpu_native.dll.a (or the
  # platform equivalent). Pre-built releases ship that layout — see
  # docs/build/wgpu-native-setup.md.
  #
  # When unset, the ON build is identical to the R1 stub: the backend
  # constructor stays in the WB_WGPU_NATIVE_AVAILABLE=0 branch and
  # isAvailable() reports false, so dependent worlds silently fall
  # back to WREN.
  # ---- AUTO-DISCOVERY, and why it is not merely a convenience -------------
  # The paragraph above is true only for a tree that has NEVER been built with
  # wgpu-native present.  Once it has, this variable is load-bearing in a way
  # the design did not intend: the objects on disk carry
  # -DWB_WGPU_NATIVE_AVAILABLE and reference wgpuDeviceCreateBuffer et al, but
  # nothing records that fact.  Drop the variable on the NEXT invocation and
  # make happily re-links those same objects without -lwgpu_native, ld reports
  # a screenful of undefined references -- and because the link is a rule with
  # a target, GNU make then DELETES omnisim-bin.exe.
  #
  # So a plain `make release`, the most obvious command in the repository,
  # destroys the binary on any box where wgpu-native was ever set up, and the
  # only recovery is knowing the magic invocation.  MEASURED 2026-08-14: that
  # is exactly what happened, and it cost a full relink to undo.
  #
  # scripts/dev/setup_wgpu_native.sh already installs to ONE canonical place
  # ($REPO_ROOT/_scratch/wgpu-native, its INSTALL_DIR).  Read it.  Discovery
  # only fires when the variable is genuinely undefined, so an explicit
  # `WGPU_NATIVE_HOME=` on the command line still opts out deliberately rather
  # than by omission -- and since 2026-08-29 that opt-out is REFUSED unless
  # OMNISIM_RENDERERLESS=ON is passed too (see the block after this one): there
  # is no WREN to fall back to any more, only a binary that draws nothing.
  ifeq ($(origin WGPU_NATIVE_HOME),undefined)
    OM_WGPU_DEFAULT := $(OMNISIM_PATH)/_scratch/wgpu-native
    ifneq ($(wildcard $(OM_WGPU_DEFAULT)/include/webgpu/webgpu.h),)
      WGPU_NATIVE_HOME := $(OM_WGPU_DEFAULT)
    endif
  endif
  WGPU_NATIVE_HOME ?=
  ifneq ($(WGPU_NATIVE_HOME),)
    ifneq ($(wildcard $(WGPU_NATIVE_HOME)/include/webgpu/webgpu.h),)
      CFLAGS   += -DWB_WGPU_NATIVE_AVAILABLE -I$(WGPU_NATIVE_HOME)/include
      CXXFLAGS += -DWB_WGPU_NATIVE_AVAILABLE -I$(WGPU_NATIVE_HOME)/include
      LIBS     += -L$(WGPU_NATIVE_HOME)/lib -lwgpu_native
      # Ship wgpu_native.dll next to omnisim-bin.exe so the wgpu-ON build runs with no
      # manual PATH/copy step (it must be discoverable at process start). Appended to the
      # existing post-link EXTRA_CMD (run by the link recipe). Guarded by OS + the dll's
      # presence, so non-wgpu / non-Windows builds are byte-for-byte unaffected.
      ifeq ($(OSTYPE),windows)
        ifneq ($(wildcard $(WGPU_NATIVE_HOME)/lib/wgpu_native.dll),)
          # Avoid rewriting an identical DLL: Windows locks loaded DLLs, so an
          # otherwise successful link used to fail whenever OmniSim was open.
          EXTRA_CMD += ; if [ ! -f $(TARGET_PATH)/wgpu_native.dll ] || ! sha256sum $(WGPU_NATIVE_HOME)/lib/wgpu_native.dll $(TARGET_PATH)/wgpu_native.dll | cut -d' ' -f1 | uniq -d | grep -q .; then cp $(WGPU_NATIVE_HOME)/lib/wgpu_native.dll $(TARGET_PATH)/; fi
        endif
      endif
    endif
  endif
endif

# ---- THERE IS NO SECOND RENDERER: a build without wgpu-native must be DELIBERATE ----
# Since the WREN deletion (976b9449d, 2026-08-23) wgpu-native is the ONLY renderer.
# Until 2026-08-29 a tree where WGPU_NATIVE_HOME did not resolve -- no
# scripts/dev/setup_wgpu_native.sh run yet, an explicit `WGPU_NATIVE_HOME=`, or
# OMNISIM_WITH_VULKAN=OFF -- built GREEN and shipped a binary that draws nothing:
# no main view, no screenshots, no Camera device, no capture service. Physics and
# controllers still ran, so the hole surfaced later as an empty screenshot
# (public issue #7). A missing MANDATORY subsystem is a configuration error, not a
# build variant, so refuse at parse time, before one TU compiles. The compute-only
# build stays available, but only by name:
#     make release OMNISIM_RENDERERLESS=ON
# which also defines -DOMNISIM_RENDERERLESS so the binary can say what it is
# (OmRenderBackend.cpp logs it on the first render request). Goals that never
# need a renderer (clean, linker-info, bundle-newton-runtime) are exempt.
OMNISIM_RENDERERLESS ?= OFF
OM_WGPU_RESOLVED :=
ifeq ($(OMNISIM_WITH_VULKAN),ON)
  ifneq ($(WGPU_NATIVE_HOME),)
    OM_WGPU_RESOLVED := $(wildcard $(WGPU_NATIVE_HOME)/include/webgpu/webgpu.h)
  endif
endif
# Empty MAKECMDGOALS is the default goal, which builds -- treat it as building.
OM_RENDER_BUILD_GOALS := $(if $(MAKECMDGOALS),$(filter-out clean linker-info bundle-newton-runtime,$(MAKECMDGOALS)),default)
ifeq ($(OM_WGPU_RESOLVED),)
  ifeq ($(OMNISIM_RENDERERLESS),ON)
    CFLAGS   += -DOMNISIM_RENDERERLESS
    CXXFLAGS += -DOMNISIM_RENDERERLESS
    ifneq ($(OM_RENDER_BUILD_GOALS),)
      $(warning OmniSim: building WITHOUT a renderer (OMNISIM_RENDERERLESS=ON) -- no main view, no screenshots, no Camera devices, no capture service)
    endif
  else ifneq ($(OM_RENDER_BUILD_GOALS),)
    define OM_NO_RENDERER_MSG

ERROR: wgpu-native was not found, and OmniSim has NO alternative renderer.
  $(if $(WGPU_NATIVE_HOME),WGPU_NATIVE_HOME=$(WGPU_NATIVE_HOME) has no include/webgpu/webgpu.h.,WGPU_NATIVE_HOME is unset and $(OMNISIM_PATH)/_scratch/wgpu-native does not exist.)$(if $(filter ON,$(OMNISIM_WITH_VULKAN)),,
  OMNISIM_WITH_VULKAN=$(OMNISIM_WITH_VULKAN) compiles the only renderer out.)
  A green build here would ship a binary that draws nothing: no main view, no
  screenshots, no Camera devices, no capture service (physics still runs).
  Fix:         bash scripts/dev/setup_wgpu_native.sh
               (installs to _scratch/wgpu-native, which this Makefile auto-discovers)
  Deliberate:  make release OMNISIM_RENDERERLESS=ON
               (compute-only binary; it logs at runtime that it has no renderer)
    endef
    $(error $(OM_NO_RENDERER_MSG))
  endif
endif

# DEFAULT ON (default-flip-plan.md §4.3 Stage 3 / engine-migration-plan.md §13.4):
# Newton is the ONLY physics backend. A no-flags `make` builds it in; an
# unspecified Solid resolves to Newton when the runtime is present AND its
# articulation is Newton-faithful (the OmSolid capability gate still routes mesh
# / non-Hinge-Slider-joint articulations away from Newton).
# ⚠ THERE IS NO FALLBACK. ODE was deleted (bdc02139) and its stub layer went on
# 2026-09-02, so `make OMNISIM_WITH_NEWTON=OFF` yields a binary with NO physics
# at all; physicsBackend "ode" in a world warns once and runs on Newton. The
# Newton runtime is staged by `make bundle-newton-runtime`: when absent,
# isAvailable() is false, the world loads and stands still, and one ERROR says
# so. Set OMNISIM_REQUIRE_NEWTON=1 at runtime to make that a FATAL (see
# docs/guide/newton-physics-backend.md).
OMNISIM_WITH_NEWTON ?= ON
ifeq ($(OMNISIM_WITH_NEWTON),ON)
  CFLAGS   += -DOMNISIM_WITH_NEWTON
  CXXFLAGS += -DOMNISIM_WITH_NEWTON
  # Newton lives behind an embedded CPython interpreter — we drive
  # newton + warp-lang from C++ via the stable CPython API
  # (no pybind11 dependency for P2 because the call surface is small:
  # Py_Initialize / PyImport_ImportModule / PyObject_CallMethod). The
  # interpreter is initialised lazily inside OmNewtonBackend so the
  # cost is paid only on worlds that actually opt into Newton.
  #
  # PYTHON_HOME is the path to a CPython install with both Python.h
  # under include/ and pythonNNN.lib under libs/. On Windows we probe
  # the python.org per-user install location for any 3.10+ version
  # (newest wins); override via the environment for anything else.
  ifeq ($(OSTYPE),windows)
    ifeq ($(origin PYTHON_HOME),undefined)
      # python.org per-user installs land in %LOCALAPPDATA%\Programs\Python;
      # probe every env spelling of that root so the build works from cmd,
      # PowerShell and stripped-down CI shells alike.
      PY_ROOTS := $(subst \,/,$(LOCALAPPDATA))/Programs/Python $(subst \,/,$(USERPROFILE))/AppData/Local/Programs/Python C:/Users/$(USERNAME)/AppData/Local/Programs/Python
      PYTHON_PROBE := $(lastword $(sort $(wildcard $(addsuffix /Python3[0-9][0-9]/include/Python.h,$(PY_ROOTS)))))
      ifeq ($(PYTHON_PROBE),)
        # Fall back to the Windows py launcher (C:\Windows, so on PATH even
        # in stripped shells, and never shadowed by the MSYS2 python).
ifdef OM_PROBES_CACHED
        PYTHON_PROBE := $(wildcard $(subst \,/,$(OM_PROBE_PY3_PREFIX))/include/Python.h)
else
        PYTHON_PROBE := $(wildcard $(subst \,/,$(shell py -3 -c "import sys; print(sys.prefix)" 2>/dev/null))/include/Python.h)
endif
      endif
      ifeq ($(PYTHON_PROBE),)
        # Last resort: whatever python is on PATH (skips the MSYS2 python
        # automatically because its include layout doesn't match).
ifdef OM_PROBES_CACHED
        PYTHON_PROBE := $(wildcard $(subst \,/,$(OM_PROBE_PYTHON_PREFIX))/include/Python.h)
else
        PYTHON_PROBE := $(wildcard $(subst \,/,$(shell python -c "import sys; print(sys.prefix)" 2>/dev/null))/include/Python.h)
endif
      endif
      ifneq ($(PYTHON_PROBE),)
        PYTHON_HOME := $(patsubst %/include/Python.h,%,$(PYTHON_PROBE))
      else
        $(error OMNISIM_WITH_NEWTON=ON embeds CPython and needs its build headers. Install Python 3 from python.org (default per-user location), or pass PYTHON_HOME=<dir containing include/Python.h and libs/pythonNNN.lib> plus PYTHON_LIB=-lpythonNNN as make arguments. OMNISIM_WITH_NEWTON=OFF builds a renderer/controller shell with no working physics backend)
      endif
    endif
    # Derive -lpythonNNN from the directory name (".../Python312" ->
    # -lpython312); set PYTHON_LIB explicitly for non-standard layouts.
    PYTHON_INCLUDE := -isystem "$(PYTHON_HOME)/include"
    PYTHON_LIBDIR  := "$(PYTHON_HOME)/libs"
    PYTHON_LIB     ?= -l$(subst Python,python,$(notdir $(PYTHON_HOME)))
  else
    # Linux/macOS: derive the embed flags from python3-config so the build
    # matches whatever Python 3 the system ships (3.10/3.12/3.13/...) instead
    # of hardcoding one version. If PYTHON_HOME is explicitly set (env or
    # command line), prefer that install's own python3-config.
    # PYTHON_CONFIG is overridable on its own (env or make arg), because the
    # right interpreter is not always the one `python3-config` resolves: on
    # Ubuntu 22.04 the system python3 is 3.10, where newton 1.5.0 raises at
    # ModelBuilder() (typing.Union rejects wp.array[wp.bool] before 3.11), so
    # linux_bootstrap.sh installs 3.12 from deadsnakes and passes
    # PYTHON_CONFIG=python3.12-config to embed THAT interpreter instead.
    ifeq ($(origin PYTHON_CONFIG),undefined)
      ifeq ($(origin PYTHON_HOME),undefined)
        PYTHON_CONFIG := python3-config
      else
        PYTHON_CONFIG := $(PYTHON_HOME)/bin/python3-config
      endif
    endif
    PYTHON_HOME ?= /usr
    PYTHON_INCLUDE := $(shell $(PYTHON_CONFIG) --includes)
    PYTHON_LIBDIR  := $(PYTHON_HOME)/lib
    PYTHON_LIB     := $(shell $(PYTHON_CONFIG) --ldflags --embed)
  endif
  ALL_INCLUDE += $(PYTHON_INCLUDE)
  LIBS += -L$(PYTHON_LIBDIR) $(PYTHON_LIB)
endif


CONTROLLER_INCLUDE = -I$(OMNISIM_PATH)/include/controller/c
QT_CORE_INCLUDE   += -I$(OBJDIR)
INCLUDE            = $(ALL_INCLUDE) $(CONTROLLER_INCLUDE) $(QT_CORE_INCLUDE) $(QT_GUI_INCLUDE) $(QT_WIDGETS_INCLUDE) $(QT_PRINT_SUPPORT_INCLUDE) $(QT_OPENGL_INCLUDE) $(QT_NETWORK_INCLUDE) $(OIS_INCLUDE) $(INSTALL_INCLUDE) $(FREETYPE_INCLUDE) $(STB_INCLUDE) $(ASSIMP_INCLUDE)

ifeq ($(BUILD_GOAL),debug)
CFLAGS += -ggdb
endif

ifeq ($(TREAT_WARNINGS_AS_ERRORS),1)
CFLAGS += -Werror
endif

ifeq ($(BUILD_GOAL),profile)
CFLAGS += -pg -O3 -DNDEBUG
ifneq ($(OSTYPE),darwin)
LD_FLAGS += -pg
endif
endif

ifeq ($(BUILD_GOAL),release)
CFLAGS += -O3 -DNDEBUG
ifneq ($(OSTYPE),darwin)
LD_FLAGS += -s
endif
endif

OBJDIR=build/$(BUILD_GOAL)
ifneq (,$(findstring $(BUILD_GOAL),debug profile release))
ifeq ($(OSTYPE),darwin)
ifeq ($(wildcard $(OBJDIR)/arm64/.deps),)
BUILD_PATH_SETUP := $(shell mkdir -p $(OBJDIR) $(OBJDIR)/x86_64 $(OBJDIR)/arm64 $(OBJDIR)/x86_64/.deps $(OBJDIR)/arm64/.deps)
endif
else
ifeq ($(wildcard $(OBJDIR)/.deps),)
BUILD_PATH_SETUP := $(shell mkdir -p $(OBJDIR) $(OBJDIR)/.deps)
endif
endif
endif

CFLAGS += -Wall -Wpointer-arith -Wcast-align -Wwrite-strings
# Emit the dependency file beside each object during compilation.  This replaces
# the old standalone `-MM` preprocessing pass below: one compiler invocation now
# creates both `Foo.o` and `Foo.d`, and ccache sees a normal cacheable compile.
#
# Most simulator rules have a basename target (`Foo.o`) and write the result to
# `$(OBJDIR)/Foo.o`; the macOS fat-binary rules instead target the full object
# path.  Preserve that distinction in both -MF and -MT.  Using GCC's default
# target here would write `$(OBJDIR)/Foo.o:` into the depfile, but no simulator
# recipe builds that target: make's real target remains `Foo.o` and vpath finds
# the output.  The conditional keeps full-path architecture targets exact.
# Keep generated depfiles in a new subdirectory.  This is also a safe migration
# boundary from the brief one-pass format that wrote exact
# `$(OBJDIR)/Foo.o:` targets into the legacy location: stale exact rules can
# hijack vpath resolution and suppress rebuilding an older object.
DEPENDENCY_OUTPUT = $(if $(filter $(OBJDIR)/%,$@),$(dir $@).deps/$(notdir $(@:.o=.d)),$(OBJDIR)/.deps/$(@:.o=.d))
DEPENDENCY_FLAGS = -MMD -MP -MF $(DEPENDENCY_OUTPUT) -MT $@
DEPENDENCY_MIGRATION_STAMP = $(OBJDIR)/.deps/.logical-targets-v1
CXXFLAGS += -std=c++17 -DQT_NO_DEBUG $(DEPENDENCY_FLAGS)

# Optional stable precompiled header for cache-miss/clean builds. It contains
# only standard-library headers, so it does not widen project dependencies or
# cause broad invalidation when an OmniSim header changes. ccache supports GCC
# PCH when -fpch-preprocess and the documented sloppiness flags are enabled.
OMNISIM_USE_PCH ?= 0
ifeq ($(OMNISIM_USE_PCH),1)
ifneq ($(OSTYPE),darwin)
PCH_SOURCE = core/OmBuildPch.hpp
PCH_WRAPPER = $(OBJDIR)/OmBuildPch.hpp
PCH_FILE = $(PCH_WRAPPER).gch
PCH_FLAG = -fpch-preprocess -include $(PCH_WRAPPER)
CXXFLAGS += $(PCH_FLAG)
export CCACHE_SLOPPINESS := $(if $(CCACHE_SLOPPINESS),$(CCACHE_SLOPPINESS),pch_defines,time_macros)
endif
endif

# GNU ld remains the conservative default. Developers with LLVM installed can
# opt into materially faster relinks using OMNISIM_LINKER=lld; fail clearly
# instead of silently falling back when an explicit request cannot be honored.
# DEFAULT CHANGED TO `auto` (2026-08-14): use lld when it is installed, GNU ld
# otherwise.  Once ccache lands the compile of a one-file edit at ~0.14 s, the
# LINK is what a C++ iteration actually costs -- MEASURED on machine
# 9722d23d12a3, the full touch-one-TU-and-rebuild loop:
#
#     bfd (GNU ld)   3738 ms      lld   2830 ms      (1.32x)
#
# i.e. roughly 3.0 s of that loop was ld, and about a second of it goes away.
# Validated on this tree before flipping: the lld-linked binary loads and
# finalises the cloth world (Newton sidecar, 289 particles registered) and
# `omnisim_dev.py test-smoke` passes all 4 non-skipped worlds.
#
# ⚠ REPRODUCIBILITY.  `auto` means two boxes can link with different linkers
# and produce binaries with different sha256 -- which env_fingerprint.py
# records.  That is why the selection is a named mode rather than a silent
# probe: pin OMNISIM_LINKER=bfd (or =lld) for anything whose binary identity
# has to match across machines, and `make linker-info` reports the choice.
OMNISIM_LINKER ?= auto
ifdef OM_PROBES_CACHED
OM_HAVE_LLD := $(OM_PROBE_LLD)
else
OM_HAVE_LLD := $(shell command -v ld.lld 2>/dev/null)
endif
ifeq ($(OMNISIM_LINKER),auto)
ifneq ($(OM_HAVE_LLD),)
LD_FLAGS += -fuse-ld=lld
OMNISIM_LINKER_RESOLVED := lld
else
OMNISIM_LINKER_RESOLVED := bfd
endif
else ifeq ($(OMNISIM_LINKER),lld)
ifeq ($(OM_HAVE_LLD),)
$(error OMNISIM_LINKER=lld requested but ld.lld is not on PATH)
endif
LD_FLAGS += -fuse-ld=lld
OMNISIM_LINKER_RESOLVED := lld
else ifeq ($(OMNISIM_LINKER),bfd)
OMNISIM_LINKER_RESOLVED := bfd
else
$(error OMNISIM_LINKER must be auto, bfd or lld)
endif

ifneq ($(OSTYPE),darwin) # Clang has these warnings by default
ifdef OM_PROBES_CACHED
GCC_VER_GREATER_OR_EQUAL_TO_5_DOT_4 := $(OM_PROBE_GCC_GE_504)
else
GCC_VER_GREATER_OR_EQUAL_TO_5_DOT_4 := $(shell expr `gcc -dumpversion | sed -e 's/\.\([0-9][0-9]\)/\1/g' -e 's/\.\([0-9]\)/0\1/g' -e 's/^[0-9]\{3,4\}$$/&00/'` \>= 50400)
endif
ifeq ($(GCC_VER_GREATER_OR_EQUAL_TO_5_DOT_4),1)
CXXFLAGS += -Wsuggest-override
endif
endif

# objects that inherit from QObject (alphabetical order)
QT_SOURCES = OmAboutBox.cpp \
  OmAbstractAppearance.cpp \
  OmAbstractDragEvent.cpp \
  OmAbstractCamera.cpp \
  OmAccelerometer.cpp \
  OmActionManager.cpp \
  OmAddInertiaMatrixDialog.cpp \
  OmAddNodeDialog.cpp \
  OmAgentHud.cpp \
  OmAltimeter.cpp \
  OmAnimationRecorder.cpp \
  OmAppearance.cpp \
  OmApplication.cpp \
  OmBackground.cpp \
  OmBallJoint.cpp \
  OmBallJointParameters.cpp \
  OmBaseNode.cpp \
  OmBasicJoint.cpp \
  OmBillboard.cpp \
  OmBoolEditor.cpp \
  OmBox.cpp \
  OmBrake.cpp \
  OmCadShape.cpp \
  OmCamera.cpp \
  OmCapsule.cpp \
  OmCharger.cpp \
  OmCloth.cpp \
  OmColor.cpp \
  OmColorEditor.cpp \
  OmCompass.cpp \
  OmCone.cpp \
  OmConnector.cpp \
  OmConsole.cpp \
  OmContactProperties.cpp \
  OmControlledWorld.cpp \
  OmController.cpp \
  OmCoordinate.cpp \
  OmCylinder.cpp \
  OmDamping.cpp \
  OmDirectionalLight.cpp \
  OmDistanceSensor.cpp \
  OmDisplay.cpp \
  OmDockTitleBar.cpp \
  OmDockWidget.cpp \
  OmDoubleEditor.cpp \
  OmDoubleSpinBox.cpp \
  OmDownloadManager.cpp \
  OmDownloader.cpp \
  OmDragPoseEvent.cpp \
  OmDragResizeEvent.cpp \
  OmDragSolidEvent.cpp \
  OmElevationGrid.cpp \
  OmEmitter.cpp \
  OmExtendedStringEditor.cpp \
  OmExternProtoEditor.cpp \
  OmInsertExternProtoDialog.cpp \
  OmField.cpp \
  OmFieldChecker.cpp \
  OmFieldDoubleSpinBox.cpp \
  OmFieldEditor.cpp \
  OmFieldIntSpinBox.cpp \
  OmFieldLineEdit.cpp \
  OmFindReplaceDialog.cpp \
  OmFieldValueRestriction.cpp \
  OmFocus.cpp \
  OmFog.cpp \
  OmGeometry.cpp \
  OmGlWindow.cpp \
  OmGps.cpp \
  OmGranularBed.cpp \
  OmGranularGroup.cpp \
  OmGroup.cpp \
  OmGuiApplication.cpp \
  OmGuidedTour.cpp \
  OmGuiRefreshOracle.cpp \
  OmGyro.cpp \
  OmHingeJoint.cpp \
  OmHingeJointParameters.cpp \
  OmHinge2Joint.cpp \
  OmImageTexture.cpp \
  OmIntEditor.cpp \
  OmIntSpinBox.cpp \
  OmIndexedFaceSet.cpp \
  OmIndexedLineSet.cpp \
  OmInertia.cpp \
  OmInertialUnit.cpp \
  OmJoint.cpp \
  OmJointDevice.cpp \
  OmJointParameters.cpp \
  OmJoystickInterface.cpp \
  OmJoystickListener.cpp \
  OmLed.cpp \
  OmLens.cpp \
  OmLensFlare.cpp \
  OmLidar.cpp \
  OmLight.cpp \
  OmLightSensor.cpp \
  OmLinearMotor.cpp \
  OmLineEdit.cpp \
  OmLog.cpp \
  OmMainWindow.cpp \
  OmMassChecker.cpp \
  OmMaterial.cpp \
  OmMatter.cpp \
  OmMesh.cpp \
  OmMFBool.cpp \
  OmMFColor.cpp \
  OmMFDouble.cpp \
  OmMFInt.cpp \
  OmMFNode.cpp \
  OmMFRotation.cpp \
  OmMFString.cpp \
  OmMFVector2.cpp \
  OmMFVector3.cpp \
  OmMotor.cpp \
  OmMouse.cpp \
  OmMultipleValue.cpp \
  OmMuscle.cpp \
  OmNewControllerWizard.cpp \
  OmNewProjectWizard.cpp \
  OmNewProtoWizard.cpp \
  OmNewWorldWizard.cpp \
  OmNewVersionDialog.cpp \
  OmNode.cpp \
  OmNodeEditor.cpp \
  OmNodeOperations.cpp \
  OmNodePane.cpp \
  OmNodeReader.cpp \
  OmNormal.cpp \
  OmOpenSampleWorldDialog.cpp \
  OmPaintTexture.cpp \
  OmPbrAppearance.cpp \
  OmPen.cpp \
  OmPhysics.cpp \
  OmPhysicsVectorRepresentation.cpp \
  OmPhysicsViewer.cpp \
  OmPlane.cpp \
  OmPointLight.cpp \
  OmPointSet.cpp \
  OmPositionSensor.cpp \
  OmPositionViewer.cpp \
  OmPose.cpp \
  OmPreferences.cpp \
  OmPreferencesDialog.cpp \
  OmPropeller.cpp \
  OmProject.cpp \
  OmProjectRelocationDialog.cpp \
  OmProtoIcon.cpp \
  OmProtoManager.cpp \
  OmProtoModel.cpp \
  OmProtoTemplateEngine.cpp \
  OmProtoTreeItem.cpp \
  OmQjsFile.cpp \
  OmRadar.cpp \
  OmRangeFinder.cpp \
  OmReceiver.cpp \
  OmRecentFilesList.cpp \
  OmRecognition.cpp \
  OmRenderingDevice.cpp \
  OmRenderingDeviceWindow.cpp \
  OmRenderingDeviceWindowFactory.cpp \
  OmResizeManipulator.cpp \
  OmRotationEditor.cpp \
  OmRobot.cpp \
  OmRobotWindow.cpp \
  OmRotationalMotor.cpp \
  OmSaveWarningDialog.cpp \
  OmSceneTree.cpp \
  OmSceneTreeModel.cpp \
  OmSelection.cpp \
  OmSensor.cpp \
  OmSFBool.cpp \
  OmSFColor.cpp \
  OmSFDouble.cpp \
  OmSFInt.cpp \
  OmSFNode.cpp \
  OmSFRotation.cpp \
  OmSFString.cpp \
  OmSFVector2.cpp \
  OmSFVector3.cpp \
  OmShape.cpp \
  OmSimulationState.cpp \
  OmSimulationStateIndicator.cpp \
  OmSimulationView.cpp \
  OmSimulationWorld.cpp \
  OmSingleValue.cpp \
  OmSingleTaskApplication.cpp \
  OmSliderJoint.cpp \
  OmSlot.cpp \
  OmSoftBody.cpp \
  OmSolid.cpp \
  OmSolidMerger.cpp \
  OmSolidReference.cpp \
  OmSpeaker.cpp \
  OmSplashScreen.cpp \
  OmSphere.cpp \
  OmSpotLight.cpp \
  OmStringEditor.cpp \
  OmTcpServer.cpp \
  OmSupervisorUtilities.cpp \
  OmSyntaxHighlighter.cpp \
  OmTemplateEngine.cpp \
  OmTemplateManager.cpp \
  OmTextFind.cpp \
  OmTextureCoordinate.cpp \
  OmTextureTransform.cpp \
  OmTouchSensor.cpp \
  OmTrack.cpp \
  OmTrackWheel.cpp \
  OmTransform.cpp \
  OmTranslateRotateManipulator.cpp \
  OmTreeItem.cpp \
  OmTreeView.cpp \
  OmTriangleMeshGeometry.cpp \
  OmUndoStack.cpp \
  OmUpdatedDialog.cpp \
  OmValue.cpp \
  OmValueEditor.cpp \
  OmVariant.cpp \
  OmVector2Editor.cpp \
  OmVector3Editor.cpp \
  OmVelocityViewer.cpp \
  OmVideoRecorder.cpp \
  OmVideoRecorderDialog.cpp \
  OmMultimediaStreamingServer.cpp \
  OmVacuumGripper.cpp \
  OmViewpoint.cpp \
  OmView3D.cpp \
  OmVisualBoundingSphere.cpp \
  OmWorld.cpp \
  OmWorldInfo.cpp \
  OmWrenAbstractManipulator.cpp \
  OmWrenAbstractResizeManipulator.cpp \
  OmWrenOpenGlContext.cpp \
  OmWrenRenderingContext.cpp \
  OmWrenTextureOverlay.cpp \
  OmW3dStreamingServer.cpp \
  OmZoom.cpp

# objects that don't inherit from QObject (alphabetical order)
OTHER_SOURCES = \
  main.cpp \
  OmAbstractPose.cpp \
  OmAddItemCommand.cpp \
  OmAffinePlane.cpp \
  OmApplicationInfo.cpp \
  OmBinaryIncubator.cpp \
  OmBoundingSphere.cpp \
  OmClipboard.cpp \
  OmConcreteNodeFactory.cpp \
  OmContextMenuGenerator.cpp \
  OmControllerPlugin.cpp \
  OmDataPacket.cpp \
  OmDataStream.cpp \
  OmDeformableFrameListener.cpp \
  OmDevice.cpp \
  OmDesktopServices.cpp \
  OmDictionary.cpp \
  OmDisplayFont.cpp \
  OmDragOverlayEvent.cpp \
  OmDragViewpointEvent.cpp \
  OmDragScaleEvent.cpp \
  OmEditCommand.cpp \
  OmFieldModel.cpp \
  OmFileUtil.cpp \
  OmHiddenKinematicParameters.cpp \
  OmHttpReply.cpp \
  OmIniParser.cpp \
  OmKinematicDifferentialWheels.cpp \
  OmLanguage.cpp \
  OmLanguageTools.cpp \
  OmLogicalDevice.cpp \
  OmLookupTable.cpp \
  OmMacAddress.cpp \
  OmMatrix3.cpp \
  OmMatrix4.cpp \
  OmMathsUtilities.cpp \
  OmMessageBox.cpp \
  OmMotorSoundManager.cpp \
  OmMultimediaStreamingLimiter.cpp \
  OmNetwork.cpp \
  OmNodeFactory.cpp \
  OmNodeModel.cpp \
  OmNodeProtoInfo.cpp \
  OmNodeUtilities.cpp \
  OmObjectDetection.cpp \
  OmParser.cpp \
  OmPerformanceLog.cpp \
  OmPerspective.cpp \
  OmPolygon.cpp \
  OmPrecision.cpp \
  OmRandom.cpp \
  OmRay.cpp \
  OmResetCommand.cpp \
  OmResizeCommand.cpp \
  OmResizeAndTranslateCommand.cpp \
  OmRotation.cpp \
  OmSolidDevice.cpp \
  OmSolidUtilities.cpp \
  OmSoundClip.cpp \
  OmSoundEngine.cpp \
  OmSoundSource.cpp \
  OmStandardPaths.cpp \
  OmSysInfo.cpp \
  OmTesselator.cpp \
  OmToken.cpp \
  OmTokenizer.cpp \
  OmTriangleMesh.cpp \
  OmTriangleMeshCache.cpp \
  OmUrdfImporter.cpp \
  OmUrl.cpp \
  OmVector4.cpp \
  OmVersion.cpp \
  OmVrmlNodeUtilities.cpp \
  OmWaveFile.cpp \
  OmWheelEvent.cpp \
  OmWrenLabelOverlay.cpp \
  OmWriter.cpp \
  OmCudaContext.cpp \
  OmCudaSmoke.cpp \
  OmPhysicsBackend.cpp \
  OmNewtonBackend.cpp \
  OmRenderBackend.cpp \
  OmVulkanBackend.cpp \
  OmniLight.cpp \
  OmDragArrowLines.cpp \
  OmGizmoLines.cpp \
  OmScenePicker.cpp \
  OmHudOverlay.cpp \
  OmWgpuGlBlit.cpp \
  OmWgpuImageAdapter.cpp \
  OmWgpuMeshAdapter.cpp \
  OmWgpuMeshCache.cpp \
  OmWgpuRenderTarget.cpp \
  OmWgpuSceneRenderer.cpp \
  OmWgpuShaders.cpp \
  OmWgpuSurface.cpp \
  OmWgpuTextureCache.cpp \
  OmWgpuView.cpp

ifeq ($(OSTYPE),windows)
OTHER_SOURCES += \
  OmMicrosoftTextToSpeech.cpp \
  OmWindowsRegistry.cpp
else
OTHER_SOURCES += \
  OmPosixMemoryMappedFile.cpp
endif

MOC_FILES=$(addprefix $(OBJDIR)/,$(notdir $(QT_SOURCES:%.cpp=%.moc.cpp)))
SOURCES=$(C_SOURCES) $(QT_SOURCES) $(OTHER_SOURCES) $(MOC_FILES)
OBJECTS=$(notdir $(C_SOURCES:.c=.o) $(QT_SOURCES:.cpp=.o) $(OTHER_SOURCES:.cpp=.o) $(MOC_FILES:.cpp=.o))
CXX_OBJECTS=$(notdir $(QT_SOURCES:.cpp=.o) $(OTHER_SOURCES:.cpp=.o) $(MOC_FILES:.cpp=.o))

ifeq ($(OSTYPE),windows)
OBJECTS += omnisim.o
LAUNCHER = ../../msys64/mingw64/bin/omnisim.exe
LAUNCHERW = ../../msys64/mingw64/bin/omnisimw.exe
# legacy launcher names, shipped as byte-identical copies of the above
LAUNCHER_LEGACY = ../../msys64/mingw64/bin/webots.exe
LAUNCHERW_LEGACY = ../../msys64/mingw64/bin/webotsw.exe
endif

# prevent 'make' from deleting these intermediate files
.PRECIOUS: $(MOC_FILES)

vpath %.o $(OBJDIR)
vpath %.hpp app compute/cuda control core widgets editor engine external/siphash gui maths nodes nodes/utils physics render scene_tree sound user_commands vrml
vpath %.cpp app compute/cuda control core widgets editor engine external/siphash gui maths nodes nodes/utils physics render scene_tree sound user_commands vrml $(OBJDIR)

default:
	@echo "Type one of the following (OSTYPE is \""$(OSTYPE)"\"):"
	@echo "  make debug                 for debug (with gdb symbols)"
	@echo "  make profile               for profiling (with gprof information)"
	@echo "  make release               for release (also bundles Newton runtime; BUNDLE_NEWTON=0 to skip)"
	@echo "  make clean                 remove build directory"
	@echo "  make count-lines           count the lines of code of OmniSim"
	@echo "  make bundle-newton-runtime vendor the Newton runtime (warp/newton) next to the binary (run by release)"
	@echo "  make dependency-stats      report compiler-process counts for dependency generation"
	@echo "  make linker-info           report the selected linker and optional PCH mode"

# Reproducible structural measurement that does not require cleaning or running
# a build.  It intentionally counts source/object relationships from the same
# variables that feed the link target, including generated MOC sources.
COMPILED_SOURCE_COUNT = $(words $(C_SOURCES) $(QT_SOURCES) $(OTHER_SOURCES) $(MOC_FILES))
OLD_DEPENDENCY_PREPASS_COUNT = $(words $(QT_SOURCES) $(OTHER_SOURCES) $(MOC_FILES))
.PHONY: dependency-stats
dependency-stats:
	@echo "compiled source objects: $(COMPILED_SOURCE_COUNT)"
	@echo "standalone dependency preprocesses now: 0"
	@echo "standalone dependency preprocesses before: $(OLD_DEPENDENCY_PREPASS_COUNT)"
	@echo "compiler processes now: $(COMPILED_SOURCE_COUNT)"
	@echo "compiler processes before: $(shell expr $(COMPILED_SOURCE_COUNT) + $(OLD_DEPENDENCY_PREPASS_COUNT))"

.PHONY: linker-info
linker-info:
	@echo "linker: $(OMNISIM_LINKER) -> $(OMNISIM_LINKER_RESOLVED)$(if $(filter auto,$(OMNISIM_LINKER)), (auto-selected; pin bfd/lld for cross-machine binary identity))"
	@echo "precompiled header: $(OMNISIM_USE_PCH)"
	@echo "ccache: $(if $(OM_CCACHE),$(OM_CCACHE),not used)$(if $(OM_CCACHE), (CCACHE_DIR=$(CCACHE_DIR)))"

ifneq ($(PCH_FILE),)
$(PCH_WRAPPER): $(PCH_SOURCE)
	@echo "# staging precompiled header" $<
	@cp $< $@

# Do not recursively inject the PCH into the command that creates the PCH.
$(PCH_FILE): DEPENDENCY_FLAGS =
$(PCH_FILE): PCH_FLAG =
$(PCH_FILE): $(PCH_WRAPPER)
	@echo "# compiling precompiled header" $<
	@$(CXX) $(CFLAGS) $(CXXFLAGS) $(INCLUDE) -x c++-header $< -o $@

$(CXX_OBJECTS): | $(PCH_FILE)
endif

ifneq (,$(filter $(BUILD_GOAL),debug release profile))
DEPENDENCY_NAMES = $(notdir $(C_SOURCES:.c=.d) $(QT_SOURCES:.cpp=.d) $(OTHER_SOURCES:.cpp=.d) $(MOC_FILES:.cpp=.d))
ifeq ($(OSTYPE),darwin)
DEPENDENCY_FILES = $(addprefix $(OBJDIR)/x86_64/.deps/,$(DEPENDENCY_NAMES)) \
                   $(addprefix $(OBJDIR)/arm64/.deps/,$(DEPENDENCY_NAMES))
else
DEPENDENCY_FILES = $(addprefix $(OBJDIR)/.deps/,$(DEPENDENCY_NAMES))
endif
-include $(DEPENDENCY_FILES)
$(DEPENDENCY_FILES): | $(DEPENDENCY_MIGRATION_STAMP)
endif

.PHONY: debug profile release check-objects clean count-lines

CHECK_OBJECTS ?=
check-objects: $(CHECK_OBJECTS)
	@echo "# checked $(words $(CHECK_OBJECTS)) affected simulator object(s); link skipped"

# A `release` build ALSO vendors the Newton runtime next to the binary. Newton
# is the only physics backend, so an unbundled clean install cannot simulate.
# Opt out with `make release BUNDLE_NEWTON=0` only for an intermediate build
# (for example, when a packaging job stages and verifies the bundle itself).
# `debug`/`profile` never bundle. See the bundle-newton-runtime target below.
BUNDLE_NEWTON ?= 1

debug profile: $(TARGET) $(LAUNCHER) $(LAUNCHERW)

release: $(TARGET) $(LAUNCHER) $(LAUNCHERW)
# The Newton-runtime bundler is a Windows-only mechanism by design: it exists
# to defeat Windows DLL/registry python resolution (python3XX._pth beside the
# loaded DLL). On Linux/macOS the embedded interpreter uses the system
# sys.path, so there is nothing to vendor -- and TARGET_PATH is only defined
# in the Windows branch anyway.
ifeq ($(OSTYPE),windows)
ifeq ($(BUNDLE_NEWTON),1)
	@if [ -d "$(TARGET_PATH)/newton-runtime" ]; then \
		echo "# release: Newton runtime already staged at $(TARGET_PATH)/newton-runtime -- skipping bundle (rm -rf it to re-stage)"; \
	else \
		echo "# release: vendoring Newton runtime (BUNDLE_NEWTON=1; set 0 to skip)"; \
		$(MAKE) bundle-newton-runtime; \
	fi
endif
endif

ifeq ($(OSTYPE),darwin)
X86_64_OBJECTS = $(addprefix $(OBJDIR)/x86_64/, $(OBJECTS))
ARM64_OBJECTS = $(addprefix $(OBJDIR)/arm64/, $(OBJECTS))

$(TARGET): $(OBJDIR)/x86_64/webots $(OBJDIR)/arm64/webots
	@echo "# creating fat " $@
	@lipo -create -output webots $(OBJDIR)/x86_64/webots $(OBJDIR)/arm64/webots
	@codesign -s - webots
	@mv webots $(TARGET)
	@$(EXTRA_CMD)
	@echo "# webots compiled in `expr \`date +%s\` - $(START)` seconds"

$(OBJDIR)/x86_64/webots: $(X86_64_OBJECTS) $(LIB_GLAD)
	@echo "# linking " $@ "(x64_64)"
	@$(CXX) -target x86_64-apple-macos12 $(LD_FLAGS) -o $@ $(X86_64_OBJECTS) $(LIBS)

$(OBJDIR)/arm64/webots: $(ARM64_OBJECTS) $(LIB_GLAD)
	@echo "# linking " $@ "(arm64)"
	@$(CXX) -target arm64-apple-macos12 $(LD_FLAGS) -o $@ $(ARM64_OBJECTS) $(LIBS)

else

$(TARGET): $(OBJECTS) $(LIB_GLAD)
	@echo "# linking " $@
	@$(CXX) $(LD_FLAGS) -o $(TARGET) $(addprefix $(OBJDIR)/, $(OBJECTS)) $(LIBS)
	@$(EXTRA_CMD)
	@echo "# webots compiled in `expr \`date +%s\` - $(START)` seconds"

endif

# ---- Newton runtime bundle (L6 / default-flip-plan.md build layer) ----------
# Vendor the Newton runtime (a self-contained CPython + warp/newton/mujoco-warp/
# usd) next to omnisim-bin.exe so a STOCK install can resolve physicsBackend
# "auto" -> Newton AT ALL. Since ODE was deleted (bdc02139) there is no second
# engine: without this runtime the world loads and then stands still. The binary brings the
# interpreter up with a bare Py_InitializeEx(0), so the bundler only has to drop
# a python3XX._pth beside the loaded DLL (standard embeddable-distribution path
# config) -- no C++/link change, which is why this is a post-build packaging
# step rather than a compile flag.
#
# WIRED INTO release: `make release` runs this by default (BUNDLE_NEWTON=1, set
# above) so a stock release ships Newton-capable rather than physics-less.
# It adds ~600 MB; `debug`/`profile` never bundle, and `make release
# BUNDLE_NEWTON=0` opts out (byte-unchanged binary). Still runnable standalone
# after any build:
#     make bundle-newton-runtime
# BUNDLE_MODE=bootstrap stages a slim first-run installer instead of vendoring
# now; PYTHON_BUNDLER overrides the launcher python (default: python).
PYTHON_BUNDLER ?= python
BUNDLE_MODE    ?= vendor
.PHONY: bundle-newton-runtime
bundle-newton-runtime:
	$(PYTHON_BUNDLER) "$(OMNISIM_PATH)/scripts/packaging/bundle_newton_runtime.py" --target "$(TARGET_PATH)" --mode $(BUNDLE_MODE) --verify

# One-time, compiler-free migration of dependency files produced before the
# `.deps` boundary above.  This is deliberately ONE shell, not one recipe per
# depfile: spawning hundreds of MSYS processes made the first no-op build look
# hung.  The read/case/printf loop uses shell builtins and rewrites only an exact
# legacy object target at the beginning of a line.  Make remakes this order-only
# stamp before evaluating object freshness, then restarts with the migrated
# logical targets.  The next real compile overwrites its migrated file via -MF.
$(DEPENDENCY_MIGRATION_STAMP):
	@for arch in '' x86_64 arm64; do \
		if [ -n "$$arch" ]; then old_dir="$(OBJDIR)/$$arch"; new_dir="$(OBJDIR)/$$arch/.deps"; else old_dir="$(OBJDIR)"; new_dir="$(OBJDIR)/.deps"; fi; \
		[ -d "$$old_dir" ] || continue; mkdir -p "$$new_dir"; \
		for dep in "$$old_dir"/*.d; do \
			[ -f "$$dep" ] || continue; name=$${dep##*/}; \
			while IFS= read -r line || [ -n "$$line" ]; do \
				case "$$line" in "$$old_dir"/*.o:*) line=$${line#"$$old_dir"/};; esac; \
				printf '%s\n' "$$line"; \
			done < "$$dep" > "$$new_dir/$$name"; \
		done; \
	done
	@touch $@

# Static pattern rules are deliberate here.  A broad
# `$(OBJDIR)/%.moc.cpp: %.hpp` implicit rule also matches speculative chains such
# as `$(OBJDIR)/nodes/OmMotor.moc.cpp`.  Once -MP adds phony header targets, GNU
# make can prefer that chain over the canonical generated source and then fail
# because the invented nested output directory does not exist.  Restricting the
# rule to the declared MOC_FILES keeps the same header -> MOC -> object rebuild
# behavior without making arbitrary nested .moc.cpp paths buildable.
$(MOC_FILES): $(OBJDIR)/%.moc.cpp: %.hpp
	@echo "# moc-generating" $@
	@$(MOC) $(MOC_FLAGS) $(INCLUDE:-isystem:-I) $< -o $@

X86_64_MOC_FILES = $(addprefix $(OBJDIR)/x86_64/,$(notdir $(QT_SOURCES:%.cpp=%.moc.cpp)))
$(X86_64_MOC_FILES): $(OBJDIR)/x86_64/%.moc.cpp: %.hpp
	@echo "# moc-generating" $@ "(x86_64)"
	@$(MOC) -target x86_64-apple-macos-11 $(MOC_FLAGS) $(INCLUDE:-isystem:-I) $< -o $@

ARM64_MOC_FILES = $(addprefix $(OBJDIR)/arm64/,$(notdir $(QT_SOURCES:%.cpp=%.moc.cpp)))
$(ARM64_MOC_FILES): $(OBJDIR)/arm64/%.moc.cpp: %.hpp
	@echo "# moc-generating" $@ "(arm64)"
	@$(MOC) -target arm64-apple-macos-11 $(MOC_FLAGS) $(INCLUDE:-isystem:-I) $< -o $@

launcher/launcher.o: launcher/launcher.rc
	@windres -i $< -o $@ --define _WIN32 --define __WIN32__ --define __WIN95__ --define __GNUWIN32__

$(LAUNCHERW): launcher/launcher.c launcher/launcher.o
	@echo "# compiling omnisimw.exe launcher (+ webotsw.exe legacy copy)"
	@gcc $(CFLAGS) -mwindows -DWEBOTSW launcher/launcher.o launcher/launcher.c -lshlwapi -o $@
	@cp -f $@ $(LAUNCHERW_LEGACY)

$(LAUNCHER): launcher/launcher.c
	@echo "# compiling omnisim.exe launcher (+ webots.exe legacy copy)"
	@gcc $(CFLAGS) launcher/launcher.c -lshlwapi -o $@
	@cp -f $@ $(LAUNCHER_LEGACY)

omnisim.o:gui/omnisim.rc gui/omnisim.ico gui/manifest.xml
	@echo "# compiling gui/"$@
	@windres -i $< -o $(OBJDIR)/$@ --define _WIN32 --define __WIN32__ --define __WIN95__ --define __GNUWIN32__

%.o: %.c
	@echo "# compiling" $<
	@$(CC) -c $(DEPENDENCY_FLAGS) $(CFLAGS) $(CONTROLLER_INCLUDE) $< -o $(OBJDIR)/$@

%.o: $(OBJDIR)/%.cpp
	@echo "# compiling" $<
	@$(CXX) -c $(CFLAGS) $(CXXFLAGS) $(INCLUDE) $< -o $(OBJDIR)/$@

%.o: app/%.cpp
	@echo "# compiling" $<
	@$(CXX) -c $(CFLAGS) $(CXXFLAGS) $(WB_APP_INCLUDE) $< -o $(OBJDIR)/$@

%.o: compute/cuda/%.cpp
	@echo "# compiling" $<
	@$(CXX) -c $(CFLAGS) $(CXXFLAGS) $(QT_CORE_INCLUDE) -Icore -Icompute/cuda $(CUDA_INCLUDE) $< -o $(OBJDIR)/$@

# Newton physics-backend abstraction (P0 of cuda-newton-physics-plan.md).
# Note: OmPhysicsBackend.hpp itself stays Qt-free, but OmNewtonBackend.cpp
# uses OmLog (which lives in core/) for the bring-up diagnostic so messages
# land in omnisim_log.txt. -Icore is therefore required, but the public
# header that callers include from elsewhere in the tree still pulls in
# zero Qt symbols.
# $(PYTHON_INCLUDE) is empty when OMNISIM_WITH_NEWTON=OFF so it's harmless;
# in =ON builds it's where Python.h lives for OmNewtonBackend's embedded
# CPython interpreter.
%.o: physics/%.cpp
	@echo "# compiling" $<
	@$(CXX) -c $(CFLAGS) $(CXXFLAGS) $(QT_CORE_INCLUDE) $(PYTHON_INCLUDE) -Iphysics -Icore $< -o $(OBJDIR)/$@

# Rendering-backend abstraction (R0 of engine-migration-plan.md).
# Twin of the physics/ rule above. R0 stays Qt-free + WREN-free at the
# header level; the .cpp only depends on OmWrenBackend.hpp (also header-
# only at R0). When R1 introduces OmVulkanBackend.cpp this rule gains
# the Vulkan SDK include path the same way physics/ gained PYTHON_INCLUDE.
%.o: render/%.cpp
	@echo "# compiling" $<
	@$(CXX) -c $(CFLAGS) $(CXXFLAGS) $(QT_CORE_INCLUDE) $(QT_GUI_INCLUDE) $(INSTALL_INCLUDE) -Irender -Icore $< -o $(OBJDIR)/$@

%.o: control/%.cpp
	@echo "# compiling" $<
	@$(CXX) -c $(CFLAGS) $(CXXFLAGS) $(WB_CONTROL_INCLUDE) $< -o $(OBJDIR)/$@

%.o: core/%.cpp
	@echo "# compiling" $<
	@$(CXX) -c $(CFLAGS) $(CXXFLAGS) $(WB_CORE_INCLUDE) $< -o $(OBJDIR)/$@

%.o: widgets/%.cpp
	@echo "# compiling" $<
	@$(CXX) -c $(CFLAGS) $(CXXFLAGS) $(WB_WIDGETS_INCLUDE) $< -o $(OBJDIR)/$@

%.o: editor/%.cpp
	@echo "# compiling" $<
	@$(CXX) -c $(CFLAGS) $(CXXFLAGS) $(WB_EDITOR_INCLUDE) $< -o $(OBJDIR)/$@

%.o: engine/%.cpp
	@echo "# compiling" $<
	@$(CXX) -c $(CFLAGS) $(CXXFLAGS) $(WB_ENGINE_INCLUDE) $< -o $(OBJDIR)/$@

%.o: external/siphash/%.cpp
	@echo "# compiling" $<
	@$(CXX) -c $(CFLAGS) $(CXXFLAGS) -isystem external/siphash $< -o $(OBJDIR)/$@

%.o: gui/%.cpp
	@echo "# compiling" $<
	@$(CXX) -c $(CFLAGS) $(CXXFLAGS) $(WB_GUI_INCLUDE) $< -o $(OBJDIR)/$@

%.o: nodes/utils/%.cpp
	@echo "# compiling" $<
	@$(CXX) -c $(CFLAGS) $(CXXFLAGS) $(WB_NODES_INCLUDE) $< -o $(OBJDIR)/$@

%.o: nodes/%.cpp
	@echo "# compiling" $<
	@$(CXX) -c $(CFLAGS) $(CXXFLAGS) $(WB_NODES_INCLUDE) $< -o $(OBJDIR)/$@

%.o: maths/%.cpp
	@echo "# compiling" $<
	@$(CXX) -c $(CFLAGS) $(CXXFLAGS) $(WB_MATHS_INCLUDE) $< -o $(OBJDIR)/$@



%.o: util/%.cpp
	@echo "# compiling" $<
	@$(CXX) -c $(CFLAGS) $(CXXFLAGS) $(WB_UTIL_INCLUDE) $< -o $(OBJDIR)/$@

%.o: scene_tree/%.cpp
	@echo "# compiling" $<
	@$(CXX) -c $(CFLAGS) $(CXXFLAGS) $(WB_SCENE_TREE_INCLUDE) $< -o $(OBJDIR)/$@

%.o: sound/%.cpp
	@echo "# compiling" $<
	@$(CXX) -c $(CFLAGS) $(CXXFLAGS) $(WB_SOUND_INCLUDE) $< -o $(OBJDIR)/$@

%.o: user_commands/%.cpp
	@echo "# compiling" $<
	@$(CXX) -c $(CFLAGS) $(CXXFLAGS) $(WB_USER_COMMANDS_INCLUDE) $< -o $(OBJDIR)/$@

%.o: vrml/%.cpp
	@echo "# compiling" $<
	@$(CXX) -c $(CFLAGS) $(CXXFLAGS) $(WB_VRML_INCLUDE) $< -o $(OBJDIR)/$@

$(OBJDIR)/x86_64/%.o: $(OBJDIR)/%.cpp
	@echo "# compiling" $< "(x86_64)"
	@$(CXX) -target x86_64-apple-macos12 -c $(CFLAGS) $(CXXFLAGS) $(INCLUDE) $< -o $@

$(OBJDIR)/x86_64/%.o: app/%.cpp
	@echo "# compiling" $< "(x86_64)"
	@$(CXX) -target x86_64-apple-macos12 -c $(CFLAGS) $(CXXFLAGS) $(WB_APP_INCLUDE) $< -o $@

$(OBJDIR)/x86_64/%.o: control/%.cpp
	@echo "# compiling" $< "(x86_64)"
	@$(CXX) -target x86_64-apple-macos12 -c $(CFLAGS) $(CXXFLAGS) $(WB_CONTROL_INCLUDE) $< -o $@

$(OBJDIR)/x86_64/%.o: core/%.cpp
	@echo "# compiling" $< "(x86_64)"
	@$(CXX) -target x86_64-apple-macos12 -c $(CFLAGS) $(CXXFLAGS) $(WB_CORE_INCLUDE) $< -o $@

$(OBJDIR)/x86_64/%.o: widgets/%.cpp
	@echo "# compiling" $< "(x86_64)"
	@$(CXX) -target x86_64-apple-macos12 -c $(CFLAGS) $(CXXFLAGS) $(WB_WIDGETS_INCLUDE) $< -o $@

$(OBJDIR)/x86_64/%.o: editor/%.cpp
	@echo "# compiling" $< "(x86_64)"
	@$(CXX) -target x86_64-apple-macos12 -c $(CFLAGS) $(CXXFLAGS) $(WB_EDITOR_INCLUDE) $< -o $@

$(OBJDIR)/x86_64/%.o: engine/%.cpp
	@echo "# compiling" $< "(x86_64)"
	@$(CXX) -target x86_64-apple-macos12 -c $(CFLAGS) $(CXXFLAGS) $(WB_ENGINE_INCLUDE) $< -o $@

$(OBJDIR)/x86_64/%.o: external/siphash/%.cpp
	@echo "# compiling" $< "(x86_64)"
	@$(CXX) -target x86_64-apple-macos12 -c $(CFLAGS) $(CXXFLAGS) -isystem external/siphash $< -o $@

$(OBJDIR)/x86_64/%.o: gui/%.cpp
	@echo "# compiling" $< "(x86_64)"
	@$(CXX) -target x86_64-apple-macos12 -c $(CFLAGS) $(CXXFLAGS) $(WB_GUI_INCLUDE) $< -o $@

$(OBJDIR)/x86_64/%.o: nodes/utils/%.cpp
	@echo "# compiling" $< "(x86_64)"
	@$(CXX) -target x86_64-apple-macos12 -c $(CFLAGS) $(CXXFLAGS) $(WB_NODES_INCLUDE) $< -o $@

$(OBJDIR)/x86_64/%.o: nodes/%.cpp
	@echo "# compiling" $< "(x86_64)"
	@$(CXX) -target x86_64-apple-macos12 -c $(CFLAGS) $(CXXFLAGS) $(WB_NODES_INCLUDE) $< -o $@

$(OBJDIR)/x86_64/%.o: maths/%.cpp
	@echo "# compiling" $< "(x86_64)"
	@$(CXX) -target x86_64-apple-macos12 -c $(CFLAGS) $(CXXFLAGS) $(WB_MATHS_INCLUDE) $< -o $@



$(OBJDIR)/x86_64/%.o: util/%.cpp
	@echo "# compiling" $< "(x86_64)"
	@$(CXX) -target x86_64-apple-macos12 -c $(CFLAGS) $(CXXFLAGS) $(WB_UTIL_INCLUDE) $< -o $@

$(OBJDIR)/x86_64/%.o: scene_tree/%.cpp
	@echo "# compiling" $< "(x86_64)"
	@$(CXX) -target x86_64-apple-macos12 -c $(CFLAGS) $(CXXFLAGS) $(WB_SCENE_TREE_INCLUDE) $< -o $@

$(OBJDIR)/x86_64/%.o: sound/%.cpp
	@echo "# compiling" $< "(x86_64)"
	@$(CXX) -target x86_64-apple-macos12 -c $(CFLAGS) $(CXXFLAGS) $(WB_SOUND_INCLUDE) $< -o $@

$(OBJDIR)/x86_64/%.o: user_commands/%.cpp
	@echo "# compiling" $< "(x86_64)"
	@$(CXX) -target x86_64-apple-macos12 -c $(CFLAGS) $(CXXFLAGS) $(WB_USER_COMMANDS_INCLUDE) $< -o $@

$(OBJDIR)/x86_64/%.o: vrml/%.cpp
	@echo "# compiling" $< "(x86_64)"
	@$(CXX) -target x86_64-apple-macos12 -c $(CFLAGS) $(CXXFLAGS) $(WB_VRML_INCLUDE) $< -o $@


$(OBJDIR)/arm64/%.o: $(OBJDIR)/%.cpp
	@echo "# compiling" $< "(arm64)"
	@$(CXX) -target arm64-apple-macos12 -c $(CFLAGS) $(CXXFLAGS) $(INCLUDE) $< -o $@

$(OBJDIR)/arm64/%.o: app/%.cpp
	@echo "# compiling" $< "(arm64)"
	@$(CXX) -target arm64-apple-macos12 -c $(CFLAGS) $(CXXFLAGS) $(WB_APP_INCLUDE) $< -o $@

$(OBJDIR)/arm64/%.o: control/%.cpp
	@echo "# compiling" $< "(arm64)"
	@$(CXX) -target arm64-apple-macos12 -c $(CFLAGS) $(CXXFLAGS) $(WB_CONTROL_INCLUDE) $< -o $@

$(OBJDIR)/arm64/%.o: core/%.cpp
	@echo "# compiling" $< "(arm64)"
	@$(CXX) -target arm64-apple-macos12 -c $(CFLAGS) $(CXXFLAGS) $(WB_CORE_INCLUDE) $< -o $@

$(OBJDIR)/arm64/%.o: widgets/%.cpp
	@echo "# compiling" $< "(arm64)"
	@$(CXX) -target arm64-apple-macos12 -c $(CFLAGS) $(CXXFLAGS) $(WB_WIDGETS_INCLUDE) $< -o $@

$(OBJDIR)/arm64/%.o: editor/%.cpp
	@echo "# compiling" $< "(arm64)"
	@$(CXX) -target arm64-apple-macos12 -c $(CFLAGS) $(CXXFLAGS) $(WB_EDITOR_INCLUDE) $< -o $@

$(OBJDIR)/arm64/%.o: engine/%.cpp
	@echo "# compiling" $< "(arm64)"
	@$(CXX) -target arm64-apple-macos12 -c $(CFLAGS) $(CXXFLAGS) $(WB_ENGINE_INCLUDE) $< -o $@

$(OBJDIR)/arm64/%.o: external/siphash/%.cpp
	@echo "# compiling" $< "(arm64)"
	@$(CXX) -target arm64-apple-macos12 -c $(CFLAGS) $(CXXFLAGS) -isystem external/siphash $< -o $@

$(OBJDIR)/arm64/%.o: gui/%.cpp
	@echo "# compiling" $< "(arm64)"
	@$(CXX) -target arm64-apple-macos12 -c $(CFLAGS) $(CXXFLAGS) $(WB_GUI_INCLUDE) $< -o $@

$(OBJDIR)/arm64/%.o: nodes/utils/%.cpp
	@echo "# compiling" $< "(arm64)"
	@$(CXX) -target arm64-apple-macos12 -c $(CFLAGS) $(CXXFLAGS) $(WB_NODES_INCLUDE) $< -o $@

$(OBJDIR)/arm64/%.o: nodes/%.cpp
	@echo "# compiling" $< "(arm64)"
	@$(CXX) -target arm64-apple-macos12 -c $(CFLAGS) $(CXXFLAGS) $(WB_NODES_INCLUDE) $< -o $@

$(OBJDIR)/arm64/%.o: maths/%.cpp
	@echo "# compiling" $< "(arm64)"
	@$(CXX) -target arm64-apple-macos12 -c $(CFLAGS) $(CXXFLAGS) $(WB_MATHS_INCLUDE) $< -o $@



$(OBJDIR)/arm64/%.o: util/%.cpp
	@echo "# compiling" $< "(arm64)"
	@$(CXX) -target arm64-apple-macos12 -c $(CFLAGS) $(CXXFLAGS) $(WB_UTIL_INCLUDE) $< -o $@

$(OBJDIR)/arm64/%.o: scene_tree/%.cpp
	@echo "# compiling" $< "(arm64)"
	@$(CXX) -target arm64-apple-macos12 -c $(CFLAGS) $(CXXFLAGS) $(WB_SCENE_TREE_INCLUDE) $< -o $@

$(OBJDIR)/arm64/%.o: sound/%.cpp
	@echo "# compiling" $< "(arm64)"
	@$(CXX) -target arm64-apple-macos12 -c $(CFLAGS) $(CXXFLAGS) $(WB_SOUND_INCLUDE) $< -o $@

$(OBJDIR)/arm64/%.o: user_commands/%.cpp
	@echo "# compiling" $< "(arm64)"
	@$(CXX) -target arm64-apple-macos12 -c $(CFLAGS) $(CXXFLAGS) $(WB_USER_COMMANDS_INCLUDE) $< -o $@

$(OBJDIR)/arm64/%.o: vrml/%.cpp
	@echo "# compiling" $< "(arm64)"
	@$(CXX) -target arm64-apple-macos12 -c $(CFLAGS) $(CXXFLAGS) $(WB_VRML_INCLUDE) $< -o $@


clean:
	@echo "# cleaning"
	@rm -rf $(TARGET) $(FILES_TO_REMOVE) $(LAUNCHER) $(LAUNCHERW) $(LAUNCHER_LEGACY) $(LAUNCHERW_LEGACY) build launcher/launcher.o $(OMNISIM_PATH)/.build_tmp/make-probes.mk $(OMNISIM_PATH)/.build_tmp/make-probes.key

count-lines:
	wc -l */*.c* */*.h*
