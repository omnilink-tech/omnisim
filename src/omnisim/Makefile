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

WEBOTS_PATH = ../..

include $(WEBOTS_PATH)/resources/Makefile.os.include

# Force-export TMP/TEMP to recipe sub-shells (msys make otherwise drops them,
# leaving g++/ld trying C:\Windows for temp files -> Permission denied builds).
export TMP ?= C:/tmp/build_tmp
export TEMP ?= C:/tmp/build_tmp
export TMPDIR ?= C:/tmp/build_tmp

WEBOTS_DEPENDENCY_PATH ?= $(WEBOTS_PATH)/dependencies

# compiler
ifeq ($(CC),cc)
CC = gcc
endif

# module dependencies: please keep these lists of dependencies up-to-date so that we don't break dependencies in the code
# listed from the module with the less dependencies to the module with the more dependencies

WB_ODE_INCLUDE           = $(QT_CORE_INCLUDE) $(QT_GUI_INCLUDE) $(ODE_INCLUDE) $(WREN_INCLUDE) $(CONTROLLER_INCLUDE) -Iapp -Icore -Imaths -Inodes -Inodes/utils -Ivrml -Iengine
WB_MATHS_INCLUDE         = $(QT_CORE_INCLUDE)
WB_CORE_INCLUDE          = $(QT_CORE_INCLUDE) $(QT_NETWORK_INCLUDE) $(QT_GUI_INCLUDE) $(QT_QML_INCLUDE) $(CONTROLLER_INCLUDE) -Iqml -Iexternal/compilation_timestamp
WB_UTIL_INCLUDE          = $(QT_CORE_INCLUDE) $(ODE_INCLUDE) $(CONTROLLER_INCLUDE) -Isound -Icore
WB_WREN_INCLUDE          = $(QT_CORE_INCLUDE) $(QT_GUI_INCLUDE) $(QT_OPENGL_INCLUDE) $(WREN_INCLUDE) $(CONTROLLER_INCLUDE) -Icore -Imaths -Ivrml -Iutil
WB_VRML_INCLUDE          = $(QT_CORE_INCLUDE) $(QT_XML_INCLUDE) $(CONTROLLER_INCLUDE) -Icore -Imaths -Ivrml
WB_USER_COMMANDS_INCLUDE = $(QT_CORE_INCLUDE) $(QT_GUI_INCLUDE) $(QT_WIDGETS_INCLUDE) $(QT_NETWORK_INCLUDE) $(CONTROLLER_INCLUDE) -Icore -isystem external/siphash -Imaths -Inodes -Inodes/utils -Iode -Iphysics -Iuser_commands -Ivrml
WB_WIDGETS_INCLUDE       = $(QT_CORE_INCLUDE) $(QT_GUI_INCLUDE) $(QT_WIDGETS_INCLUDE) $(CONTROLLER_INCLUDE) -Icore -Imaths -Iuser_commands -Ivrml
WB_EDITOR_INCLUDE        = $(QT_CORE_INCLUDE) $(QT_GUI_INCLUDE) $(QT_WIDGETS_INCLUDE) $(QT_PRINT_SUPPORT_INCLUDE) $(CONTROLLER_INCLUDE) -Icore -Iwidgets -Imaths -Inodes -Inodes/utils -Iode -Iphysics -Iuser_commands -Ivrml
WB_PLUGINS_INCLUDE       = $(QT_CORE_INCLUDE) $(ODE_INCLUDE) $(CONTROLLER_INCLUDE) -Icore -Imaths -Inodes -Inodes/utils -Iphysics -Ivrml
WB_ENGINE_INCLUDE        = $(QT_CORE_INCLUDE) $(QT_NETWORK_INCLUDE) $(ODE_INCLUDE) $(WREN_INCLUDE) $(CONTROLLER_INCLUDE) -Icore -Imaths -Inodes -Inodes/utils -Iphysics -Irender -Iplugins -Isound -Iuser_commands -Iutil -Ivrml
WB_CONTROL_INCLUDE       = $(QT_CORE_INCLUDE) $(QT_NETWORK_INCLUDE) $(CONTROLLER_INCLUDE) -Icore -Iengine -Imaths -Inodes -Inodes/utils -Iode -Iphysics -Ivrml -Iutil
# dependency of NODES on QT_GUI is necessary in WbImageTexture to retrieve the size of a texture before loading it in Wren
WB_NODES_INCLUDE         = $(QT_CORE_INCLUDE) $(QT_NETWORK_INCLUDE) $(QT_GUI_INCLUDE) $(ODE_INCLUDE) $(CONTROLLER_INCLUDE) $(OIS_INCLUDE) $(FREETYPE_INCLUDE) $(WREN_INCLUDE) $(STB_INCLUDE) $(ASSIMP_INCLUDE) $(CUDA_INCLUDE) -Iapp -Icompute/cuda -Icore -Iengine -isystem external/siphash -Imaths -Inodes -Inodes/utils -Iode -Iphysics -Irender -Iplugins -Isound -Iutil -Ivrml
WB_APP_INCLUDE           = $(QT_CORE_INCLUDE) $(QT_NETWORK_INCLUDE) $(ODE_INCLUDE) $(WREN_INCLUDE) $(CONTROLLER_INCLUDE) -Icompute/cuda -Icontrol -Icore -Ieditor -Iengine -Imaths -Inodes -Inodes/utils -Iphysics -Iplugins -Iscene_tree -Ivrml
WB_SCENE_TREE_INCLUDE    = $(QT_CORE_INCLUDE) $(QT_GUI_INCLUDE) $(QT_WIDGETS_INCLUDE) $(ASSIMP_INCLUDE) $(CONTROLLER_INCLUDE) -isystem external/siphash -Iapp -Icompute/cuda -Icore -Ieditor -Imaths -Inodes -Inodes/utils -Iode -Iphysics -Iuser_commands -Ivrml -Iwidgets
WB_SOUND_INCLUDE         = $(QT_CORE_INCLUDE) $(QT_XML_INCLUDE) $(ODE_INCLUDE) $(CONTROLLER_INCLUDE) -Icore -Imaths -Inodes -Inodes/utils -Iphysics -Isound -Ivrml
WB_GUI_INCLUDE           = $(QT_CORE_INCLUDE) $(QT_NETWORK_INCLUDE) $(QT_WEBSOCKETS_INCLUDE) $(QT_GUI_INCLUDE) $(QT_WIDGETS_INCLUDE) $(QT_OPENGL_INCLUDE) $(QT_OPENGL_WIDGETS_INCLUDE) $(ODE_INCLUDE) $(WREN_INCLUDE) $(CONTROLLER_INCLUDE) -Iapp -Icontrol -Icore -Iwidgets -Ieditor -Iengine -isystem external/siphash -Imaths -Inodes -Inodes/utils -Iphysics -Irender -Iplugins -Iscene_tree -Isound -Iuser_commands -Iutil -Ivrml
WB_RENDER_INCLUDE        = $(QT_CORE_INCLUDE) $(QT_GUI_INCLUDE) $(QT_OPENGL_INCLUDE) $(QT_WIDGETS_INCLUDE) $(WREN_INCLUDE) -isystem external/siphash

HOSTTYPE ?= $(shell uname -m)

# clear out all suffixes for implicit rules to speed up Makefile
.SUFFIXES:
%:: %,v
%:: RCS/%,v
%:: RCS/%
%:: s.%
%:: SCCS/s.%

OIS_INCLUDE = -isystem $(WEBOTS_PATH)/include/libOIS
WREN_INCLUDE = -I$(WEBOTS_PATH)/include
STB_INCLUDE  = -isystem $(WEBOTS_PATH)/src/stb
OPEN_VR_INCLUDE = -isystem $(WEBOTS_PATH)/include/openvr

ifeq ($(OSTYPE),darwin)
EXECUTABLE_PATH    = $(WEBOTS_PATH)/Contents/MacOS
EXECUTABLE_PATH_SETUP := $(shell mkdir -p $(EXECUTABLE_PATH))
FRAMEWORKS         = $(WEBOTS_PATH)/Contents/Frameworks
TARGET             = $(EXECUTABLE_PATH)/webots
ODE                = $(WEBOTS_PATH)/src/ode
LIB_WREN           = ../wren/libwren.a ../glad/libglad.a
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
FREETYPE_INCLUDE   = -isystem $(WEBOTS_DEPENDENCY_PATH)/freetype2
ASSIMP_INCLUDE     = -I$(WEBOTS_DEPENDENCY_PATH)/assimp/include
LD_FLAGS           = -mmacosx-version-min=$(MACOSX_MIN_SDK_VERSION) -L$(WEBOTS_LIB_PATH) -lOIS -lassimp -Xlinker -rpath -Xlinker @loader_path/../..
MOC                =  $(WEBOTS_PATH)/bin/qt/moc
LIBS_FRAMEWORKS    = -F$(FRAMEWORKS) -framework OpenAL -framework OpenGL -framework QtCore -framework QtGui -framework QtPrintSupport -framework QtQml -framework QtNetwork -framework QtOpenGL -framework QtOpenGLWidgets -framework QtWebSockets -framework QtWidgets -framework QtXml -framework IOKit -framework CoreFoundation -framework Cocoa
LIBS              += -bind_at_load $(LIBS_FRAMEWORKS) $(LIB_WREN)
CFLAGS             = -isysroot $(MACOSX_SDK_PATH) -mmacosx-version-min=$(MACOSX_MIN_SDK_VERSION)
CXXFLAGS           = -std=c++11 -F$(FRAMEWORKS) -x objective-c++
EXTRA_CMD          = cp launcher/webots-mac.sh $(WEBOTS_PATH)/webots && chmod 755 $(WEBOTS_PATH)/webots
FILES_TO_REMOVE    = $(WEBOTS_PATH)/webots
ifeq ($(MAKECMDGOALS), profile)
LIB_ODE = ../ode/libode.a
LIBS += $(LIB_ODE)
LD_FLAGS += -lpthread
else
LD_FLAGS += -lode
endif
else ifeq ($(OSTYPE),linux)
LIB_WREN           = ../wren/libwren.a ../glad/libglad.a
ASSIMP_INCLUDE     = -I$(WEBOTS_PATH)/include/libassimp/include
QT_CORE_INCLUDE    = -isystem $(WEBOTS_PATH)/include/qt/QtCore
QT_GUI_INCLUDE     = -isystem $(WEBOTS_PATH)/include/qt/QtGui
QT_WIDGETS_INCLUDE = -isystem $(WEBOTS_PATH)/include/qt/QtWidgets
QT_PRINT_SUPPORT_INCLUDE = -isystem $(WEBOTS_PATH)/include/qt/QtPrintSupport
QT_QML_INCLUDE     = -isystem $(WEBOTS_PATH)/include/qt/QtQml
QT_OPENGL_INCLUDE  = -isystem $(WEBOTS_PATH)/include/qt/QtOpenGL
QT_OPENGL_WIDGETS_INCLUDE = -isystem $(WEBOTS_PATH)/include/qt/QtOpenGLWidgets
QT_NETWORK_INCLUDE = -isystem $(WEBOTS_PATH)/include/qt/QtNetwork
QT_WEBSOCKETS_INCLUDE = -isystem $(WEBOTS_PATH)/include/qt/QtWebSockets
QT_XML_INCLUDE     = -isystem $(WEBOTS_PATH)/include/qt/QtXml
FREETYPE_INCLUDE   = -I/usr/include/freetype2
TARGET             = $(WEBOTS_PATH)/bin/omnisim-bin
CFLAGS             = -fPIC
CXXFLAGS           = -std=c++11
MOC                = $(WEBOTS_PATH)/bin/qt/moc
LIBS              += $(LIB_WREN) -Wl,-rpath $(WEBOTS_LIB_PATH) -L$(WEBOTS_LIB_PATH) -lQt6Core -lQt6Network -lQt6Gui -lQt6OpenGL -lQt6OpenGLWidgets -lQt6WebSockets -lQt6Widgets -lQt6PrintSupport -lQt6Qml -lQt6Xml -lopenal -lGL -lGLU -ldl -lOIS -lfreetype -lassimp -lrt
LD_FLAGS           = -rdynamic
EXTRA_CMD          = cp launcher/webots-linux.sh $(WEBOTS_PATH)/webots && chmod 755 $(WEBOTS_PATH)/webots
FILES_TO_REMOVE    = $(WEBOTS_PATH)/webots
ifeq ($(MAKECMDGOALS), profile)
LIB_ODE = ../ode/libode.a
LIBS += $(LIB_ODE) -lpthread
else
LIBS += -lode -lpthread
endif

else ifeq ($(OSTYPE),windows)
vpath %.c ../controller/c
WB_NODES_INCLUDE         += $(OPEN_VR_INCLUDE)
C_SOURCES                += system.c
LIB_WREN                  = ../wren/wren.a ../glad/glad.a
QT_CORE_INCLUDE           = -isystem $(WEBOTS_PATH)/include/qt/QtCore
QT_GUI_INCLUDE            = -isystem $(WEBOTS_PATH)/include/qt/QtGui
QT_WIDGETS_INCLUDE        = -isystem $(WEBOTS_PATH)/include/qt/QtWidgets
QT_PRINT_SUPPORT_INCLUDE  = -isystem $(WEBOTS_PATH)/include/qt/QtPrintSupport
QT_QML_INCLUDE            = -isystem $(WEBOTS_PATH)/include/qt/QtQml
QT_OPENGL_INCLUDE         = -isystem $(WEBOTS_PATH)/include/qt/QtOpenGL
QT_OPENGL_WIDGETS_INCLUDE = -isystem $(WEBOTS_PATH)/include/qt/QtOpenGLWidgets
QT_NETWORK_INCLUDE        = -isystem $(WEBOTS_PATH)/include/qt/QtNetwork
QT_WEBSOCKETS_INCLUDE     = -isystem $(WEBOTS_PATH)/include/qt/QtWebSockets
QT_XML_INCLUDE            = -isystem $(WEBOTS_PATH)/include/qt/QtXml
ASSIMP_INCLUDE            = -I$(WEBOTS_PATH)/dependencies
FREETYPE_INCLUDE          = -I/mingw64/include/freetype2
TARGET_PATH               = $(WEBOTS_PATH)/msys64/mingw64/bin
TARGET                    = $(TARGET_PATH)/omnisim-bin.exe
CFLAGS                    = -isystem /mingw64/lib/libzip/include
CXXFLAGS                  = -std=c++11 -D_USE_MATH_DEFINES
MOC                       = /mingw64/share/qt6/bin/moc
MOC_FLAGS                += -D_WIN32
LIBS                     += $(LIB_WREN) -L/mingw64/lib -L/mingw64/bin -L$(TARGET_PATH) -lQt6Core -lQt6Network -lQt6Gui -lQt6OpenGL -lQt6OpenGLWidgets -lQt6WebSockets -lQt6Widgets -lQt6PrintSupport -lQt6Qml -lQt6Xml -lode -lopenal -lopengl32 -liphlpapi -ld3d9 -lgdi32 -lglu32 -lOIS -ldinput8 -ldxguid -lole32 -lsapi -loleaut32 -luuid -lfreetype-6 -lopenvr_api -lassimp-5
LD_FLAGS                  = -Wl,--enable-auto-import
EXTRA_CMD                 = cp launcher/omnisim-bin.exe.config $(TARGET_PATH)/
FILES_TO_REMOVE           = $(TARGET_PATH)/omnisim-bin.exe.config
ifeq ($(MAKECMDGOALS),release)
# -fipa-cp-clone included in -O3 seems to cause some stack overflow on Windows,
# in particular in WbController::setProcessEnvironment(), however increasing the
# stack size doesn't seems to help, so we need to remove this optimization
CFLAGS                   += -fno-ipa-cp-clone
LD_FLAGS                 += -Wl,-subsystem,windows
else
EXTRA_CMD                += ; cp launcher/omnisim_debug.bat $(TARGET_PATH)/omnisim_debug.bat
endif
ifeq ($(MAKECMDGOALS), profile)
LIB_ODE = ../ode/ode.a
LIBS += $(LIB_ODE) -lpthread
else
LIBS += -lode
endif
endif

ALL_INCLUDE        = -Iapp -Icontrol -Icore -Iwidgets -Ieditor -Iengine -isystem external/siphash -Igui -Imaths -Inodes -Inodes/utils -Iphysics -Irender -Iplugins -Iscene_tree -Isound -Iuser_commands -Ivrml -Iutil -Iexternal/compilation_timestamp -Icompute/cuda

# ---- CUDA compute layer (M0 of cuda-compute-infrastructure-plan.md) ---------
# OMNISIM_WITH_CUDA defaults to ON if `nvcc` is on PATH, OFF otherwise. Users
# can force either with `make OMNISIM_WITH_CUDA=ON` / `=OFF`.
ifeq ($(origin OMNISIM_WITH_CUDA), undefined)
  ifneq ($(shell command -v nvcc 2>/dev/null),)
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
  ifeq ($(OSTYPE),windows)
    CUDA_HOME    := $(shell p="$$(command -v $(CUDA_NVCC) 2>/dev/null)"; [ -n "$$p" ] && cygpath -m -d "$$(dirname "$$(dirname "$$p")")" 2>/dev/null)
    CUDA_LIB_DIR := $(CUDA_HOME)/lib/x64
  else
    CUDA_HOME    := $(shell p="$$(command -v $(CUDA_NVCC) 2>/dev/null)"; [ -n "$$p" ] && dirname "$$(dirname "$$p")")
    CUDA_LIB_DIR := $(CUDA_HOME)/lib64
  endif
  CUDA_INCLUDE    = -isystem $(CUDA_HOME)/include
  CFLAGS += -DOMNISIM_WITH_CUDA
  CXXFLAGS += -DOMNISIM_WITH_CUDA
  # cudart    — runtime API (cudaMalloc, cudaMemcpy, ...) used by WbCudaBuffer
  # cuda      — driver API (cuLaunchKernel, cuModuleLoadDataEx, ...) used by
  #             WbGranularGroup so we can dispatch NVRTC-compiled kernels
  #             without dragging nvcc-compiled .obj files into the mingw link.
  # nvrtc     — runtime kernel compilation, sidesteps the MSVC/mingw ABI gap.
  LIBS += -L$(CUDA_LIB_DIR) -lcudart -lcuda -lnvrtc
  ALL_INCLUDE += $(CUDA_INCLUDE)
endif

# ---- NVIDIA Newton physics backend (P0 of cuda-newton-physics-plan.md) ------
# DEFAULT BACKEND as of the Stage 3 flip (see the OMNISIM_WITH_NEWTON block
# below). ODE remains the permanent, byte-identical fallback (one flag away).
# OMNISIM_WITH_NEWTON=ON compiles the Newton backend in; the runtime field
# `physicsBackend "newton"` (or the flipped `"auto"` default) opts a Solid into
# the GPU solver. With the flag OFF, every Newton-related .cpp file is a no-op
# stub and the binary is byte-equivalent to a pure-ODE build.
#
# Newton itself is a Python package (newton + warp-lang on PyPI; the former
# was renamed from newton-physics in late 2025). Driven from C++ via the
# stable CPython API in WbNewtonBackend.cpp -- no pybind11 dep at P2 because
# the call surface is small. The flag now defaults to ON (Stage 3 flip);
# OMNISIM_WITH_NEWTON=OFF restores the pure-ODE legacy build.
# ---- Vulkan/wgpu render backend (R1 of engine-migration-plan.md) ------------
# DEFAULT ON as of the architectural-baseline C1 flip (2026-06-07) -- symmetric
# with OMNISIM_WITH_NEWTON: a no-flags `make` now COMPILES the wgpu backend in,
# so the new render architecture is "live by default." This does NOT change the
# runtime default: WREN stays the canonical renderer on every Camera/Viewpoint
# (the `renderBackend` field defaults to "wren"); only a world that opts in with
# `renderBackend "wgpu"` uses the GPU path (the Phase-ζ default flip is separate
# and gated on lighting parity). The wgpu code is DOUBLE-GUARDED:
#   * OMNISIM_WITH_VULKAN       -- compiles the backend TUs in (this flag), and
#   * WB_WGPU_NATIVE_AVAILABLE  -- set only when WGPU_NATIVE_HOME points at a real
#     wgpu-native lib (below); every wgpu* API call lives behind it.
# So with the flag ON but WGPU_NATIVE_HOME unset (the plain-`make` case on a box
# without wgpu-native), every wgpu* call compiles out, isAvailable() reports
# false, and the binary is byte-equivalent to a WREN-only build -- no link
# against wgpu_native, no runtime change. OMNISIM_WITH_VULKAN=OFF restores the
# pre-flip build. (The "vulkan" name is historical; the backend is wgpu-native --
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
          EXTRA_CMD += ; cp $(WGPU_NATIVE_HOME)/lib/wgpu_native.dll $(TARGET_PATH)/
        endif
      endif
    endif
  endif
endif

# DEFAULT ON (default-flip-plan.md §4.3 Stage 3 / engine-migration-plan.md §13.4):
# Newton is the default physics backend. A no-flags `make` builds it in; an
# unspecified Solid resolves to Newton when the runtime is present AND its
# articulation is Newton-faithful (the WbSolid capability gate routes mesh /
# non-Hinge-Slider-joint articulations to ODE), else ODE. ODE stays the
# permanent, byte-identical fallback: build-time `make OMNISIM_WITH_NEWTON=OFF`
# for a pure-ODE binary, or runtime OMNISIM_LEGACY=1 / OMNISIM_FORCE_ODE=1 /
# physicsBackend "ode". The Newton runtime (newton + warp-lang on PyPI) is NOT
# bundled — when absent, isAvailable() is false and `auto` falls back to ODE
# (safe, but not actually Newton; see docs/guide/newton-physics-backend.md).
OMNISIM_WITH_NEWTON ?= ON
ifeq ($(OMNISIM_WITH_NEWTON),ON)
  CFLAGS   += -DOMNISIM_WITH_NEWTON
  CXXFLAGS += -DOMNISIM_WITH_NEWTON
  # Newton lives behind an embedded CPython interpreter — we drive
  # newton + warp-lang from C++ via the stable CPython API
  # (no pybind11 dependency for P2 because the call surface is small:
  # Py_Initialize / PyImport_ImportModule / PyObject_CallMethod). The
  # interpreter is initialised lazily inside WbNewtonBackend so the
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
        PYTHON_PROBE := $(wildcard $(subst \,/,$(shell py -3 -c "import sys; print(sys.prefix)" 2>/dev/null))/include/Python.h)
      endif
      ifeq ($(PYTHON_PROBE),)
        # Last resort: whatever python is on PATH (skips the MSYS2 python
        # automatically because its include layout doesn't match).
        PYTHON_PROBE := $(wildcard $(subst \,/,$(shell python -c "import sys; print(sys.prefix)" 2>/dev/null))/include/Python.h)
      endif
      ifneq ($(PYTHON_PROBE),)
        PYTHON_HOME := $(patsubst %/include/Python.h,%,$(PYTHON_PROBE))
      else
        $(error OMNISIM_WITH_NEWTON=ON (the default) embeds CPython and needs its build headers. Install Python 3 from python.org (default per-user location), or pass PYTHON_HOME=<dir containing include/Python.h and libs/pythonNNN.lib> plus PYTHON_LIB=-lpythonNNN as make arguments, or build the pure-ODE legacy stack with OMNISIM_WITH_NEWTON=OFF)
      endif
    endif
    # Derive -lpythonNNN from the directory name (".../Python312" ->
    # -lpython312); set PYTHON_LIB explicitly for non-standard layouts.
    PYTHON_INCLUDE := -isystem "$(PYTHON_HOME)/include"
    PYTHON_LIBDIR  := "$(PYTHON_HOME)/libs"
    PYTHON_LIB     ?= -l$(subst Python,python,$(notdir $(PYTHON_HOME)))
  else
    PYTHON_HOME ?= /usr
    PYTHON_INCLUDE := -isystem $(PYTHON_HOME)/include/python3.12
    PYTHON_LIBDIR  := $(PYTHON_HOME)/lib
    PYTHON_LIB     := -lpython3.12
  endif
  ALL_INCLUDE += $(PYTHON_INCLUDE)
  LIBS += -L$(PYTHON_LIBDIR) $(PYTHON_LIB)
endif

CONTROLLER_INCLUDE = -I$(WEBOTS_PATH)/include/controller/c
ODE_INCLUDE        = -Iode -isystem $(WEBOTS_PATH)/include/ode
QT_CORE_INCLUDE   += -I$(OBJDIR)
WREN_INCLUDE      += -Iwren
INCLUDE            = $(ALL_INCLUDE) $(CONTROLLER_INCLUDE) $(ODE_INCLUDE) $(QT_CORE_INCLUDE) $(QT_GUI_INCLUDE) $(QT_WIDGETS_INCLUDE) $(QT_PRINT_SUPPORT_INCLUDE) $(QT_OPENGL_INCLUDE) $(QT_NETWORK_INCLUDE) $(OIS_INCLUDE) $(WREN_INCLUDE) $(FREETYPE_INCLUDE) $(STB_INCLUDE) $(ASSIMP_INCLUDE)

ifeq ($(MAKECMDGOALS),debug)
CFLAGS += -ggdb
endif

ifeq ($(TREAT_WARNINGS_AS_ERRORS),1)
CFLAGS += -Werror
endif

ifeq ($(MAKECMDGOALS),profile)
CFLAGS += -pg -O3 -DNDEBUG
ifneq ($(OSTYPE),darwin)
LD_FLAGS += -pg
endif
endif

ifeq ($(MAKECMDGOALS),release)
CFLAGS += -O3 -DNDEBUG
ifneq ($(OSTYPE),darwin)
LD_FLAGS += -s
endif
endif

OBJDIR=build/$(MAKECMDGOALS)
ifneq (,$(findstring $(MAKECMDGOALS),debug profile release))
ifeq ($(OSTYPE),darwin)
BUILD_PATH_SETUP := $(shell mkdir -p $(OBJDIR) $(OBJDIR)/x86_64 $(OBJDIR)/arm64)
else
BUILD_PATH_SETUP := $(shell mkdir -p $(OBJDIR))
endif
endif

CFLAGS += -Wall -Wpointer-arith -Wcast-align -Wwrite-strings
CXXFLAGS += -std=c++17 -DQT_NO_DEBUG

ifneq ($(OSTYPE),darwin) # Clang has these warnings by default
GCC_VER_GREATER_OR_EQUAL_TO_5_DOT_4 := $(shell expr `gcc -dumpversion | sed -e 's/\.\([0-9][0-9]\)/\1/g' -e 's/\.\([0-9]\)/0\1/g' -e 's/^[0-9]\{3,4\}$$/&00/'` \>= 50400)
ifeq ($(GCC_VER_GREATER_OR_EQUAL_TO_5_DOT_4),1)
CXXFLAGS += -Wsuggest-override
endif
endif

# objects that inherit from QObject (alphabetical order)
QT_SOURCES = WbAboutBox.cpp \
  WbAbstractAppearance.cpp \
  WbAbstractDragEvent.cpp \
  WbAbstractCamera.cpp \
  WbAccelerometer.cpp \
  WbActionManager.cpp \
  WbAddInertiaMatrixDialog.cpp \
  WbAddNodeDialog.cpp \
  WbAgentHud.cpp \
  WbAltimeter.cpp \
  WbAnimationRecorder.cpp \
  WbAppearance.cpp \
  WbApplication.cpp \
  WbBackground.cpp \
  WbBallJoint.cpp \
  WbBallJointParameters.cpp \
  WbBaseNode.cpp \
  WbBasicJoint.cpp \
  WbBillboard.cpp \
  WbBoolEditor.cpp \
  WbBox.cpp \
  WbBrake.cpp \
  WbCadShape.cpp \
  WbCamera.cpp \
  WbCapsule.cpp \
  WbCharger.cpp \
  WbColor.cpp \
  WbColorEditor.cpp \
  WbCompass.cpp \
  WbCone.cpp \
  WbConnector.cpp \
  WbConsole.cpp \
  WbContactPointsRepresentation.cpp \
  WbContactProperties.cpp \
  WbControlledWorld.cpp \
  WbController.cpp \
  WbCoordinate.cpp \
  WbCoordinateSystem.cpp \
  WbCylinder.cpp \
  WbDamping.cpp \
  WbDirectionalLight.cpp \
  WbDistanceSensor.cpp \
  WbDisplay.cpp \
  WbDockTitleBar.cpp \
  WbDockWidget.cpp \
  WbDoubleEditor.cpp \
  WbDoubleSpinBox.cpp \
  WbDownloadManager.cpp \
  WbDownloader.cpp \
  WbDragPoseEvent.cpp \
  WbDragResizeEvent.cpp \
  WbDragSolidEvent.cpp \
  WbElevationGrid.cpp \
  WbEmitter.cpp \
  WbExtendedStringEditor.cpp \
  WbExternProtoEditor.cpp \
  WbInsertExternProtoDialog.cpp \
  WbField.cpp \
  WbFieldChecker.cpp \
  WbFieldDoubleSpinBox.cpp \
  WbFieldEditor.cpp \
  WbFieldIntSpinBox.cpp \
  WbFieldLineEdit.cpp \
  WbFindReplaceDialog.cpp \
  WbFieldValueRestriction.cpp \
  WbFluid.cpp \
  WbFocus.cpp \
  WbFog.cpp \
  WbGeometry.cpp \
  WbGps.cpp \
  WbGranularGroup.cpp \
  WbGroup.cpp \
  WbGuiApplication.cpp \
  WbGuidedTour.cpp \
  WbGuiRefreshOracle.cpp \
  WbGyro.cpp \
  WbHingeJoint.cpp \
  WbHingeJointParameters.cpp \
  WbHinge2Joint.cpp \
  WbImageTexture.cpp \
  WbImmersionProperties.cpp \
  WbIntEditor.cpp \
  WbIntSpinBox.cpp \
  WbIndexedFaceSet.cpp \
  WbIndexedLineSet.cpp \
  WbInertialUnit.cpp \
  WbJoint.cpp \
  WbJointDevice.cpp \
  WbJointParameters.cpp \
  WbJoystickInterface.cpp \
  WbJoystickListener.cpp \
  WbLed.cpp \
  WbLens.cpp \
  WbLensFlare.cpp \
  WbLidar.cpp \
  WbLight.cpp \
  WbLightSensor.cpp \
  WbLinearMotor.cpp \
  WbLineEdit.cpp \
  WbLog.cpp \
  WbMainWindow.cpp \
  WbMassChecker.cpp \
  WbMaterial.cpp \
  WbMatter.cpp \
  WbMesh.cpp \
  WbMFBool.cpp \
  WbMFColor.cpp \
  WbMFDouble.cpp \
  WbMFInt.cpp \
  WbMFNode.cpp \
  WbMFRotation.cpp \
  WbMFString.cpp \
  WbMFVector2.cpp \
  WbMFVector3.cpp \
  WbMicrophone.cpp \
  WbMotor.cpp \
  WbMouse.cpp \
  WbMultipleValue.cpp \
  WbMuscle.cpp \
  WbNewControllerWizard.cpp \
  WbNewPhysicsPluginWizard.cpp \
  WbNewProjectWizard.cpp \
  WbNewProtoWizard.cpp \
  WbNewWorldWizard.cpp \
  WbNewVersionDialog.cpp \
  WbNode.cpp \
  WbNodeEditor.cpp \
  WbNodeOperations.cpp \
  WbNodePane.cpp \
  WbNodeReader.cpp \
  WbNormal.cpp \
  WbOdeContext.cpp \
  WbOpenSampleWorldDialog.cpp \
  WbPaintTexture.cpp \
  WbPbrAppearance.cpp \
  WbPen.cpp \
  WbPhysics.cpp \
  WbPhysicsPlugin.cpp \
  WbPhysicsVectorRepresentation.cpp \
  WbPhysicsViewer.cpp \
  WbPlane.cpp \
  WbPlugin.cpp \
  WbPointLight.cpp \
  WbPointSet.cpp \
  WbPositionSensor.cpp \
  WbPositionViewer.cpp \
  WbPose.cpp \
  WbPreferences.cpp \
  WbPreferencesDialog.cpp \
  WbPropeller.cpp \
  WbProject.cpp \
  WbProjectRelocationDialog.cpp \
  WbProtoIcon.cpp \
  WbProtoManager.cpp \
  WbProtoModel.cpp \
  WbProtoTemplateEngine.cpp \
  WbProtoTreeItem.cpp \
  WbQjsFile.cpp \
  WbRadar.cpp \
  WbRadio.cpp \
  WbRadioPlugin.cpp \
  WbRangeFinder.cpp \
  WbReceiver.cpp \
  WbRecentFilesList.cpp \
  WbRecognition.cpp \
  WbRenderingDevice.cpp \
  WbRenderingDeviceWindow.cpp \
  WbRenderingDeviceWindowFactory.cpp \
  WbResizeManipulator.cpp \
  WbRotationEditor.cpp \
  WbRobot.cpp \
  WbRobotWindow.cpp \
  WbRotationalMotor.cpp \
  WbSaveWarningDialog.cpp \
  WbSceneTree.cpp \
  WbSceneTreeModel.cpp \
  WbSelection.cpp \
  WbSensor.cpp \
  WbSFBool.cpp \
  WbSFColor.cpp \
  WbSFDouble.cpp \
  WbSFInt.cpp \
  WbSFNode.cpp \
  WbSFRotation.cpp \
  WbSFString.cpp \
  WbSFVector2.cpp \
  WbSFVector3.cpp \
  WbShape.cpp \
  WbSimulationState.cpp \
  WbSimulationStateIndicator.cpp \
  WbSimulationView.cpp \
  WbSimulationWorld.cpp \
  WbSingleValue.cpp \
  WbSingleTaskApplication.cpp \
  WbSkin.cpp \
  WbSliderJoint.cpp \
  WbSlot.cpp \
  WbSolid.cpp \
  WbSolidMerger.cpp \
  WbSolidReference.cpp \
  WbSpeaker.cpp \
  WbSplashScreen.cpp \
  WbSphere.cpp \
  WbSpotLight.cpp \
  WbStringEditor.cpp \
  WbTcpServer.cpp \
  WbSupervisorUtilities.cpp \
  WbSyntaxHighlighter.cpp \
  WbTemplateEngine.cpp \
  WbTemplateManager.cpp \
  WbTextFind.cpp \
  WbTextureCoordinate.cpp \
  WbTextureTransform.cpp \
  WbTouchSensor.cpp \
  WbTrack.cpp \
  WbTrackWheel.cpp \
  WbTransform.cpp \
  WbTranslateRotateManipulator.cpp \
  WbTreeItem.cpp \
  WbTreeView.cpp \
  WbTriangleMeshGeometry.cpp \
  WbUndoStack.cpp \
  WbUpdatedDialog.cpp \
  WbValue.cpp \
  WbValueEditor.cpp \
  WbVariant.cpp \
  WbVector2Editor.cpp \
  WbVector3Editor.cpp \
  WbVelocityViewer.cpp \
  WbVideoRecorder.cpp \
  WbVideoRecorderDialog.cpp \
  WbMultimediaStreamingServer.cpp \
  WbVacuumGripper.cpp \
  WbViewpoint.cpp \
  WbView3D.cpp \
  WbVisualBoundingSphere.cpp \
  WbWebotsUpdateDialog.cpp \
  WbWebotsUpdateManager.cpp \
  WbWorld.cpp \
  WbWorldInfo.cpp \
  WbWrenAbstractManipulator.cpp \
  WbWrenAbstractResizeManipulator.cpp \
  WbWrenCamera.cpp \
  WbWrenOpenGlContext.cpp \
  WbWrenFullScreenOverlay.cpp \
  WbWrenRenderingContext.cpp \
  WbWrenTextureOverlay.cpp \
  WbWrenWindow.cpp \
  WbW3dStreamingServer.cpp \
  WbZoom.cpp

# objects that don't inherit from QObject (alphabetical order)
OTHER_SOURCES = \
  main.cpp \
  WbAbstractPose.cpp \
  WbAddItemCommand.cpp \
  WbAffinePlane.cpp \
  WbApplicationInfo.cpp \
  WbBinaryIncubator.cpp \
  WbBoundingSphere.cpp \
  WbClipboard.cpp \
  WbConcreteNodeFactory.cpp \
  WbContactSoundManager.cpp \
  WbContactSound.cpp \
  WbContextMenuGenerator.cpp \
  WbControllerPlugin.cpp \
  WbDataPacket.cpp \
  WbDataStream.cpp \
  WbDevice.cpp \
  WbDesktopServices.cpp \
  WbDictionary.cpp \
  WbDisplayFont.cpp \
  WbDragOverlayEvent.cpp \
  WbDragViewpointEvent.cpp \
  WbDragScaleEvent.cpp \
  WbEditCommand.cpp \
  WbFieldModel.cpp \
  WbFileUtil.cpp \
  WbHiddenKinematicParameters.cpp \
  WbHttpReply.cpp \
  WbIniParser.cpp \
  WbKinematicDifferentialWheels.cpp \
  WbLanguage.cpp \
  WbLanguageTools.cpp \
  WbLightRepresentation.cpp \
  WbLogicalDevice.cpp \
  WbLookupTable.cpp \
  WbMacAddress.cpp \
  WbMatrix3.cpp \
  WbMatrix4.cpp \
  WbMathsUtilities.cpp \
  WbMessageBox.cpp \
  WbMotorSoundManager.cpp \
  WbMultimediaStreamingLimiter.cpp \
  WbNetwork.cpp \
  WbNodeFactory.cpp \
  WbNodeModel.cpp \
  WbNodeProtoInfo.cpp \
  WbNodeUtilities.cpp \
  WbObjectDetection.cpp \
  WbOdeDebugger.cpp \
  WbOdeUtilities.cpp \
  WbParser.cpp \
  WbPerformanceLog.cpp \
  WbPerspective.cpp \
  WbPolygon.cpp \
  WbPrecision.cpp \
  WbRandom.cpp \
  WbRay.cpp \
  WbResetCommand.cpp \
  WbResizeCommand.cpp \
  WbResizeAndTranslateCommand.cpp \
  WbRotation.cpp \
  WbSimulationCluster.cpp \
  WbSolidDevice.cpp \
  WbSolidUtilities.cpp \
  WbSoundClip.cpp \
  WbSoundEngine.cpp \
  WbSoundSource.cpp \
  WbSpotLightRepresentation.cpp \
  WbStandardPaths.cpp \
  WbSupportPolygonRepresentation.cpp \
  WbSysInfo.cpp \
  WbTesselator.cpp \
  WbToken.cpp \
  WbTokenizer.cpp \
  WbTriangleMesh.cpp \
  WbTriangleMeshCache.cpp \
  WbUrdfImporter.cpp \
  WbUrl.cpp \
  WbVector4.cpp \
  WbVersion.cpp \
  WbVrmlNodeUtilities.cpp \
  WbWaveFile.cpp \
  WbWheelEvent.cpp \
  WbWrenAbstractPostProcessingEffect.cpp \
  WbWrenAtmosphericSky.cpp \
  WbWrenBloom.cpp \
  WbWrenColorNoise.cpp \
  WbWrenDepthOfField.cpp \
  WbWrenGtao.cpp \
  WbWrenHdr.cpp \
  WbWrenLabelOverlay.cpp \
  WbWrenLensDistortion.cpp \
  WbWrenLensFlare.cpp \
  WbWrenMeshBuffers.cpp \
  WbWrenMotionBlur.cpp \
  WbWrenNoiseMask.cpp \
  WbWrenPicker.cpp \
  WbWrenPostProcessingEffects.cpp \
  WbWrenRangeNoise.cpp \
  WbWrenRangeQuantization.cpp \
  WbWrenShaders.cpp \
  WbWrenSmaa.cpp \
  WbWrenVertexArrayFrameListener.cpp \
  WbWriter.cpp \
  WbCudaContext.cpp \
  WbCudaSmoke.cpp \
  WbPhysicsBackend.cpp \
  WbOdeBackend.cpp \
  WbNewtonBackend.cpp \
  WbRenderBackend.cpp \
  WbVulkanBackend.cpp \
  WbWgpuGlBlit.cpp \
  WbWgpuImageAdapter.cpp \
  WbWgpuMeshAdapter.cpp \
  WbWgpuMeshCache.cpp \
  WbWgpuRenderTarget.cpp \
  WbWgpuSceneRenderer.cpp \
  WbWgpuShaders.cpp \
  WbWgpuSurface.cpp \
  WbWgpuTextureCache.cpp \
  WbWgpuView.cpp \
  GpuPassTimer.cpp \
  CpuPassTimer.cpp

ifeq ($(OSTYPE),windows)
QT_SOURCES += \
  WbVirtualRealityHeadset.cpp
OTHER_SOURCES += \
  WbMicrosoftTextToSpeech.cpp \
  WbWindowsRegistry.cpp
else
OTHER_SOURCES += \
  WbPosixMemoryMappedFile.cpp
endif

MOC_FILES=$(addprefix $(OBJDIR)/,$(notdir $(QT_SOURCES:%.cpp=%.moc.cpp)))
SOURCES=$(C_SOURCES) $(QT_SOURCES) $(OTHER_SOURCES) $(MOC_FILES)
OBJECTS=$(notdir $(C_SOURCES:.c=.o) $(QT_SOURCES:.cpp=.o) $(OTHER_SOURCES:.cpp=.o) $(MOC_FILES:.cpp=.o))

ifeq ($(OSTYPE),windows)
OBJECTS += webots.o
LAUNCHER = ../../msys64/mingw64/bin/webots.exe
LAUNCHERW = ../../msys64/mingw64/bin/webotsw.exe
endif

# prevent 'make' from deleting these intermediate files
.PRECIOUS: $(MOC_FILES)

vpath %.o $(OBJDIR)
vpath %.hpp app compute/cuda control core widgets editor engine external/siphash gui maths nodes nodes/utils ode physics render wren plugins scene_tree sound user_commands vrml
vpath %.cpp app compute/cuda control core widgets editor engine external/siphash gui maths nodes nodes/utils ode physics render wren plugins scene_tree sound user_commands vrml $(OBJDIR)

default:
	@echo "Type one of the following (OSTYPE is \""$(OSTYPE)"\"):"
	@echo "  make debug                 for debug (with gdb symbols)"
	@echo "  make profile               for profiling (with gprof information)"
	@echo "  make release               for release (also bundles Newton runtime; BUNDLE_NEWTON=0 to skip)"
	@echo "  make clean                 remove build directory"
	@echo "  make count-lines           count the lines of code of OmniSim"
	@echo "  make bundle-newton-runtime vendor the Newton runtime (warp/newton) next to the binary (run by release)"

ifneq (,$(filter $(MAKECMDGOALS),debug release profile))
-include $(addprefix $(OBJDIR)/,$(notdir $(SOURCES:.cpp=.d)))
endif

.PHONY: debug profile release clean count-lines

# A `release` build ALSO vendors the Newton runtime next to the binary so a
# stock install resolves physicsBackend "auto" -> Newton instead of a silent
# ODE fallback. Opt out with `make release BUNDLE_NEWTON=0` (e.g. on a build
# host without warp/newton, or to keep the ~600 MB out of a slim build).
# `debug`/`profile` never bundle. See the bundle-newton-runtime target below.
BUNDLE_NEWTON ?= 1

debug profile: $(TARGET) $(LAUNCHER) $(LAUNCHERW)

release: $(TARGET) $(LAUNCHER) $(LAUNCHERW)
ifeq ($(BUNDLE_NEWTON),1)
	@if [ -d "$(TARGET_PATH)/newton-runtime" ]; then \
		echo "# release: Newton runtime already staged at $(TARGET_PATH)/newton-runtime -- skipping bundle (rm -rf it to re-stage)"; \
	else \
		echo "# release: vendoring Newton runtime (BUNDLE_NEWTON=1; set 0 to skip)"; \
		$(MAKE) bundle-newton-runtime; \
	fi
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

$(OBJDIR)/x86_64/webots: $(X86_64_OBJECTS) $(LIB_WREN) $(LIB_ODE)
	@echo "# linking " $@ "(x64_64)"
	@$(CXX) -target x86_64-apple-macos12 $(LD_FLAGS) -o $@ $(X86_64_OBJECTS) $(LIBS)

$(OBJDIR)/arm64/webots: $(ARM64_OBJECTS) $(LIB_WREN) $(LIB_ODE)
	@echo "# linking " $@ "(arm64)"
	@$(CXX) -target arm64-apple-macos12 $(LD_FLAGS) -o $@ $(ARM64_OBJECTS) $(LIBS)

else

$(TARGET): $(OBJECTS) $(LIB_WREN) $(LIB_ODE)
	@echo "# linking " $@
	@$(CXX) $(LD_FLAGS) -o $(TARGET) $(addprefix $(OBJDIR)/, $(OBJECTS)) $(LIBS)
	@$(EXTRA_CMD)
	@echo "# webots compiled in `expr \`date +%s\` - $(START)` seconds"

endif

# ---- Newton runtime bundle (L6 / default-flip-plan.md build layer) ----------
# Vendor the Newton runtime (a self-contained CPython + warp/newton/mujoco-warp/
# usd) next to omnisim-bin.exe so a STOCK install resolves physicsBackend "auto"
# -> Newton instead of silently falling back to ODE. The binary brings the
# interpreter up with a bare Py_InitializeEx(0), so the bundler only has to drop
# a python3XX._pth beside the loaded DLL (standard embeddable-distribution path
# config) -- no C++/link change, which is why this is a post-build packaging
# step rather than a compile flag.
#
# WIRED INTO release: `make release` runs this by default (BUNDLE_NEWTON=1, set
# above) so a stock release ships Newton-capable instead of falling back to ODE.
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
	$(PYTHON_BUNDLER) "$(WEBOTS_PATH)/scripts/packaging/bundle_newton_runtime.py" --target "$(TARGET_PATH)" --mode $(BUNDLE_MODE) --verify

$(OBJDIR)/%.d: %.cpp
	@echo "# updating" $@
	@$(CXX) $(CFLAGS) $(CXXFLAGS) $(INCLUDE) -MM -w $< >> $@

$(OBJDIR)/%.moc.cpp: %.hpp
	@echo "# moc-generating" $@
	@$(MOC) $(MOC_FLAGS) $(INCLUDE:-isystem:-I) $< -o $@

$(OBJDIR)/x86_64/%.moc.cpp: %.hpp
	@echo "# moc-generating" $@ "(x86_64)"
	@$(MOC) -target x86_64-apple-macos-11 $(MOC_FLAGS) $(INCLUDE:-isystem:-I) $< -o $@

$(OBJDIR)/arm64/%.moc.cpp: %.hpp
	@echo "# moc-generating" $@ "(arm64)"
	@$(MOC) -target arm64-apple-macos-11 $(MOC_FLAGS) $(INCLUDE:-isystem:-I) $< -o $@

launcher/launcher.o: launcher/launcher.rc
	@windres -i $< -o $@ --define _WIN32 --define __WIN32__ --define __WIN95__ --define __GNUWIN32__

$(LAUNCHERW): launcher/launcher.c launcher/launcher.o
	@echo "# compiling webotsw.exe launcher"
	@gcc $(CFLAGS) -mwindows -DWEBOTSW launcher/launcher.o launcher/launcher.c -lshlwapi -o $@

$(LAUNCHER): launcher/launcher.c
	@echo "# compiling webots.exe launcher"
	@gcc $(CFLAGS) launcher/launcher.c -lshlwapi -o $@

webots.o:gui/webots.rc gui/omnisim.ico gui/manifest.xml
	@echo "# compiling gui/"$@
	@windres -i $< -o $(OBJDIR)/$@ --define _WIN32 --define __WIN32__ --define __WIN95__ --define __GNUWIN32__

%.o: %.c
	@echo "# compiling" $<
	@$(CC) -c $(CFLAGS) $(CONTROLLER_INCLUDE) $< -o $(OBJDIR)/$@

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
# Note: WbPhysicsBackend.hpp itself stays Qt-free, but WbNewtonBackend.cpp
# uses WbLog (which lives in core/) for the bring-up diagnostic so messages
# land in omnisim_log.txt. -Icore is therefore required, but the public
# header that callers include from elsewhere in the tree still pulls in
# zero Qt symbols.
# $(PYTHON_INCLUDE) is empty when OMNISIM_WITH_NEWTON=OFF so it's harmless;
# in =ON builds it's where Python.h lives for WbNewtonBackend's embedded
# CPython interpreter.
# $(ODE_INCLUDE) is here for WbOdeBackend.cpp (P1.5 widening: the ODE
# backend gained a .cpp once its first method needed to call into ODE).
# WbPhysicsBackend.hpp + WbOdeBackend.hpp still stay ODE-header-free so
# the abstraction's containment of ODE doesn't leak to other TUs.
%.o: physics/%.cpp
	@echo "# compiling" $<
	@$(CXX) -c $(CFLAGS) $(CXXFLAGS) $(QT_CORE_INCLUDE) $(PYTHON_INCLUDE) $(ODE_INCLUDE) -Iphysics -Icore $< -o $(OBJDIR)/$@

# Rendering-backend abstraction (R0 of engine-migration-plan.md).
# Twin of the physics/ rule above. R0 stays Qt-free + WREN-free at the
# header level; the .cpp only depends on WbWrenBackend.hpp (also header-
# only at R0). When R1 introduces WbVulkanBackend.cpp this rule gains
# the Vulkan SDK include path the same way physics/ gained PYTHON_INCLUDE.
%.o: render/%.cpp
	@echo "# compiling" $<
	@$(CXX) -c $(CFLAGS) $(CXXFLAGS) $(QT_CORE_INCLUDE) $(QT_GUI_INCLUDE) $(WREN_INCLUDE) -Irender -Icore $< -o $(OBJDIR)/$@

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

%.o: ode/%.cpp
	@echo "# compiling" $<
	@$(CXX) -c $(CFLAGS) $(CXXFLAGS) $(WB_ODE_INCLUDE) $< -o $(OBJDIR)/$@

%.o: wren/%.cpp
	@echo "# compiling" $<
	@$(CXX) -c $(CFLAGS) $(CXXFLAGS) $(WB_WREN_INCLUDE) $< -o $(OBJDIR)/$@

%.o: util/%.cpp
	@echo "# compiling" $<
	@$(CXX) -c $(CFLAGS) $(CXXFLAGS) $(WB_UTIL_INCLUDE) $< -o $(OBJDIR)/$@

%.o: plugins/%.cpp
	@echo "# compiling" $<
	@$(CXX) -c $(CFLAGS) $(CXXFLAGS) $(WB_PLUGINS_INCLUDE) $< -o $(OBJDIR)/$@

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

$(OBJDIR)/x86_64/%.o: ode/%.cpp
	@echo "# compiling" $< "(x86_64)"
	@$(CXX) -target x86_64-apple-macos12 -c $(CFLAGS) $(CXXFLAGS) $(WB_ODE_INCLUDE) $< -o $@

$(OBJDIR)/x86_64/%.o: wren/%.cpp
	@echo "# compiling" $< "(x86_64)"
	@$(CXX) -target x86_64-apple-macos12 -c $(CFLAGS) $(CXXFLAGS) $(WB_WREN_INCLUDE) $< -o $@

$(OBJDIR)/x86_64/%.o: util/%.cpp
	@echo "# compiling" $< "(x86_64)"
	@$(CXX) -target x86_64-apple-macos12 -c $(CFLAGS) $(CXXFLAGS) $(WB_UTIL_INCLUDE) $< -o $@

$(OBJDIR)/x86_64/%.o: plugins/%.cpp
	@echo "# compiling" $< "(x86_64)"
	@$(CXX) -target x86_64-apple-macos12 -c $(CFLAGS) $(CXXFLAGS) $(WB_PLUGINS_INCLUDE) $< -o $@

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

$(OBJDIR)/arm64/%.o: ode/%.cpp
	@echo "# compiling" $< "(arm64)"
	@$(CXX) -target arm64-apple-macos12 -c $(CFLAGS) $(CXXFLAGS) $(WB_ODE_INCLUDE) $< -o $@

$(OBJDIR)/arm64/%.o: wren/%.cpp
	@echo "# compiling" $< "(arm64)"
	@$(CXX) -target arm64-apple-macos12 -c $(CFLAGS) $(CXXFLAGS) $(WB_WREN_INCLUDE) $< -o $@

$(OBJDIR)/arm64/%.o: util/%.cpp
	@echo "# compiling" $< "(arm64)"
	@$(CXX) -target arm64-apple-macos12 -c $(CFLAGS) $(CXXFLAGS) $(WB_UTIL_INCLUDE) $< -o $@

$(OBJDIR)/arm64/%.o: plugins/%.cpp
	@echo "# compiling" $< "(arm64)"
	@$(CXX) -target arm64-apple-macos12 -c $(CFLAGS) $(CXXFLAGS) $(WB_PLUGINS_INCLUDE) $< -o $@

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
	@rm -rf $(TARGET) $(FILES_TO_REMOVE) $(LAUNCHER) $(LAUNCHERW) build launcher/launcher.o

count-lines:
	wc -l */*.c* */*.h*
