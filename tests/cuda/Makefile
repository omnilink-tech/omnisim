# M0 smoke test for the CUDA compute layer.
# Standalone build — uses nvcc directly; does NOT depend on the rest of the
# simulator's build system. See docs/developer/physics-roadmap.md §2.

NVCC ?= nvcc

# Target SM list per the plan: Turing through Hopper. Pascal/Kepler are not
# supported. Adding sm_75 covers RTX 20-series; sm_86 covers RTX 30-series
# (the dev box this was bootstrapped on).
NVCC_SM_FLAGS = \
  -gencode=arch=compute_75,code=sm_75 \
  -gencode=arch=compute_80,code=sm_80 \
  -gencode=arch=compute_86,code=sm_86 \
  -gencode=arch=compute_89,code=sm_89 \
  -gencode=arch=compute_90,code=sm_90

NVCC_FLAGS = -O2 -std=c++14 $(NVCC_SM_FLAGS)

ifeq ($(OS),Windows_NT)
  EXE = cuda_smoke.exe
else
  EXE = cuda_smoke
endif

.PHONY: all run clean

all: $(EXE)

$(EXE): cuda_smoke.cu
	$(NVCC) $(NVCC_FLAGS) cuda_smoke.cu -o $(EXE)

run: $(EXE)
	./$(EXE)

clean:
	rm -f cuda_smoke cuda_smoke.exe cuda_smoke.exp cuda_smoke.lib
