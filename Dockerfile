# SushiEngine dev image.
#
# The engine is the head of the stack; SushiRuntime is the plugged-in battery
# that supplies the SYCL toolchain and the add_sycl_to_target() command. So this
# image provisions the same intel/llvm clang++ -fsycl toolchain the runtime is
# built and tested against, then clones the runtime as a SIBLING checkout next to
# the engine (the layout SushiEngine's CMake and `se` CLI expect by default:
# ../sushiruntime). Build on demand inside the container with `se project build`.
#
# Base: CUDA runtime (GTX 1060 = sm_61, Pascal), matching the runtime image.
FROM nvidia/cuda:12.4.1-devel-ubuntu22.04

# GCC 13 for a recent C++17 stdlib + base tools (any GCC >= 9 suffices; 13 is a
# known-good libstdc++ for the clang -fsycl host pass).
RUN apt-get update && apt-get install -y software-properties-common && \
    add-apt-repository -y ppa:ubuntu-toolchain-r/test && \
    apt-get update && apt-get install -y \
    python3 python3-pip build-essential ninja-build git \
    pkg-config libhwloc-dev libgtest-dev \
    dos2unix wget \
    gcc-13 g++-13 ocl-icd-opencl-dev ocl-icd-libopencl1 \
    && rm -rf /var/lib/apt/lists/*

# CMake 3.31 — Ubuntu 22.04 apt ships 3.22; pull a current CMake from Kitware.
RUN apt-get update && apt-get install -y curl ca-certificates gpg && \
    curl -fsSL https://apt.kitware.com/keys/kitware-archive-latest.asc \
        | gpg --dearmor -o /usr/share/keyrings/kitware-archive-keyring.gpg && \
    echo 'deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ jammy main' \
        > /etc/apt/sources.list.d/kitware.list && \
    apt-get update && apt-get install -y cmake && \
    rm -rf /var/lib/apt/lists/*

# Intel OpenCL CPU runtime — exposes the host CPU as a SYCL device.
RUN apt-get update && apt-get install -y curl ca-certificates && \
    curl -fsSL https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB \
        | gpg --dearmor > /usr/share/keyrings/oneapi-archive-keyring.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/oneapi-archive-keyring.gpg] https://apt.repos.intel.com/oneapi all main" \
        > /etc/apt/sources.list.d/oneAPI.list && \
    apt-get update && apt-get install -y intel-oneapi-runtime-opencl && \
    rm -rf /var/lib/apt/lists/*

# intel/llvm nightly SYCL bundle — the pre-built, vendor-neutral clang++ -fsycl
# toolchain (CUDA + OpenCL backends). This is the engine's primary, tested path;
# the CMake default picks `clang++` off PATH, which this provides.
ARG INTEL_LLVM_DATE=2026-06-12
RUN wget -q \
    https://github.com/intel/llvm/releases/download/nightly-${INTEL_LLVM_DATE}/sycl_linux.tar.gz \
    -O /tmp/sycl_linux.tar.gz && \
    mkdir -p /opt/llvm-sycl && \
    tar -xzf /tmp/sycl_linux.tar.gz -C /opt/llvm-sycl --strip-components=1 && \
    rm /tmp/sycl_linux.tar.gz

ENV PATH="/opt/llvm-sycl/bin:$PATH"
ENV LD_LIBRARY_PATH="/opt/llvm-sycl/lib:$LD_LIBRARY_PATH"

# Keep GPU kernel launches ASYNC. CUDA_LAUNCH_BLOCKING=1 serialises every launch,
# which makes async/overlap behaviour look blocking. Pin it off; override at
# `docker run` only when deliberately debugging a kernel.
ENV CUDA_LAUNCH_BLOCKING=0

WORKDIR /workspace

# Clone the SushiRuntime sibling the engine builds against. Pinned to a ref via
# --build-arg SUSHIRUNTIME_REF=<branch|tag|sha> (default: main). Lives at
# /workspace/sushiruntime so the engine's default ../sushiruntime resolves to it.
ARG SUSHIRUNTIME_REF=main
RUN git clone --depth 1 --branch ${SUSHIRUNTIME_REF} \
        https://github.com/SushiSystems/SushiRuntime.git /workspace/sushiruntime

# The engine source. Copied last so a source change does not bust the toolchain
# or runtime-clone layers above.
COPY . /workspace/sushiengine/
WORKDIR /workspace/sushiengine

# Normalize line endings on any shell helpers carried in from a Windows checkout.
RUN find . -type f -name "*.sh" -exec sed -i 's/\r$//' {} + 2>/dev/null || true

# Install the `se` CLI system-wide (the container is already isolated, so no venv
# is needed). Pin /usr/bin/python3: the intel/llvm bundle prepends its own bin to
# PATH, and python3-pip belongs to the system python — install and run under the
# same interpreter. Verify the import here so a bad build context fails loudly.
RUN /usr/bin/python3 -m pip install --no-cache-dir --upgrade pip setuptools wheel && \
    /usr/bin/python3 -m pip install --no-cache-dir /workspace/sushiengine/cli && \
    /usr/bin/python3 -c "import sushiengine.cli; print('se CLI installed OK')" && \
    printf "alias se='/usr/bin/python3 -m sushiengine'\nalias sushiengine='/usr/bin/python3 -m sushiengine'\n" >> ~/.bashrc

# The image is a ready build/run *environment*, not a pre-built artifact: the
# clang++ -fsycl toolchain, the runtime sibling, the deps and the `se` CLI are all
# installed. Build on demand inside the container:
#   se project build
#   se project test
#
CMD ["/bin/bash"]
