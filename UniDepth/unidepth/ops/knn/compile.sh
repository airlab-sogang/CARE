#!/usr/bin/env bash

# Limit template instantiation by targeting only your specific GPU
export TORCH_CUDA_ARCH_LIST="8.6"

# Force PyTorch to use the system CUDA version
export CUDA_HOME=$CONDA_PREFIX
export TORCH_CUDA_VERSION="11.8"

# Force CUDA to use the system CUDA compiler
export PATH=$CONDA_PREFIX/bin:$PATH

# Add more compiler memory
export PYTORCH_CUDA_ALLOC_CONF=max_split_size_mb:512

# Potentially reduce template instantiation depth
export CUDA_LAUNCH_BLOCKING=1

# Enable verbose output for debugging
#export VERBOSE=1

python setup.py build install