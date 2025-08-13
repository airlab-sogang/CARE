#!/usr/bin/env bash

# if [ -z "$TORCH_CUDA_ARCH_LIST" ]; then
#     export TORCH_CUDA_ARCH_LIST="7.5 8.0 8.6+PTX"
# fi

# python setup.py build install
# Target only your GPU architecture
export TORCH_CUDA_ARCH_LIST="8.6"

# Force PyTorch to use the system CUDA version
export CUDA_HOME=$CONDA_PREFIX
export TORCH_CUDA_VERSION="12.8"

# Additional compilation settings
export PYTORCH_CUDA_ALLOC_CONF=max_split_size_mb:512
export CUDA_LAUNCH_BLOCKING=1

python setup.py build install