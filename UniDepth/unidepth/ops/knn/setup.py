import glob
import os
import types
import torch
from setuptools import find_packages, setup
from torch.utils.cpp_extension import CUDA_HOME, CppExtension, CUDAExtension

# Patch the CUDA version check function to avoid the version mismatch error
original_check_cuda_version = torch.utils.cpp_extension._check_cuda_version


# Define a function that does nothing
def _patched_check_cuda_version(*args, **kwargs):
    pass


# Replace the original function with our patched version
torch.utils.cpp_extension._check_cuda_version = _patched_check_cuda_version

requirements = ["torch", "torchvision"]


def get_extensions():
    this_dir = os.path.dirname(os.path.abspath(__file__))
    extensions_dir = os.path.join(this_dir, "src")

    main_file = glob.glob(os.path.join(extensions_dir, "*.cpp"))
    source_cpu = glob.glob(os.path.join(extensions_dir, "*.cpp"))
    source_cuda = glob.glob(os.path.join(extensions_dir, "*.cu"))

    sources = main_file + source_cpu
    extension = CppExtension
    extra_compile_args = {"cxx": ["-O3", "-std=c++17"]}
    define_macros = []

    if torch.cuda.is_available() and CUDA_HOME is not None:
        extension = CUDAExtension
        sources += source_cuda
        define_macros += [("WITH_CUDA", None)]
        extra_compile_args["nvcc"] = [
            "-O3",
            "-std=c++17",  # Keep C++17 for PyTorch compatibility
            "-Xcompiler",
            "-fPIC",
            "--expt-relaxed-constexpr",  # Help with template instantiation
            "--expt-extended-lambda",  # Help with complex lambdas
            "--use_fast_math",  # Optimize math operations
            "--relocatable-device-code=true",  # Help with linking
            "--generate-line-info",  # For debugging
            "-DCUDA_HAS_FP16=1",  # Enable FP16 support
            "--maxrregcount=64",  # Limit register usage for better occupancy
        ]
    else:
        raise NotImplementedError("Cuda is not available")

    sources = list(set([os.path.join(extensions_dir, s) for s in sources]))
    include_dirs = [extensions_dir]
    ext_modules = [
        extension(
            "KNN",
            sources,
            include_dirs=include_dirs,
            define_macros=define_macros,
            extra_compile_args=extra_compile_args,
        )
    ]

    return ext_modules


setup(
    name="KNN",
    version="0.1",
    author="Luigi Piccinelli",
    ext_modules=get_extensions(),
    packages=find_packages(
        exclude=(
            "configs",
            "tests",
        )
    ),
    cmdclass={"build_ext": torch.utils.cpp_extension.BuildExtension},
)
