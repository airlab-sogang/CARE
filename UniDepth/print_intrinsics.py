import numpy as np


def print_intrinsics(file_path):
    # Load the intrinsics from the .npy file
    intrinsics = np.load(file_path)

    # Print the intrinsics
    print("Intrinsics:")
    print(intrinsics)


if __name__ == "__main__":
    # Path to the intrinsics.npy file
    file_path = "assets/demo/intrinsics.npy"

    # Print the intrinsics
    print_intrinsics(file_path)
