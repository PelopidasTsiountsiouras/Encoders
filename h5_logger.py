import functools

import h5py
import numpy as np


class H5Logger:
    """
    Context manager for logging scalar or array-like values into an HDF5 file.

    It can be used like this:
    with H5Logger("data.h5", dset="measurements") as h5:
        h5.append(3.14)
        h5.append([1.0, 2.0, 3.0])

    Alternatively, use the `log_h5` decorator to log function return values easier.
    Example:
    @log_h5("data.h5", dset="results")
    def compute_value(x):
        return x ** 2
    """

    def __init__(self, path, dset="values", dtype=np.float64):
        self.path = path
        self.dset = dset
        self.dtype = dtype
        self.file = None

    def __enter__(self):
        # "a" = create if doesn't exist, else open for read/write
        self.file = h5py.File(self.path, "a")

        # If dataset doesn't exist → create dynamically on first append
        if self.dset not in self.file:
            # Create an empty dataset with unlimited growth
            self.file.create_dataset(
                self.dset, shape=(0,), maxshape=(None,), dtype=self.dtype
            )

        return self

    def __exit__(self, exc_type, exc_value, traceback):
        if self.file is not None:
            self.file.close()

    def append(self, value):
        """
        Appends a scalar or array-like value.
        If `value` is a vector, dtype must match (e.g., dtype=float with shape=3).
        """
        dset = self.file[self.dset]

        value = np.asarray(value, dtype=self.dtype)

        # If first write: adjust dataset shape for vector data
        if dset.shape[0] == 0 and value.ndim > 0:
            dset.resize((0,) + value.shape)
            # Now dataset shape is (N, *value.shape)

        # Resize to add 1 record
        dset.resize((dset.shape[0] + 1,) + dset.shape[1:])
        dset[-1] = value


def log_h5(path, dset="values", dtype=np.float64):
    """
    Decorator that logs the RETURN VALUE of the wrapped function
    into an HDF5 file on each call.
    """

    def decorator(func):
        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            result = func(*args, **kwargs)

            # Append result to HDF5
            with H5Logger(path, dset=dset, dtype=dtype) as logger:
                logger.append(result)

            return result

        return wrapper

    return decorator