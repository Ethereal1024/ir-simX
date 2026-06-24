import importlib.metadata

try:
    __version__ = importlib.metadata.version("ir-simX")
except importlib.metadata.PackageNotFoundError:
    try:
        __version__ = importlib.metadata.version("ir-sim")
    except importlib.metadata.PackageNotFoundError:
        __version__ = "0.0.0"
