import os
import sys
from collections.abc import Callable
from typing import Any, Optional

# ── C++ core guard: irsim-x requires the compiled extension ──
try:
    from cpp._core import SimWorld as _SimWorld
except ImportError:
    raise ImportError(
        "irsim-x requires the compiled C++ core.\n"
        "Build it with:  pip install -e .\n"
        "To install from PyPI:  pip install irsim-x\n"
        "If you have the original ir-sim installed, uninstall it first:\n"
        "  pip uninstall ir-sim\n"
        "  pip install irsim-x"
    )

from irsim.env import EnvBase, EnvBase3D

try:
    from irsim.env.batch_env_base import BatchEnvBase
    _HAS_BATCH = True
except ImportError:
    _HAS_BATCH = False

from .version import __version__


class _EnvFactory:
    """
    Internal object-oriented factory that creates IR-SIM environments.

    Create an environment by the given world file and projection.

    Env candidates:
        - EnvBase (2D default)
        - EnvBase3D (3D)
    """

    def __init__(
        self, default_projection: str | None = None, **default_kwargs: Any
    ) -> None:
        self.default_projection = default_projection
        self.default_kwargs = default_kwargs

        self._registry: dict[str, Callable[..., EnvBase]] = {
            "2d": EnvBase,
            "3d": EnvBase3D,
        }

    def _resolve_world_name(self, world_name: str | None) -> str:
        return world_name or os.path.basename(sys.argv[0]).split(".")[0] + ".yaml"

    def register(self, key: str, ctor: Callable[..., EnvBase]) -> None:
        self._registry[key.strip().lower()] = ctor

    def create(
        self,
        world_name: str | None = None,
        projection: str | None = None,
        batch_size: int = 1,
        share_obstacles: bool = True,
        **kwargs: Any,
    ) -> EnvBase:
        resolved_world = self._resolve_world_name(world_name)

        if batch_size > 1:
            if not _HAS_BATCH:
                raise ImportError(
                    "BatchEnvBase requires compiled C++ core with BatchSimWorld.\n"
                    "Run: pip install -e ."
                )
            return BatchEnvBase(
                resolved_world,
                batch_size=batch_size,
                share_obstacles=share_obstacles,
                **kwargs,
            )

        options: dict[str, Any] = {**self.default_kwargs, **kwargs}
        key = (projection or self.default_projection or "2d").strip().lower()
        try:
            ctor = self._registry[key]
        except KeyError as e:
            raise ValueError(
                f"Unknown projection {projection!r}. Allowed: {', '.join(self._registry)}"
            ) from e
        return ctor(resolved_world, **options)


_env_factory = _EnvFactory()


def make(
    world_name: str | None = None,
    projection: str | None = None,
    batch_size: int = 1,
    share_obstacles: bool = True,
    **kwargs: Any,
) -> EnvBase:
    """
    Create an environment by the given world file and projection.

    This function serves as the main entry point for creating simulation environments.
    It automatically selects between 2D and 3D environments based on the projection parameter.

    When ``batch_size > 1``, returns a :py:class:`.BatchEnvBase` that wraps
    N identical environments with a shared C++ SIMD backend.

    Args:
        world_name (str, optional): The name of the world YAML configuration file.
            If not specified, the default name of the current Python script with
            '.yaml' extension will be used.
        projection (str, optional): The projection type of the environment.
            Default is None for 2D environment. If set to "3d", creates a 3D
            plot environment.
        batch_size (int, optional): Number of parallel environments (default 1).
            When >1, the C++ BatchSimWorld backend is used with SoA layout
            and SIMD acceleration across environments.
        share_obstacles (bool, optional): If True (default), all environments
            share the same obstacle set, enabling cross-environment SIMD LiDAR.
            If False, each environment has its own obstacle set (mode B).
        **kwargs: Additional keyword arguments passed to :py:class:`.EnvBase`
            or :py:class:`.EnvBase3D`. Common options include:

            - display (bool): Whether to display the environment visualization
            - save_ani (bool): Whether to save animation
            - log_level (str): Logging level for the environment
            - seed (int, optional): Seed for IR-SIM's random number generator
              to make runs reproducible when using IR-SIM randomness.

    Returns:
        EnvBase: The created environment object. Returns :py:class:`.BatchEnvBase`
        if ``batch_size > 1``, :py:class:`.EnvBase3D` if projection is "3d",
        otherwise :py:class:`.EnvBase`.

    Example:
        >>> # Create a 2D environment with default world file
        >>> env = make()
        >>>
        >>> # Create a batch of 16 environments (shared obstacles, SIMD LiDAR)
        >>> env = make("world.yaml", batch_size=16)
        >>>
        >>> # Batch with per-environment obstacles (mode B, scalar LiDAR fallback)
        >>> env = make("world.yaml", batch_size=16, share_obstacles=False)
    """
    return _env_factory.create(
        world_name=world_name,
        projection=projection,
        batch_size=batch_size,
        share_obstacles=share_obstacles,
        **kwargs,
    )
