# core/surface

De-globalized port of the original `Surface` class. Represents one walkable surface: its 3D/2D vertex loops, plane, centroid, and the world<->surface-frame transforms, shrunk by half the foot dimensions so a footstep placed on the shrunk boundary still has the full foot resting on the original surface.

## API

See [`include/nas/core/surface.hpp`](include/nas/core/surface.hpp). One class, `Surface(points, surface_idx, foot_length, foot_width)`. Foot dimensions are constructor parameters rather than globals read from `constants.hpp` — this is the module that will eventually take them from `RobotModel` (phase 9).

## Dependencies

`core/geometry`, CGAL.

## Testing standalone

```sh
mkdir build && cd build && cmake .. && cmake --build . -j2 && ctest
```
