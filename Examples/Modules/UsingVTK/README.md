# Using VTK example

This example shows how using VTK doesn't require the use of the module system.

## Borehole widget demo

This folder also contains `vtk_borehole_widget_demo`, an interactive demo for
`vtkBoreholeWidget` / `vtkBoreholeRepresentation`.

After configuring and building this example project, run:

```sh
./vtk_borehole_widget_demo
```

The demo opens a render window where you can:

- hover and drag top/bottom caps to move the interval,
- hover and drag the wall to change the tube radius.
- drag endpoint glyphs (spheres) to move interval ends along the trajectory.
- observe cap orientation adapting to interpolated geological structural surfaces
  (configured from 3 synthetic horizons in the demo) using finite-difference
  sampling controlled by `SurfaceDx` / `SurfaceDy`.
- glyph size is independent from tube diameter and configured via `GlyphRadius`.
