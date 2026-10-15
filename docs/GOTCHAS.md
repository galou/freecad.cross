# Gotchas

## Geometry of the elements of `CrossLink.Group`

- The elements of `CrossLink.Group` are `App::Link` objects with `LinkTransform` false: the placement of the linked
  object is ignored, only `LinkPlacement` counts.
- In a robot, `link.Placement` already includes `link.MountedPlacement` (set by `RobotProxy.compute_poses()`) and
  `LinkPlacement` is `link.Placement`. Outside a robot, `LinkPlacement` is `link.Placement * link.MountedPlacement`.
  The URDF frame of a link in a robot, used by IK and `Cross::Pose`, is `link.Placement * MountedPlacement⁻¹`.
- With robots imported from URDF, the linked objects are `App::Part` containers holding links to meshes.
  `Part.getShape()` returns a null shape for meshes, also when they are nested in containers.
- In FreeCAD 1.x, `App::Part` has a `Shape` attribute: use the type (`Part::Feature`, `Mesh::Feature`), not
  `hasattr(obj, 'Shape')`, to tell geometry from containers.
- `Shape.writeInventor()` and `Mesh.writeInventor()` bake the shape or mesh placement into their output. Adding the
  object's `Placement` as an extra `SoTransform` applies it twice.
- To walk the displayed geometry generically, use `fc_link.getSubObjects()` and
  `fc_link.getSubObject(subname, retType=2, transform=False)` (matrix in the frame of `fc_link`, including the
  placement of the sub-object), then `obj.getLinkedObject(True, Matrix(), False)` (adds the placements brought in
  by `LinkTransform`, not the one of `obj`).

## fpo view proxies

- Display-mode builders (`@fpo.DisplayMode(...).builder`) are only called in `attach()`. `Proxy.create()` already
  attaches a view proxy: instantiating the view proxy again (`ViewProxy(obj.ViewObject)`) replaces it with an
  instance that was never attached.
- Draw into the display-mode nodes, not into `vobj.RootNode`: clearing `RootNode` removes the display-mode switch
  (all modes shown at once) and the transform of `Placement` for `App::GeometryPython` objects. With that transform
  kept, do not apply `obj.Placement` again.
