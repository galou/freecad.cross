# Gotchas

## Geometry of the elements of `CrossLink.Group`

- The elements of `CrossLink.Group` are `App::Link` objects with `LinkTransform` false: the placement of the linked
  object is ignored, only `LinkPlacement` (`link.Placement * link.MountedPlacement`) counts.
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
