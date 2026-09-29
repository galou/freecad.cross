from __future__ import annotations

from typing import NewType
from typing import Optional
from typing import TYPE_CHECKING

import FreeCAD as fc

from pivy import coin

from .coin_utils import transform_from_placement
from .freecad_utils import error
from .freecad_utils import warn
from .wb_utils import ICON_PATH
from .wb_utils import is_link
from freecad.cross.vendor.fcapi import fpo  # Cf. https://github.com/mnesarco/fcapi

# Typing hints.
from .rgb_camera import RgbCamera as CrossRgbCamera  # A Cross::RgbCamera, i.e. a DocumentObject with Proxy "RgbCamera". # noqa: E501

if TYPE_CHECKING and hasattr(fc, 'GuiUp') and fc.GuiUp:
    from FreeCADGui import ViewProviderDocumentObject as VPDO
else:
    VPDO = NewType('VPDO', object)

# Width in pixels of the images rendered from the camera, the height is
# deduced from the field of view.
IMAGE_WIDTH = 640


@fpo.view_proxy(
        icon=str(ICON_PATH / 'camera-photo-symbolic.svg'),
)
class RgbCameraViewProxy:
    frustum_display_mode = fpo.DisplayMode(name='Frustum', is_default=True)

    frustum_length = fpo.PropertyLength(
            name='FrustumLength',
            section='Display Options',
            default=5000,
            description=(
                'Length of the frustum representation (relative to the camera center)'
            ),
    )

    frustum_start = fpo.PropertyLength(
            name='FrustumStart',
            section='Display Options',
            default=0,
            description=(
                'Start of the frustum representation'
            ),
    )

    color = fpo.PropertyColor(
            name='Color',
            section='Display Options',
            default=(0.0, 0.5, 0.0),
            description=(
                'Color of the frustum representation'
            ),
    )

    transparency = fpo.PropertyIntegerConstraint(
            name='Transparency',
            section='Display Options',
            default=0,
            description=(
                'Transparency of the frustum representation'
            ),
    )

    def on_start(self) -> None:
        # Set the transparency to a range of 0-100 with a step of 1.
        # Implementation note: getter is an int, setter can be (val, min, max, step).
        self.transparency = (self.transparency, 0, 100, 1)

    def on_change(self) -> None:
        self._redraw()

    def on_object_change(self) -> None:
        self._redraw()

    def on_context_menu(self, event: fpo.events.ContextMenuEvent) -> None:
        event.menu.addAction('Save Camera Image...', self.save_image_dialog)

    def save_image_dialog(self) -> None:
        # Import late to avoid slowing down workbench start-up.
        import FreeCADGui as fcgui
        from PySide.QtWidgets import QFileDialog  # FreeCAD's PySide

        filename, _ = QFileDialog.getSaveFileName(
            fcgui.getMainWindow(),
            'Save the image seen by the camera',
            f'{self.Object.Label}.png',
            'Images (*.png *.jpg *.bmp);;All files (*.*)',
        )
        if not filename:
            return
        self.save_image(filename)

    def save_image(self, filename: str, width: int = IMAGE_WIDTH) -> bool:
        """Render the scene seen by the camera and save it to `filename`.

        The height of the image is deduced from `width` and the fields of
        view.

        """
        image = self.render_image(width)
        if image is None:
            return False
        if not image.save(filename):
            error(f'Cannot save the image to "{filename}"', True)
            return False
        return True

    def render_image(self, width: int = IMAGE_WIDTH):
        """Render the scene seen by the camera and return it as QImage.

        The scene is the one displayed in the 3D view, without the frustum of
        this camera.
        The camera looks along its z-axis, with the image's x-axis along the
        camera's x-axis and the image's y-axis along the camera's y-axis
        (i.e. ROS' optical frame convention).

        Return None on failure.

        """
        import math

        from PySide.QtGui import QImage  # FreeCAD's PySide

        obj = self.Object
        views = self.ViewObject.Document.mdiViewsOfType('Gui::View3DInventor')
        if not views:
            error('No 3D view to render the camera image from', True)
            return None
        scene_graph = views[0].getSceneGraph()

        tan_half_hfov = math.tan(obj.HFov.getValueAs('rad').Value / 2.0)
        tan_half_vfov = math.tan(obj.VFov.getValueAs('rad').Value / 2.0)
        if tan_half_hfov <= 0.0 or tan_half_vfov <= 0.0:
            error('The fields of view of the camera must be in ]0, 180[ deg', True)
            return None
        height = max(1, round(width * tan_half_vfov / tan_half_hfov))

        # Coin cameras look along their -z-axis with the image's up along
        # their y-axis.
        rotation = obj.Placement.Rotation * fc.Rotation(fc.Vector(1, 0, 0), 180)
        coin_rotation = coin.SbRotation(*rotation.Q)
        camera = coin.SoPerspectiveCamera()
        camera.position.setValue(coin.SbVec3f(*obj.Placement.Base))
        camera.orientation.setValue(coin_rotation)
        # Leave the camera alone, i.e. don't let Coin adapt the field of
        # view to the viewport.
        camera.viewportMapping.setValue(coin.SoCamera.LEAVE_ALONE)
        camera.aspectRatio.setValue(tan_half_hfov / tan_half_vfov)
        camera.heightAngle.setValue(2.0 * math.atan(tan_half_vfov))

        # A headlight.
        light_sep = coin.SoTransformSeparator()
        light_rotation = coin.SoRotation()
        light_rotation.rotation.setValue(coin_rotation)
        light_sep.addChild(light_rotation)
        light_sep.addChild(coin.SoDirectionalLight())

        root = coin.SoSeparator()
        root.ref()
        root.addChild(camera)
        root.addChild(light_sep)
        root.addChild(scene_graph)

        viewport = coin.SbViewportRegion(width, height)
        renderer = coin.SoOffscreenRenderer(viewport)
        renderer.setComponents(coin.SoOffscreenRenderer.RGB)
        renderer.setBackgroundColor(coin.SbColor(*_get_background_color()))

        # Hide the frustum, which would occlude the view.
        self.ViewObject.RootNode.removeAllChildren()
        try:
            # Clip the scene between 1 mm and its farthest point.
            camera.nearDistance.setValue(1.0)
            camera.farDistance.setValue(_get_far_distance(root, viewport, camera))
            ok = renderer.render(root)
        finally:
            self._redraw()
            root.unref()
        if not ok:
            error('Rendering of the camera image failed', True)
            return None

        # `copy()` because the QImage doesn't own the buffer.
        image = QImage(
            renderer.getBuffer(), width, height, width * 3,
            QImage.Format_RGB888,
        ).copy()
        # OpenGL's origin is at the bottom left.
        return image.mirrored(False, True)

    def _redraw(self) -> None:
        """Draw the frustum."""
        import math
        obj = self.Object
        view = self.ViewObject
        if not hasattr(view, 'RootNode'):
            return
        root = view.RootNode
        root.removeAllChildren()

        if not view.Visibility:
            return

        sep = coin.SoSeparator()

        # Add a material node.
        material = coin.SoMaterial()
        material.diffuseColor = self.color[:3]
        if self.transparency is not None:
            # Implementation note: self.on_change is called before
            # self.on_start.
            # TODO: Fix this in fcapi.
            material.transparency = self.transparency / 100.0
        sep.addChild(material)

        # Add a transform node.
        transform = transform_from_placement(self.Object.Placement)
        sep.addChild(transform)

        # Represent the frustum as a truncated pyramid because
        # `SoFrustumCamera` is not supported by the coin3D version used in
        # FreeCAD.
        # Calculate the vertices of the truncated pyramid
        x_angle = math.radians(obj.HFov / 2)
        y_angle = math.radians(obj.VFov / 2)
        start = max(0.0, self.frustum_start)
        height = max(start, self.frustum_length)
        start_half_width = start * math.tan(x_angle)
        start_half_height = start * math.tan(y_angle)
        base_half_width = height * math.tan(x_angle)
        base_half_height = height * math.tan(y_angle)

        vertices = [
            ( start_half_width,  start_half_height, start),  # Vertices near summit.
            (-start_half_width,  start_half_height, start),
            (-start_half_width, -start_half_height, start),
            ( start_half_width, -start_half_height, start),
            ( base_half_width,  base_half_height, height),  # Vertices far.
            (-base_half_width,  base_half_height, height),
            (-base_half_width, -base_half_height, height),
            ( base_half_width, -base_half_height, height),
        ]

        # Create the faces of the frustum
        coord = coin.SoCoordinate3()
        coord.point.setValues(0, len(vertices), vertices)

        # Define the indices for the faces
        indices = [
            [0, 1, 2, 3, -1],  # Face near summit
            [0, 1, 5, 4, -1],  # Face 1
            [1, 2, 6, 5, -1],  # Face 2
            [2, 3, 7, 6, -1],  # Face 3
            [3, 0, 4, 7, -1],  # Face 4
            [4, 5, 6, 7, -1],  # Far face
        ]

        face_set = coin.SoIndexedFaceSet()
        face_set.coordIndex.setValues(0, len(indices) * 5, [i for face in indices for i in face])

        sep.addChild(coord)
        sep.addChild(face_set)
        root.addChild(sep)


@fpo.proxy(
    object_type='App::FeaturePython',
    subtype='Cross::RgbCamera',
    view_proxy=RgbCameraViewProxy,
)
class RgbCameraProxy:

    link = fpo.PropertyLink(
            name='Link',
            section='Elements',
            description='Cross::Link to attach to',
    )

    placement = fpo.PropertyPlacement(
            name='Placement',
            section='Internal',
            mode=fpo.PropertyMode.ReadOnly,
            description='Placement of the sensor in the robot frame',
    )

    hfov = fpo.PropertyAngle(
            name='HFov',
            section='Sensor Options',
            default=70,
            description='Horizontal field of view of the camera',
    )

    vfov = fpo.PropertyAngle(
            name='VFov',
            section='Sensor Options',
            default=70,
            description='Vertical field of view of the camera',
    )


def _get_background_color() -> tuple[float, float, float]:
    """Return the simple background color of FreeCAD's 3D views."""
    param = fc.ParamGet('User parameter:BaseApp/Preferences/View')
    # Packed as 0xRRGGBBAA, default is FreeCAD's.
    rgba = param.GetUnsigned('BackgroundColor', 0x333333ff)
    return (
        ((rgba >> 24) & 0xff) / 255.0,
        ((rgba >> 16) & 0xff) / 255.0,
        ((rgba >> 8) & 0xff) / 255.0,
    )


def _get_far_distance(
        root: coin.SoNode,
        viewport: coin.SbViewportRegion,
        camera: coin.SoCamera,
) -> float:
    """Return a far clipping distance that includes the whole scene."""
    default_distance = 1e6  # mm.
    action = coin.SoGetBoundingBoxAction(viewport)
    action.apply(root)
    bbox = action.getBoundingBox()
    if bbox.isEmpty():
        return default_distance
    position = camera.position.getValue()
    corners = [
        coin.SbVec3f(x, y, z)
        for x in (bbox.getMin()[0], bbox.getMax()[0])
        for y in (bbox.getMin()[1], bbox.getMax()[1])
        for z in (bbox.getMin()[2], bbox.getMax()[2])
    ]
    distance = max((c - position).length() for c in corners)
    # Margin to avoid clipping the farthest point.
    return max(distance * 1.01, 10.0)


def make_rgb_camera(
        name: str,
        doc: Optional[fc.Document] = None,
) -> CrossRgbCamera:
    """Add a CROSS::::RgbCamera to the current document."""
    if doc is None:
        doc = fc.activeDocument()
    if doc is None:
        warn('No active document, doing nothing', False)
        return
    obj: CrossRgbCamera = RgbCameraProxy.create(name=name, doc=doc)

    if hasattr(fc, 'GuiUp') and fc.GuiUp:
        import FreeCADGui as fcgui

        # Make `obj` part of the selected `Cross::Robot`.
        sel = fcgui.Selection.getSelection()
        if sel:
            candidate = sel[0]
            if is_link(candidate):
                obj.Link = candidate
                try:
                    obj.Link.Proxy.get_robot().addObject(obj)
                except AttributeError:
                    pass

    return obj
