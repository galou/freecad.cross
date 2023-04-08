# This module should not be imported before the GUI is up otherwise, it'll be
# imported without `tr` and later import attempts will be ignored because this
# is own Python works.
# TODO: solve the import mess.

from __future__ import annotations

from itertools import zip_longest
from pathlib import Path
import string
from typing import Any, Iterable
import xml.etree.ElementTree as et
from xml.dom import minidom

import FreeCAD as fc

from .freecad_utils import is_box
from .freecad_utils import is_cylinder
from .freecad_utils import is_sphere
from .freecad_utils import label_or
from .freecad_utils import warn

# Typing hints.
DO = fc.DocumentObject
DOList = Iterable[DO]
RosLink = DO  # A Ros::Link, i.e. a DocumentObject with Proxy "Link".
RosJoint = DO  # A Ros::Joint, i.e. a DocumentObject with Proxy "Joint".
RosRobot = DO  # A Ros::Robot, i.e. a DocumentObject with Proxy "Robot".

MOD_PATH = Path(fc.getUserAppDataDir()) / 'Mod/freecad.workbench_ros'
RESOURCES_PATH = MOD_PATH / 'resources'
UI_PATH = RESOURCES_PATH / 'ui'
ICON_PATH = RESOURCES_PATH / 'icons'

# Otherwise et.tostring uses xlmns:ns0 as xacro namespace.
et.register_namespace('xacro', 'http://ros.org/wiki/xacro')


def get_valid_filename(text: str) -> str:
    """Return a string that is a valid file name."""
    valids = string.ascii_letters + string.digits + '_-.'
    return ''.join(c if c in valids else '_' for c in text)


def xml_comment(comment: str) -> str:
    """Returns the string without '--'."""
    return f'{comment.replace("--", "⸗⸗")}'


def get_valid_urdf_name(name: str) -> str:
    if not name:
        return 'no_label'
    return name.replace(' ', '_')


def warn_unsupported(objects: [DO, DOList],
                     by: str = '',
                     gui: bool = False,
                     ) -> None:
    """Warn the user of an unsupported object type."""
    if not isinstance(objects, list):
        objects = [objects]
    for o in objects:
        by_txt = f' by {by}' if by else ''
        try:
            label = o.Label
        except AttributeError:
            label = str(o)
        try:
            warn(f'Object "{label}" of type {o.TypeId}'
                 f' not supported{by_txt}\n',
                 gui=gui)
        except AttributeError:
            warn(f'Object "{label}" not supported{by_txt}\n', gui=gui)


def _has_ros_type(obj: DO, type_: str) -> bool:
    """Return True if the object is an object from this workbench."""
    if not isinstance(obj, DO):
        return False
    return hasattr(obj, '_Type') and (obj._Type == type_)


def is_robot(obj: DO) -> bool:
    """Return True if the object is a Ros::Robot."""
    return _has_ros_type(obj, 'Ros::Robot')


def is_link(obj: DO) -> bool:
    """Return True if the object is a Ros::Link."""
    return _has_ros_type(obj, 'Ros::Link')


def is_joint(obj: DO) -> bool:
    """Return True if the object is a Ros::Link."""
    return _has_ros_type(obj, 'Ros::Joint')


def is_xacro(obj: DO) -> bool:
    """Return True if the object is a Ros::Xacro."""
    return _has_ros_type(obj, 'Ros::Xacro')


def is_simple_joint(obj: DO) -> bool:
    """Return True if prismatic, revolute, or continuous."""
    return (is_joint(obj)
            and (obj.Type in ['prismatic', 'revolute', 'continuous']))


def is_primitive(obj: DO) -> bool:
    """Return True if the object is a 'Part::{Box,Cylinder,Sphere}'."""
    return is_box(obj) or is_sphere(obj) or is_cylinder(obj)


def is_robot_selected() -> bool:
    """Return True if the first selected object is a Ros::Object."""
    if not fc.GuiUp:
        return False
    if fc.activeDocument() is None:
        return False
    import FreeCADGui as fcgui
    sel = fcgui.Selection.getSelection()
    if not sel:
        return False
    return is_robot(sel[0])


def get_links(objs: DOList) -> list[RosLink]:
    """Return only the objects that are Ros::Link instances."""
    return [o for o in objs if is_link(o)]


def get_joints(objs: DOList) -> list[RosJoint]:
    """Return only the objects that are Ros::Joint instances."""
    return [o for o in objs if is_joint(o)]


def get_chains(
        links: list[RosLink],
        joints: list[RosJoint],
        ) -> list[DOList]:
    base_links: list[RosLink] = []
    tip_links: list[RosLink] = []
    for link in links:
        if link.Proxy.may_be_base_link():
            base_links.append(link)
        if link.Proxy.is_tip_link():
            tip_links.append(link)
    if len(base_links) > 1:
        # At least two root links found, not supported.
        return []
    chains: list[DOList] = []
    for link in tip_links:
        chain = get_chain(link)
        if chain:
            chains.append(chain)
    return chains


def get_chain(link: RosLink) -> DOList:
    """Return the chain from base_link to link, excluded.

    The chain start with the base link, then alternates a joint and a link. The
    last item is the joint that has `link` as child.

    """
    chain: DOList = []
    ref_joint = link.Proxy.get_ref_joint()
    if not ref_joint:
        # A root link.
        return [link]
    if not ref_joint.Parent:
        warn(f'{label_or(ref_joint)} has no parent', True)
        # Return only ref_joint to indicate an error.
        return [ref_joint]
    subchain = get_chain(ref_joint.Parent)
    if subchain and is_joint(subchain[0]):
        # Propagate the error of missing joint.Parent.
        return subchain
    chain += subchain + [ref_joint] + [link]
    return chain


def is_subchain(subchain: DOList, chain: DOList) -> bool:
    """Return True if all items in `subchain` are in `chain`."""
    for link_or_joint in subchain:
        if link_or_joint not in chain:
            return False
    return True


def hasallattr(obj: Any, attrs: list[str]):
    """Return True if object has all attributes."""
    if isinstance(attrs, str):
        # Developer help, call error.
        warn(f'hasallattr({attrs}) was replaced by hasallattr([{attrs}])')
        attrs = [attrs]
    for attr in attrs:
        if not hasattr(obj, attr):
            return False
    return True


def save_xml(
        xml: et.ElementTree,
        filename: [Path | str],
        ) -> None:
    """Save the xml element into a file."""
    file_path = Path(filename)
    file_path.parent.mkdir(parents=True, exist_ok=True)
    txt = minidom.parseString(et.tostring(xml)).toprettyxml(indent='  ')
    file_path.write_text(txt)


def grouper(iterable, n, fillvalue=None):
    """Collect data into fixed-length chunks or blocks."""
    # grouper('ABCDEFG', 3, 'x') --> ABC DEF Gxx"
    # From https://docs.python.org/3.8/library/itertools.html.
    args = [iter(iterable)] * n
    return zip_longest(*args, fillvalue=fillvalue)
