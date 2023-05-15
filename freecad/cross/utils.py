# This module should not be imported before the GUI is up otherwise, it'll be
# imported without `tr` and later import attempts will be ignored because this
# is own Python works.
# TODO: solve the import mess.

from __future__ import annotations

from itertools import zip_longest
from pathlib import Path
import string
from typing import Any, Iterable, Optional
import xml.etree.ElementTree as et
from xml.dom import minidom

import FreeCAD as fc

# Typing hints.
DO = fc.DocumentObject
DOList = Iterable[DO]
RosLink = DO  # A Ros::Link, i.e. a DocumentObject with Proxy "Link".
RosJoint = DO  # A Ros::Joint, i.e. a DocumentObject with Proxy "Joint".
RosRobot = DO  # A Ros::Robot, i.e. a DocumentObject with Proxy "Robot".
RosXacroObject = DO  # Ros::XacroObject, i.e. DocumentObject with Proxy "XacroObject".

# Otherwise et.tostring uses xlmns:ns0 as xacro namespace.
et.register_namespace('xacro', 'http://ros.org/wiki/xacro')


def get_valid_filename(text: str) -> str:
    """Return a string that is a valid file name."""
    valids = string.ascii_letters + string.digits + '_-.'
    return ''.join(c if c in valids else '_' for c in text)


def xml_comment(comment: str) -> str:
    """Returns the string without '--'."""
    return f'{comment.replace("--", "⸗⸗")}'


def warn_unsupported(objects: [DO, DOList],
                     by: str = '',
                     gui: bool = False,
                     ) -> None:
    """Warn the user of an unsupported object type."""
    # Import here otherwise not fc.GuiUp.
    from .freecad_utils import warn

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


def attr_equals(instance: Any, attr: str, value: Any):
    return hasattr(instance, attr) and getattr(instance, attr) == value


def attr_is(instance: Any, attr: str, value: Any):
    return hasattr(instance, attr) and getattr(instance, attr) is value


def hasallattr(obj: Any, attrs: list[str]):
    """Return True if object has all attributes."""
    # Import here otherwise not fc.GuiUp.
    from .freecad_utils import warn

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


def get_parent_by_pattern(
        file_path: [Path | str],
        pattern: str,
        type: Optional[str] = None,
        ) -> tuple[Path, str]:
    """Return the parent directory of the given file containing pattern.

    Return the directory that is parent (possibly indirect) of `filepath` and
    that contains the directory or file `pattern`.

    If the file path is relative, return `(Path(), file_path)`.

    If the pattern was not found, return `(Path(), '')`.

    """
    file_path = Path(file_path).expanduser()
    if not file_path.is_absolute():
        return Path(), str(file_path)
    if type in ['f', 'file']:
        def is_correct_type(p: Path):
            return p.is_file()
    elif type in ['d', 'directory']:
        def is_correct_type(p: Path):
            return p.is_dir()
    else:
        def is_correct_type(p: Path):
            return p.exists()
    relative_file_path = ''
    while True:
        candidate_path_to_pattern = file_path / pattern
        if is_correct_type(candidate_path_to_pattern):
            return file_path, relative_file_path
        relative_file_path = (f'{file_path.name}/{relative_file_path}'
                              if relative_file_path else file_path.name)
        file_path = file_path.parent
        if file_path.samefile(Path(file_path.root)):
            # We are at the root.
            return Path(), relative_file_path
