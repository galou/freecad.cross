"""Compat layer for xacro processing with a fallback parser."""

from __future__ import annotations

from pathlib import Path
from xml.dom.minidom import Document
from xml.dom.minidom import parse as parse_xml
import os
import re
import shlex
import xml.dom

try:
    import xacro as ros_xacro
except ImportError:
    ros_xacro = None


def process_file(filename: Path | str) -> Document:
    """Process a xacro file and return a minidom document."""
    if ros_xacro is not None:
        return ros_xacro.process_file(Path(filename).expanduser())

    filename = Path(filename).expanduser()
    with open(filename) as f:
        doc = parse_xml(f)
    _process_includes(doc, str(filename.parent))
    _eval_self_contained(doc)
    return doc


def process_doc(doc: Document, base_dir: Path | str | None = None) -> Document:
    """Process a xacro minidom document in place."""
    if ros_xacro is not None:
        ros_xacro.process_doc(doc)
        return doc

    if base_dir is None:
        base_dir = Path.cwd()
    _process_includes(doc, str(base_dir))
    _eval_self_contained(doc)
    return doc


class _Table:
    def __init__(self, parent=None):
        self.parent = parent
        self.table = {}

    def __getitem__(self, key):
        if key in self.table:
            return self.table[key]
        if self.parent:
            return self.parent[key]
        raise KeyError(key)

    def __setitem__(self, key, value):
        self.table[key] = value

    def __contains__(self, key):
        return key in self.table or (self.parent and key in self.parent)


class _QuickLexer:
    def __init__(self, **res):
        self._str = ""
        self._top = None
        self._res = []
        for k, v in res.items():
            self.__setattr__(k, len(self._res))
            self._res.append(v)

    def lex(self, value: str):
        self._str = value
        self._top = None
        self.next()

    def peek(self):
        return self._top

    def next(self):
        result = self._top
        self._top = None
        for i in range(len(self._res)):
            m = re.match(self._res[i], self._str)
            if m:
                self._top = (i, m.group(0))
                self._str = self._str[m.end():]
                break
        return result


def _first_child_element(elt):
    c = elt.firstChild
    while c:
        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
            return c
        c = c.nextSibling
    return None


def _next_sibling_element(elt):
    c = elt.nextSibling
    while c:
        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
            return c
        c = c.nextSibling
    return None


def _next_element(elt):
    child = _first_child_element(elt)
    if child:
        return child
    while elt and elt.nodeType == xml.dom.Node.ELEMENT_NODE:
        next_elt = _next_sibling_element(elt)
        if next_elt:
            return next_elt
        elt = elt.parentNode
    return None


def _next_node(node):
    if node.firstChild:
        return node.firstChild
    while node:
        if node.nextSibling:
            return node.nextSibling
        node = node.parentNode
    return None


def _child_elements(elt):
    c = elt.firstChild
    while c:
        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
            yield c
        c = c.nextSibling


def _process_includes(doc, base_dir: str):
    previous = doc.documentElement
    elt = _next_element(previous)
    while elt:
        if elt.tagName in ['include', 'xacro:include']:
            filename = _eval_text(elt.getAttribute('filename'), {})
            if not os.path.isabs(filename):
                filename = os.path.join(base_dir, filename)
            with open(filename) as f:
                included = parse_xml(f)
            for c in _child_elements(included.documentElement):
                elt.parentNode.insertBefore(c.cloneNode(True), elt)
            elt.parentNode.removeChild(elt)
            elt = None
        else:
            previous = elt
        elt = _next_element(previous)


def _grab_macros(doc):
    macros = {}
    previous = doc.documentElement
    elt = _next_element(previous)
    while elt:
        if elt.tagName in ['macro', 'xacro:macro']:
            name = elt.getAttribute('name')
            macros[name] = elt
            macros[f'xacro:{name}'] = elt
            elt.parentNode.removeChild(elt)
            elt = None
        else:
            previous = elt
        elt = _next_element(previous)
    return macros


def _grab_properties(doc):
    table = _Table()
    previous = doc.documentElement
    elt = _next_element(previous)
    while elt:
        if elt.tagName in ['property', 'xacro:property']:
            name = elt.getAttribute('name')
            if elt.hasAttribute('value'):
                value = elt.getAttribute('value')
            else:
                name = '**' + name
                value = elt
            table[name] = value
            elt.parentNode.removeChild(elt)
            elt = None
        else:
            previous = elt
        elt = _next_element(previous)
    return table


def _eat_ignore(lex):
    while lex.peek() and lex.peek()[0] == lex.IGNORE:
        lex.next()


def _eval_lit(lex, symbols):
    _eat_ignore(lex)
    if lex.peek()[0] == lex.NUMBER:
        return float(lex.next()[1])
    if lex.peek()[0] == lex.SYMBOL:
        value = symbols[lex.next()[1]]
        try:
            return int(value)
        except (TypeError, ValueError):
            try:
                return float(value)
            except (TypeError, ValueError):
                return value
    raise RuntimeError("Bad literal")


def _eval_factor(lex, symbols):
    _eat_ignore(lex)
    neg = 1
    if lex.peek()[1] == '-':
        lex.next()
        neg = -1
    if lex.peek()[0] in [lex.NUMBER, lex.SYMBOL]:
        return neg * _eval_lit(lex, symbols)
    if lex.peek()[0] == lex.LPAREN:
        lex.next()
        _eat_ignore(lex)
        result = _eval_expr(lex, symbols)
        _eat_ignore(lex)
        if lex.next()[0] != lex.RPAREN:
            raise RuntimeError("Unmatched left paren")
        _eat_ignore(lex)
        return neg * result
    raise RuntimeError("Misplaced operator")


def _eval_term(lex, symbols):
    _eat_ignore(lex)
    result = 0
    if lex.peek()[0] in [lex.NUMBER, lex.SYMBOL, lex.LPAREN] or lex.peek()[1] == '-':
        result = _eval_factor(lex, symbols)
    _eat_ignore(lex)
    while lex.peek() and lex.peek()[1] in ['*', '/']:
        op = lex.next()[1]
        n = _eval_factor(lex, symbols)
        result = float(result) * float(n) if op == '*' else float(result) / float(n)
        _eat_ignore(lex)
    return result


def _eval_expr(lex, symbols):
    _eat_ignore(lex)
    op = None
    if lex.peek()[0] == lex.OP:
        op = lex.next()[1]
    result = _eval_term(lex, symbols)
    if op == '-':
        result = -float(result)
    _eat_ignore(lex)
    while lex.peek() and lex.peek()[1] in ['+', '-']:
        op = lex.next()[1]
        n = _eval_term(lex, symbols)
        result = float(result) + float(n) if op == '+' else float(result) - float(n)
        _eat_ignore(lex)
    return result


def _find_package_path(package_name: str) -> str:
    try:
        from ament_index_python.packages import get_package_share_directory  # type: ignore

        return get_package_share_directory(package_name)
    except ImportError:
        pass
    except Exception:
        pass

    ws = os.environ.get('ROS_WORKSPACE', '')
    if ws:
        candidate = Path(ws) / 'src' / package_name
        if candidate.exists():
            return str(candidate)
    return package_name


def _eval_extension(ext: str):
    if ext == '$(cwd)':
        return os.getcwd()
    find_match = re.fullmatch(r'\$\(find\s+([^)]+)\)', ext.strip())
    if find_match:
        return _find_package_path(find_match.group(1).strip())
    return ''


def _eval_text(text: str, symbols):
    def _handle_expr(s):
        lex = _QuickLexer(
            IGNORE=r"\s+",
            NUMBER=r"(\d+(\.\d*)?|\.\d+)([eE][-+]?\d+)?",
            SYMBOL=r"[a-zA-Z_]\w*",
            OP=r"[\+\-\*/]",
            LPAREN=r"\(",
            RPAREN=r"\)",
        )
        lex.lex(s)
        return _eval_expr(lex, symbols)

    def _handle_extension(s):
        return _eval_extension(f"$({s})")

    results = []
    lex = _QuickLexer(
        DOLLAR_DOLLAR_BRACE=r"\$\$+\{",
        EXPR=r"\$\{[^\}]*\}",
        EXTENSION=r"\$\([^\)]*\)",
        TEXT=r"([^\$]|\$[^{(]|\$$)+",
    )
    lex.lex(text)
    while lex.peek():
        if lex.peek()[0] == lex.EXPR:
            results.append(_handle_expr(lex.next()[1][2:-1]))
        elif lex.peek()[0] == lex.EXTENSION:
            results.append(_handle_extension(lex.next()[1][2:-1]))
        elif lex.peek()[0] == lex.TEXT:
            results.append(lex.next()[1])
        elif lex.peek()[0] == lex.DOLLAR_DOLLAR_BRACE:
            results.append(lex.next()[1][1:])
    return ''.join(map(str, results))


def _eval_all(root, macros, symbols):
    for at in root.attributes.items():
        root.setAttribute(at[0], _eval_text(at[1], symbols))
    previous = root
    node = _next_node(previous)
    while node:
        if node.nodeType == xml.dom.Node.ELEMENT_NODE:
            if node.tagName in macros:
                body = macros[node.tagName].cloneNode(deep=True)
                params = shlex.split(body.getAttribute('params'))
                scoped = _Table(symbols)
                for name, value in node.attributes.items():
                    if name not in params:
                        raise RuntimeError(
                            f'Invalid parameter "{name}" while expanding macro "{node.tagName}"',
                        )
                    params.remove(name)
                    scoped[name] = _eval_text(value, symbols)
                cloned = node.cloneNode(deep=True)
                _eval_all(cloned, macros, symbols)
                block = cloned.firstChild
                for param in params[:]:
                    if param[0] == '*':
                        while block and block.nodeType != xml.dom.Node.ELEMENT_NODE:
                            block = block.nextSibling
                        if not block:
                            raise RuntimeError(
                                f'Not enough blocks while evaluating macro {node.tagName}',
                            )
                        params.remove(param)
                        scoped[param] = block
                        block = block.nextSibling
                if params:
                    raise RuntimeError(
                        f'Some parameters were not set for macro {node.tagName}',
                    )
                _eval_all(body, macros, scoped)
                for e in list(_child_elements(body)):
                    node.parentNode.insertBefore(e, node)
                node.parentNode.removeChild(node)
                node = None
            elif node.tagName in ['insert_block', 'xacro:insert_block']:
                name = node.getAttribute('name')
                if ("**" + name) in symbols:
                    block = symbols['**' + name]
                    for e in list(_child_elements(block)):
                        node.parentNode.insertBefore(e.cloneNode(deep=True), node)
                elif ("*" + name) in symbols:
                    block = symbols['*' + name]
                    node.parentNode.insertBefore(block.cloneNode(deep=True), node)
                else:
                    raise RuntimeError(f'Block "{name}" was never declared')
                node.parentNode.removeChild(node)
                node = None
            else:
                for at in node.attributes.items():
                    node.setAttribute(at[0], _eval_text(at[1], symbols))
                previous = node
        elif node.nodeType == xml.dom.Node.TEXT_NODE:
            node.data = _eval_text(node.data, symbols)
            previous = node
        else:
            previous = node
        node = _next_node(previous)


def _eval_self_contained(doc):
    macros = _grab_macros(doc)
    symbols = _grab_properties(doc)
    _eval_all(doc.documentElement, macros, symbols)
