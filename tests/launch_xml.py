"""Read <arg> and <param> attributes from roslaunch XML in static tests.

A missing attribute reads as None, so an assertion on it fails loudly.
"""


def arg_defaults(element):
    """``name -> default`` of the <arg> children of a launch root."""
    return _attributes(element, "arg", "default")


def arg_values(element):
    """``name -> value`` of the <arg> children of an <include>."""
    return _attributes(element, "arg", "value")


def param_values(node):
    """``name -> value`` of the <param> children of a <node>."""
    return _attributes(node, "param", "value")


def _attributes(element, tag, attribute):
    return {item.attrib["name"]: item.attrib.get(attribute) for item in element.findall(tag)}
