"""Change single values in a YAML file and keep its comments and layout."""

import yaml


def child(node: yaml.Node | None, key: str) -> yaml.Node | None:
    """The value node of key in a mapping node, or None."""
    if not isinstance(node, yaml.MappingNode):
        return None
    for key_node, value_node in node.value:
        if isinstance(key_node, yaml.ScalarNode) and key_node.value == key:
            return value_node
    return None


def find(node: yaml.Node | None, *keys: str) -> yaml.Node | None:
    for key in keys:
        node = child(node, key)
    return node


def replace(text: str, edits: list[tuple[yaml.Node, str]]) -> str:
    """Put each new value in place of its scalar node, from the end of the text back."""
    spans = sorted(
        ((node.start_mark.index, node.end_mark.index, value) for node, value in edits),
        reverse=True,
    )
    for start, end, value in spans:
        text = text[:start] + value + text[end:]
    return text
