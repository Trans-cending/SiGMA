"""Naming helpers shared by Blender and Sionna modules."""
from __future__ import annotations

import re


def strip_blender_suffix(name):
    return re.sub(r"\.\d{3}$", "", str(name))


def local_name(tag):
    tag = str(tag)
    return tag.rsplit("}", 1)[-1] if "}" in tag else tag
