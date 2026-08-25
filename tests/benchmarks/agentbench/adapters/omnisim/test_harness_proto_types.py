"""Regression guards for PROTO-derived scene classification in the harness."""

from __future__ import annotations

import sys
from pathlib import Path


HARNESS = (Path(__file__).resolve().parents[5] / "projects" / "default" /
           "controllers" / "harness_supervisor")
sys.path.insert(0, str(HARNESS))

import observe  # noqa: E402


class Node:
    def __init__(self, type_name, base_type):
        self.type_name = type_name
        self.base_type = base_type

    def getTypeName(self):
        return self.type_name

    def getBaseTypeName(self):
        return self.base_type


def test_proto_derived_robot_is_a_robot():
    assert observe._is_robot(Node("ScaleBot", "Robot"))


def test_proto_derived_solid_is_a_solid():
    assert observe._is_solid(Node("CrateProto", "Solid"))


def test_plain_group_is_neither():
    node = Node("Group", "Group")
    assert not observe._is_robot(node)
    assert not observe._is_solid(node)
