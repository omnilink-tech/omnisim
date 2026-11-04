"""Background consumers cannot touch the non-thread-safe native supervisor."""
import ast
from concurrent.futures import ThreadPoolExecutor
import math
from pathlib import Path
import threading
from typing import Tuple

import pytest


def bridge():
    path=Path(__file__).resolve().parents[1]/'projects/samples/demos/controllers/omnilink_mobile_bridge/omnilink_mobile_bridge.py'
    tree=ast.parse(path.read_text(encoding='utf-8'))
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='MobileBridge')
    method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='_read_pose')
    module=ast.fix_missing_locations(ast.Module(body=[ast.ClassDef(name='Bridge',bases=[],keywords=[],body=[method],decorator_list=[])],type_ignores=[]))
    namespace={'threading':threading,'Tuple':Tuple,'yaw_from_orientation':lambda value:value,
               'wrap_pi':lambda v:(v+math.pi)%(2*math.pi)-math.pi}
    exec(compile(module,str(path),'exec'),namespace)
    obj=namespace['Bridge'](); obj.yaw_offset=.1
    class Node:
        calls=0
        def getPosition(self):
            assert threading.current_thread() is threading.main_thread()
            self.calls+=1
            return [2.,3.,.1]
        def getOrientation(self):
            assert threading.current_thread() is threading.main_thread()
            self.calls+=1
            return .4
    obj.self_node=Node()
    return obj


def test_many_concurrent_observers_only_read_published_snapshot():
    obj=bridge(); measured=obj._read_pose()
    with ThreadPoolExecutor(max_workers=8) as pool:
        values=list(pool.map(lambda _:obj._read_pose(),range(1000)))
    assert all(value==measured for value in values)
    assert obj.self_node.calls==2


def test_unsampled_background_pose_is_refused_not_invented():
    obj=bridge()
    with ThreadPoolExecutor(max_workers=1) as pool:
        with pytest.raises(RuntimeError,match='not been sampled'):
            pool.submit(obj._read_pose).result()
    assert obj.self_node.calls==0
