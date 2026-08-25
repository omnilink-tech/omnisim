"""Source-level guards for cold-load and hot traversal optimizations."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_newton_preload_starts_before_gui_application_construction():
    main = (ROOT / "src/omnisim/gui/main.cpp").read_text(encoding="utf-8")
    preload = main.index("OmPhysicsBackendRegistry::startNewtonRuntimePreload();")
    application = main.index("OmGuiApplication app(argc, argv);")
    assert preload < application


def test_async_preload_has_synchronous_escape_and_gil_handoff():
    registry = (ROOT / "src/omnisim/physics/OmPhysicsBackend.cpp").read_text(encoding="utf-8")
    backend = (ROOT / "src/omnisim/physics/OmNewtonBackend.cpp").read_text(encoding="utf-8")
    assert "OMNISIM_NEWTON_ASYNC_PRELOAD" in registry
    assert "std::launch::async" in registry
    assert "PyEval_SaveThread()" in backend
    assert "PyGILState_Ensure()" in backend


def test_descendant_walk_uses_constant_time_queue_and_visited_set():
    source = (ROOT / "src/omnisim/nodes/utils/OmNodeUtilities.cpp").read_text(encoding="utf-8")
    function = source[source.index("QList<OmNode *> OmNodeUtilities::findDescendantNodesOfType"):]
    assert "QQueue<OmNode *> queue" in function
    assert "QSet<OmNode *> visited" in function
    assert "queue.takeFirst()" not in function
