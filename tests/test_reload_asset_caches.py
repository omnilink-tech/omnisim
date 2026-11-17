"""Source-contract tests for process-lived reload caches.

These intentionally stay simulator-free: they pin the invalidation and lifecycle
seams which are otherwise difficult to observe without a graphics context.
Runtime timing belongs to the same-process harness benchmark documented in
``docs/developer/world-reload-cache.md``.
"""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def source(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


def test_triangle_mesh_cache_retains_zero_user_entries_but_is_bounded():
    cpp = source("src/omnisim/nodes/utils/OmTriangleMeshCache.cpp")
    hpp = source("src/omnisim/nodes/utils/OmTriangleMeshCache.hpp")

    assert 'std::getenv("OMNISIM_TRIANGLE_MESH_CACHE_SIZE")' in cpp
    assert "pruneUnused(map);" in cpp
    assert "triangleMeshInfo.mNumUsers == 0" not in cpp
    assert "void clear();" in hpp


def test_decoded_texture_identity_invalidates_file_and_preference_changes():
    cpp = source("src/omnisim/nodes/OmImageTexture.cpp")

    assert "info.size()" in cpp
    assert "info.lastModified().toMSecsSinceEpoch()" in cpp
    assert 'value("OpenGL/textureQuality", 4)' in cpp
    assert "OMNISIM_DECODED_TEXTURE_CACHE_MB" in cpp
    assert "gDecodedImages.find(cacheKey)" in cpp


def test_changed_local_proto_is_evicted_while_unchanged_model_is_retained():
    cpp = source("src/omnisim/vrml/OmProtoManager.cpp")

    assert "QCryptographicHash::Sha256" in cpp
    assert "mModelFingerprints.value(model) != currentFingerprint" in cpp
    assert "if (changed || difference.contains(model->url()))" in cpp
    assert "model->unref();" in cpp
    assert "!OmUrl::isWeb((*modelIt)->url())" not in cpp


def test_application_clears_process_cache_after_final_world_teardown():
    cpp = source("src/omnisim/app/OmApplication.cpp")
    destructor = cpp[cpp.index("OmApplication::~OmApplication()") : cpp.index("void OmApplication::setup()")]

    assert destructor.index("delete mWorld;") < destructor.index("OmTriangleMeshCache::clear();")
