#ifndef WR_SCENE_H
#define WR_SCENE_H

#ifdef __cplusplus
extern "C" {
#endif

struct WrScene;
typedef struct WrScene WrScene;

struct WrTransform;
typedef struct WrTransform WrTransform;

struct WrViewport;
typedef struct WrViewport WrViewport;

struct WrRenderable;
typedef struct WrRenderable WrRenderable;

struct WrShaderProgram;
typedef struct WrShaderProgram WrShaderProgram;

typedef enum WrSceneFogType {
  WR_SCENE_FOG_TYPE_NONE,
  WR_SCENE_FOG_TYPE_EXPONENTIAL,
  WR_SCENE_FOG_TYPE_EXPONENTIAL2,
  WR_SCENE_FOG_TYPE_LINEAR
} WrSceneFogType;

/* Defines the basis for the fog distance computation */
typedef enum WrSceneFogDepthType { WR_SCENE_FOG_DEPTH_TYPE_PLANE, WR_SCENE_FOG_DEPTH_TYPE_POINT } WrSceneFogDepthType;

WrScene *wr_scene_get_instance();
void wr_scene_destroy();

// The OpenGL context must be active when calling these functions
void wr_scene_init(WrScene *scene);
void wr_scene_reset(WrScene *scene);

// Apply pending OpenGL state changes
void wr_scene_apply_pending_updates(WrScene *scene);

/* The 'materialName' parameter is optional (can be set to NULL for default), and if set, force
the renderables to use the named material */
void wr_scene_render(WrScene *scene, const char *material_name, bool culling, bool offScreen);
void wr_scene_render_to_viewports(WrScene *scene, int count, WrViewport **viewports, const char *material_name, bool culling,
                                  bool offScreen);

void wr_scene_set_ambient_light(const float *ambient_light);

int wr_scene_get_active_spot_light_count(WrScene *scene);
int wr_scene_get_active_point_light_count(WrScene *scene);
int wr_scene_get_active_directional_light_count(WrScene *scene);

/* Expects a 3-component array for 'color' */
void wr_scene_set_fog(WrScene *scene, WrSceneFogType fogType, WrSceneFogDepthType depthType, const float *color, float density,
                      float start, float end);
void wr_scene_set_skybox(WrScene *scene, WrRenderable *renderable);
void wr_scene_set_hdr_clear_quad(WrScene *scene, WrRenderable *renderable);
void wr_scene_set_fog_program(WrScene *scene, WrShaderProgram *program);
void wr_scene_set_shadow_volume_program(WrScene *scene, WrShaderProgram *program);

void wr_scene_enable_skybox(WrScene *scene, bool enable);
void wr_scene_enable_hdr_clear(WrScene *scene, bool enable);
void wr_scene_enable_translucence(WrScene *scene, bool enable);
void wr_scene_enable_depth_reset(WrScene *scene, bool enable);

void wr_scene_add_frame_listener(WrScene *scene, void (*listener)());
void wr_scene_remove_frame_listener(WrScene *scene, void (*listener)());

void wr_scene_get_main_buffer(int width, int height, unsigned int format, unsigned int data_type, void *buffer);

void wr_scene_init_frame_capture(WrScene *scene, int pixel_buffer_count, unsigned int *pixel_buffer_ids, int frame_size);
void wr_scene_bind_pixel_buffer(WrScene *scene, int buffer);
void *wr_scene_map_pixel_buffer(WrScene *scene, unsigned int access_mode);
void wr_scene_unmap_pixel_buffer(WrScene *scene);
void wr_scene_terminate_frame_capture(WrScene *scene);

int wr_scene_compute_node_count(WrScene *scene);

WrTransform *wr_scene_get_root(WrScene *scene);
WrViewport *wr_scene_get_viewport(WrScene *scene);

/* Per-pass GPU timing — Tier 2 instrumentation, off by default.
 *
 * When enabled, Scene::renderToViewport issues GL_TIMESTAMP queries
 * around the forward pass and the applyPostProcessing call. Results are
 * harvested on a 4-frame ring so the GPU never stalls; the most recent
 * complete pair is exposed via wr_scene_get_last_pass_timings_ns.
 *
 * No effect on rendering output; behaviour is byte-identical to the
 * disabled case other than ~1 microsecond of extra GPU command-buffer
 * work per frame for the queries themselves.
 *
 * forwardNs covers the forward render of the current viewport up to and
 * including its translucency / shadow / fog passes. postProcessNs covers
 * the applyPostProcessing call in isolation. The two together do not
 * include the auxiliary draws that follow post-processing (overlays,
 * non-main drawing-order queues) — those are part of the full
 * wr_scene_render time but are not broken out by this API. */
void wr_scene_set_pass_timing_enabled(bool enabled);
void wr_scene_get_last_pass_timings_ns(unsigned long long *forward_ns, unsigned long long *post_process_ns);

/* Forward-pass sub-bucket breakdown for the most recent harvested frame:
 *   ambient_ns           - ambient/emissive draw + ambient-occlusion apply
 *   per_light_ns         - stencil shadow volumes + diffuse-specular per light
 *                          (sum across all directional/point/spot lights)
 *   forward_residual_ns  - fog + translucent + no-stencil-program + auxiliary
 *                          draws between the per-light loop and post-processing
 * The three together equal forward_ns (modulo nanosecond rounding). */
void wr_scene_get_last_forward_subpass_timings_ns(unsigned long long *ambient_ns, unsigned long long *per_light_ns,
                                                  unsigned long long *forward_residual_ns);

/* Aggregate render-call totals since the last reset. Sums across every
 * Scene::renderToViewport invocation — the main window viewport plus
 * every WbWrenCamera / WbLidar / WbRangeFinder sensor render — so we can
 * see the *full* per-frame rendering load, not just the most recent
 * viewport. The single-viewport timings above only reflect the last
 * call processed.
 *
 * call_count is total attempts; harvested_count is calls whose timing
 * query came back before the 4-slot ring lapped. forward_ns and post_ns
 * are summed across harvested calls only — divide by harvested_count
 * for per-call average. */
void wr_scene_get_aggregate_render_timings_ns(unsigned long long *forward_ns, unsigned long long *post_ns,
                                              unsigned int *call_count, unsigned int *harvested_count);
void wr_scene_reset_aggregate_render_timings();

/* Per-frame render counters — Tier 2 instrumentation, off by default.
 *
 * When enabled, Scene::renderToViewport records the size of the main
 * draw queue (enqueued), how many of those passed the visibility-flag
 * gate (visible), and how many survived frustum culling and were
 * actually submitted as draw calls (drawn).
 *
 * The three numbers together tell us where the CPU is going:
 *   - enqueued >> visible: many invisible-flagged renderables (cheap)
 *   - visible >> drawn:    frustum culling is doing real work
 *   - visible == drawn:    every visible renderable draws every frame —
 *                          the win is in draw-call submission, not culling
 *
 * Drawn count corresponds to the main queue only; auxiliary drawing
 * orders (lens flares, overlays) are not included. */
void wr_scene_set_render_counts_enabled(bool enabled);
void wr_scene_get_last_render_counts(int *enqueued, int *visible, int *drawn);

/* Total triangles submitted from the main draw queue last frame. Used
 * with the GPU forward time to back-compute triangle throughput and
 * disambiguate submission-bound from triangle-bound forward cost. */
long long wr_scene_get_last_triangles_drawn();

/* Per-Mesh draw-call histogram. Top N most-drawn meshes from the most
 * recent main-queue render, sorted by draw count descending.
 *
 * Built only when render counts are enabled (set above). Used to identify
 * instancing candidates: a mesh that draws 100 times in a frame is a
 * 100-to-1 win once instanced.
 *
 * Pointer identity (mesh_id_low32) is the wren::Mesh* low 32 bits — useful
 * as a stable group key within a single run; not stable across runs. The
 * vertex_count and triangle_count fields are diagnostic, so the operator
 * can tell at a glance whether the dominant mesh is a small repeated
 * primitive (cube, sphere) or a heavy unique prop. */
typedef struct WrSceneMeshHistogramEntry {
  unsigned int mesh_id_low32;
  int draw_count;
  int vertex_count;
  int triangle_count;
} WrSceneMeshHistogramEntry;

/* Fills 'out' with up to 'max_entries' entries; writes the actual count
 * to '*filled'. Pass max_entries=0 to query the count without copying. */
void wr_scene_get_top_mesh_histogram(WrSceneMeshHistogramEntry *out, int max_entries, int *filled);

/* Instancing-candidate run detector (item 1 of large-world-optimization-
 * plan.md). Counts runs of consecutive renderables sharing the same
 * sortingId in the post-state-sort opaque draw range — i.e. the
 * candidates that would collapse into one glDrawElementsInstanced call
 * once the draw-path merger lands.
 *
 *   run_count: number of runs whose length is ≥ the instancing threshold
 *   max_run_length: peak length of any run last frame
 *   mergeable: total renderables inside ≥-threshold runs (upper bound on
 *              draw calls saved by full merging)
 *
 * Built only when render counts are enabled. Used to size the threshold
 * and confirm the merger is worth shipping on real workloads. */
void wr_scene_get_last_instancing_stats(int *run_count, int *max_run_length, int *mergeable);

#ifdef __cplusplus
}
#endif

#endif  // WR_SCENE_H
