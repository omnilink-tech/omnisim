#ifndef WR_TEXTURE_CUBEMAP_BAKER_H
#define WR_TEXTURE_CUBEMAP_BAKER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <wren/texture_2d.h>
#include <wren/texture_cubemap.h>
#include <wren/texture_rtt.h>

WrTextureCubeMap *wr_texture_cubemap_bake_diffuse_irradiance(WrTextureCubeMap *input_cubemap, WrShaderProgram *shader,
                                                             unsigned int size);
WrTextureCubeMap *wr_texture_cubemap_bake_specular_irradiance(WrTextureCubeMap *input_cubemap, WrShaderProgram *shader,
                                                              unsigned int size);
WrTextureRtt *wr_texture_cubemap_bake_brdf(WrShaderProgram *shader, unsigned int size);

// Bake a cubemap whose six faces are rendered by ``shader`` while
// ``input2D`` is bound at slot 0 (the standard ``inputTextures[0]``
// sampler).  Per-face view + projection matrix uniforms follow the
// same convention as the IBL bakers above.  Used by the procedural
// atmospheric-sky pipeline to feed sky-view-LUT colour into the
// existing specular-irradiance bake.
WrTextureCubeMap *wr_texture_cubemap_bake_from_2d(WrTextureRtt *input2D, WrShaderProgram *shader, unsigned int size);

#ifdef __cplusplus
}
#endif

#endif
