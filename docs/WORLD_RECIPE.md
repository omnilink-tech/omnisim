# OmniSim World Recipe — the canonical sky-and-sun lighting

**Every `.wbt` in this repo — demos, samples, generated worlds, RL worlds, manually-authored examples — uses the same three-PROTO lighting recipe.** New worlds, whether written by a human or by an AI agent, MUST follow it. Test/regression worlds under `tests/` are exempt (they exist to exercise specific engine behaviour, not to look pretty).

> **Known exemption (as of 2026-07-09): omniworld-generated worlds.** The tracked worlds under
> `distribution/generated_worlds/` predate this recipe and use `TexturedBackground`-style lighting
> (`mars.wbt` even carries hand-written `Background` + `DirectionalLight` blocks), and the omniworld
> emitter (`src/python/omniworld/emit/wbt.py`) still emits that style — so regenerating does not fix it.
> Migrating the emitter to the OmniSimSky recipe is pending. Until then, the MUST above applies to
> **hand-authored** worlds; don't copy a generated world's lighting block into a new hand-authored one.

This page is the source of truth. If it disagrees with any world file, the recipe wins — fix the world.

---

## The recipe (copy-pasteable)

```vrml
EXTERNPROTO "omnisim://projects/objects/backgrounds/protos/OmniSimSky.proto"
EXTERNPROTO "omnisim://projects/objects/lights/protos/OmniSimSun.proto"
EXTERNPROTO "omnisim://projects/objects/lights/protos/OmniSimSunMarker.proto"

WorldInfo { basicTimeStep 16  title "Your world title" }
Viewpoint { orientation -0.5773 0.5773 0.5773 2.0944  position 0 0 10 }

OmniSimSky { }
DEF SUN        OmniSimSun       { }
DEF SUN_MARKER OmniSimSunMarker { }

# ... rest of your world ...
```

> ⚠️ **The `Viewpoint` line above is a placeholder, and it is the single most
> copy-pasted bug in this tree.** `orientation -0.5773 0.5773 0.5773 2.0944`
> decodes to forward `(0, 0, -1)` — **straight down**. It is correct *only* with
> a directly-overhead `position` like the `0 0 10` shown here. Move the position
> to an oblique eye point and keep the orientation and the camera will stare at
> empty floor metres from your robot — which is exactly what happened to 160
> worlds before the framing gate landed. **Do not hand-tune the two numbers.**
> Bake the camera instead:
> `PYTHONPATH=src/python python scripts/dev/set_viewpoint.py <world> --auto`.
> See [docs/developer/viewpoint-convention.md](developer/viewpoint-convention.md);
> a world whose robot opens out of frame now fails `omniworld validate` and the
> pre-push hook.

That is the whole recipe. Three PROTOs, three lines. With defaults it produces:

- Daylight blue sky with the atmospheric-sky shader (`atmosphericSky "earth"`).
- Warm directional sun (slightly yellow, intensity 2.5, casts shadows).
- A glowing emissive sphere ~8 m above the origin that the user can drag in the 3D viewport to orbit the sun live. Press `V` to hide/show.

The look is the same one shipped with the OmniQuad demo at [`projects/robots/omnisim/omniquad/worlds/omniquad.omniworld`](../projects/robots/omnisim/omniquad/worlds/omniquad.omniworld) — that's the reference world.

---

## The three PROTOs

| PROTO | What it wraps | Where it lives |
|---|---|---|
| [`OmniSimSky`](../projects/objects/backgrounds/protos/OmniSimSky.proto) | One `Background` node. Daylight blue sky + atmospheric shader. | `projects/objects/backgrounds/protos/` |
| [`OmniSimSun`](../projects/objects/lights/protos/OmniSimSun.proto) | One `DirectionalLight`. Warm sunlight, casts shadows. | `projects/objects/lights/protos/` |
| [`OmniSimSunMarker`](../projects/objects/lights/protos/OmniSimSunMarker.proto) | One `Robot` (driver) wrapping the visual emissive sphere. The `sun_marker` supervisor controller reads its translation each step and writes it into `DEF SUN`'s `direction` field, so dragging the marker orbits the sun. | `projects/objects/lights/protos/` |

PROTOs are single-rooted (a constraint of the PROTO system OmniSim inherited from upstream Webots), which is why the recipe is three nodes rather than one — `Background`, `DirectionalLight`, and `Robot` can't share a parent.

### Why those DEF names matter

The supervisor controller [`projects/default/controllers/sun_marker/sun_marker.py`](../projects/default/controllers/sun_marker/sun_marker.py) hard-codes `getFromDef("SUN")` and `getFromDef("SUN_MARKER")`. Instantiating without those DEFs still loads — the controller prints an idle message and the sun stops being live-draggable, but the visual recipe still works.

---

## Extending the recipe

The recipe is **extensible by adding new fields** with sensible defaults. That's a backward-compatible change — existing worlds keep their look, new worlds opt in.

**Allowed extensions:**

1. **New field on an existing PROTO.** Add `field SF... newField <default>` to the header and `newField IS newField` to the body. Defaults must match the current visual output. Update this doc to list the new field.
2. **New PROTO in the same recipe.** For new categories of lighting (fog, post-fx, ambient fill, secondary lights). Add the EXTERNPROTO line above and the instantiation line below the existing three. Update this doc and ship a default-noop instantiation pattern.
3. **Per-world overrides via PROTO fields.** A Mars world legitimately wants a red sky — override `skyColor` and `atmosphericSky "mars"` on its `OmniSimSky` instance. Don't fork the PROTO; pass field values.

**NOT allowed:**

- **Don't change the defaults on the existing PROTOs.** Every world inherits them; a default change silently re-skins ~100 worlds. Add a new field instead.
- **Don't reach for `NightSky`, `TexturedBackground`, `TexturedBackgroundLight`, or hand-written `Background { ... }` / `DirectionalLight { ... }` blocks in new demos.** If you find yourself wanting one of these, you're either building a test world (under `tests/`, exempt) or you've found a real recipe gap — extend the recipe per the rules above.
- **Don't author a fourth lighting recipe.** One canonical look across all user-facing worlds is the point.

---

## For AI agents authoring worlds

When you are asked to create or substantially edit a `.wbt` outside `tests/`, your default behaviour MUST be:

1. Open this file, copy the three-line recipe block above.
2. Paste it in. Don't generate `Background { ... }` / `DirectionalLight { ... }` / `DEF SUN_MARKER Solid { ... }` inline blocks.
3. Override PROTO fields only when the world has a documented stylistic reason to differ (Mars worlds, night-themed showcases). Note the deviation in the world's header comment.

If you find a world that doesn't follow the recipe, migrating it is a safe, low-risk change — there are no behavioural differences from the inline form.

---

## Reference world

[`projects/robots/omnisim/omniquad/worlds/omniquad.omniworld`](../projects/robots/omnisim/omniquad/worlds/omniquad.omniworld) is the canonical recipe consumer. When in doubt, copy its lighting block.
