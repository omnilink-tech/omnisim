#include <omnisim/camera.h>
#include <omnisim/led.h>
#include <omnisim/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  WbDeviceTag led_phong = wb_robot_get_device("phong");
  WbDeviceTag led_pbr = wb_robot_get_device("pbr");
  WbDeviceTag led_light = wb_robot_get_device("light");

  WbDeviceTag camera = wb_robot_get_device("camera");

  wb_led_set(led_phong, 0xff0000);
  wb_led_set(led_pbr, 0xff0000);
  wb_led_set(led_light, 0xff0000);

  wb_camera_enable(camera, TIME_STEP);

  wb_robot_step(TIME_STEP);
  wb_robot_step(TIME_STEP);

  ts_assert_int_equal(wb_led_get(led_phong), 0xff0000, "The phong LED has the wrong stored color.");
  ts_assert_int_equal(wb_led_get(led_pbr), 0xff0000, "The pbr LED has the wrong stored color.");
  ts_assert_int_equal(wb_led_get(led_light), 0xff0000, "The light LED has the wrong stored color.");

  const unsigned char *camera_image = wb_camera_get_image(camera);

  unsigned char phong_red = wb_camera_image_get_red(camera_image, 128, 19, 64);
  unsigned char pbr_red = wb_camera_image_get_red(camera_image, 128, 64, 64);
  unsigned char light_red = wb_camera_image_get_red(camera_image, 128, 103, 61);

  unsigned char phong_green = wb_camera_image_get_green(camera_image, 128, 19, 64);
  unsigned char phong_blue = wb_camera_image_get_blue(camera_image, 128, 19, 64);

  unsigned char pbr_green = wb_camera_image_get_green(camera_image, 128, 64, 64);
  unsigned char pbr_blue = wb_camera_image_get_blue(camera_image, 128, 64, 64);

  // fuzzy color check for HDR
  ts_assert_int_is_bigger(phong_red, 0xaa, "The phong material should be bright red (red=%d)", phong_red);
  ts_assert_int_is_bigger(pbr_red, 0xaa, "The pbr material should be bright red (red=%d)", pbr_red);
  ts_assert_int_is_bigger(light_red, 0x80, "The spotlight's illuminated circle should be bright red (red=%d)", light_red);

  // check the spheres are red-dominant.
  // F2.5 RE-GOLDEN (wren-deletion-runbook, 2026-08-23). OLD EXPECTATION: green == 0x00 and
  // blue == 0x00 exactly -- WREN's per-channel exposure+sRGB encode maps a pure (1,0,0)
  // emissive to (207,0,0), zeros staying zero. The wgpu sensor pipeline tonemaps through
  // AgX (OmWgpuShaders.cpp kAgxTonemapPost), whose 3x3 INSET matrix mixes ~4-8% of a pure
  // primary into the other channels BEFORE the log2 sigmoid, so the outset cannot restore
  // an exact zero: a pure red emissive measures (207,20,20) -- same 207 red as WREN, plus
  // the structural AgX crosstalk. 24 bounds that crosstalk with headroom for the +-0.5 LSB
  // dither while still failing hard on any real regression (a diffuse-lit sphere reads
  // green ~= red, an order of magnitude over the bound).
  const int agx_crosstalk_max = 24;
  ts_assert_int_is_bigger(agx_crosstalk_max, phong_green,
                          "The phong material should have (near) no green (green=%d, red=%d, blue=%d)", phong_green,
                          phong_red, phong_blue);
  ts_assert_int_is_bigger(agx_crosstalk_max, phong_blue, "The phong material should have (near) no blue (blue=%d)",
                          phong_blue);
  ts_assert_int_is_bigger(agx_crosstalk_max, pbr_green,
                          "The pbr material should have (near) no green (green=%d, red=%d, blue=%d)", pbr_green, pbr_red,
                          pbr_blue);
  ts_assert_int_is_bigger(agx_crosstalk_max, pbr_blue, "The pbr material should have (near) no blue (blue=%d)", pbr_blue);

  ts_send_success();
  return EXIT_SUCCESS;
}
