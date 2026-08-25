/*
 * Copyright 2026 OmniLink
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * W3/P3 gate: a Pen's paint must be READABLE FROM A wgpu CAMERA IMAGE.
 *
 * The assertions are DIFFERENTIAL on purpose (see the world header): the same frame is measured
 * with the pen off and then with the pen on, and what is pinned is the change the ink makes, not
 * an absolute colour. That keeps the test a test of the PEN rather than of whichever ambient
 * model the active renderer happens to use -- the reason it cannot simply live in pen.c.
 *
 * The pen is pure RED at density 1 and the camera looks straight down at the board from 77 mm,
 * so one write covers the whole 64x64 frame. Before the write no channel dominates (a white board
 * under a blue-ish sky reads blue-heavy); after it, red must dominate by a wide margin.
 */

#include <omnisim/camera.h>
#include <omnisim/pen.h>
#include <omnisim/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

static WbDeviceTag cam;
static const int width = 64;
static const int height = 64;
static const int size = 4096;

typedef struct {
  int r;
  int g;
  int b;
} Color;

static Color mean_color(void) {
  int x, y;
  Color color = {0, 0, 0};
  const unsigned char *image = wb_camera_get_image(cam);
  for (y = 0; y < height; y++) {
    for (x = 0; x < width; x++) {
      color.r += wb_camera_image_get_red(image, width, x, y);
      color.g += wb_camera_image_get_green(image, width, x, y);
      color.b += wb_camera_image_get_blue(image, width, x, y);
    }
  }
  color.r /= size;
  color.g /= size;
  color.b /= size;
  return color;
}

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  WbDeviceTag pen0 = wb_robot_get_device("pen0");
  wb_pen_write(pen0, false);

  cam = wb_robot_get_device("camera0");
  wb_camera_enable(cam, TIME_STEP);

  wb_robot_step(TIME_STEP);

  const Color before = mean_color();
  ts_assert_int_is_bigger(before.b, before.r,
                          "Before the pen writes, the white board under a blue sky should NOT read "
                          "red-dominant; got (r=%d,g=%d,b=%d)",
                          before.r, before.g, before.b);

  wb_pen_set_ink_color(pen0, 0xFF0000, 1.0);
  wb_pen_write(pen0, true);

  wb_robot_step(TIME_STEP);

  const Color after = mean_color();

  /* The ink is mixed into the base colour and then LIT, so its absolute level depends on the
     renderer's ambient term. What cannot depend on that is the ORDERING: pure red ink can only
     make the red channel dominate. */
  ts_assert_int_is_bigger(after.r, after.g + 20,
                          "After pen0 writes pure red, the camera image should be red-dominant. "
                          "Got (r=%d,g=%d,b=%d) after, (r=%d,g=%d,b=%d) before -- the paint layer "
                          "is not reaching the camera image",
                          after.r, after.g, after.b, before.r, before.g, before.b);
  ts_assert_int_is_bigger(after.r, after.b + 20,
                          "After pen0 writes pure red, the camera image should be red-dominant. "
                          "Got (r=%d,g=%d,b=%d) after, (r=%d,g=%d,b=%d) before -- the paint layer "
                          "is not reaching the camera image",
                          after.r, after.g, after.b, before.r, before.g, before.b);
  ts_assert_int_is_bigger(before.b, after.b + 20,
                          "Red ink should REMOVE blue from the image. Got b=%d after vs b=%d "
                          "before (r=%d,g=%d after)",
                          after.b, before.b, after.r, after.g);

  ts_send_success();
  return EXIT_SUCCESS;
}
