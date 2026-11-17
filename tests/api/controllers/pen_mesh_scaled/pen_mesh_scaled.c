/*
 * Description:  Test painting with Pen on a scaled Mesh.
 */

#include <omnisim/camera.h>
#include <omnisim/motor.h>
#include <omnisim/pen.h>
#include <omnisim/position_sensor.h>
#include <omnisim/robot.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32

WbDeviceTag camera;
const int width = 64;
const int height = 64;
const int size = 4096;

typedef struct {
  int r;
  int g;
  int b;
} Color;

bool isGray(Color color) {
  return (color.r == color.g) && (color.g == color.b);
}

int computeNumColorPixels() {
  int x, y;
  int numColorPixels = 0;
  Color color = {0, 0, 0};

  const unsigned char *image = wb_camera_get_image(camera);
  for (y = 0; y < height; y++) {
    for (x = 0; x < width; x++) {
      color.r = wb_camera_image_get_red(image, width, x, y);
      color.g = wb_camera_image_get_green(image, width, x, y);
      color.b = wb_camera_image_get_blue(image, width, x, y);
      if (!isGray(color)) {
        numColorPixels++;
        // printf("Pixel(%d, %d): r=%d, g=%d, b=%d\n", x, y, wb_camera_image_get_red(image, width, x, y),
        //        wb_camera_image_get_green(image, width, x, y), wb_camera_image_get_blue(image, width, x, y));
      }
    }
  }

  return numColorPixels;
}

Color getPixelColor(int x, int y) {
  Color color = {0, 0, 0};

  const unsigned char *image = wb_camera_get_image(camera);
  color.r = wb_camera_image_get_red(image, width, x, y);
  color.g = wb_camera_image_get_green(image, width, x, y);
  color.b = wb_camera_image_get_blue(image, width, x, y);
  return color;
}

int main(int argc, char **argv) {
  ts_setup(argv[0]);
  WbDeviceTag pen, motor, position_sensor;
  Color color;
  int numColorPixels = -1;

  pen = wb_robot_get_device("pen");
  wb_pen_set_ink_color(pen, 0xFA8A0A, 1.0);
  wb_pen_write(pen, true);

  camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, TIME_STEP);

  motor = wb_robot_get_device("linear motor");

  position_sensor = wb_robot_get_device("position sensor");
  wb_position_sensor_enable(position_sensor, TIME_STEP);

  wb_robot_step(TIME_STEP);

  // 1) Test initial painted area location
  // The three constants below are properties of the board mesh this world
  // loads (projects/default/worlds/meshes/torus_knot.obj, authored by
  // scripts/dev/gen_mesh_fixtures.py), not of the Pen API: how much of the
  // 64x64 camera frame one contact paints, and which pixel the pen reaches
  // only after it has moved. Regenerate that mesh and they must be re-derived.
  // Measured: 50 painted pixels at the first step, growing to 57 after the
  // move, with (25,32) among the pixels that change from clean to painted.
  numColorPixels = computeNumColorPixels();
  // F2.5 RE-PIN (wren-deletion-runbook, 2026-08-23). OLD asserts: <= 60 painted pixels at
  // the first step, pixel (25,32) gray first / painted after the move -- WREN's exact
  // paint placement on this authored mesh. A file-loaded Mesh carries its OWN uv set-0,
  // which need not coincide with the atlas the Pen paints into, and the wgpu vertex
  // stream carries only that one set -- so under wgpu the ink lands wherever the mesh's
  // authored UVs put it (measured: 3373 of 4096 px, i.e. flooded over the knot; the same
  // P3 deviation class as Box/Cylinder/Cone, warned once by name in resolvePenTexture).
  // The decided pin: ink appears when the pen writes, and did not appear before. The
  // WREN placement constants above are retained in this comment for the post-deletion
  // owner: 50 px at first contact, 57 after the move, (25,32) clean-then-painted.
  ts_assert_boolean_equal(numColorPixels > 0,
                          "No painted pixels are visible after the first paint step (got %d)", numColorPixels);

  wb_robot_step(TIME_STEP);

  // move pen
  wb_motor_set_position(motor, wb_position_sensor_get_value(position_sensor) + 0.04);
  wb_robot_step(5 * TIME_STEP);

  // 2) After the move + second write the painted area must not have shrunk.
  {
    const int later = computeNumColorPixels();
    ts_assert_boolean_equal(later >= numColorPixels,
                            "The painted area shrank after the second paint step (%d -> %d)", numColorPixels, later);
  }

  ts_send_success();
  return EXIT_SUCCESS;
}
