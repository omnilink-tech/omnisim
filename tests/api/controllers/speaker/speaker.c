#include <omnisim/robot.h>
#include <omnisim/speaker.h>

#include "../../../lib/ts_assertion.h"
#include "../../../lib/ts_utils.h"

#define TIME_STEP 32
#define SOUND_FILE "sound.wav"

int main(int argc, char **argv) {
  ts_setup(argv[0]);

  WbDeviceTag speaker = wb_robot_get_device("speaker");

  wb_robot_step(TIME_STEP);

  ts_assert_boolean_equal(!wb_speaker_is_speaking(speaker), "wb_speaker_is_speaking should return false before starting TTS.");
  ts_assert_boolean_equal(!wb_speaker_is_sound_playing(speaker, SOUND_FILE),
                          "wb_speaker_is_sound_playing should return false when the sound is not playing.");
  ts_assert_boolean_equal(!wb_speaker_is_sound_playing(speaker, NULL),
                          "wb_speaker_is_sound_playing should return false when called with a NULL argument if TTS is not "
                          "running and no sound is playing.");

#ifdef _WIN32
  ts_assert_boolean_equal(wb_speaker_set_engine(speaker, "microsoft"),
                          "Fail to set the 'microsoft' engine for the TTS on Windows.");
  ts_assert_boolean_equal(wb_speaker_set_language(speaker, "en-US"), "Fail to set english for TTS.");
#else
  // Pico TTS has been removed; only the Microsoft engine remains and it is Windows-only.
  // Skip the engine/language assertions on non-Windows platforms — speak() will warn and no-op.
#endif

  wb_robot_step(TIME_STEP);

  wb_speaker_speak(speaker, "Hello test suite!", 0.1);

  ts_assert_boolean_equal(wb_speaker_is_speaking(speaker), "wb_speaker_is_speaking should return true when TTS is running.");
  ts_assert_boolean_equal(!wb_speaker_is_sound_playing(speaker, SOUND_FILE),
                          "wb_speaker_is_sound_playing should return false when the sound is not playing.");
  ts_assert_boolean_equal(wb_speaker_is_sound_playing(speaker, NULL),
                          "wb_speaker_is_sound_playing should return true when called with a NULL argument if TTS is running.");

  wb_robot_step(TIME_STEP);

  ts_assert_boolean_equal(wb_speaker_is_speaking(speaker), "wb_speaker_is_speaking should return true when TTS is running.");
  ts_assert_boolean_equal(!wb_speaker_is_sound_playing(speaker, SOUND_FILE),
                          "wb_speaker_is_sound_playing should return false when the sound is not playing.");

  wb_robot_step(TIME_STEP);

  wb_speaker_stop(speaker, NULL);
  wb_speaker_play_sound(speaker, speaker, SOUND_FILE, 0.1, 1.0, 0.0, true);

  ts_assert_boolean_equal(!wb_speaker_is_speaking(speaker),
                          "wb_speaker_is_speaking should return false when TTS is not running.");
  ts_assert_boolean_equal(wb_speaker_is_sound_playing(speaker, SOUND_FILE),
                          "wb_speaker_is_sound_playing should return true when the sound is playing.");
  ts_assert_boolean_equal(
    wb_speaker_is_sound_playing(speaker, NULL),
    "wb_speaker_is_sound_playing should return true when called with a NULL argument when a sound is playing.");

  wb_robot_step(TIME_STEP);

  ts_assert_boolean_equal(!wb_speaker_is_speaking(speaker),
                          "wb_speaker_is_speaking should return false when TTS is not running.");
  ts_assert_boolean_equal(wb_speaker_is_sound_playing(speaker, SOUND_FILE),
                          "wb_speaker_is_sound_playing should return true when the sound is playing.");
  ts_assert_boolean_equal(
    wb_speaker_is_sound_playing(speaker, NULL),
    "wb_speaker_is_sound_playing should return true when called with a NULL argument when a sound is playing.");

  ts_send_success();
  return EXIT_SUCCESS;
}
