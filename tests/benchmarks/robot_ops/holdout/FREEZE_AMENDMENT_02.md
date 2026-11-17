# FREEZE AMENDMENT 02 — real-time speed: reporting and a sensitivity result

Recorded 2026-09-26 while holdout v1 run 2 was in progress, after 124 of 336
episodes. It only adds analysis: nothing about how run 2 executes changes,
and no code is touched. It amends [FREEZE.md](FREEZE.md) and
[FREEZE_AMENDMENT_01.md](FREEZE_AMENDMENT_01.md).

## Why

Episodes run in REALTIME, and the grader's timing checks use wall-clock time
(for example: moved within a window, idle triggers, halts). If the machine is
busy and the simulator falls behind real time, the robot moves more slowly in
wall time, and a task can get harder for reasons that have nothing to do with
the agent.

During run 2, CPU sat at 100% because of load outside the benchmark:
- an orphaned `find` scanning the whole filesystem since 2026-09-25 19:56;
- VS Code's C/C++ extension indexing the repository.

Both have been handed to another session to remove.

## The measure

Every pose sample carries both wall time (`t`) and the simulator's clock
(`sim`). An episode's **real-time factor** is Δsim/Δwall over a sliding
5-second wall-clock window. Its **worst window** is the minimum of that
factor over the episode.

An episode **lagged** if its worst window is below **0.9×**.

## What is reported

1. For every episode:
   - the real-time factor over the whole episode;
   - the worst window;
   - the largest gap between pose samples.
2. The **main result**, exactly as preregistered, over every episode. The
   claims (A and B) are decided from this result only.
3. A **sensitivity result**. It runs the same analysis without any task block
   (one task in one repeat, all 7 arms) that contains a lagged episode. A
   whole block is dropped, so both sides of every pair go together, the same
   way an excluded ERROR episode is handled. If a task loses one repeat, it
   stays in the task bootstrap with the repeat that remains.

   If the main and sensitivity results disagree on a claim, that disagreement
   is reported next to the claim.

ERROR re-runs are measured too, and their lag is reported the same way.

## What was known when this was recorded

The developer had already computed the measure for the first 124 episodes:
- 114 episodes never fell below 0.9×. The median real-time factor was 1.00,
  and the largest gap between pose samples was 0.38 s.
- 10 episodes lagged, with the worst window at 0.68×. Nine were in two task
  blocks, where every arm lagged together:
  - `f6_report_blocked_husky`: all four lagged arms passed.
  - `f4_shove_correction_tb3`: six arms failed on the same reply check, and
    one (`lobster_basic`) was a provider ERROR.

  The tenth was one `plain_basic` episode of
  `f1_negative_control_forklift_husky`, which passed.
- In both blocks, every arm got the same result apart from provider errors.
  So choosing the 0.9× rule after seeing them cannot move the difference
  between OmniLink and any competitor in either direction.
- The same two tasks ran at 0.98–1.00× in the calibration runs
  (`holdout-calibration-01` to `-04`). So the lag came from machine load, not
  from anything in the tasks.

The 0.9× threshold and the 5-second window were chosen before the rest of
run 2 was seen, and are not changed afterwards.
