import unittest

from blender_extension.bone_physics.timeline import (
    CacheIndex,
    Checkpoint,
    TimelineActionKind,
    TimelineState,
    TimelineTime,
)


DT = 1.0 / 120.0


def observation(seconds, *, fps=24.0, approximate=False):
    return TimelineTime(seconds=seconds, fps=fps, approximate_preview=approximate)


class TimelineTests(unittest.TestCase):
    def state(self):
        return TimelineState(
            start_time=0.0,
            fixed_dt=DT,
            configuration_hash="world-a",
        )

    def index(self, *checkpoints):
        return CacheIndex(tuple(checkpoints))

    def checkpoint(
        self,
        seconds,
        *,
        identifier=None,
        configuration_hash="world-a",
        fps=24.0,
        fixed_dt=DT,
        authoritative=True,
    ):
        return Checkpoint(
            identifier=identifier or f"cp-{seconds}",
            time=seconds,
            configuration_hash=configuration_hash,
            fps=fps,
            fixed_dt=fixed_dt,
            authoritative=authoritative,
        )

    def test_sequential_forward_playback_advances_exact_fixed_points(self):
        state = self.state()
        initial = state.observe(observation(0.0), self.index())
        action = state.observe(observation(1.0 / 24.0), self.index())

        self.assertEqual(initial.kind, TimelineActionKind.RESET)
        self.assertEqual(action.kind, TimelineActionKind.ADVANCE)
        self.assertEqual(len(action.fixed_times), 5)
        self.assertAlmostEqual(action.fixed_times[-1], 1.0 / 24.0)
        self.assertTrue(action.authoritative)

    def test_skipped_future_frame_restores_and_replays_in_fixed_steps(self):
        state = self.state()
        state.observe(observation(0.0), self.index())
        state.observe(observation(1.0 / 24.0), self.index())
        checkpoint = self.checkpoint(1.0 / 24.0)

        action = state.observe(observation(4.0 / 24.0), self.index(checkpoint))

        self.assertEqual(action.kind, TimelineActionKind.RESTORE_AND_REPLAY)
        self.assertEqual(action.checkpoint, checkpoint)
        self.assertEqual(len(action.fixed_times), 15)
        self.assertTrue(
            all(
                abs((right - left) - DT) < 1.0e-9
                for left, right in zip(
                    (checkpoint.time,) + action.fixed_times,
                    action.fixed_times,
                )
            )
        )

    def test_backward_seek_uses_nearest_compatible_checkpoint(self):
        state = self.state()
        state.observe(observation(10.0 / 24.0), self.index())
        older = self.checkpoint(2.0 / 24.0)
        nearer = self.checkpoint(4.0 / 24.0)

        action = state.observe(
            observation(5.0 / 24.0),
            self.index(older, nearer),
        )

        self.assertEqual(action.kind, TimelineActionKind.RESTORE_AND_REPLAY)
        self.assertEqual(action.checkpoint, nearer)
        self.assertEqual(len(action.fixed_times), 5)

    def test_unchanged_time_is_noop(self):
        state = self.state()
        state.observe(observation(1.0), self.index())

        action = state.observe(observation(1.0), self.index())

        self.assertEqual(action.kind, TimelineActionKind.NO_OP)
        self.assertEqual(action.fixed_times, ())

    def test_incompatible_checkpoint_resets_from_start(self):
        state = self.state()
        state.observe(observation(0.0), self.index())
        invalid = self.checkpoint(0.25, configuration_hash="world-b")

        action = state.observe(observation(0.5), self.index(invalid))

        self.assertEqual(action.kind, TimelineActionKind.RESET)
        self.assertIsNone(action.checkpoint)
        self.assertEqual(len(action.fixed_times), 60)

    def test_unaligned_checkpoint_cannot_create_a_non_fixed_replay_delta(self):
        state = self.state()
        state.observe(observation(0.0), self.index())
        unaligned = self.checkpoint(0.01)

        action = state.observe(observation(0.5), self.index(unaligned))

        self.assertEqual(action.kind, TimelineActionKind.RESET)
        self.assertIsNone(action.checkpoint)
        self.assertTrue(all(
            abs(time / DT - round(time / DT)) < 1.0e-9
            for time in action.fixed_times
        ))

    def test_changed_fps_resets_even_at_a_forward_time(self):
        state = self.state()
        state.observe(observation(0.0, fps=24.0), self.index())
        state.observe(observation(1.0 / 24.0, fps=24.0), self.index())

        action = state.observe(observation(2.0 / 30.0, fps=30.0), self.index())

        self.assertEqual(action.kind, TimelineActionKind.RESET)
        self.assertIn("FPS", action.reason)

    def test_approximate_preview_is_never_authoritative(self):
        state = self.state()
        state.observe(observation(0.0, approximate=True), self.index())

        action = state.observe(
            observation(1.0 / 24.0, approximate=True),
            self.index(),
        )

        self.assertEqual(action.kind, TimelineActionKind.ADVANCE)
        self.assertFalse(action.authoritative)


if __name__ == "__main__":
    unittest.main()
