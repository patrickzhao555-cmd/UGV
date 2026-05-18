# Competition Mission V2 Cleanup / Deprecation Report

CompetitionMissionV2 is now the authoritative mission layer for `round1`,
`round2`, and `round3` when `--competition-mission-v2-enabled true`.

Legacy code intentionally retained for fallback:

- `CompetitionMission._update_straight_round`: old round1/round2 straight-goal behavior. V2 replaces this with round1 goal extension and round2 sweep coverage.
- `CompetitionMission._update_round3`: old `startup_to_center` / `search_expand` behavior. V2 replaces this with obstacle-aware sweep coverage.
- `CompetitionMission._search_goal`: old center-expanding search pattern. No longer used by round3 while V2 is enabled.
- `CompetitionMission._loiter_goal`: old target loiter behavior. V2 stops on target/landing/complete criteria by default instead of loitering for competition rounds.
- `CompetitionMission._update_indoor`, `_indoor_search_goal`, `_choose_indoor_goal`, and `_mark_indoor_visited`: retained for indoor mode only. Indoor remains a non-competition/deprecated behavior path.
- Legacy constructor fields such as `center_loiter_radius_m`, `search_radius_step_m`, and `waypoint_switch_radius_m`: still used by legacy/indoor fallback, but not by V2 competition rounds.

Deletion is deferred so `--competition-mission-v2-enabled false` can still run
the previous mission behavior for comparison and emergency fallback.
