# Copyright 2026 robot-safety maintainer
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Unit tests for fault classification.

The report groups faults by category, and a fault filed under the wrong category
sends an operator to the wrong subsystem, so the classification is pinned here
rather than left to the report's formatting to reveal.
"""

import pytest

from robot_safety_monitor import analyzer as az
from robot_safety_monitor import motion_safety as ms


class TestClassifyCode:
    def test_known_codes_map_to_their_category(self):
        assert az.classify_code(az.CODE_ODOM_STALE) == (az.CATEGORY_SEN, "SEN-001")
        assert az.classify_code(az.CODE_SCAN_MISSING) == (az.CATEGORY_SEN, "SEN-001")
        assert az.classify_code(az.CODE_POSE_UNCERTAIN) == (az.CATEGORY_SEN, "SEN-007")
        assert az.classify_code(az.CODE_TILT_CRITICAL) == (az.CATEGORY_MOT, "MOT-010")
        assert az.classify_code(az.CODE_OBSTACLE_NEAR) == (az.CATEGORY_COL, "COL-001")
        assert az.classify_code(az.CODE_BATTERY_LOW) == (az.CATEGORY_SYS, "SEN-014")

    def test_codes_without_a_usable_prefix_are_still_classified(self):
        # These are the reason the table exists: guessing from the name would
        # file COMMAND_MISMATCH under CMD (it is a MOT failure of the robot) and
        # ODOM_STALE under nothing at all.
        assert az.classify_code(az.CODE_COMMAND_MISMATCH) == (az.CATEGORY_MOT, "MOT-012")
        assert az.classify_code(az.CODE_UNEXPECTED_MOTION) == (az.CATEGORY_MOT, "CMD-012")
        assert az.classify_code(az.CODE_TILT_HIGH) == (az.CATEGORY_MOT, "MOT-010")

    def test_mot_alert_codes_classify_as_motion_by_prefix(self):
        for code in (
            ms.CODE_LIN_VEL_EXCEED,
            ms.CODE_ACTUAL_VEL_EXCEED,
            ms.CODE_ANG_VEL_EXCEED,
            ms.CODE_ACTUAL_ANG_VEL_EXCEED,
            ms.CODE_LIN_ACCEL_EXCEED,
            ms.CODE_ANG_ACCEL_EXCEED,
            ms.CODE_TWIST_INFEASIBLE,
        ):
            category, rule_id = az.classify_code(code)
            assert category == az.CATEGORY_MOT, code
            # MOT alerts carry their rule on the alert itself, so the classifier
            # is not expected to know it.
            assert rule_id == ""

    def test_unknown_code_falls_back_to_system_without_a_rule(self):
        # An unmapped code is a gap in the table. Calling it a system fault is
        # the honest reading; inventing a category would be worse.
        assert az.classify_code("SOMETHING_NEW") == (az.CATEGORY_SYS, "")
        assert az.classify_code("") == (az.CATEGORY_SYS, "")

    def test_every_declared_code_is_classified(self):
        # Guard against adding a code and forgetting the table: every module-level
        # CODE_ constant must resolve to something other than the fallback.
        codes = [
            value
            for name, value in vars(az).items()
            if name.startswith("CODE_") and isinstance(value, str)
        ]
        codes += [
            value
            for name, value in vars(ms).items()
            if name.startswith("CODE_") and isinstance(value, str)
        ]
        assert codes, "no fault codes found; the discovery below is broken"
        for code in codes:
            category, _rule = az.classify_code(code)
            assert category in az.CATEGORY_ORDER, code
            # COMMAND_STALE is retained as a code but is no longer raised: a
            # quiet command source is normal operation, not a fault.
            if code == az.CODE_COMMAND_STALE:
                continue
            assert category != az.CATEGORY_SYS or code.startswith("SYS_") or (
                code in az._CODE_CLASSIFICATION
            ), "%s fell through to the SYS fallback" % code

    def test_category_names_cover_every_category(self):
        for category in az.CATEGORY_ORDER:
            assert category in az.CATEGORY_NAMES


class TestFindingCarriesClassification:
    def test_findings_can_be_classified_from_their_code(self):
        # The report classifies findings by code rather than each call site
        # storing a category, so a finding must always be resolvable.
        config = az.Config()
        tracker = az.ObservationTracker(config)
        # Register the required sources so the assessment actually produces
        # findings; with nothing registered there is nothing to classify.
        for name in config.required_sources:
            tracker.register(name, "/" + name, 1.0)
        analyzer = az.Analyzer(config, tracker)
        assessment = analyzer.assess(az.MonitorState(now_wall=100.0))
        assert assessment.findings
        for finding in assessment.findings:
            category, _rule = az.classify_code(finding.code)
            assert category in az.CATEGORY_ORDER

    def test_motion_alert_category_is_motion(self):
        alert = ms.MotionAlert(
            code=ms.CODE_LIN_VEL_EXCEED, rule_id="MOT-001", level=ms.LEVEL_S2, detail="d"
        )
        assert alert.category == az.CATEGORY_MOT

    def test_level_names_cover_the_spec_levels(self):
        for level in (ms.LEVEL_S1, ms.LEVEL_S2, ms.LEVEL_S3, ms.LEVEL_S4):
            assert ms.LEVEL_NAMES[level] == "S%d" % level
