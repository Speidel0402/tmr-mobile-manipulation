from rim_grasp_perception.selection import select_best_result
from rim_grasp_perception.types import GraspResult


def _result(object_id, valid, detection, geometry, depth=0.0, edge=0.0):
    return GraspResult(
        timestamp=1.0, frame_id="camera", camera_id="left", object_id=object_id,
        category="bowl", valid=valid, invalid_reason="" if valid else "rejected",
        detection_score=detection, geometry_score=geometry,
        diagnostics={"rim_depth_support": depth, "edge_support": edge})


def test_invalid_object_is_never_selected():
    bad = _result("bad", False, 1.0, 1.0, 1.0, 1.0)
    good = _result("good", True, 0.5, 0.5, 0.5, 0.5)
    assert select_best_result([bad, good]) is good
    assert good.diagnostics["selected"]
    assert not bad.diagnostics["selected"]


def test_geometry_quality_dominates_detection_tie_tradeoff():
    strong_geometry = _result("geometry", True, 0.70, 0.95, 0.9, 0.9)
    strong_detection = _result("detection", True, 0.99, 0.50, 0.9, 0.9)
    assert select_best_result([strong_detection, strong_geometry]) is strong_geometry
