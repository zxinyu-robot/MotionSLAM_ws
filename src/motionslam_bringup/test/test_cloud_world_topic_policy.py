from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
PARAMETER = "lio.output.cloud_world_topic.enabled"


def _parameter_value(relative_path: str) -> str:
    for line in (PACKAGE_ROOT / relative_path).read_text(encoding="utf-8").splitlines():
        stripped = line.strip()
        if stripped.startswith(f"{PARAMETER}:"):
            return stripped.split(":", 1)[1].strip().lower()
    raise AssertionError(f"{PARAMETER} missing from {relative_path}")


def test_demo_mapping_enables_cloud_topic():
    assert _parameter_value("config/super_lio_mid360.yaml") == "true"
