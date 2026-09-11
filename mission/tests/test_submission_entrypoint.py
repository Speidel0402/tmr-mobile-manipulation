from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
ENTRYPOINT = (ROOT / "docker" / "run_task3.sh").read_text(encoding="utf-8")
README = (ROOT / "README.md").read_text(encoding="utf-8")
DOCKERFILE = (ROOT / "Dockerfile").read_text(encoding="utf-8")


def test_container_entrypoint_has_non_motion_checks() -> None:
    assert 'preflight|check|execute' in ENTRYPOINT
    assert '"physical_motion_commanded":false' in ENTRYPOINT
    assert "check_remote_runtime.sh" in ENTRYPOINT
    assert 'CMD ["--cup-letter", "B", "--bowl-letter", "A", "--plate-letter", "D"]' in DOCKERFILE
    assert "ssh_wrapper.sh" in DOCKERFILE


def test_container_entrypoint_does_not_require_private_arm_files() -> None:
    assert "/home/aup/tmr_env.sh" not in ENTRYPOINT
    assert "/home/aup/tmr-mobile-manipulation" not in ENTRYPOINT
    assert "load_arm_environment.sh" in ENTRYPOINT
    assert "base grasp mission tools docker" in ENTRYPOINT


def test_credentials_are_documented_as_evaluator_owned() -> None:
    assert "never\nstored in this repository or image" in README
    assert "pre-populated" in README and "known_hosts" in README
