from piper_on_bunker.data.dataset_validator import create_synthetic_episode, summarize_dataset, validate_episode_directory


def test_synthetic_episode_validates(tmp_path):
    episode_dir = create_synthetic_episode(tmp_path / "dataset")
    result = validate_episode_directory(episode_dir)
    assert result.ok
    assert result.frame_count == 3


def test_summary_reports_invalid_episode(tmp_path):
    dataset_root = tmp_path / "dataset"
    episode_dir = create_synthetic_episode(dataset_root)
    (episode_dir / "frames" / "frame_000001.jpg").unlink()
    summary = summarize_dataset(dataset_root)
    assert summary.invalid_episodes == 1
    assert any("missing image" in issue for issue in summary.issues)
