from piper_on_bunker.data.piper_openpi_dataset import make_synthetic_episode
from piper_on_bunker.data.piper_openpi_dataset import validate_episode
from piper_on_bunker.data.piper_openpi_dataset import write_synthetic_lerobot_like_dataset


def test_synthetic_episode_validates_and_converts(tmp_path):
    frames = make_synthetic_episode()
    validate_episode(frames)
    output = write_synthetic_lerobot_like_dataset(tmp_path / "dataset", frames)
    assert (output / "features.json").exists()
    assert (output / "episode_000000.json").exists()

