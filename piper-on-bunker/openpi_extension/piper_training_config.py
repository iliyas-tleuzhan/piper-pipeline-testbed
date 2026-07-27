from __future__ import annotations

import dataclasses

from openpi import transforms as _transforms
from openpi.models import pi0_config
from openpi.training import config as _config
from openpi.training import optimizer as _optimizer
from openpi.training import weight_loaders

from piper_policy import PiperInputs
from piper_policy import PiperOutputs


@dataclasses.dataclass(frozen=True)
class LeRobotPiperDataConfig(_config.DataConfigFactory):
    repo_id: str = "iliyas-tleuzhan/piper_openpi_single_arm"
    assets: _config.AssetsConfig = dataclasses.field(default_factory=_config.AssetsConfig)

    def create(self, assets_dirs, model_config):
        base = dataclasses.replace(self.create_base_config(assets_dirs, model_config), asset_id="piper_single_arm_v1")
        return dataclasses.replace(
            base,
            data_transforms=_transforms.Group(
                inputs=[PiperInputs(model_config.model_type)],
                outputs=[PiperOutputs()],
            ),
            model_transforms=_config.ModelTransformFactory()(model_config),
            use_quantile_norm=True,
            prompt_from_task=True,
        )


PIPER_PI05_LOW_MEM_CONFIG = _config.TrainConfig(
    name="pi05_piper_single_arm_low_mem",
    model=pi0_config.Pi0Config(
        pi05=True,
        action_dim=7,
        action_horizon=10,
        discrete_state_input=False,
        paligemma_variant="gemma_2b_lora",
        action_expert_variant="gemma_300m_lora",
    ),
    data=LeRobotPiperDataConfig(),
    weight_loader=weight_loaders.CheckpointWeightLoader("gs://openpi-assets/checkpoints/pi05_base/params"),
    batch_size=32,
    num_train_steps=30_000,
    optimizer=_optimizer.AdamW(clip_gradient_norm=1.0),
    freeze_filter=pi0_config.Pi0Config(
        pi05=True,
        action_dim=7,
        action_horizon=10,
        discrete_state_input=False,
        paligemma_variant="gemma_2b_lora",
        action_expert_variant="gemma_300m_lora",
    ).get_freeze_filter(),
    ema_decay=None,
    policy_metadata={
        "openpi_commit": "15a9616a00943ada6c20a0f158e3adb39df2ccac",
        "base_checkpoint": "gs://openpi-assets/checkpoints/pi05_base",
        "action_semantics": "absolute_piper_joint_targets",
        "joint_names": ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper"],
        "control_frequency_hz": 20.0,
        "piper_compatible": False,
    },
)

PIPER_PI05_FULL_CONFIG = dataclasses.replace(
    PIPER_PI05_LOW_MEM_CONFIG,
    name="pi05_piper_single_arm_full",
    model=pi0_config.Pi0Config(pi05=True, action_dim=7, action_horizon=10, discrete_state_input=False),
    batch_size=64,
    freeze_filter=None,
    ema_decay=0.999,
)
