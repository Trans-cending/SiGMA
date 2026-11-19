"""Synchronous multi-modal sensing using official CARLA 0.9.16."""

def run_stage2(config, output_dir, seed=0):
    from .runner import run_stage2 as run
    return run(config, output_dir, seed)


def validate_stage2_config(config):
    from .config import validate_stage2_config as validate
    return validate(config)
