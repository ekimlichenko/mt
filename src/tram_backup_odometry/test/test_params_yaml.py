"""config/params.yaml must mirror the Params dataclass (regenerate with tools/gen_params_yaml.py)."""
import os
import sys
from dataclasses import fields

import yaml

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PKG_DIR)
from tram_backup_odometry.core.config import Params  # noqa: E402

PARAMS_YAML = os.path.join(PKG_DIR, 'config', 'params.yaml')


def test_params_yaml_has_every_field_with_the_default_value_and_type():
    with open(PARAMS_YAML) as f:
        d = yaml.safe_load(f)['backup_odometry']['ros__parameters']
    defaults = Params()
    names = [f.name for f in fields(Params)]
    assert sorted(d) == sorted(names), 'params.yaml out of date: run tools/gen_params_yaml.py'
    for n in names:
        want, got = getattr(defaults, n), d[n]
        assert type(got) is type(want) and got == want, (n, got, want)


def test_params_yaml_loads_through_params_from_yaml():
    assert Params.from_yaml(PARAMS_YAML) == Params()
