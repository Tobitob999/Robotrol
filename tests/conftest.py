import pytest
from pathlib import Path

@pytest.fixture
def base_dir():
    return Path(__file__).parent.parent

@pytest.fixture
def eb300_profile(base_dir):
    import json
    with open(base_dir / "profiles" / "EB300.json") as f:
        return json.load(f)

@pytest.fixture
def moveo_profile(base_dir):
    import json
    with open(base_dir / "profiles" / "Moveo.json") as f:
        return json.load(f)
