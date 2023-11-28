import pytest


@pytest.mark.copyright
@pytest.mark.linter
def test_simple():
    rc = "Hello World"
    assert rc == "Hello World"
