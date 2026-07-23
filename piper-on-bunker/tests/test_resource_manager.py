from piper_on_bunker.resource_manager import ResourceManager


def test_resource_authority():
    rm = ResourceManager()
    assert rm.acquire("arm", "a")
    assert not rm.acquire("arm", "b")
    assert rm.release("arm", "a")
    assert rm.acquire("arm", "b")
