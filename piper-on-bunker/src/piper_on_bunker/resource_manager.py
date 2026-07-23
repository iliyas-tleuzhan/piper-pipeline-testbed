from __future__ import annotations

from dataclasses import dataclass


@dataclass
class Lease:
    owner: str
    resource: str


class ResourceManager:
    def __init__(self) -> None:
        self._leases: dict[str, Lease] = {}

    def acquire(self, resource: str, owner: str) -> bool:
        existing = self._leases.get(resource)
        if existing and existing.owner != owner:
            return False
        self._leases[resource] = Lease(owner=owner, resource=resource)
        return True

    def release(self, resource: str, owner: str) -> bool:
        existing = self._leases.get(resource)
        if not existing:
            return True
        if existing.owner != owner:
            return False
        del self._leases[resource]
        return True

    def owner(self, resource: str) -> str | None:
        lease = self._leases.get(resource)
        return lease.owner if lease else None
