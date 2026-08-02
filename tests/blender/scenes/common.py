from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class ScenarioFixture:
    name: str
    body_count: int
    chain_count: int
    contact_required: bool = False
    expected_contacts: int = 0

    def build_batch(self) -> dict[str, object]:
        return {
            "scenario": self.name,
            "body_count": self.body_count,
            "chain_count": self.chain_count,
            "contact_required": self.contact_required,
        }
