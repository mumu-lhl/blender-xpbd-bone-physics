from .common import ScenarioFixture


def closed_skirt() -> ScenarioFixture:
    return ScenarioFixture("closed_skirt", body_count=30, chain_count=1)


def moving_legs() -> ScenarioFixture:
    return ScenarioFixture("moving_legs", body_count=30, chain_count=1, contact_required=True, expected_contacts=8)
