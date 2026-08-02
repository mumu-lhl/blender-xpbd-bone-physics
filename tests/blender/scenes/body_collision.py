from .common import ScenarioFixture


def fast_arm() -> ScenarioFixture:
    return ScenarioFixture("fast_arm", body_count=12, chain_count=1, contact_required=True, expected_contacts=3)


def initial_penetration() -> ScenarioFixture:
    return ScenarioFixture("initial_penetration", body_count=8, chain_count=1, contact_required=True, expected_contacts=2)


def deforming_body() -> ScenarioFixture:
    return ScenarioFixture("deforming_body", body_count=10, chain_count=1)


def breast_torso() -> ScenarioFixture:
    return ScenarioFixture("breast_torso", body_count=6, chain_count=1, contact_required=True, expected_contacts=1)
