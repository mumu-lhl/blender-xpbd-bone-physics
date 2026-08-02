from .common import ScenarioFixture


def single_hair() -> ScenarioFixture:
    return ScenarioFixture("single_hair", body_count=8, chain_count=1)


def multiple_hair() -> ScenarioFixture:
    return ScenarioFixture("multiple_hair", body_count=24, chain_count=2, contact_required=True, expected_contacts=18)
