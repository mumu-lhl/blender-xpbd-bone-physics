from .common import ScenarioFixture


def graded_tail() -> ScenarioFixture:
    return ScenarioFixture("graded_tail", body_count=16, chain_count=1)


def single_pendulum() -> ScenarioFixture:
    return ScenarioFixture("single_pendulum", body_count=2, chain_count=1)
