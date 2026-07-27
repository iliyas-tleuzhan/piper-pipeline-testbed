from piper_on_bunker.openclaw.semantic_plan import SemanticSkill
from piper_on_bunker.openclaw.semantic_plan import next_phase_allowed
from piper_on_bunker.openclaw.semantic_plan import plan_manipulation_task
from piper_on_bunker.openclaw.semantic_plan import validate_phase_sequence


def test_cup_task_becomes_semantic_phases():
    plan = plan_manipulation_task("Move the red cup onto the paper.")
    assert [phase.skill for phase in plan.phases] == [
        SemanticSkill.APPROACH_OBJECT,
        SemanticSkill.GRASP_OBJECT,
        SemanticSkill.TRANSPORT_OBJECT,
        SemanticSkill.RELEASE_OBJECT,
    ]
    assert plan.phases[0].target == "red cup"
    assert plan.phases[2].destination == "paper"
    validate_phase_sequence(plan.phases)


def test_transport_requires_grasp_confirmation():
    plan = plan_manipulation_task("Move the red cup onto the paper.")
    transport = plan.phases[2]
    assert not next_phase_allowed(transport, {"approach_complete"})
    assert next_phase_allowed(transport, {"approach_complete", "grasp_confirmed"})

