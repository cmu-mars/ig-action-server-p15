# flake8: noqa
from ig_server.instructions import common, sei
from ig_server.ig_evaluator_lark import IGEvaluator

from ig_server.ros_wrappers import PortedNode

GOOD_INSTRUCTION_GRAPH = "P(V(0, do MoveAbs(10, 10, 0.5) then 1), V(1, do DispenseMedicine() then 2)::V(2, end)::nil)"
DUPLICATE_LABEL = "P(V(0, do MoveAbs(10, 10, 0.5) then 1), V(1, do DispenseMedicine() then 1)::V(" \
                 "1, end)::nil)"
MISSING_LABEL = "P(V(0, do MoveAbs(10, 10, 0.5) then 1), V(1, do DispenseMedicine() then 3)::V(2, end)::nil)"
MISSING_ACTION = "P(V(0, do MoveAbsolute(10, 10, 0.5) then 1), V(1, do DispenseMedicine() then " \
                "2)::V(2, end)::nil)"
WRONG_PARAM_COUNT = "P(V(0, do MoveAbs(10, 10) then 1), V(1, do DispenseMedicine() then 2)::V(2, end)::nil)"
WRONG_PARAM_TYPE = 'P(V(0, do MoveAbs(10, 10, "hello") then 1), V(1, do DispenseMedicine() then " \
                   "2)::V(2, end)::nil)'


def get_evaluator(inst):
    return IGEvaluator(None, inst, modules=[common, sei])


def test_ok():
    ig_eval = get_evaluator(GOOD_INSTRUCTION_GRAPH)
    parsed = ig_eval.parse()
    assert parsed


def test_duplicate():
    ig_eval = get_evaluator(DUPLICATE_LABEL)
    assert not ig_eval.parse()
