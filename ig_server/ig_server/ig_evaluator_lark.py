import inspect
import traceback
from typing import Dict, List

import attr
from lark import Transformer, v_args, Visitor, Tree, Lark
from lark.visitors import Interpreter
from lark.exceptions import VisitError

from ig_server.exceptions import IGException
from ig_server.abstract_instruction import AbstractInstruction
from ig_server.ros_wrappers import PortedNode


IGGrammar = IGGrammar = '''
    program: "P" "(" vertex "," vertices ")"
    vertices: "nil" -> nil
            | vertex "::" vertices
    vertex: "V" "(" label "," content ")"
    content: "do" action "then" label -> dothen
           | "do" action "until" action "then" label -> dountil
           | "if" action "then" label "else" label -> if_
           | "goto" label -> goto
           | "end" -> end
    action: NAME params
    params: "(" [param ("," param)*] ")"
    ?param: string
         | number
    string: ESCAPED_STRING
    number: SIGNED_NUMBER
    label: INT

    %import common.CNAME -> NAME
    %import common.INT -> INT
    %import common.SIGNED_NUMBER
    %import common.WS
    %import common.ESCAPED_STRING
    %ignore WS
'''


@attr.s(slots=True)
class IGTypeChecker(Visitor):
    actions: Dict[str, AbstractInstruction] = attr.ib()
    labels: List[int] = attr.ib()

    _errors: List[str] = []
    _has_end: bool = False

    def typecheck(self, tree):
        self._errors = []
        self._has_end = False
        self.visit(tree)
        if not self._has_end:
            self._errors.append('The instruction graph does not have an end vertex')
        return len(self._errors) == 0

    def errors(self):
        return self._errors

    def dothen(self, tree):
        if tree.children[1] not in self.labels:
            self._errors.append(f'do .. then refers to unknown label: {tree.children[1]}')

    def dountil(self, tree):
        if tree.children[2] not in self.labels:
            self._errors.append(f'do .. then refers to unknown label: {tree.children[2]}')

    def if_(self, tree):  # cnd, tLabel, fLabel):
        if tree.children[1] not in self.labels:
            self._errors.append(f'if else refers to unknown label: {tree.children[1]}')
        if tree.children[2] not in self.labels:
            self._errors.append(f'if else refers to unknown label: {tree.children[2]}')

    def goto(self, tree):
        if tree.children[0] not in self.labels:
            self._errors.append(f'goto refers to unknown label: {tree.children[0]}')

    def end(self, tree):
        self._has_end = True


@attr.s(slots=True, frozen=True)
class TransformData(Transformer):
    operations: Dict[str, AbstractInstruction] = attr.ib()
    node: PortedNode = attr.ib()

    @v_args(inline=True)
    def action(self, name: str, params: List[any]) -> AbstractInstruction:
        """
        Replace the action node in the AST to an AbstractInstruction that matches the name.

        :param name: the name of the action in the instruction graph
        :param params: the parameters (which have been transformed into a list by this)
        :return: the AbstractInstruction that can be called to construct this
        :raises: Exception if the operation is not in self.operations or the parameters are of the
                 wrong number or type
        """
        if name not in self.operations:
            raise IGException(f'No operation "{name}"')
        cls = self.operations[name]
        return cls.load_from_params(self.node, params)

    @v_args(inline=True)
    def string(self, s) -> str:
        """
        Replace the string node in the AST with a string value.

        :param s: The string node
        :return: the str representation
        """
        return s[1:-1].replace('\\"', '"')

    params = list
    label = v_args(inline=True)(int)

    @v_args(inline=True)
    def number(self, n):
        """
        Replace the number in the AST with the float or int representation.

        :param n: The number
        :return:
        """
        return int(n) if '.' not in n else float(n)


class VertexState:
    """The state of execution of a vertex kept as the IG executes recording the result."""

    result: bool = False
    vertex = None

    def __init__(self, vertex, result):
        self.result = result
        self.vertex = vertex


class VertexInterpreter(Interpreter):
    """Interprets the instruction graph."""

    _next_vertex = None
    _current_vertex = None
    _execution_state: List[VertexState] = []
    _success: bool = True

    def __init__(self, vertices: Dict[str, Tree],
                 check_canceled=None, feedback=None):
        self._vertices = vertices

        def default_feedback(msg: str):
            print(msg)

        def default_cc() -> bool:
            return False

        self.feedback = feedback if feedback is not None else default_feedback
        self.check_canceled = check_canceled if check_canceled is not None else default_cc

    def next_vertex(self):
        return self._next_vertex

    def success(self):
        return self._success

    def _update_state(self, result: bool):
        self._execution_state.append(VertexState(self._current_vertex, result))

    def _update_next_vertex(self, label: int):
        if label is None or label == -1:
            self._next_vertex = None
        else:
            self._next_vertex = self._vertices[label]

    @v_args(inline=True)
    def dothen(self, action: AbstractInstruction, label: int):
        result = self.do_action(action)
        self._update_state(result)
        if result:
            self._update_next_vertex(label)
        else:
            self._update_next_vertex(-1)

    @v_args(inline=True)
    def dountil(self, action, cnd, label):
        go = True
        result = True
        while go:
            result = self.do_action(action)
            self._update_state(result)
            go = result and cnd.evaluate() and not self.check_canceled()

        if self.check_canceled():
            self._update_next_vertex(-1)
        else:
            if result:
                self._update_next_vertex(label)
            else:
                self._update_next_vertex(-1)

    @v_args(inline=True)
    def if_(self, action, t_label, f_label):
        result = self.do_condition(action)
        self._update_state(True)
        if result:
            self._update_next_vertex(t_label)
        else:
            self._update_next_vertex(f_label)

    @v_args(inline=True)
    def goto(self, label):
        self._update_next_vertex(label)
        self._update_state(True)

    def end(self, tree):
        self._update_next_vertex(-1)
        self._update_state(True)

    def vertex(self, tree):
        self._current_vertex = tree
        self.visit_children(tree)

    def do_action(self, action: AbstractInstruction) -> bool:
        label = self._current_vertex.children[0]
        str_rep = action.to_pretty_string()
        self.feedback(f'{label}:{str_rep : START}')
        result = action.execute()
        self.feedback(
            f'{label}:{str_rep : {"SUCCESS" if result else "FAILED"}}')

    def do_condition(self, action: AbstractInstruction):
        return action.execute()


class IGEvaluator:
    _node: PortedNode
    _actions: Dict[str, AbstractInstruction]
    _errors: List[str]

    def __init__(self,
                 node: PortedNode,
                 instructions: str,
                 check_canceled=None,
                 feedback=None,
                 modules=[]):
        def default_feedback(msg: str):
            print(msg)

        def default_cc() -> bool:
            return False

        self.feedback = feedback if feedback is not None else default_feedback
        self.check_canceled = check_canceled if check_canceled is not None else default_cc

        self._instructions = instructions
        self._node = node
        self._parser = Lark(IGGrammar, start='program')

        self._actions = self.load_actions(modules)
        self._errors = []

        self._ast = None

    @staticmethod
    def load_actions(modules) -> Dict[str, AbstractInstruction]:
        actions: Dict[str, AbstractInstruction] = {}

        for module in modules:
            for name, obj in inspect.getmembers(module):
                if inspect.isclass(obj) and issubclass(obj, AbstractInstruction):
                    actions[obj.__name__] = obj
        print(f"Loaded operations for {','.join(c.__name__ for c in actions.values())}")

        return actions

    def error(self, err: str):
        self.feedback(err)
        self._errors.append(err)

    def errors(self):
        return self._errors

    def parse(self):
        self.feedback("Parsing instruction graph")
        try:
            self._ast = self._parser.parse(self._instructions)
        except Exception as e:
            self.error(f'Parsing instruction graph failed: {e}')
            traceback.print_exc()
            return False

        self.feedback("Validating instructions")
        rep_data = TransformData(node=self._node, operations=self._actions)
        try:
            self._ast = rep_data.transform(self._ast)
        except IGException as e:
            self.error(str(e))
            return False
        except VisitError as ve:
            self.error(str(ve))
            return False

        # Collect all the valid labels and check for duplicates
        raw_labels = [v.children[0] for v in self._ast.find_data('vertex')]
        labels = set(raw_labels)
        duplicates = set([x for x in raw_labels if raw_labels.count(x) > 1])
        if len(duplicates) != 0:
            dup_nodes = ",".join([str(i) for i in duplicates])
            self.error(
                f'Instruction graph is not well formed: has duplicate labels: {dup_nodes}')
            return False

        # Typecheck the tree
        typechecker = IGTypeChecker(labels=labels, actions=self._actions)
        if not typechecker.typecheck(self._ast):
            errors = "\n  ".join(typechecker.errors())
            self.error(f'The instruction graph failed to typecheck: \n  {errors}')
            return False

        return True

    def eval_instructions(self):
        vertices = {v.children[0]: v for v in self._ast.find_data('vertex')}
        interpreter = VertexInterpreter(self._node, vertices)
        current_vertex = self._ast.children[0]
        while current_vertex is not None and not self.check_canceled():
            interpreter.visit(current_vertex)
            current_vertex = interpreter.next_vertex()

        if self.check_canceled:
            self.feedback("Instruction evaluation canceled")


if __name__ == '__main__':
    eval = IGEvaluator(None,
                       "P(V(0, do MoveAbs(10, 10, 0.5) then 1), "
                       "V(1, do DispenseMedicine() then 2)::V(2, end)::nil)")
    eval.parse()
