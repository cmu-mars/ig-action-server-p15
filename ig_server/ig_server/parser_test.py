from typing import List, Dict
from abc import ABC, abstractmethod
import attr

from lark import Lark, Transformer, v_args, Visitor
from lark.visitors import Interpreter

from rclpy.node import Node


def check_args(expected, *received):
    if len(expected) != len(received):
        raise Exception(f"Must specify arguments: {','.join(expected)}")


@attr.s(slots=True)
class AbstractInstruction(ABC):
    node: Node = attr.ib()

    _is_active: bool = attr.ib(init=False, default=False)

    @classmethod
    def load_from_params(cls, params: List[any]):
        pass

    @abstractmethod
    def execute(self):
        pass

    @abstractmethod
    def cancel(self):
        pass

    @abstractmethod
    def to_pretty_string(self):
        pass

    def active(self):
        return self._is_active

    def activate(self):
        self._is_active = True

    def deactivate(self):
        self._is_active = False


@attr.s(frozen=True, slots=True)
class Move(AbstractInstruction):
    x: float = attr.ib()
    y: float = attr.ib()
    velocity: float = attr.ib()
    action: str = attr.ib()
    yaw: float = attr.ib(default=0.0)

    @classmethod
    def load_from_params(cls, params: List[any]):
        if len(params) < 4 or len(params) > 5:
            raise Exception(f'Move: found {len(params)} parameters, expecting 4 or 5')
        x = float(params[0])
        y = float(params[1])
        v = float(params[2])
        a = str(params[3])
        w = float(params[4]) if len(params) == 5 else 0.0
        return Move(node=None, #Change
                    x=x, y=y, velocity=v, action=a)

    def execute(self):
        print(f'Executing {self.to_pretty_string()}')

    def cancel(self):
        pass

    def to_pretty_string(self):
        return f'Move({self.x}, {self.y}, {self.velocity}, {self.action}, {self.yaw})'
    # def __init__(self, node: Node, x: float, y: float, v: float, action: str, w: float):
    #     super().__init__(node, x, y, v, action, str)
    #     self._x = x
    #     self._y = y
    #     self._velocity = v
    #     self._action = action
    #     self._w = w


@attr.s(slots=True, frozen=True)
class MoveAbs(Move):

    @classmethod
    def load_from_params(cls, params: List[any]):
        if len(params) != 3:
            raise Exception(f"Received {len(params)} parameters, expected 3")
        x = float(params[0])
        y = float(params[1])
        v = float(params[2])
        return MoveAbs(node=None, #Change
                       x=x, y=y, velocity=v, action="Absolute")

    def to_pretty_string(self):
        return f'MoveAbs({self.x}, {self.y}, {self.velocity})'


@attr.s(slots=True, frozen=True)
class DispenseMedicine(AbstractInstruction):

    @classmethod
    def load_from_params(cls, params: List[any]):
        if len(params) != 0:
            raise Exception(f'Receive {len(params)} parameters, expected 0')
        return DispenseMedicine(node=None) # change

    def execute(self):
        print(f'Executing {self.to_pretty_string()}')

    def cancel(self):
        pass

    def to_pretty_string(self):
        return f'DispenseMedicine()'


IGGrammar = '''
    program: "P" "(" vertex "," vertices ")"
    vertices: "nil" -> nil
            | vertex "::" vertices
    vertex: "V" "(" label "," content ")"
    content: "do" action "then" label -> dothen
           | "do" action "until" cnd "then" label -> dountil
           | "if" cnd "then" label "else" label -> if_
           | "goto" label -> goto
           | "end" -> end
    action: NAME params 
    params: "(" [param ("," param)*] ")"
    ?param: string 
         | number
    cnd: "visible" "(" string ")" 
       | "stop" "(" number "," string ")"
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


class TypeChecker(Visitor):

    def __init__(self, valid_labels, valid_actions):
        self._actions = valid_actions
        self._labels = valid_labels
        self._errors = []
        self._has_end = False

    def typecheck(self, tree):
        self._errors = []
        self._has_end = False
        self.visit(tree)
        if not self._has_end:
            self._errors.append('The instruction graph does not have a required "end" vertex')
        return len(self._errors) == 0

    def dothen(self, tree):
        if tree.children[1] not in self._labels:
            self._errors.append(f'do .. then refers to unknown label: {tree.children[1]}')

    def dountil(self, tree):
        if tree.children[2] not in self._labels:
            self._errors.append(f'do .. then refers to unknown label: {tree.children[2]}')

    def if_(self, tree):  # cnd, tLabel, fLabel):
        if tree.children[1] not in self._labels:
            self._errors.append(f'if else refers to unknown label: {tree.children[1]}')
        if tree.children[2] not in self._labels:
            self._errors.append(f'if else refers to unknown label: {tree.children[2]}')

    def goto(self, tree):
        if tree.children[0] not in self._labels:
            self._errors.append(f'goto refers to unknown label: {tree.children[0]}')

    def end(self, tree):
        self._has_end = True


@attr.s(slots=True, frozen=True)
class TransformData(Transformer):
    operations: Dict[str, AbstractInstruction] = attr.ib()

    @v_args(inline=True)
    def action(self, name, params):
        if name not in self.operations:
            raise Exception(f'No operation "{name}"')
        cls = self.operations[name]
        return cls.load_from_params(params)

    @v_args(inline=True)
    def string(self, s):
        return s[1:-1].replace('\\"', '"')

    params = list
    label = v_args(inline=True)(int)

    @v_args(inline=True)
    def number(self, n):
        return int(n) if '.' not in n else float(n)


class VertexState:
    result: bool = False
    vertex = None

    def __init__(self, vertex, result):
        self.result = result
        self.vertex = vertex


class VertexInterpreter(Interpreter):
    _next_vertex = None
    _current_vertex = None
    _execution_state: List[VertexState] = []

    def __init__(self,
                 actions, vertices,
                 check_canceled=None,
                 feedback=None):
        self._actions = actions
        self._vertices = vertices
        self._success = True

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

    @v_args(inline=True)
    def dothen(self, action: AbstractInstruction, label):
        result = self.doaction(action)
        self._execution_state.append(VertexState(self._current_vertex, result))
        if result:
            self._next_vertex = self._vertices[label]
        else:
            self._next_vertex = None

    @v_args(inline=True)
    def dountil(self, action, cnd, label):
        go = True
        result = True
        while go:
            result = self.doaction(action)
            self._execution_state.append(VertexState(self._current_vertex, result))
            go = result and cnd.evaluate()

        if not result:
            self._next_vertex = None
        else:
            self._next_vertex = self._vertices[label]

    @v_args(inline=True)
    def if_(self, cnd, tLabel, fLabel):
        result = cnd.evaluate()
        self._execution_state.append(VertexState(self._current_vertex, result))
        if result:
            self._next_vertex = self._vertices[tLabel]
        else:
            self._next_vertex = self._vertices[fLabel]

    @v_args(inline=True)
    def goto(self, label):
        self._next_vertex - self._vertices[label]
        self._execution_state.append(VertexState(self._current_vertex, True))

    def end(self, tree):
        self._next_vertex = None
        self._execution_state.append(VertexState(self._current_vertex, True))

    def vertex(self, tree):
        self._current_vertex = tree
        self.visit_children(tree)

    def doaction(self, action: AbstractInstruction) -> bool:
        self.feedback(f"Doing action {action.to_pretty_string()}")
        action.execute()
        return True


parser = Lark(IGGrammar, start='program')
operations = {DispenseMedicine.__name__: DispenseMedicine,
              MoveAbs.__name__: MoveAbs,
              Move.__name__: Move}

good = "P(V(0, do MoveAbs(10, 10, 0.5) then 1), V(1, do DispenseMedicine() then 2)::V(2, end)::nil)"
bad = "P(V(0, do MoveAbs(10, 10, 0.5) then 3), V(1, do DispenseMedicine() then 2)::V(2, end)::nil)"

gast = TransformData(operations=operations).transform(parser.parse(good))
bast = TransformData(operations=operations).transform(parser.parse(bad))

tc = TypeChecker(set([v.children[0] for v in gast.find_data('vertex')]), ["MoveAbs", "DispenseMedicine"])

tc.visit(gast)

tc.visit(bast)

interpreter = VertexInterpreter(None, {v.children[0]: v for v in gast.find_data('vertex')})
current_vertex = gast.children[0]
while current_vertex is not None:
    interpreter.visit(current_vertex)
    current_vertex = interpreter.next_vertex()
