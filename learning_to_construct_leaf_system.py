import numpy as np
from pydrake.common.containers import namedview
from pydrake.common.value import AbstractValue
from pydrake.math import RigidTransform, RotationMatrix
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import BasicVector, LeafSystem
from pydrake.trajectories import PiecewisePolynomial


class MyAdder(LeafSystem):
    def __init__(self):
        super().__init__()
        self._a_port = self.DeclareVectorInputPort(name="a", size=2)
        self._b_port = self.DeclareVectorInputPort(name="b", size=2)
        self.DeclareVectorOutputPort(name="sum", size=2, calc=self.CalcSum)
        self.DeclareVectorOutputPort(name="difference",
                                     size=2,
                                     calc=self.CalcDifference)

    def CalcSum(self, context, output):
        a = self._a_port.Eval(context)
        b = self._b_port.Eval(context)
        output.SetFromVector(a + b)

    def CalcDifference(self, context, output):
        a = self._a_port.Eval(context)
        b = self._b_port.Eval(context)
        output.SetFromVector(a - b)

system = MyAdder()
context = system.CreateDefaultContext()

system.GetInputPort("a").FixValue(context, [3, 4])
system.GetInputPort("b").FixValue(context, [1, 2])

print(f'sum:{system.GetOutputPort("sum").Eval(context)}')
print(f'difference:{system.GetOutputPort("difference").Eval(context)}')


