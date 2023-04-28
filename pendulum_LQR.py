import os
import pydot
import numpy as np
import matplotlib.pyplot as plt
from pydrake.multibody.meshcat import ContactVisualizer, ContactVisualizerParams
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import MultibodyPlant, AddMultibodyPlantSceneGraph
from pydrake.systems.primitives import SymbolicVectorSystem
from pydrake.systems.framework import LeafSystem
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.primitives import LogVectorOutput
from pydrake.systems.controllers import LinearQuadraticRegulator
from pydrake.systems.primitives import ConstantVectorSource
from pydrake.common import temp_directory
from pydrake.geometry import (
    MeshcatVisualizer,
    MeshcatVisualizerParams,
    Role,
    StartMeshcat,
    SceneGraph,
)
from pydrake.math import RigidTransform, RollPitchYaw
from pydrake.visualization import ModelVisualizer

model_sdf_file_path = './cartpole.sdf'

'''
SymbolicVectorSystem is a simplified description of a dynamic system
'''
# x = Variable("x")
# continuous_vector_system = SymbolicVectorSystem(state=[x], dynamics=[-x + x ** 3], output=[x])

'''
we can have a more complicated dynamic system from deriving from leaf system
'''


class SimpleContinuousTimeSystem(LeafSystem):
    def __init__(self):
        LeafSystem.__init__(self)

        # declare that there is only one state in the continuous dynamic system
        state_index = self.DeclareContinuousState(1)
        # declare the output port name and input the state_index
        # To demonstrate the state
        self.DeclareStateOutputPort("y", state_index)

    def DoCalcTimeDerivatives(self, context, derivatives):
        x = context.get_continuous_state_vector().GetAtIndex(0)
        xdot = -x + x ** 3
        derivatives.get_mutable_vector().SetAtIndex(0, xdot)


class LQRControlSystem(LeafSystem):
    def __init__(self):
        LeafSystem.__init__(self)
        state_index = self.DeclareContinuousState(2)
        self.DeclareStateOutputPort("x", state_index)

        self.A = np.array([[0, 1],
                           [10, 0]])
        self.B = np.array([[0], [-1]])
        self.Q = np.eye(2)
        self.R = np.array([100])
        K, S = LinearQuadraticRegulator(self.A, self.B, self.Q, self.R)
        self.K = K

    def DoCalcTimeDerivatives(self, context, derivatives):
        x = context.get_continuous_state_vector().GetAtIndex(0)
        u = -self.K @ x
        xdot = self.A @ x + self.B @ u
        derivatives.get_mutable_vector().SetAtIndex(0, xdot)


if __name__ == "__main__":
    meshcat = StartMeshcat()
    # test_mode = True if "TEST_SRCDIR" in os.environ else False

    # set the simulation time step to 0.001 seconds
    sim_time_step = 0.001

    builder = DiagramBuilder()
    # Add robot multibody plant and scene graph
    plant, scene_graph = AddMultibodyPlantSceneGraph(
        builder, time_step=sim_time_step
    )

    parser = Parser(plant)
    parser.AddModelFromFile(model_sdf_file_path, "sdf")
    plant.Finalize()
    assert plant.geometry_source_is_registered()

    # controller = builder.AddSystem(LQRControlSystem())
    # controller = builder.AddSystem(ConstantVectorSource(np.zeros(2)))
    controller = builder.AddSystem(ConstantVectorSource(np.array([0, 0])))
    controller.set_name("controller")

    logger = LogVectorOutput(controller.get_output_port(0), builder)
    logger.set_name("logger")

    # connect the subsystems
    builder.Connect(controller.get_output_port(), plant.get_actuation_input_port())
    builder.ExportOutput(plant.get_state_output_port())
    # define the visualizer
    visualizer = MeshcatVisualizer.AddToBuilder(
        builder, scene_graph, meshcat, MeshcatVisualizerParams(role=Role.kPerception, prefix="visual")
    )
    diagram = builder.Build()
    # To visual the whole system connectivity between subsystems
    graph = pydot.graph_from_dot_data(diagram.GetGraphvizString(max_depth=2))[0]
    graph.write('./system_graph_output.png', format='png')

    # Start to construct simulation process
    diagram_context = diagram.CreateDefaultContext()
    plant_content = diagram.GetMutableSubsystemContext(plant, diagram_context)
    input_port_index = plant.get_actuation_input_port().get_index()

    simulator = Simulator(diagram)
    simulator.Initialize()
    simulator.set_target_realtime_rate(1.)
    simulator.AdvanceTo(5.0)

'''
# create a simple block diagram containing our system
builder = DiagramBuilder()
system = builder.AddSystem(SimpleContinuousTimeSystem())
logger = LogVectorOutput(system.get_output_port(0), builder)
diagram = builder.Build()

# Set the initial conditions
context = diagram.CreateDefaultContext()
context.SetContinuousState([0.5])

simulator = Simulator(diagram, context)
# set_target_realtime_rate function can set the simulation time to several times of the real world time
# simulator.set_target_realtime_rate(2)
simulator.AdvanceTo(10)

log = logger.FindLog(context)
plt.figure()
plt.plot(log.sample_times(), log.data().transpose())
plt.xlabel('t')
plt.ylabel('y(t)')
plt.show()

'''
