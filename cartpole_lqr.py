import numpy as np
import matplotlib.pyplot as plt
import pydot
import os
import pydrake

from pydrake.geometry import (
    MeshcatVisualizer,
    MeshcatVisualizerParams,
    Role,
    StartMeshcat,
)
from pydrake.math import RigidTransform, RollPitchYaw
from pydrake.multibody.meshcat import JointSliders
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder, LeafSystem, PortDataType, BasicVector
from pydrake.systems.primitives import ConstantVectorSource, LogVectorOutput
from pydrake.systems.controllers import PidController
from pydrake.systems.controllers import LinearQuadraticRegulator
from pydrake.systems.drawing import plot_system_graphviz
from pydrake.systems.framework import GenerateHtml


cart_mass = 10.0
pole_mass = 1.0

class LQRController(LeafSystem):
    def __init__(self):
        LeafSystem.__init__(self)
        state_index = self.DeclareContinuousState(2)
        self.DeclareInputPort(
            name="state",
            size=4
        )
        self.DeclareStateOutputPort("x", state_index)

    def DoCalcTimeDerivatives(self, context, derivatives):
        x = context.get_state_vector().GetAtIndex(0)
        u = -self.K @ x


def create_scene(meshcat, model_location, sim_time_step):
    meshcat.Delete()
    meshcat.DeleteAddedControls()

    builder = DiagramBuilder()

    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=sim_time_step)
    parser = Parser(plant)

    parser.AddModelFromFile(model_location)

    plant.mutable_gravity_field().set_gravity_vector([0, 0, -9.8])
    plant.Finalize()
    # assert plant.geometry_source_is_registred()
    context = plant.CreateDefaultContext()
    # # plant state_vector stands for [x_cart, x_pole, theta_pole, theta_dot_pole]
    eps_ = np.pi / 3
    joint_0 = plant.GetJointByName('CartSlider')
    joint_0.set_default_translation(0.0)
    joint_1 = plant.GetJointByName('PolePin')
    joint_1.set_default_angle(np.pi - eps_)

    plant.get_actuation_input_port().FixValue(context, [0])
    context.get_mutable_continuous_state_vector().SetFromVector((0, np.pi , 0, 0))

    visualizer = MeshcatVisualizer.AddToBuilder(
        builder, scene_graph, meshcat, MeshcatVisualizerParams(role=Role.kPerception, prefix="visual")
    )


    pid_desired_state = ConstantVectorSource(np.array([0, np.pi, 0, 0], dtype=float))


    Q = np.diag([1000, 10000., 0., 0.])
    R = np.array([[1]])
    lqr_default_controller = builder.AddSystem(LinearQuadraticRegulator(plant, context, Q, R, input_port_index=plant.get_actuation_input_port().get_index()))

    # LQR default controller
    builder.Connect(
        plant.get_state_output_port(),
        lqr_default_controller.get_input_port()
    )
    builder.Connect(
        lqr_default_controller.get_output_port(),
        plant.get_actuation_input_port()
    )

    logger = LogVectorOutput(plant.get_state_output_port(), builder)
    logger.set_name("logger")

    diagram = builder.Build()
    return diagram, visualizer, plant, context, logger


def initialize_simulation(diagram):
    simulator = Simulator(diagram)
    simulator.Initialize()
    simulator.set_target_realtime_rate(1.)
    return simulator


if __name__ == "__main__":
    meshcat = StartMeshcat()
    model_location = './cartpole.sdf'
    # sim_time_step = 0.0001
    sim_time_step = 0.

    diagram, visualizer, plant, context, logger = create_scene(meshcat, model_location, sim_time_step)

    simulator = initialize_simulation(diagram)
    visualizer.StartRecording()
    meshcat.AddButton('Stop Simulation')
    while meshcat.GetButtonClicks('Stop Simulation') < 1:
        simulator.AdvanceTo(simulator.get_context().get_time() + 1.0)
    visualizer.PublishRecording()



    state_log = logger.FindLog(simulator.get_context())
    time = state_log.sample_times()
    state_data = state_log.data()

    plt.figure()
    plt.plot(time, state_data[1], '.-')
    plt.plot([time[0], time[-1]], [np.pi, np.pi], c='g')
    plt.title('pole angle plot')

    plt.figure()
    plt.plot(time, state_data[0], '.-')
    plt.plot([time[0], time[-1]], [0, 0], c='g')
    plt.title('cart distance plot')

    # graph = pydot.graph_from_dot_data(diagram.GetGraphvizString(max_depth=2))[0]
    #
    # graph.write('./cart_pole_pid_system.png', format='png')

    plt.show()

