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
from pydrake.systems.drawing import plot_system_graphviz

class custom_leaf_system(LeafSystem):
    def __init__(self, num_in, num_out):
        LeafSystem.__init__(self)
        input_port = self.DeclareVectorInputPort(
            name="input",
            size=num_in
        )

        def output(context, output):
            x = input_port.Eval(context)
            y = np.array([x[1], x[0]], dtype=float)
            output.set_value(y)

        output_port = self.DeclareVectorOutputPort(
            "output",
            BasicVector(num_out),
            output
        )

def create_scene(meshcat, model_location, sim_time_step):
    meshcat.Delete()
    meshcat.DeleteAddedControls()

    builder = DiagramBuilder()

    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=sim_time_step)
    parser = Parser(plant)

    parser.AddModelFromFile(model_location)

    plant.mutable_gravity_field().set_gravity_vector([0, 0, -9.8])
    plant.Finalize()

    visualizer = MeshcatVisualizer.AddToBuilder(
        builder, scene_graph, meshcat, MeshcatVisualizerParams(role=Role.kPerception, prefix="visual")
    )

    kp = np.array([0, 500], dtype=float)
    kd = np.array([0, 100], dtype=float)
    ki = np.array([0, 0], dtype=float)

    desired_state = ConstantVectorSource(np.array([0, np.pi, 0, 0], dtype=float))

    pid_controller = builder.AddSystem(PidController(kp, ki, kd))
    desired_trajectory = builder.AddSystem(desired_state)

    data_bus = custom_leaf_system(2, 2)
    builder.AddSystem(data_bus)
    builder.Connect(
        desired_trajectory.get_output_port(),
        pid_controller.get_input_port_desired_state()
    )

    builder.Connect(
        plant.get_state_output_port(),
        pid_controller.get_input_port_estimated_state()
    )

    builder.Connect(
        pid_controller.get_output_port_control(),
        data_bus.get_input_port()
    )

    builder.Connect(
        data_bus.get_output_port(),
        plant.get_actuation_input_port()
    )

    logger = LogVectorOutput(plant.get_state_output_port(), builder)
    logger.set_name("logger")

    diagram = builder.Build()


    context = plant.CreateDefaultContext()

    eps_ = np.pi / 3
    joint_0 = plant.GetJointByName('CartSlider')
    joint_0.set_default_translation(0.0)
    joint_1 = plant.GetJointByName('PolePin')
    joint_1.set_default_angle(np.pi - eps_)

    return diagram, visualizer, plant, context, logger


def initialize_simulation(diagram):
    simulator = Simulator(diagram)
    simulator.Initialize()
    simulator.set_target_realtime_rate(10.0)
    return simulator


if __name__ == "__main__":
    meshcat = StartMeshcat()
    model_location = './cartpole.sdf'
    sim_time_step = 0.0001

    diagram, visualizer, plant, context, logger = create_scene(meshcat, model_location, sim_time_step)

    simulator = initialize_simulation(diagram)
    visualizer.StartRecording()
    simulator.AdvanceTo(10.0)
    visualizer.PublishRecording()


    state_log = logger.FindLog(simulator.get_context())
    time = state_log.sample_times()
    state_data = state_log.data()
    print(state_data)
    print(state_data.shape)
    print(time)

    plt.figure()
    plt.plot(time, state_data[1], '.-')
    plt.plot([time[0], time[-1]], [np.pi, np.pi], c='g')
    plt.title('pole angle plot')

    plt.figure()
    plt.plot(time, state_data[0], '.-')
    plt.plot([time[0], time[-1]], [0, 0], c='g')
    plt.title('cart distance plot')

    graph = pydot.graph_from_dot_data(diagram.GetGraphvizString(max_depth=2))[0]

    graph.write('./cart_pole_pid_system.png', format='png')

    plt.show()

