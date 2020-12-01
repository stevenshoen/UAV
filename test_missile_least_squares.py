from simulation.simcraft.cessna_172 import Cessna172
from environment.environment import Environment
from environment.atmosphere import ISA1976
from environment.gravity import VerticalConstant
from environment.wind import NoWind, Wind
from state.position import EarthPosition
from simulation.trimmer import steady_state_trim
from systems.odeint_system import OdeIntSystem
from state.aircraft_state import AircraftState
from state.velocity import BodyVelocity
from state.attitude import EulerAttitude
from simulation.simtools import Target
from simulation.flightsim import TakeoffSim
#from simulation.simulator import Simulation
from ground_station.ground_station import GeodeticWaypoint, FlightPlan
from simulation.simcraft.glider import ElectricGlider
from simulation.simcraft.rocket import Missile
from simulation.autopilot import PDAutopilot, LeastSquaresAutopilot


GROUND_LEVEL = 5000.0
FLIGHT_LEVEL = 6000.0

init_pos = EarthPosition(0.0, 0.0, FLIGHT_LEVEL)

INITIAL_HEADING = 0.0
TRIM_SPEED = 55.0

atmosphere = ISA1976()
gravity = VerticalConstant()
#wind = NoWind()
wind = Wind()

environment = Environment(atmosphere, gravity, wind)

aircraft = Missile()

controls0 = {'delta_elevator': -0.01, 'delta_aileron': 0, 'delta_rudder': 0, 'delta_t': 0.75}

trimmed_state, trimmed_controls = steady_state_trim(
    aircraft, environment, init_pos, psi=INITIAL_HEADING, TAS=TRIM_SPEED,
    controls=controls0
)

aircraft.alpha = trimmed_state.attitude.theta

init_vel = BodyVelocity(u=trimmed_state.velocity.u, v=0, w=0, attitude=trimmed_state.attitude)

init_state = AircraftState(init_pos, trimmed_state.attitude, init_vel)

system = OdeIntSystem(t0=0, full_state=init_state)
system.z_ground = -GROUND_LEVEL

system.full_state.position.update((0, 0, -FLIGHT_LEVEL))

controller = LeastSquaresAutopilot(trimmed_controls)
controller.target = Target(pos=[5000.0, -1000.0, -6100.0], velocity=(0.0, 20.0, 0.0))

sim = TakeoffSim(aircraft, system, environment, controller, dt=0.01)

res_df = sim.propagate(4.0)

res = sim.results

from simulation import plots

plots.target_flight_map(res)

plots.flight_plot(res, every=200)
#plots.environ_plot(res_df)

#plots.fls_plot(res_df)

plots.pitch_dampers_plot(res)
plots.roll_dampers_plot(res)


