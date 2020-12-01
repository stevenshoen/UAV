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

from simulation.flightsim import TakeoffSim
#from simulation.simulator import Simulation
from ground_station.ground_station import GeodeticWaypoint, FlightPlan
from simulation.simcraft.glider import ElectricGlider
from simulation.autopilot import FlightplanAutopilot


GROUND_LEVEL = 5000.0
FLIGHT_LEVEL = 6000.0

lat0 = 45.675
lon0 = -108.770
hdg0 = 0.00001

lat1 = 45.690
lon1 = -108.770
hdg1 = 0.0

lat2 = 45.710
lon2 = -108.790
hdg2 = 0.0

lat3 = 45.730
lon3 = -108.770
hdg3 = 0.0

lat4 = 45.745
lon4 = -108.770
hdg4 = 0.0

wps = [GeodeticWaypoint(lat0, lon0, FLIGHT_LEVEL, hdg0), # x_earth=0.0, y_earth=0.0),
       GeodeticWaypoint(lat1, lon1, FLIGHT_LEVEL, hdg1),
       GeodeticWaypoint(lat2, lon2, FLIGHT_LEVEL, hdg2),
        GeodeticWaypoint(lat3, lon3, FLIGHT_LEVEL, hdg3)]

fp = FlightPlan(wps, active_waypoint=1)

init_pos = EarthPosition(0.0, 0.0, FLIGHT_LEVEL)

INITIAL_HEADING = hdg0
TRIM_SPEED = 55.0

atmosphere = ISA1976()
gravity = VerticalConstant()
#wind = NoWind()
wind = Wind()

environment = Environment(atmosphere, gravity, wind)

#aircraft = Cessna172()
aircraft = ElectricGlider()
#aircraft = Missile()

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

controller = FlightplanAutopilot(trimmed_controls)
controller.flightplan = fp
controller.mode = 'flightplan'

sim = TakeoffSim(aircraft, system, environment, controller, dt=0.01)

res_df = sim.propagate(120.0)

res = sim.results

from simulation import plots

#plots.flight_plot(res, every=200)
plots.environ_plot(res_df)

plots.pitch_dampers_plot(res)
plots.roll_dampers_plot(res)
plots.yaw_dampers_plot(res)

plots.flightplan_map(fp, res_df)
plots.timer_plot(res_df)


