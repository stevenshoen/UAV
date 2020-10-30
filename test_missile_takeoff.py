#from pyfme.aircrafts import Cessna172
from environment.environment import Environment
from environment.atmosphere import ISA1976
from environment.gravity import VerticalConstant
from environment.wind import NoWind
from state.position import EarthPosition
from simulation.trimmer import steady_state_trim
from systems.euler_flat_earth_ground import EulerFlatEarthGround
#from pyfme.simulator import Simulation
from simulation.modsim import ModSim, Wind

from simulation.flightsim_takeoff import TakeoffSim
from simulation.simulator import Simulation

from simulation.simcraft.rocket import Missile
from math import pi


atmosphere = ISA1976()
gravity = VerticalConstant()
wind = NoWind()
#wind = Wind()

environment = Environment(atmosphere, gravity, wind)
#aircraft = Cessna172()
#aircraft = ModCraft()
aircraft = Missile()

#aircraft = Cessna172()

initial_position = EarthPosition(0, 0, 1000)
initial_heading = pi / 2
controls_0 = {'delta_elevator': 0.01,
              'delta_aileron': 0,
              'delta_rudder': 0,
              'delta_t': 0.75,
              }

trimmed_state, trimmed_controls = steady_state_trim(
    aircraft, environment, initial_position, psi=initial_heading, TAS=80,
    controls=controls_0
)

system = EulerFlatEarthGround(t0=0, full_state=trimmed_state)

#trimmed_controls['delta_elevator'] *= -1 # not sure why this is coming out backwards


#simulation = ModSim(aircraft, system, environment, trimmed_controls)
simulation = TakeoffSim(aircraft, system, environment, trimmed_controls, dt=0.01)
#simulation = Simulation(aircraft, system, environment, trimmed_controls)

#simulation.ground_station = GroundStation()

simulation.propagate(5.0)

res = simulation.results

from simulation import plots

#plots.effection_plot(simulation.controller.pitch_effection)
#plots.elev_plot(res)
#plots.ail_plot(res)
#plots.rud_plot(res)

plots.telem_plot(res)
plots.controls_plot(res)
##plots.flight_plot(res, every=10)
#plots.environ_plot(res)

