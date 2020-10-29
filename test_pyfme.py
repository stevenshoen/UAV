#from pyfme.aircrafts import Cessna172
from environment.environment import Environment
from environment.atmosphere import ISA1976
from environment.gravity import VerticalConstant
from environment.wind import NoWind
from state.position import EarthPosition
from simulation.trimmer import steady_state_trim
from systems.euler_flat_earth import EulerFlatEarth
#from pyfme.simulator import Simulation
from simulation.modsim import ModSim, Wind

from simulation.flightsim import FlightSim
from simulation.simulator import Simulation

from simulation.simcraft.modcraft import ModCraft
#from simulation.simcraft.cessna_172 import Cessna172
#from pyfme.utils.input_generator import Constant, Doublet


atmosphere = ISA1976()
gravity = VerticalConstant()
wind = NoWind()
#wind = Wind()

environment = Environment(atmosphere, gravity, wind)
#aircraft = Cessna172()
aircraft = ModCraft()

#aircraft = Cessna172()

initial_position = EarthPosition(0, 0, 1000)

controls_0 = {'delta_elevator': 0.05,
              'delta_aileron': 0,
              'delta_rudder': 0,
              'delta_t': 0.5,
              }

trimmed_state, trimmed_controls = steady_state_trim(
    aircraft, environment, initial_position, psi=0.0, TAS=50,
    controls=controls_0
)

system = EulerFlatEarth(t0=0, full_state=trimmed_state)

#controls = {
#    'delta_elevator': Doublet(2, 1, 0.1,
#                              trimmed_controls['delta_elevator']),
#    'delta_aileron': Constant(trimmed_controls['delta_aileron']),
#    'delta_rudder': Constant(trimmed_controls['delta_rudder']),
#    'delta_t': Constant(trimmed_controls['delta_t'])
#}
#controls = {
#    'delta_elevator': trimmed_controls['delta_elevator'],
#    'delta_aileron': trimmed_controls['delta_aileron'],
#    'delta_rudder': trimmed_controls['delta_rudder'],
#    'delta_t': trimmed_controls['delta_t']
#}


#simulation = ModSim(aircraft, system, environment, trimmed_controls)
simulation = FlightSim(aircraft, system, environment, trimmed_controls)
#simulation = Simulation(aircraft, system, environment, trimmed_controls)
simulation.propagate(0.02)

res = simulation.results

#import plots

#plots.elev_plot(res)
#plots.ail_plot(res)
#plots.rud_plot(res)

#plots.telem_plot(res)
#plots.controls_plot(res)
##plots.flight_plot(res, every=10)
#plots.environ_plot(res)

