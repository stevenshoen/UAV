#!/usr/bin/env python3
# -*- coding: utf-8 -*-


#from pyfme.aircrafts import Cessna172
from src.environment import Environment
from src.atmosphere import ISA1976
from src.gravity import VerticalConstant
from src.wind import NoWind
from src.position import EarthPosition
from src.trimmer import steady_state_trim
from src.euler_flat_earth import EulerFlatEarth
#from pyfme.simulator import Simulation
from modsim import ModSim, Wind
from modcraft import ModCraft

atmosphere = ISA1976()
gravity = VerticalConstant()
wind = NoWind()
#wind = Wind()

environment = Environment(atmosphere, gravity, wind)
#aircraft = Cessna172()
aircraft = ModCraft()

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


sim = ModSim(aircraft, system, environment, trimmed_controls)

time=0.0


dt = sim.dt
half_dt = sim.dt/2

#bar = tqdm.tqdm(total=time, desc='time', initial=self.system.time)
#fig, axes = plt.subplots(1, 3)
# To deal with floating point issues we cannot check equality to
# final time to finish propagation
time_plus_half_dt = time + half_dt
print('running..')
#while self.system.time + dt < time_plus_half_dt:
t = sim.system.time
sim.environment.update(sim.system.full_state)
#            controls = self._get_current_controls(t)
#            self.aircraft.controls = self._get_current_controls(t)
sim.controller.update(t)
sim.aircraft._set_current_controls(sim.controller.controls)
#            print('--', self.aircraft.controls)
#            print('--', self.controller.controls)
sim.aircraft.calculate_forces_and_moments(sim.system.full_state,
sim.environment, sim.controller.controls)
sim.system.time_step(dt)

"""
        sol = solve_ivp(self.fun_wrapped, t_span, x0, method=method,
                        **self._options)
        
        self.fun_wrapped is derivatives
            -function of t and x, time and state vec?
            
            calls
            system._update_full_system_state_from_state
            
            calls update_simulation
                -updates environment
                -calculates forces and moments given
                    controls
                    environemnt
                    current state
            
            calls and returns
            system._system_equations(t, x, mass, inertia, forces, moments)
            
                takes current state + forces and moments to get new state
            
                returns np.array([du_dt, dv_dt, dw_dt, dp_dt, dq_dt, dr_dt, dtheta_dt,
                     dphi_dt, dpsi_dt, dx_dt, dy_dt, dz_dt])
                body vel, body ang vel, world ang vel, world body vel
            ** the equations to solve **            

        t_span = (t_ini, t_ini + dt)
        x0 = self.state_vector

        method='RK45'
        

solution of the ivp is new system._state_vector


"""




sim._save_time_step()


#simulation.propagate(0.2)

res = sim.results













#
#
#    def propagate(self, time):
#        """Run the simulation by integrating the system until time t.
#
#        Parameters
#        ----------
#        time : float
#            Final time of the simulation
#
#        Notes
#        -----
#        The propagation relies on the dense output of the integration
#        method, so that the number and length of the time steps is
#        automatically chosen.
#        """
#        dt = self.dt
#        half_dt = self.dt/2
#
#        bar = tqdm.tqdm(total=time, desc='time', initial=self.system.time)
#        fig, axes = plt.subplots(1, 3)
#        # To deal with floating point issues we cannot check equality to
#        # final time to finish propagation
#        time_plus_half_dt = time + half_dt
#        print('running..')
#        while self.system.time + dt < time_plus_half_dt:
#            t = self.system.time
#            self.environment.update(self.system.full_state)
##            controls = self._get_current_controls(t)
##            self.aircraft.controls = self._get_current_controls(t)
#            self.controller.update(t)
#            self.aircraft._set_current_controls(self.controller.controls)
##            print('--', self.aircraft.controls)
##            print('--', self.controller.controls)
#            self.aircraft.calculate_forces_and_moments(self.system.full_state,
#                                                       self.environment, self.controller.controls)
#            self.system.time_step(dt)
#            self._save_time_step()
#            bar.update(dt)
#            self.update_hud(axes)
##            plt.pause(0.05)
#        plt.show()
#        bar.close()






