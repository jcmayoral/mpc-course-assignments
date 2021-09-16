import numpy as np
from sim.sim1d import sim_run

# Simulator options.
options = {}
options['FIG_SIZE'] = [8,8]
options['FULL_RECALCULATE'] = False

class ModelPredictiveControl:
    def __init__(self):
        self.horizon = 20
        self.dt = 0.2

        # Reference or set point the controller will achieve.
        self.reference = [50, 0, 0]

    def plant_model(self, prev_state, dt, pedal, steering):
        x_t = prev_state[0]
        v_t = prev_state[3]
        a_t = pedal
        x_t_1 = x_t + v_t *dt
        v_t_1 = v_t + a_t*dt - v_t/25.0# m/s
        return [x_t_1, 0, 0, v_t_1]

    def cost_function(self,u, *args):
        state = args[0]
        ref = args[1]
        cost = 0.0
        # u is a tuple size of u is double
        for k in range(0,self.horizon):
            state = self.plant_model(state, self.dt, u[k*2],u[k*2+1])
            cost += (ref[0] - state[0])**2
            #cost += 1-u[k*2]
            if state[3]*self.dt > (ref[0] - state[0]):
               cost+=10000 
            
            #m/s to km/hr
            if state[3]*3.6 > 10:
                cost += 2000*np.fabs(state[3] - 10)
        return cost

sim_run(options, ModelPredictiveControl)
