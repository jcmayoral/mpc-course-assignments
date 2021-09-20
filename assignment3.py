import numpy as np
from sim.sim2d import sim_run

# Simulator options.
options = {}
options['FIG_SIZE'] = [8,8]
options['OBSTACLES'] = True

class ModelPredictiveControl:
    def __init__(self):
        self.horizon = 15
        self.dt = 0.2

        # Reference or set point the controller will achieve.
        self.reference1 = [10, 0, 0]
        self.reference2 = None

        self.x_obs = 5
        self.y_obs = 0.1

    def plant_model(self,prev_state, dt, pedal, steering):
        x_t = prev_state[0]
        y_t = prev_state[1]
        psi_t = prev_state[2]
        v_t = prev_state[3]
        x_t_1 = x_t + v_t * np.cos(psi_t) * dt
        y_t_1 = y_t + v_t * np.sin(psi_t) * dt
        v_t_1 = v_t + pedal*dt - v_t/25.0# m/s
        
        L=1.0
        R = L/np.tan(steering)
        R=2.5 #HACK according to the instructions
        psi_t_1 = psi_t + v_t* np.tan(steering)/R

        return [x_t_1, y_t_1, psi_t_1, v_t_1]

    def cost_function(self,u, *args):
        state = args[0]
        ref = args[1]
        cost = 0.0
        
        for k in range(0,self.horizon):
            vt_ref = state[3]
            state = self.plant_model(state, self.dt, u[k*2],u[k*2+1])
            cost += 1.0*abs(ref[0] - state[0])**2
            cost += 1.0*abs(ref[1] - state[1])**2
            cost += 1.0*abs(ref[2] - state[2])**2
            cost += self.calculate_distance(state, ref[1], ref[2])
        return cost
    
    def dist2goal(self, x, y, goalx, goaly):
        return 0.1*np.sqrt(abs(goalx - x)**2 + abs(goaly - y)**2)

    
    def calculate_distance(self,state, goalx, goaly):
        distance = np.sqrt(np.power(state[0]-self.x_obs,2)+ np.power(state[1]-self.y_obs,2))
        return self.dist2goal(state[0], state[1], goalx, goaly) if distance > 3 else 10./distance

sim_run(options, ModelPredictiveControl)
