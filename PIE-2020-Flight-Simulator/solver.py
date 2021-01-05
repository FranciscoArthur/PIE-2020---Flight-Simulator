
def rk4 ( state_var_derivatives, parameters, forces, tspan, state_var0):
    '''rk4 approximates the solution to an ODE using the RK4 method.

      Licensing:

          This code is distributed under the GNU LGPL license.

      Modified:

          22 April 2020

      Author:

          John Burkardt

      Input:

        function state_var_derivatives: points to a function that computes the state varaible derivatives.

        parameters: input for derivatives computation function
        
        forces: input for derivatives computation function
        
        real tspan[2]: contains the initial and final times.

        real state_var0[m]: an array containing the initial condition.


      Output:

          real t, state_var[m]: the times and solution values in the new step.'''
          
          
    import numpy as np

    if ( np.ndim ( state_var0 ) == 0 ):
        m = 1
    else:
        m = len ( state_var0 )

    tfirst = tspan[0]
    tlast = tspan[1]
    dt = ( tlast - tfirst )
    t = np.zeros (2)
    state_var = np.zeros ( [2, m ] )
    t[0] = tspan[0]
    state_var[0,:] = state_var0


    k1 = state_var_derivatives ( t[0],            state_var[0,:], parameters, forces)
    k2 = state_var_derivatives ( t[0] + dt / 2.0, state_var[0,:] + dt * k1 / 2.0,  parameters, forces)
    k3 = state_var_derivatives ( t[0] + dt / 2.0, state_var[0,:] + dt * k2 / 2.0,  parameters, forces)
    k4 = state_var_derivatives ( t[0] + dt,       state_var[0,:] + dt * k3,  parameters, forces)

    t[1] = t[0] + dt
    state_var[1,:] = state_var[0,:] + dt * ( k1 + 2.0 * k2 + 2.0 * k3 + k4 ) / 6.0

    return t[1], state_var[1,:]

