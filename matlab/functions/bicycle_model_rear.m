function dState = bicycle_model_rear(state, v, delta, L)
    % BICYCLE_MODEL_REAR Calculates the kinematic derivatives for the rear-axle model
    % Inputs:
    %   state - Current state vector [X; Y; psi]
    %   v     - Longitudinal velocity (m/s)
    %   delta - Front steering angle (radians)
    %   L     - Wheelbase (meters)
    % Output:
    %   dState - State derivatives [dX; dY; dpsi]

    % 1. Extract the current heading (psi) from the state vector
    psi = state(3);
    
    % 2. Calculate the derivatives (The Physics)
    dX   = v * cos(psi);
    dY   = v * sin(psi);
    dpsi = (v / L) * tan(delta);
    
    % 3. Pack the derivatives into the output column vector
    dState = [dX; dY; dpsi];
end