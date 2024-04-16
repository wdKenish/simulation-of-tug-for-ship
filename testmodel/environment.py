def calculate_wind_force(U_w, angle_w, params):
    rho_air = 1.225
    A_frontal, A_lateral, L_pp = params['A_frontal'], params['A_lateral'], params['L_pp']
    Cx, Cy = 0.6, 0.8

    X_wind = 0.5 * rho_air * A_frontal * Cx * U_w**2 * np.cos(angle_w)
    Y_wind = 0.5 * rho_air * A_lateral * Cy * U_w**2 * np.sin(angle_w)
    N_wind = Y_wind * L_pp / 2
    return X_wind, Y_wind, N_wind

def calculate_wave_force(params):
    amplitude, wavelength, wave_direction, L_pp = params['amplitude'], 200, params['wave_direction'], params['L_pp']
    X_wave = 0.1 * amplitude * wavelength * np.cos(wave_direction)
    Y_wave = 0.1 * amplitude * wavelength * np.sin(wave_direction)
    N_wave = Y_wave * L_pp / 4
    return X_wave, Y_wave, N_wave

def calculate_current_effect(U_c, angle_c, params):
    rho_water, C_current, A_submerged, L_pp = 1025, 1.0, params['A_submerged'], params['L_pp']
    X_current = 0.5 * rho_water * A_submerged * C_current * U_c**2 * np.cos(angle_c)
    Y_current = 0.5 * rho_water * A_submerged * C_current * U_c**2 * np.sin(angle_c)
    N_current = Y_current * L_pp / 2
    return X_current, Y_current, N_current