if __name__ == "__main__":
    total_time, time_step, ship_state, target_pos, environmental_conditions = initialize_simulation()
    tug = setup_tug_control()
    positions = simulate(total_time, time_step, ship_params, ship_state, target_pos, environmental_conditions, tug)
    plot_results(positions)  # Assuming a function to visualize the results