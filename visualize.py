import os

import os

def display_grid(state, problem):
    num_rows, num_cols = problem.grid_size
    cell_width = 7  # width of each cell for better visuals
    cell_height = 3  # height of each cell

    # Create empty grid of strings
    grid_cells = [[[' ' * cell_width for _ in range(cell_height)]
                   for _ in range(num_cols)] for _ in range(num_rows)]

    # Fill obstacles
    for r, c in problem.obstacles:
        for i in range(cell_height):
            grid_cells[r][c][i] = 'X' * cell_width

    # Fill packing stations
    for r, c in problem.packing_stations:
        grid_cells[r][c][1] = 'P'.center(cell_width)

    # Fill shelves
    shelf_positions = {}  # keep track of shelves for robot-under-shelf check
    for shelf_id, pos in state['shelf_positions'].items():
        r, c = pos
        # Check if robot is carrying this shelf
        carrying_r1 = state['robot1']['shelf_carried'] == shelf_id
        carrying_r2 = state['robot2']['shelf_carried'] == shelf_id
        if carrying_r1 or carrying_r2:
            continue  # will show with robot
        grid_cells[r][c][1] = f"S{shelf_id}".center(cell_width)
        shelf_positions[(r, c)] = shelf_id

    # Fill robots
    robots = [('R1', state['robot1']['position'], state['robot1']['shelf_carried']),
              ('R2', state['robot2']['position'], state['robot2']['shelf_carried'])]
    for r_name, (r, c), shelf in robots:
        if shelf:  # robot is carrying a shelf
            cell_content = f"{r_name}/S{shelf}"
        elif (r, c) in shelf_positions:  # robot is under a shelf
            cell_content = f"{r_name}↓S{shelf_positions[(r, c)]}"
        else:
            cell_content = r_name
        grid_cells[r][c][1] = cell_content.center(cell_width)

    # Clear terminal
    os.system('cls' if os.name == 'nt' else 'clear')

    # Print the grid row by row
    horizontal_border = '+' + '+'.join(['-' * cell_width for _ in range(num_cols)]) + '+'
    print(horizontal_border)
    for r in range(num_rows):
        for h in range(cell_height):
            row_str = '|'
            for c in range(num_cols):
                row_str += grid_cells[r][c][h] + '|'
            print(row_str)
        print(horizontal_border)
    print()

