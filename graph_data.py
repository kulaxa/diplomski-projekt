import sys

import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import GraphicsContextPdf


# Function to parse the data from a line
def parse_line(line):
    values = line.strip().split(';')
    fps = float(values[0].split(':')[1])
    particle_counter = float(values[2].split(':')[1])
    grid_size = int(values[1].split(':')[1])
    thread_count = int(values[3].split(':')[1])
    return fps, particle_counter, grid_size, thread_count


# Read data from the file
file_path = 'results.txt'
with open(file_path, 'r') as file:
    lines = file.readlines()

# Parse data and create lists for FPS and PARTICLE_COUNTER
fps_values = []
particle_counter_values = []

for line in lines:
    fps, particle_counter, grid_size, thread_count = parse_line(line)
    if particle_counter != 0:
        fps_values.append(fps)
        particle_counter_values.append(particle_counter)

# Create and save the plot
plt.plot(particle_counter_values, fps_values, marker='o')
plt.title(f'FPS vs Particle Count (Grid Size: {grid_size}, Thread Count: {thread_count})')
plt.xlabel('Number of Particles')
plt.ylabel('Physics Time (ms)')
plt.ylim(0, 200)  # Set y-axis limits

plt.grid(True)
plt.savefig(f'graphs/fps_vs_particle_counter_plot_{grid_size}x{grid_size}_{thread_count}.png')  # Save as PNG file
