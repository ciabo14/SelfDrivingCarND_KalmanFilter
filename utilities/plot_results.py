import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np

input_file =  "..\..\data\sample-laser-radar-measurement-data-2.txt"
output_file =  "..\..\data\sample-laser-radar-measurement-data-2-output.txt"

i_f = open(input_file, 'r')
o_f = open(output_file, 'r')

input_px = []
input_py = []

output_px = []
output_py = []

for line in i_f:
	entry_values = line.split()
	if(entry_values[0] == "L"):
		input_px.append(float(entry_values[1]))
		input_py.append(float(entry_values[2]))
	if(entry_values[0] == "R"):
		input_px.append(float(entry_values[1]) * np.cos(float(entry_values[2])))
		input_py.append(float(entry_values[1]) * np.sin(float(entry_values[2])))

for line in o_f:
	entry_values = line.split()
	output_px.append(entry_values[0])
	output_py.append(entry_values[1])


original_data = mpatches.Patch(color='red', label='The original data')
computed_data = mpatches.Patch(color='blue', label='The computed data')

plt.plot(input_px,input_py,'ro',output_px,output_py,'bx')

plt.legend(handles=[original_data])
plt.legend(handles=[computed_data])
plt.show()