#  Live graphs
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
import os.path




fig = plt.figure(figsize=(11.5,10),facecolor='white')

ax1 = fig.add_subplot(2,2,1) # consolidation curve (time-vertical stress) -> consolidation phase
ax2 = fig.add_subplot(2,2,2) # mobilized angle of friction curve (mobangle-strain) -> shearing phase
ax3 = fig.add_subplot(2,2,3) # deviatoric stress (vertical stress-strain) -> shearing phase
ax4 = fig.add_subplot(2,2,4) # coordination number (coordination number-strain) -> shearing phase


# The animation stuff
def animate(i):

	# ax1
	if os.path.isfile('../triaxialOnBeads/consolidation.txt'):
		source_data_consolidation= open('../triaxialOnBeads/consolidation.txt').readlines()
		firstline_consolidation= source_data_consolidation.pop(0)
		# lines = source_data.split('\n')
		xs_c = []
		ys_c = []
		for line in source_data_consolidation:
			if len(line) > 1:
				time, verticalDisp, strain, verticalForce, verticalStress, mobAngle, numManifolds, numContacts, CM = line.split(' ')
				xs_c.append(time)
				ys_c.append(verticalStress)

		ax1.clear()
		ax1.plot(xs_c,ys_c, color='k', linewidth=2)
		ax1.set_xlabel('Time(s)')
		ax1.set_ylabel('Detected vertical stress (kPa)')
		ax1.set_title('Consolidation phase')

		# To change the characteristics of each axis line, thickness, color and ticks
		ax1.spines['left'].set_color('#3a3a3a')
		ax1.spines['right'].set_color('#3a3a3a')
		ax1.spines['bottom'].set_color('#3a3a3a')
		ax1.spines['top'].set_color('#3a3a3a')

		ax1.spines['left'].set_linewidth('2.5')
		ax1.spines['bottom'].set_linewidth('2.5')
		ax1.spines['right'].set_linewidth('2.5')
		ax1.spines['top'].set_linewidth('2.5')

		ax1.tick_params(axis='x', colors='#3a3a3a')
		ax1.tick_params(axis='y', colors='#3a3a3a')


		# plt.title('The stress-strain behaviour of soil sample]')
		plt.subplots_adjust(left=0.1, bottom= 0.1, right= 0.95, top=0.95, wspace=0.2, hspace= 0.2)

		ax1.grid(True,color='grey', linestyle=':')


	# ax2
	# Checks if the file exists first
	if os.path.isfile('../triaxialOnBeads/shearing.txt'):
		source_data= open('../triaxialOnBeads/shearing.txt').readlines()
		firstline= source_data.pop(0)
		# lines = source_data.split('\n')
		xs = []
		ys = []
		zs = []
		ks = []
		for line in source_data:
			if len(line) > 1:
				time, verticalDisp, strain, verticalForce, verticalStress, mobAngle, numManifolds, numContacts, CM = line.split(' ')
				xs.append(strain)
				ys.append(mobAngle)
				zs.append(verticalStress)
				ks.append(CM)

		ax2.clear()
		ax2.plot(xs,ys, color='k', linewidth=2)
		ax2.set_xlabel('Strain')
		ax2.set_ylabel('Mobilized angle of friction (deg)')
		ax2.set_title('Shearing phase')

		# To change the characteristics of each axis line, thickness, color and ticks
		ax2.spines['left'].set_color('#3a3a3a')
		ax2.spines['right'].set_color('#3a3a3a')
		ax2.spines['bottom'].set_color('#3a3a3a')
		ax2.spines['top'].set_color('#3a3a3a')

		ax2.spines['left'].set_linewidth('2.5')
		ax2.spines['bottom'].set_linewidth('2.5')
		ax2.spines['right'].set_linewidth('2.5')
		ax2.spines['top'].set_linewidth('2.5')

		ax2.tick_params(axis='x', colors='#3a3a3a')
		ax2.tick_params(axis='y', colors='#3a3a3a')


		# plt.title('The stress-strain behaviour of soil sample]')
		plt.subplots_adjust(left=0.1, bottom= 0.1, right= 0.95, top=0.95, wspace=0.2, hspace= 0.2)

		ax2.grid(True,color='grey', linestyle=':')

		# ax3
		ax3.clear()
		ax3.plot(xs,zs, color='k', linewidth=2)
		ax3.set_xlabel('Strain')
		ax3.set_ylabel('Vertical stress (kPa)')
		ax3.set_title('Shearing phase')

		# To change the characteristics of each axis line, thickness, color and ticks
		ax3.spines['left'].set_color('#3a3a3a')
		ax3.spines['right'].set_color('#3a3a3a')
		ax3.spines['bottom'].set_color('#3a3a3a')
		ax3.spines['top'].set_color('#3a3a3a')

		ax3.spines['left'].set_linewidth('2.5')
		ax3.spines['bottom'].set_linewidth('2.5')
		ax3.spines['right'].set_linewidth('2.5')
		ax3.spines['top'].set_linewidth('2.5')

		ax3.tick_params(axis='x', colors='#3a3a3a')
		ax3.tick_params(axis='y', colors='#3a3a3a')


		# plt.title('The stress-strain behaviour of soil sample]')
		plt.subplots_adjust(left=0.1, bottom= 0.1, right= 0.95, top=0.95, wspace=0.2, hspace= 0.2)

		ax3.grid(True,color='grey', linestyle=':')

		# ax4
		ax4.clear()
		ax4.plot(xs,ks, color='k', linewidth=2)
		ax4.set_xlabel('Strain')
		ax4.set_ylabel('Average coordination number')
		ax4.set_title('Shearing phase')

		# To change the characteristics of each axis line, thickness, color and ticks
		ax4.spines['left'].set_color('#3a3a3a')
		ax4.spines['right'].set_color('#3a3a3a')
		ax4.spines['bottom'].set_color('#3a3a3a')
		ax4.spines['top'].set_color('#3a3a3a')

		ax4.spines['left'].set_linewidth('2.5')
		ax4.spines['bottom'].set_linewidth('2.5')
		ax4.spines['right'].set_linewidth('2.5')
		ax4.spines['top'].set_linewidth('2.5')

		ax4.tick_params(axis='x', colors='#3a3a3a')
		ax4.tick_params(axis='y', colors='#3a3a3a')


		# plt.title('The stress-strain behaviour of soil sample]')
		plt.subplots_adjust(left=0.1, bottom= 0.1, right= 0.95, top=0.95, wspace=0.2, hspace= 0.2)

		ax4.grid(True,color='grey', linestyle=':')

#  the animation function 3rd parameter is the interval time in milisecond
ani = animation.FuncAnimation(fig,animate,interval=1000)



plt.show()