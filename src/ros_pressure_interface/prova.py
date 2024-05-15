import rospy

DEFAULT_CHAMBERS = 9		# Default number of chambers

PMAX      = [1.68, 1.68, 1.68, 1.08, 1.68, 1.68, 4.4, 4.4, 1.02]
PMIN      = [0, 0, 0, 0, 0, 0, 0, 0, 0] 
MAX_DIGIT = [255, 255, 255, 255, 255, 255, 255, 255, 4095]
MIN_DIGIT = [10, 10, 10, 15, 10, 10, 10, 10, 50]

bar = [1.68, 1.68, 1.68, 1.08, 1.68, 1.68, 4.4, 4.4, 1.02] # prova con valore max in bar 
#bar = [0, 0, 0, 0, 0, 0, 0, 0, 0] # prova con valore max in bar 

# Saturation on max value
for i in range(DEFAULT_CHAMBERS):
    if bar[i] > PMAX[i]:
	    bar[i] = PMAX[i] 		
   

		
# Initialization
digit = [0] * DEFAULT_CHAMBERS
		
for i in range(DEFAULT_CHAMBERS):
	digit[i] = int((MAX_DIGIT[i] - MIN_DIGIT[i]) * (bar[i] / PMAX[i]) + MIN_DIGIT[i])


print(digit)






  


