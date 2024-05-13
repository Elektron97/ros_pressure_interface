MAX_DIGIT			= 255   # Max value of pressure in digit 
MIN_DIGIT			= 10   	# Min value of pressure in digit 		
  

pressures = [0] * 9
print(pressures)

pressures = [1.68, 1.68, 1.68, 1.08, 1.68, 1.68, 4.4, 4.4, 1.02] # prova con valore max in bar 

print(pressures)

pressure_digit = [0] * 9
print(pressure_digit)


  
pressure_digit[0] = int((MAX_DIGIT - MIN_DIGIT) * (pressures[0] / 1.68) + MIN_DIGIT)
pressure_digit[1] = int((MAX_DIGIT - MIN_DIGIT) * (pressures[1] / 1.68) + MIN_DIGIT)
pressure_digit[2] = int((MAX_DIGIT - MIN_DIGIT) * (pressures[2] / 1.68) + MIN_DIGIT)
  
pressure_digit[3] = int((MAX_DIGIT - 15) * (pressures[3] / 1.08) + 15)
  
pressure_digit[4] = int((MAX_DIGIT - MIN_DIGIT) * (pressures[4] / 1.68) + MIN_DIGIT)
pressure_digit[5] = int((MAX_DIGIT - MIN_DIGIT) * (pressures[5] / 1.68) + MIN_DIGIT)

pressure_digit[6] = int((MAX_DIGIT - MIN_DIGIT) * (pressures[6] / 4.4) + MIN_DIGIT)
pressure_digit[7] = int((MAX_DIGIT - MIN_DIGIT) * (pressures[7] / 4.4) + MIN_DIGIT)

pressure_digit[8] = int((4095 - 50) * (pressures[8] / 1.02) + 50)


print(pressure_digit)