voltage = float(input("Give voltage: "))


#print(voltage)
if(voltage >= 0.4):
    pressure = float((voltage-0.4)/0.4)*0.5
    print(pressure)

    depth = float(pressure * 10)
    print("depth: " + str(round(depth,2)) + " meters")
else:
    print("Sensor error")