
import math as mt
header=True
goals=[]
aisle=[]
aisle_id="A1"# could change later
with open('/home/kendell/retail_assistant_ws/Staff-interface/Staff-interface/aisles.csv', 'r') as file:
    for line in file:
        #print(line)
        if header:
            header = False
            continue
        else:# not header
            line=line.split(',')
            if(line[0]!=aisle_id):# if new aisle append
              goals.append(aisle)  
              aisle=[]# reset aisle
            x=round(float(line[2]),3)
            y=round(float(line[3]),3)
            aisle.append((x,y))
            aisle_id=line[0]
    goals.append(aisle)# add last aisle
print(len(goals))
print(goals[6])
    