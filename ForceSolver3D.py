import numpy as np
import pandas as pd#For storing data and finding highest values
import csv


class force():
    def __init__(self,direction,position):
        self.direction = direction/np.linalg.norm(direction)
        self.position = position
        #Sets moments relative to (0,0) in 3 directions, scaled by distance in metres
        self.moments = np.dot(np.identity(3),np.cross(self.position,self.direction))
        print(np.linalg.norm(direction))


class force_calculator():
    
    def __init__(self):
        self.input_forces = []
        self.output_forces = []
        self.M = None

    def set_input_forces(self,list):
        self.input_forces = list

    def set_output_forces(self,list):
        self.output_forces = list

    def form_matrix(self):
        #x components are the first index of force,y the second and so on, moments are harder
        A_1 = np.stack((i.direction for i in self.output_forces), axis = -1)
        #Take moments around 0,0
        A_2 = np.stack((i.moments for i in self.output_forces), axis = -1)
        A = np.concatenate((A_1,A_2))

        B_1 = np.stack((i.direction for i in self.input_forces), axis = -1)
        B_2 = np.stack((i.moments for i in self.input_forces), axis = -1)
        B = np.concatenate((B_1,B_2))

        M = np.matmul(np.linalg.inv(A),B)
        self.M = M

    def get_forces(self,input):
        return np.transpose(np.matmul(self.M,np.transpose(input)))

class data_manager():

    def forces_from_file(self,path):
        with open(path, newline='') as csvfile:
            data = list(csv.reader(csvfile, delimiter=','))#
            #print(np.array([i[1:] for i in data[1:]]))
        return np.array([i[0] for i in data[1:]]), np.array([i[1:] for i in data[1:]]).astype(float)


if __name__ == "__main__":
    
    #These values need to be set to define the VD geometry
    #Origin is defined as the middle of the two front wheel contact patches
    #####################################################################

    #Set wheel center and contact patch position
    contact = np.array([0,605.3,0])
    other_contact = np.array([1550,605.3,0])#Contact at other set of wheels for force calcs
    wheel_center = np.array([245.5,605.3,1])

    #Set VD points, Damper OB is assumed to be the same as the LBJ
    front_upper_IB = np.array([-150,308.5,329.1])
    front_lower_IB = np.array([-171.27,208.9,127.5])
    rear_upper_IB = np.array([175,308.5,329.1])
    rear_lower_IB = np.array([241.13,208.9,127.5])
    UBJ = np.array([10,528.53,368])
    LBJ = np.array([-10,564.85,140])
    DamperIB = np.array([115.45,188.86,568.76])

    #Set steering track rod coordinates
    steeringOB = np.array([62.76,531,203.41])
    steeringIB = np.array([62.76,203.47,169.4])

    #####################################################################

    calculator = force_calculator()

    #Input forces 

    #Need to check x y z are in right directions
    F_0x = force(np.array([1,0,0]),contact) #Longitudinal Force (brake/accelerate)
    F_0y = force(np.array([0,1,0]),contact) #Lateral force (turning)
    F_0z = force(np.array([0,0,1]),contact) #Up and down force (bump)
    F_1x = force(np.array([1,0,0]),wheel_center) #Force applied at front of tyre (hitting kerb)

    calculator.set_input_forces([F_0x,F_0y,F_0z,F_1x])
    input_labels = ["F_0x","F_0y","F_0z","F_1x"]

    #Output forces
    F_front_upper = force(front_upper_IB - UBJ,UBJ)
    F_rear_upper = force(rear_upper_IB - UBJ,UBJ)
    F_front_lower = force(front_lower_IB - LBJ,LBJ)
    F_rear_lower = force(rear_lower_IB - LBJ,LBJ)
    F_damper = force(DamperIB - LBJ,LBJ)
    F_steering = force(steeringIB - steeringOB, steeringIB)

    calculator.set_output_forces([F_front_upper,F_rear_upper,F_front_lower,F_rear_lower,F_damper,F_steering])
    output_labels = ["F_front_upper - rod end", "F_rear_upper - rod end", "F_front_lower - rod end", "F_rear_lower - rod end", "F_damper", "F_steering"]
    
    #Solve values
    calculator.form_matrix()

    manager = data_manager()
    scenario_labels, input_forces = manager.forces_from_file("InputForces.csv")
    output_forces = calculator.get_forces(input_forces)

    df = pd.DataFrame()
    df["Scenario"] = scenario_labels

    
    for i in range(4):
        df[input_labels[i]] = [x[i] for x in input_forces]

    for i in range(len(output_labels)):
        df[output_labels[i]] = [x[i] for x in output_forces]

    #Forces on sperical joints
    cos_UWB_angle = np.linalg.norm(np.dot(F_front_upper.direction,F_rear_upper.direction))
    cos_LWB_angle = np.linalg.norm(np.dot(F_front_lower.direction,F_rear_lower.direction))

    ffu = df["F_front_upper - rod end"]
    fru = df["F_rear_upper - rod end"]
    ffl = df["F_front_lower - rod end"]
    frl = df["F_rear_lower - rod end"]

    df["UBJ - spherical"] = (ffu**2+fru**2-2*ffu*fru*cos_UWB_angle)**0.5
    df["LBJ - spherical"]=(ffl**2+frl**2-2*ffl*frl*cos_LWB_angle)**0.5

    df = df.append(df.min(axis = 0),ignore_index=True)
    df = df.append(df.max(axis = 0),ignore_index=True)

    df.at[len(scenario_labels) + 1, 'Scenario'] = "Max tension"
    df.at[len(scenario_labels), 'Scenario'] = "Max compression"

    df.to_csv("OutputForces.csv")
    print("Completed")


    #generator = force_scenario_generator(COG,friction_coefficient,weight,contact,other_contact)
    #forces = generator.generate_forces()
    #print(calculator.get_forces(forces))

    #Need forces on sperical + rod ends from max forces
    """
    COG = np.array([855.2,0,260])

    friction_coefficient = 1.2
    safety_factor = 2
    weight = 140+80#kg
    """



# class force_scenario_generator():

#     def __init__(self,COG,friction_coefficient,weight,contact,other_contact):
#         self.COG = COG
#         self.wheel_base = abs(contact[0]-other_contact[0])
#         self.wheel_track = contact[1] * 2
#         self.friction_coefficient = friction_coefficient
#         self.weight = weight
#         self.contact = contact
#         self.other_contact = other_contact



    
#     #Needs to deal with front or rear wheel contact

#     #corner brake balance depends on how much of friction is used to brake and how much for cornering
#     # -90 < theta < 90
#     def get_wheel_balance(self,corner_brake_balance):
#         #Check directions right as well
#         #mv^2/gr = u sin(theta), a/g = u cos(theta)
#         longitudinal_force = np.cos(corner_brake_balance) * 9.81 * self.friction_coefficient * self.weight
#         lateral_force = np.sin(corner_brake_balance) * 9.81 * self.friction_coefficient * self.weight
#         F_0z = 9.81 * self.weight *#Need expression for F/B weight distribution # + longitudinal_force * COG[1] / (self.COG[0]-self.contact[0]) / 2 + 
#         #F_0x = 
#         #F_0y = 
#         #F_1x = 

#     #Assume a circle with car cornering at full speed or accelerating/braking at full speed
    
#     def generate_forces(self):
#         pass