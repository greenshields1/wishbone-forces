# Wishbone-Forces
 
1.Change input forces in InputForces(Front/Rear).csv

Important to note that since only the right wheels are used, the cornering force sign will change dependent on whether it is an inner or outer wheel

Cornering outer:
Fy = -ve
Cornering
Fy = +ve
Braking
Fx: +ve
Accelerating
Fx: -ve

2. Suspension geometry

Change Suspension Geometry in ForceSolver3D
Use solidworks to get the coordinates of the VD points


3. Generate forces
Select whether you want to generate front or rear forces with the position ="" variable
This generates OutputForces.csv

4. Get wishbone and bearing sizing
Copy and paste generated block to (Front/Rear)VD.xlsx

Adjust green values as needed 
- wishbone lengths are printed in the output of the script
- for safety factors go for around 2
- adjust diameter until reasonable gauge tubes appear