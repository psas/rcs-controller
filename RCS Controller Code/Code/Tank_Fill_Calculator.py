p0 = 14
p1 = 4500
cfm= 3.5
v1 = 0.0509259 # 88 cubic inches in cubic feet
# v1 = 0.162 # 280 cubic inches in cubic feet

cfm = 0.27
# v1 = 88

t = p1/p0*v1/cfm

v0 = p1*v1/p0 
weight = 0.0765*v0

# x = p1/p0*0.017/20

minutes = int(t)
seconds = (t-minutes)*60

print(f"Tank will fill in {minutes} minutes {seconds:.0f} seconds")
