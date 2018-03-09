import matlab.engine
eng = matlab.engine.start_matlab()

print("Creating random camera points:")
A = eng.rand(8,3)
one = eng.ones(8,1)
A = eng.cat(2,A,one)
for arow in A:
    print(arow)

print("\nCreating a random transformation matrix:")
T = eng.rand(3,4)
row = matlab.double([0,0,0,1])
T = eng.cat(1,T,row)
for arow in T:
    print(arow)

print("\nCalculating the corresponding robot positions...")
B = eng.mtimes(A,T)

print("\nGenerate a transformation matrix given camera and robot points and B:")
newT = eng.mldivide(A,B)
for arow in newT:
    print(arow)

print("\nThe new transformation matrix should be the same as the random one.")
