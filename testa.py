import numpy
import scipy 
import math

data = [[1,23,5],[5,9,7],[4,6,55]]
x = [1,2,3,4,5,6,7,8,9]

grad = numpy.zeros(9)
sxx = x[0]
syy = x[1]
szz = x[2]
sxy = x[3]
sxz = x[4]
syz = x[5]
hx = x[6]
hy = x[7]
hz = x[8]
S = numpy.array([[sxx, sxy, sxz],[sxy, syy, syz], [sxz, syz, szz]])
h = numpy.array(x[6:])
data = numpy.array(data)

Ui = S @ (data[0] - h)
Ri = numpy.linalg.norm( Ui)

grad[0] += numpy.array([data[0][0]-h[0], 0, 0]) @ Ui * (Ri-1)/Ri #sxx
grad[1] += numpy.array([0,data[0][1]-h[1], 0]) @ Ui * (Ri-1)/Ri #syy
grad[2] += numpy.array([0, 0,data[0][2]-h[2]]) @ Ui * (Ri-1)/Ri #szz
grad[3] += numpy.array([data[0][1]-h[1],data[0][0]-h[0], 0]) @ Ui * (Ri-1)/Ri #sxy
grad[4] += numpy.array([data[0][2]-h[2], 0,data[0][0]-h[0]]) @ Ui * (Ri-1)/Ri #sxz
grad[5] += numpy.array([0,data[0][2]-h[2],data[0][1]-h[1]]) @ Ui * (Ri-1)/Ri #syz
grad[6] += numpy.array([-sxx, -sxy, -sxz]) @ Ui * (Ri-1)/Ri #hx
grad[7] += numpy.array([-sxy, -syy, -syz]) @ Ui * (Ri-1)/Ri #hy
grad[8] += numpy.array([-sxz, -syz, -szz]) @ Ui * (Ri-1)/Ri #hz

j = numpy.array([data[0][0]-h[0], 0, 0]) @ Ui * (Ri-1)/Ri

print(grad)
print(j)

A = numpy.array([1,1,1])
B = numpy.array([2,3,4])
C = A @ B
D = B @ A

print(C)
print(D)

'''
for(data[0] in data:
    Ui = S * (data[0] - h).T
    #Ri = math.sqrt(Ui.T * Ui)
    Ri = numpy.linalg.norm(Ui.T * Ui)
    grad[0] += (Ui/Ri) * (Ri-1) * numpy.array((data[0][0]-h[0], 0, 0]) #sxx
'''