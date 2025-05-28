import numpy
import scipy 
import math


def costfuncmag (x , data ):
  cost = 0
  sxx = x[0]
  syy = x[1]
  szz = x[2]
  sxy = x[3]
  sxz = x[4]
  syz = x[5]
  hx = x[6]
  hy = x[7]
  hz = x[8]
  h = numpy.array(x[6:])
  S = numpy.array([[sxx, sxy, sxz],[sxy, syy, syz], [sxz, syz, szz]])
  data = numpy.array(data)
  sumg = 0
  for e in data:
    diff = numpy.array([e[0] - h[0], e[1]-h[1], e[2]-h[2]])
    prod = S * diff
    norm = numpy.linalg.norm(prod)
    sumg += (norm - 1)**2
  G = sumg/len(data)
  return G

def grad_costfuncmag (x , data ):
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

  ## Atenção: conferir se os vetores grad[i] são linha ou coluna
  for e in data:
    Ui = S @ (e - h)
    #Ri = math.sqrt(Ui.T * Ui)
    Ri = numpy.linalg.norm(Ui)
    '''
    grad[0] += (Ui/Ri) * (Ri-1) * numpy.array([e[0]-h[0], 0, 0]) #sxx
    grad[1] += (Ui/Ri) * (Ri-1) * numpy.array([0, e[1]-h[1], 0]) #syy
    grad[2] += (Ui/Ri) * (Ri-1) * numpy.array([0, 0, e[2]-h[2]]) #szz
    grad[3] += (Ui/Ri) * (Ri-1) * numpy.array([e[1]-h[1], e[0]-h[0], 0]) #sxy
    grad[4] += (Ui/Ri) * (Ri-1) * numpy.array([e[2]-h[2], 0, e[0]-h[0]]) #sxz
    grad[5] += (Ui/Ri) * (Ri-1) * numpy.array([0, e[2]-h[2], e[1]-h[1]]) #syz
    grad[6] += (Ui/Ri) * (Ri-1) * numpy.array([-sxx, -sxy, -sxz]) #hx
    grad[7] += (Ui/Ri) * (Ri-1) * numpy.array([-sxy, -syy, -syz]) #hy
    grad[8] += (Ui/Ri) * (Ri-1) * numpy.array([-sxz, -syz, -szz]) #hz
    '''
    grad[0] += numpy.array([e[0]-h[0], 0, 0]) @ Ui * (Ri-1)/Ri #sxx
    grad[1] += numpy.array([0, e[1]-h[1], 0]) @ Ui * (Ri-1)/Ri #syy
    grad[2] += numpy.array([0, 0, e[2]-h[2]]) @ Ui * (Ri-1)/Ri #szz
    grad[3] += numpy.array([e[1]-h[1], e[0]-h[0], 0]) @ Ui * (Ri-1)/Ri #sxy
    grad[4] += numpy.array([e[2]-h[2], 0, e[0]-h[0]]) @ Ui * (Ri-1)/Ri #sxz
    grad[5] += numpy.array([0, e[2]-h[2], e[1]-h[1]]) @ Ui * (Ri-1)/Ri #syz
    grad[6] += numpy.array([-sxx, -sxy, -sxz]) @ Ui * (Ri-1)/Ri #hx
    grad[7] += numpy.array([-sxy, -syy, -syz]) @ Ui * (Ri-1)/Ri #hy
    grad[8] += numpy.array([-sxz, -syz, -szz]) @ Ui * (Ri-1)/Ri #hz

  i = 0
  while i <= 8:
    grad[i] = grad[i] * 2 / len(data)
    i += 1
  return grad

dados_brutos = numpy.loadtxt("C:/Users/rober/Entregas Poli/9º semestre/PMR3502/dados_mag_rand.txt")
captura = dados_brutos[:, 5:8]

lxi_max = numpy.max(captura[:, 0])
lxi_min = numpy.min(captura[:, 0])
lyi_max = numpy.max(captura[:, 1])
lyi_min = numpy.min(captura[:, 1])
lzi_max = numpy.max(captura[:, 2])
lzi_min = numpy.min(captura[:, 2])

sxx0 = 2/(lxi_max-lxi_min)
syy0 = 2/(lyi_max-lyi_min)
szz0 = 2/(lzi_max-lzi_min)
hx0 = (lxi_max+lxi_min)/2
hy0 = (lyi_max+lyi_min)/2
hz0 = (lzi_max+lzi_min)/2
sol_inic = numpy.array([sxx0, syy0, szz0, 0, 0, 0, hx0, hy0, hz0])

cal_mag = scipy.optimize.minimize(costfuncmag , sol_inic , captura , jac = grad_costfuncmag)

print(cal_mag.x)

