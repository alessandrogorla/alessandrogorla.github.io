from fenics import*
import dlroms.fespaces as fe
import numpy as np
from IPython.display import clear_output as clc

mesh = fe.unitsquaremesh(80, 80)
Vh = fe.space(mesh, 'CG', 1)
xb = fe.coordinates(Vh)
clc()

def solveStokes(hpipe):    
    domain = fe.rectangle((0, 0), (1, 1)) - fe.rectangle((0, hpipe), (1-hpipe, 1-hpipe))
    mesh = fe.mesh(domain, stepsize = 0.009)
    
    pP1  = FiniteElement("CG", mesh.ufl_cell(), 1)
    vP1B = VectorElement("CG",  mesh.ufl_cell(), 2)

    pspace, vspace = pP1, vP1B
    W = FunctionSpace(mesh, vspace * pspace)
    (b, p) = TrialFunctions(W)
    (v, q) = TestFunctions(W)

    space = fe.space(mesh, "CG", 2, scalar = False)
    
    a = 100*inner(grad(b), grad(v))*dx - div(v)*p*dx - q*div(b)*dx
    L = inner(Constant((0.0, 0.0)), v)*dx
    dbc = DirichletBC(W.sub(0), Constant((0.0, 0.0)), lambda x, on: on and x[0]>1e-12)
    dbc_in = DirichletBC(W.sub(0), Expression(("x[1]*(%.2f-x[1])/pow(%.2f, 3)" % (hpipe, hpipe), "0.0"), degree = 2),
                         lambda x, on: on and (x[0]<=1e-12) and (x[1]<0.5))
    dbc_out = DirichletBC(W.sub(1), Constant(0.0), lambda x, on: on and (x[0]<1e-12) and (x[1]>0.5))

    bp = Function(W)
    solve(a == L, bp, [dbc, dbc_out, dbc_in])
    clc()
    return bp


def FOMstep(u0, t, dt, bfield, sigma, hpipe):
    u, v  = TrialFunction(Vh), TestFunction(Vh)
    u0f = fe.asvector(u0, Vh)
    a = dt*(inner(sigma*grad(u), grad(v))*dx + 10*inner(bfield, grad(u))*v*dx) + u*v*dx
    f = u0f*v*dx

    dbc = DirichletBC(Vh, Constant(1.0-np.exp(-t)), lambda x, on: on and x[0]<=1e-12 and x[1]<=hpipe)

    u = Function(Vh)
    solve(a == f, u, dbc)
    return u.vector()[:]

        
def FOMsolver(hpipe, dpipe, dbulk):
    bp = solveStokes(hpipe)
    b = []
    for x in xb:
        try:
            b.append(list(bp(*x))[:2])
        except:
            b.append([0.0, 0.0])
    b = np.stack(b).reshape(-1)
    Vb = fe.space(Vh.mesh(), 'CG', 1, scalar = False)
    bfield = fe.asvector(b, Vb)

    x, y = fe.coordinates(Vh).T
    ind = (x>(1.0-hpipe)) + (x<=(1.0-hpipe))*(np.abs(y-0.5)>(0.5-hpipe))
    sigma = dpipe*ind + (1-ind)*dbulk
    sigma = fe.asvector(sigma, Vh)
              
    u = [np.zeros(Vh.dim())]
    t = 0.0
    dt = 5e-2
    for j in range(100):
        u.append(FOMstep(u[-1], t, dt, bfield, sigma, hpipe))
        t += dt
    return 1.0 - np.stack(u)