from fenics import *
from ufl_legacy import nabla_div
from scipy.sparse.linalg import spsolve
from scipy.sparse import csr_matrix

L = 1
W = 0.2
rho = 1
delta = W/L
gamma = 0.4*delta**2
beta = 1.25
g = gamma

mesh = BoxMesh(Point(0, 0, 0), Point(L, W, W), 10, 3, 3)
Vh = VectorFunctionSpace(mesh, 'P', 1)

def Elastic_solver(lambda_, nu):

    # Boundary conditions
    tol = 1E-14
    def clamped_boundary(x, on_boundary):
        return on_boundary and x[0] < tol
    bc = DirichletBC(Vh, Constant((0, 0, 0)), clamped_boundary)

    # Auxiliary definitions (see https://fenicsproject.org/pub/tutorial/sphinx1/._ftut1004.html#the-equations-of-linear-elasticity)
    def epsilon(u):
        return 0.5*(nabla_grad(u) + nabla_grad(u).T)

    def sigma(u):
        return lambda_*nabla_div(u)*Identity(d) + 2*nu*epsilon(u)

    # Variational problem
    u = TrialFunction(Vh)
    d = u.geometric_dimension() # space dimension
    v = TestFunction(Vh)
    f = Constant((0, 0, -rho*g))
    T = Constant((0, 0, 0))
    a = inner(sigma(u), epsilon(v))*dx
    L = dot(f, v)*dx + dot(T, v)*ds

    # Assembling and adjusting
    A = assemble(a)
    F = assemble(L)
    bc.apply(A)
    bc.apply(F)

    A = csr_matrix(A.array())
    F = F[:]

    # Solving
    u = spsolve(A, F)
    return u