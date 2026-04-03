from sympy.solvers import solve
from sympy import Symbol as sym
from sympy import ln, sqrt, pi 
import sympy

def inspect_vars(*args): # returns name of variable for which function is being solved.
    for arg in args:
        if isinstance(arg, sympy.core.symbol.Symbol):
            return arg

def hoop_stress(sigma, P_i, P_o, r_i, r_o, FOS = 1):
    numerator = (P_i*r_i**2-P_o*r_o**2) + (P_i-P_o)*r_o**2
    denominator = (r_o**2-r_i**2)
    x = solve(sigma / FOS - numerator / denominator)
    var = inspect_vars(sigma, P_i, P_o, r_i, r_o)
    print(f"{var}:", x[-1])
    return x[-1]

def thrust(F, p0, At, Ae, L, k, pe, pa):
    print("\nWARNING: Using numerical solve!!!\n")
    from sympy import lambdify
    from scipy.optimize import fsolve
    func = lambdify(p0, F - p0*At*(L*(2*k/(k-1))**(0.5)*(1-(pe/p0)**((k-1)/k)) + Ae/At*((pe-pa)/p0)))
    x = fsolve(func, 100) # initial guess doesn't always work, guess 100 PSI
    var = inspect_vars(F, p0, At, Ae, L, k, pe, pa)
    print(f"{var}:", x[-1])
    return x[-1]

def m_dot(mdot, At, p0, lam, R, T0):
    x = solve(mdot - At*p0*lam*sqrt(1/(R*T0)))
    var = inspect_vars(mdot, At, p0, lam, R, T0)
    print(f"{var}:", x[-1])
    return x[-1]

def lam(k):
    return sqrt(k*(2/(k+1))**((k+1)/(k-1)))

def radius_to_area(r):
    return pi*r**2

def Ae_At(Ae, At, lam, pe, pctot, k):
    x = solve(Ae/At - lam / ((pe/pctot)**(1/k)*sqrt(2*k/(k-1)*(1-(pe/pctot)**((k-1)/k)))))
    var = inspect_vars(Ae, At, lam, pe, pctot, k)
    print(f"{var}:", x[-1])
    return x[-1]    



if __name__ == "__main__":
    # Clinging to Imperial units because Elvis is tragically allergic to metric elegance
    P_outer = 14.6959 # psi
    P_outer = 0
    P_inner = 580
    r_inner = 0.25
    sigma_max = 5000
    FOS = 2
    
    # r_outer = hoop_stress(sigma_max, P_inner, P_outer, r_inner, sym("r_outer"), FOS)
    
    F = 3.965
    k=1.4
    L = lam(k)
    At = radius_to_area(0.08)
    Ae = radius_to_area(0.204)
    pe = 30
    pa = 14.7
    p0 = thrust(F, sym("p0"), At, Ae, L, k, pe, pa)