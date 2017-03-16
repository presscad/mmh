import copy

# Ordinary Differential Equation
# solve dy/dt = f(t, y(t))
# solution y is sequential form : y1, y2, y3, ...

# f : differential equations. dy/dt = f(t, y(t)) [n dim]
# t : independent variable (only one because it's ordinary differential equation)
# y : y_k as input, y_k+1 as output (in/out) [n dim]
# h : spacing of independent variable (time step)

def euler(f, t, y, h):
    n = len(y)
    y_old = copy.copy(y)
    for i in range(n):
        y[i] = y_old[i] + h * f[i](t, y)
    
def rk4(f, t, y, h):
    n = len(y)
    y_old = copy.copy(y)
    half_h = h/2.
    k1 = [None]*n
    k2 = [None]*n
    k3 = [None]*n
    k4 = [None]*n
    for i in range(n):
        k1[i] = h * f[i](t, y)
        y[i] += k1[i]/2.
        k2[i] = h * f[i](t+half_h, y)
        y[i] = y_old[i] + k2[i]/2.
        k3[i] = h * f[i](t+half_h, y)
        y[i] = y_old[i] + k3[i]
        k4[i] = h * f[i](t+h, y)
        y[i] = y_old[i] + (k1[i] + 2*(k2[i]+k3[i]) + k4[i]) / 6.
        

if __name__=='__main__':
    g = -9.8
    h = .01
    
    def test_analytic():
        global g, h
        
        x0 = 0
        v0 = 0
        t = 0
    
        for i in range(100):
            t += h
            print 't:', t, 'x:', x0 + t*v0 + .5*g*t*t, 'v:', v0 + t*g
        
    def test_euler():
        global g, h
        
        x0 = 0
        v0 = 0
        t = 0
    
        y = [x0, v0]
        dydt = [lambda t, y: y[1], lambda t, y: g]
        
        for i in range(100):
            euler(dydt, t, y, h)
            t += h
            print 't:', t, 'x:', y[0], 'v:', y[1]
            
    def test_rk4():
        global g, h
        
        x0 = 0
        v0 = 0
        t = 0
    
        y = [x0, v0]
        dydt = [lambda t, y: y[1], lambda t, y: g]
        
        for i in range(100):
            rk4(dydt, t, y, h)
            t += h
            print 't:', t, 'x:', y[0], 'v:', y[1]
            
#    test_analytic()
#    test_euler()
    test_rk4()