import numpy as np
import numpy.linalg as npl

# Equality Constrained Least Square
# minimize |C1x-d1|^2 + ... + |Cnx-dn|^2
# subject to Ax=b
class LSE:
    def __init__(self, varNum, consNum):
        self.varNum = varNum
        self.consNum = consNum
    
        self.Cs = []
        self.ds = []
        self.A = None
        self.b = None
        
    # add |Cix-di|^2
    # C: matrix, d: vector
    def addObjective_matrix(self, Ci, di, w=1.):
        if Ci.shape[1]!=self.varNum:
            print 'varNum mismatched 1'
            return
        if Ci.shape[0]!=len(di):
            print 'Ci & di mismatched'
            return
        self.Cs.append(w*Ci)
        self.ds.append(w*di)
        
    # subject to Ax=b
    # A: matrix, b: vector
    def setConstraint_matrix(self, A, b):
        if A.shape[1]!=self.varNum:
            print 'varNum mismatched 2'
            return
        if len(A)!=self.consNum or len(b)!=self.consNum:
            print 'consNum misnatched'
            return
        self.A = A
        self.b = b
    
    def solve(self):
#        \underset{x}{\operatorname{min}}\left \|  C_{1}x-d_{1}\right \|^{2} + \cdots  + \left \|  C_{n}x-d_{n)}\right \|^{2} \newline
#        subject \; to : Ax-b=0 \newline
#        \newline
#        FONC : \newline
#        2C_{1}^{T}C_{1}x-2C_{1}^{T}d_{1} + \cdots + 2C_{n}^{T}C_{n}x-2C_{n}^{T}d_{n} 
#        
#        + A^
#        
#        {T}\lambda =0 \newline
#        Ax-b=0 \newline
#        \newline
#        \Rightarrow
#        \begin{pmatrix}
#         2C_{1}^{T}C_{1} + \cdots + 2C_{n}^{T}C_{n} & A^{T} \\ 
#         A & 0
#        \end{pmatrix}
#        \begin{pmatrix}
#        x\\ 
#        \lambda
#        \end{pmatrix}
#        =
#        \begin{pmatrix}
#        2C_{1}^{T}d_{1} + \cdots + 2C_{n}^{T}d_{n} \\ 
#        b
#        \end{pmatrix}

        # build system
        # A_large * x_large = b_large
        A11 = sum([2*np.dot(Ci.T, Ci) for Ci in self.Cs])
        A12 = self.A.T
        A21 = self.A
        A22 = np.zeros((self.A.shape[0], self.A.shape[0]))
        A_large = np.vstack((np.hstack((A11,A12)), np.hstack((A21,A22))))
        
        b1 = sum([2*np.dot(self.Cs[i].T, self.ds[i]) for i in range(len(self.Cs))])
        b2 = self.b
        b_large = np.hstack((b1,b2))
        
        x_large = npl.solve(A_large, b_large)
        
        result = {}
        result['x'] = x_large[:self.varNum]
        result['lambda'] = x_large[self.varNum:]
        return result
    
    def clear(self):
        del self.Cs[:]
        del self.ds[:]
        self.A = None
        self.b = None
    
    
if __name__ == '__main__':
    import psyco; psyco.full()
    
    def test_LSE():
        
        # minimize :    f(x,y) = x^2 + y^2 + z^2
        # subject to :    x + y + z = 1
        
        # convert to form of
        # min |Cx-d|^2     s.t Ax=b
        
        # minimize |[1 0 0]*[x] - [0]|^2 
        #           [0 1 0] [y]   [0]
        #           [0 0 1] [z]   [0]
        # subject to  [1 1 1] * [x y z].T = 1
        
        p = LSE(3, 1)
        p.addObjective_matrix(np.eye(3), np.zeros(3))
        p.setConstraint_matrix(np.array([[1,1,1]]), [1])
        r = p.solve()
        print r 
        
        #    subject to :    x + y = 1
        #                    y + z = 1
        #                    x + z = 1
        p = LSE(3, 3)
        p.addObjective_matrix(np.eye(3), np.zeros(3))
        p.setConstraint_matrix(np.array([[1,1,0],
                                         [0,1,1],
                                         [1,0,1]]), [1,1,1])
        r = p.solve()
        print r 
        
        #    subject to :    x + y = 1
        #                    y + z = 1
        p = LSE(3, 2)
        p.addObjective_matrix(np.eye(3), np.zeros(3))
        p.setConstraint_matrix(np.array([[1,1,0],
                                         [0,1,1]]), [1,1])
        r = p.solve()
        print r 
    
        
    test_LSE()
