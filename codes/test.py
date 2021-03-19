import modern_robotics as mr
import numpy as np
import math
import sympy as sym

class MatrixOperations():
    def __init__(self,**kwargs):
        super(MatrixOperations, self).__init__(**kwargs)

    def multiplicationTransMat(self, A1, A2):
        print(np.matmul(A1, A2))
        return np.matmul(A1, A2)

    def symMatMultiplication(self, A1, A2):
        pass

def main(args=None):
    MO = MatrixOperations()
    Tdb = np.array([[0, 0, -1, 250],
                     [0, -1, 0, -150],
                     [-1, 0, 0, 200],
                     [0, 0, 0, 1]])
    
    Tde = np.array([[0, 0, -1, 300],
                     [0, -1, 0, 100],
                     [-1, 0, 0, 120],
                     [0, 0, 0, 1]])
    
    Tad = np.array([[0, 0, -1, 400],
                     [0, -1, 0, 50],
                     [-1, 0, 0, 300],
                     [0, 0, 0, 1]])

    Tbc = np.array([[0, -1/math.sqrt(2), -1/math.sqrt(2), 30],
                     [0, 1/math.sqrt(2), -1/math.sqrt(2), -40],
                     [1, 0, 0, 25],
                     [0, 0, 0, 1]])

    print("Multiplication of: Tad * Tdb =")
    Tab = MO.multiplicationTransMat(Tad, Tdb)
    print("Multiplication of: Tab * Tbc =")
    Tac = MO.multiplicationTransMat(Tab, Tbc)
    print("Inverse of Tac =")
    Tac_inv = mr.TransInv(Tac)
    print(Tac_inv)
    print("Multiplication of: Tac_inv * Tad * Tde =")
    Tec = MO.multiplicationTransMat(Tac_inv, MO.multiplicationTransMat(Tad, Tde))


if __name__=="__main__":
    main()
