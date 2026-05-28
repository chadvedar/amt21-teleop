import sympy as sp

def dh_transformation_matrix(theta, alpha, d, a):
    Tx = sp.Matrix([
                [1,             0,               0,  a],
                [0, sp.cos(alpha),  -sp.sin(alpha),  0],
                [0, sp.sin(alpha),   sp.cos(alpha),  d],
                [0,             0,               0,  1]
    ])

    Tz = sp.Matrix([
                [sp.cos(theta), -sp.sin(theta),  0,  0],
                [sp.sin(theta),  sp.cos(theta),  0,  0],
                [0,                          0,  1,  0],
                [0,                          0,  0,  1]
    ])

    return Tx * Tz

def compute_transformation(dh_table):
    T = sp.eye(4)
    for params in dh_table:
        T *= dh_transformation_matrix(*params)
    return T

def get_end_effector_eq(T):
    return T @ sp.Matrix([0.0, 0.0, 0.0, 1.0])

def compute_jacobian(T):
    x = T[0]
    y = T[1]
    z = T[2]

    th1, th2, th3, th4, th5, th6, th7 = sp.symbols('theta1, theta2, theta3, theta4, theta5, theta6, theta7')

    J = sp.Matrix([
        [sp.diff(x, th1), sp.diff(x, th2), sp.diff(x, th3), sp.diff(x, th4), sp.diff(x, th5), sp.diff(x, th6), sp.diff(x, th7)],
        [sp.diff(y, th1), sp.diff(y, th2), sp.diff(y, th3), sp.diff(y, th4), sp.diff(y, th5), sp.diff(y, th6), sp.diff(y, th7)],
        [sp.diff(z, th1), sp.diff(z, th2), sp.diff(z, th3), sp.diff(z, th4), sp.diff(z, th5), sp.diff(z, th6), sp.diff(z, th7)]
    ])

    J = sp.trigsimp(J)

    return J

if __name__ == "__main__":
    theta1 = sp.symbols('theta1')
    theta2 = sp.symbols('theta2')
    theta3 = sp.symbols('theta3')
    theta4 = sp.symbols('theta4')
    theta5 = sp.symbols('theta5')
    theta6 = sp.symbols('theta6')
    theta7 = sp.symbols('theta7')

    J_ANGLE  = [0.0 for _ in range(7)]
    J_OFFSET = [0.1, 0.20, 0.20, 0.07]
    dh_table  = [ [ 0.0,         0.0,    0.0,          1.0 ],
                  [ 0.0,         sp.pi/2,   0.0,  J_OFFSET[0] ],
                  [ theta1,  0.0,    0.0,          0.0 ],
                  [ sp.pi/2,        0.0,    0.0,          0.0 ],
                  [ 0.0,         sp.pi/2,   0.0,          0.0 ],
                  [ theta2,  0.0,    0.0,          0.0 ],
                  [ sp.pi/2,        0.0,    0.0,          0.0 ],
                  [ 0.0,        -sp.pi/2,   0.0,          0.0 ],
                  [ theta3,  0.0,    0.0,          0.0 ],
                  [ 0.0,         sp.pi/2,   J_OFFSET[1],  0.0 ],
                  [ theta4,  0.0,    0.0,          0.0 ],
                  [ sp.pi/2,        0.0,    0.0,  J_OFFSET[2] ],
                  [ 0.0,         sp.pi/2,   0.0,          0.0 ],
                  [ theta5,  0.0,    0.0,          0.0 ],
                  [ sp.pi/2,        0.0,    0.0,          0.0 ],
                  [ 0.0,         sp.pi/2,   0.0,          0.0 ],
                  [ theta6,  0.0,    0.0,          0.0 ],
                  [ sp.pi/2,        0.0,    0.0,          0.0 ],
                  [ 0.0,         sp.pi/2,   0.0,          0.0 ],
                  [ theta7,  0.0,    0.0,          0.0 ],
                  [ 0.0,         0.0,    0.0,  J_OFFSET[3] ]               
                 ]
    
    T = compute_transformation(dh_table)
    eq = get_end_effector_eq(T)
    J = compute_jacobian(eq)

    # sp.pprint(J)
    
    print(J.shape)
    for i in range(3):
        for j in range(7):
            print(f'row : {i}, col : {j}')
            sp.pprint(J[i, j])
            print("-------------------------------------")
    
    # test value substitution
    # result = J.subs({theta1: 0.0, theta2: 0.0, theta3: 0.0, theta4: 0.0, theta5: 0.0})
    # print(result)