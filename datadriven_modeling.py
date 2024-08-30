import numpy as np
import matplotlib.pyplot as plt
import control
import zmq


def Hankel_matrix(z: np.array, i: int, N: int, t: int) -> np.array:
    '''
    arguments:
    z is a row array (len > 1) where each element is a column array of the same length
    i is a natural number
    N (number of columns) is a natural number >= 1
    t (number of rows) is a natural number >= 1

    precondition:
    i+t+N-2 must be less than len(z)

    return:
    matrix:
    ```
    [[z(i)      z(i+1)    z(i+2)    ...   z(i+N-1)]
    [z(i+1)    z(i+2)    z(i+3)    ...   z(i+N)  ]
    [z(i+2)    z(i+3)    z(i+4)    ...   z(i+N+1)]
                    .....
    [z(i+t-1)  z(i+t)    z(i+t+1)  ...   z(i+t+N-2)]]
    ```
    '''
    if i + t + N - 2 >= z.shape[1]:
        raise Exception(f"source array ({z.shape[1]} elements) can't build Hankel matrix {t}x{N}!")

    sigma = z.shape[0]

    Hz = np.zeros((t*sigma, N))

    for tt in range(t):
        for nn in range(N):
            for s in range(sigma):
                Hz[tt*sigma+s, nn] = z[s, nn + tt]

    return Hz


def Make_data_driven_dynamics(x_0: np.array, x_1: np.array, u_0: np.array) -> np.array:
    i = 0
    #Nt = lambda z: (z.shape[1] - (z.shape[0] - 1), z.shape[0])
    #Nx0, tx0 = Nt(x_0)
    #Nx1, tx1 = Nt(x_1)
    #Nu0, tu0 = Nt(u_0)

    Nx0 = x_0.shape[1]
    Nx1 = x_1.shape[1]
    Nu0 = u_0.shape[1]

    tx0 = tx1 = tu0 = 1

    num_cols = np.min((Nx0, Nx1, Nu0))

    X_0 = Hankel_matrix(x_0, i, num_cols, tx0) # sinal, i, N, t
    X_1 = Hankel_matrix(x_1, i, num_cols, tx1)
    U_0 = Hankel_matrix(u_0, i, num_cols, tu0)

    Inverse = np.block([[U_0], [X_0]])
    Inverse = np.linalg.pinv(Inverse)

    dynamics = X_1 @ Inverse

    m = u_0.shape[0]

    A = dynamics[:, m:]
    B = dynamics[:, :m]

    return B, A


def Model_01(x0: np.array, N: int) -> np.array:
    A = np.array([[0.2]])
    B = np.array([[0.5]])

    X = np.zeros((N, 1))
    U = np.zeros((N, 1))
    X[0, 0] = x0[0, 0]
    x = x0

    for i in range(N - 1):
        u = np.array([[np.sin(2 * i)]])
        x = A @ x + B @ u
        X[i + 1, 0] = x[0, 0]
        U[i, 0] = u[0, 0]

    return X, U


def Model_02(x0: np.array, N: int) -> np.array:
    A = np.array([[1, 0.05],
                  [0, 1]])
    B = np.array([[0],
                  [10 / 3]])

    X = np.zeros((N, 2))
    U = np.zeros((N, 1))
    X[0, :] = x0[:, 0]
    x = x0

    for i in range(N - 1):
        u = np.array([[np.sin(2 * i)]])
        x = A @ x + B @ u
        X[i + 1, :] = x[:, 0]
        U[i, 0] = u[0, 0]

    return X, U


def manopla_model(experiment_log) -> np.array:
    import pandas as pd
    df = pd.read_csv(experiment_log)
    # df.rename(str.strip, axis = 'columns')
    n_points, n_columns = df.shape


    time = np.asarray(df["time_us"]).reshape((1, n_points))
    pulse = np.asarray(df["pulse_qc"]).reshape((1, n_points))
    setpoint_current = np.asarray(df["setpoint_current_mA"]).reshape((1, n_points))
    actual_current = np.asarray(df["actual_current_mA"]).reshape((1, n_points))
    epos_velocity = np.asarray(df["epos_velocity_unfiltered_rpm"]).reshape((1, n_points))
    calculated_velocity = np.asarray(df["calculated_velocity_rad/s"]).reshape((1, n_points))

    outputs = np.concatenate((pulse, calculated_velocity), axis=0)
    output_names = ["pulse [qc]", "calculated velocity [rpm]"]
    
    inputs = setpoint_current
    input_names = ["setpoint_current [mA]"]

    return time, outputs, inputs, output_names, input_names


def main():

    time, X, U, output_names, input_names = manopla_model(
        '/home/grilo/Git/manopla-system-identification-test/cmake-build-debug-arm-docker/KEEPIT-log_2023_09_06-13_17_50-openloop.log')

    x0 = np.array([[0]])
    start_idx=0
    N = start_idx+190
    start_idx = 20
    N=start_idx+1000

    plt.figure()
    plt.plot(time[0], X[0], time[0], X[1], time[0], U[0])
    plt.legend([*output_names, *input_names])
    #plt.show(block=False)

    x_0 = X[:, start_idx  :N - 1]
    x_1 = X[:, start_idx+1:N]
    u_0 = U[:, start_idx  :N - 1]

    B, A = Make_data_driven_dynamics(x_0, x_1, u_0)

    print('A Good: ', A)
    print('B Good: ', B)

    C = np.eye(2)
    D = np.array([[0],[0]])

    time_dd,output_dd=control.step_response(control.ss(A, B, C, D, 0.01))

    x=output_dd[0,0,:]
    xdot=output_dd[1,0,:]

    plt.figure()
    plt.plot(time_dd,x, time_dd,xdot)
    plt.title('datadriven step response')
    #plt.show(block=False)

    time_dd = np.arange(0, 40, .01)
    input_dd = 400*np.sin(time_dd*2*np.pi-np.pi-np.pi/4)

    #time_dd,output_dd=control.forced_response(control.ss(A,B,C,D,0.01), time_dd, X0=[0,0], U=input_dd)
    time_dd,output_dd_0=control.forced_response(control.ss(A,B,C,D,0.01), time_dd, X0=[0,0], U=U[0][0:len(time_dd)])

    x=output_dd_0[0,:]
    xdot=output_dd_0[1,:]

    plt.figure()
    plt.plot(time_dd,x, time_dd,xdot, 'b')
    plt.plot(time_dd,input_dd, 'r')
    plt.plot((time[0][0:len(time_dd)]-time[0][0])/1e6, X[0][0:len(time_dd)], '--')
    plt.legend(['dd pos output', 'dd vel output', 'dd cur input', 'exp pos output'])
    plt.title('datadriven sin response')
    #plt.show(block=False)

    #mag, phase, omega = control.bode_plot(control.ss(A,B,C,D,0.01))

    #-------------------------------------------------------------------------------------------------------------------

    x0 = np.array([[0]])
    N = 190
    # N=4000

    plt.figure()
    plt.plot(time[0], X[0], time[0], X[1], time[0], U[0])
    plt.legend([*output_names, *input_names])
    #plt.show(block=False)

    x_0 = X[:, 0:N - 1]
    x_1 = X[:, 1:N]
    u_0 = U[:, 0:N - 1]

    B, A = Make_data_driven_dynamics(x_0, x_1, u_0)

    print('A Good: ', A)
    print('B Good: ', B)

    C = np.eye(2)
    D = np.array([[0], [0]])

    time_dd, output_dd = control.step_response(control.ss(A, B, C, D, 0.01))

    x = output_dd[0, 0, :]
    xdot = output_dd[1, 0, :]

    plt.figure()
    plt.plot(time_dd, x, time_dd, xdot)
    plt.title('datadriven step response')
    #plt.show(block=False)

    time_dd = np.arange(0, 40, .01)
    input_dd = 400 * np.sin(time_dd * 2 * np.pi - np.pi - np.pi / 4)

    # time_dd,output_dd=control.forced_response(control.ss(A,B,C,D,0.01), time_dd, X0=[0,0], U=input_dd)
    time_dd, output_dd_1 = control.forced_response(control.ss(A, B, C, D, 0.01), time_dd, X0=[0, 0],
                                                   U=U[0][0:len(time_dd)])

    x = output_dd_1[0, :]
    xdot = output_dd_1[1, :]

    plt.figure()
    plt.plot(time_dd, x, time_dd, xdot, 'b')
    plt.plot(time_dd, input_dd, 'r')
    plt.plot((time[0][0:len(time_dd)] - time[0][0]) / 1e6, X[0][0:len(time_dd)], '--')
    plt.legend(['dd pos output', 'dd vel output', 'dd cur input', 'exp pos output'])
    plt.title('datadriven sin response')
    #plt.show(block=False)

    # mag, phase, omega = control.bode_plot(control.ss(A,B,C,D,0.01))

    plt.figure()
    plt.plot(time_dd[0:len(time_dd)],output_dd_0[0,:len(time_dd)],'*')
    plt.plot(time_dd[0:len(time_dd)],output_dd_1[0,:len(time_dd)],'*')
    plt.plot((time[0][0:len(time_dd)] - time[0][0]) / 1e6, X[0][0:len(time_dd)], '*')
    plt.legend(['output N=4000', 'output N=190'])
    plt.show()

    #error_0 = np.trapz(np.square(output_dd_0[0,:len(time_dd)] - X[0][0:len(time_dd)]),




if __name__ == '__main__':
    main()
