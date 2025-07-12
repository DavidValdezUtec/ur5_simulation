import numpy as np
import matplotlib.pyplot as plt

data = np.loadtxt('/tmp/ur5_log.txt', skiprows=1)

t = data[:,0]
# Comandos y actuales
q_cmd = data[:,1:7]
q_cur = data[:,7:13]

# Pose base (constante)
p_des = data[:,13:16]
q_des = data[:,16:20]
# Trayectoria deseada
p_traj = data[:,20:23]
q_traj = data[:,23:27]
# Actual
p_act = data[:,27:30]
q_act = data[:,30:34]
# Errores
pos_error = data[:,34:37]
ori_error = data[:,37:40]
v_cart_actual = data[:,40:46]
v_cart_desired = data[:,46:52]


plt.figure(figsize=(12,6))
plt.subplot(2,1,1)
plt.plot(t, p_traj[:,0], label='x_trayectoria')
plt.plot(t, p_act[:,0], '--', label='x_efector')
plt.plot(t, p_traj[:,1], label='y_trayectoria')
plt.plot(t, p_act[:,1], '--', label='y_efector')
plt.plot(t, p_traj[:,2], label='z_trayectoria')
plt.plot(t, p_act[:,2], '--', label='z_efector')
plt.ylabel('Posición (m)')
plt.legend()
plt.grid()
plt.title('Trayectoria deseada vs actual')

plt.subplot(2,1,2)
plt.plot(t, pos_error[:,0], label='Error x')
plt.plot(t, pos_error[:,1], label='Error y')
plt.plot(t, pos_error[:,2], label='Error z')
plt.ylabel('Error (m)')
plt.xlabel('Tiempo (s)')
plt.legend()
plt.grid()
plt.title('Error cartesiano')

plt.tight_layout()
plt.show()

plt.figure()
plt.plot(t, ori_error)
plt.xlabel('Tiempo (s)')
plt.ylabel('Error de orientación (rad)')
plt.title('Error de orientación (log3)')
plt.legend(['x','y','z'])
plt.grid()
plt.show()


plt.figure(figsize=(10,7))
plt.subplot(2,1,1)
plt.plot(t, v_cart_actual[:,0], label='Vx actual')
plt.plot(t, v_cart_desired[:,0], '--', label='Vx deseada')
plt.plot(t, v_cart_actual[:,1], label='Vy actual')
plt.plot(t, v_cart_desired[:,1], '--', label='Vy deseada')
plt.plot(t, v_cart_actual[:,2], label='Vz actual')
plt.plot(t, v_cart_desired[:,2], '--', label='Vz deseada')
plt.ylabel('Velocidad (m/s)')
plt.legend()
plt.grid()
plt.title('Velocidades cartesianas: Actual vs Deseada')

plt.subplot(2,1,2)
plt.plot(t, v_cart_actual[:,3], label='Wx actual')
plt.plot(t, v_cart_desired[:,3], '--', label='Wx deseada')
plt.plot(t, v_cart_actual[:,4], label='Wy actual')
plt.plot(t, v_cart_desired[:,4], '--', label='Wy deseada')
plt.plot(t, v_cart_actual[:,5], label='Wz actual')
plt.plot(t, v_cart_desired[:,5], '--', label='Wz deseada')
plt.ylabel('Velocidad angular (rad/s)')
plt.xlabel('Tiempo (s)')
plt.legend()
plt.grid()
plt.title('Velocidades angulares cartesianas')

plt.tight_layout()
plt.show()