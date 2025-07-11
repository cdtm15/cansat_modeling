#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jul  7 11:07:10 2025

@author: cristiantobar
"""

import numpy as np
from scipy.signal import resample
import matplotlib.pyplot as plt
import pandas as pd


def wind_components(wind_type, num_data):
    np.random.seed()
    t = np.linspace(0, 6*np.pi, num_data)
    
    base = 5 * np.cos(t)
    noise = np.random.normal(0, 1, num_data)
    
    if wind_type == 'sinuidal':
        windx = base
        windy = -base
    elif wind_type == 'noise':
        windx = base + 10 * noise
        windy = -base + 10 * np.random.normal(0, 1, num_data)
    elif wind_type == 'step':
        windx = base + np.concatenate([np.zeros(num_data//2), np.ones(num_data//2)])
        windy = base - np.concatenate([np.zeros(num_data//2), np.ones(num_data//2)])
    elif wind_type == 'turbulence':
        # Suavizado para simular turbulencia
        from scipy.ndimage import gaussian_filter1d
        turbulent_noise = gaussian_filter1d(np.random.normal(0, 3, num_data), sigma=10)
        windx = base + turbulent_noise
        windy = -base + gaussian_filter1d(np.random.normal(0, 3, num_data), sigma=10)
    else:
        windx = np.zeros(num_data)
        windy = np.zeros(num_data)

    return windx, windy


def dropping(yaw_rate_max, GRmax, num_intentos, num_data, deltaT, wind_type):
    ro = 1.29
    S = 0.2358**2
    Cd = (2 * 1.726) / (ro * (5.3**2) * S)
    m = 0.5
    g = 9.8

    A, B, C = 0.5240, -8.3341, -1.043

    time = np.arange(0.01, num_data * deltaT + 0.01, deltaT)
    landing_x = np.zeros(num_intentos)
    landing_y = np.zeros(num_intentos)
    tiempo_caida = 0
    v_terminal = 0
    # Nuevo: almacenar todas las trayectorias si se requiere
    all_trajs = {
        'x': [],
        'y': [],
        'z': []
    }
    tiempos_caida = []
    velocidades_terminales = []
    eficiencias_control = []


    for intento in range(num_intentos):
        delta_c = np.zeros(num_data)
        yaw_rate = np.zeros(num_data)
        yaw_acc_act = np.zeros(num_data)
        yaw = np.zeros(num_data)
        GR = np.zeros(num_data)
        x_dot = np.zeros(num_data)
        y_dot = np.zeros(num_data)
        x = np.zeros(num_data)
        y = np.zeros(num_data)
        z = np.zeros(num_data)
        zp = np.zeros(num_data)
        zpp = np.zeros(num_data)

        z[0] = 1000

        windx, windy = wind_components(wind_type, num_data)

        for k in range(1, num_data):
            zpp[k] = -((0.5) * np.sign(zp[k-1]) * ro * Cd * S * zp[k-1]**2) / m - g
            zp[k] = zp[k-1] + zpp[k]*deltaT
            z[k] = z[k-1] + zp[k]*deltaT + 0.5*zpp[k]*deltaT**2

            if z[k] <= 0 and tiempo_caida == 0:
                tiempo_caida = time[k]
                v_terminal = abs(zp[k])
                break

        for k in range(1, num_data - 1):
            delta_c[k+1] = 0.057 + 0.052 + np.sin(2*np.pi*0.04*(k+1) + 2.999772) + np.sin(2*np.pi*0.2*(k+1) + 4.952040)
            yaw_rate[k+1] = delta_c[k+1] * 160
            yaw_acc_act[k+1] = A * yaw_rate[k] + B * (delta_c[k] - 0.3) + C
            yaw[k+1] = yaw[k] + yaw_rate[k+1]*deltaT
            GR[k+1] = np.abs((yaw_rate[k] - yaw_rate_max) / yaw_rate_max) * GRmax

            x_dot[k+1] = windx[k+1] - zp[k+1]*GR[k+1]*np.sin(np.deg2rad(yaw[k+1]))
            y_dot[k+1] = windy[k+1] - zp[k+1]*GR[k+1]*np.sin(np.deg2rad(yaw[k+1]))
            x[k+1] = x[k] + x_dot[k+1]*deltaT
            y[k+1] = y[k] + y_dot[k+1]*deltaT

        landing_x[intento] = x[-1]
        landing_y[intento] = y[-1]
        
        #AQUÍ debes colocar el bloque de guardado de trayectorias por intento:
        z_valid = np.where(z > 0)[0]
        last_k = z_valid[-1] if len(z_valid) > 0 else num_data - 1
    
        all_trajs['x'].append(x[:last_k])
        all_trajs['y'].append(y[:last_k])
        all_trajs['z'].append(z[:last_k])
        
        # Métrica de eficiencia del control: cuán dispersos están los puntos de aterrizaje
        distancia_media = np.mean(np.sqrt((landing_x - np.mean(landing_x))**2 + (landing_y - np.mean(landing_y))**2))
        eficiencia_control = 1 / (1 + distancia_media)  # más cerca a 1, mejor
        
        tiempos_caida.append(tiempo_caida)
        velocidades_terminales.append(v_terminal)
        eficiencias_control.append(eficiencia_control)

    return landing_x, landing_y, x, y, z, tiempos_caida, velocidades_terminales, eficiencias_control, all_trajs




# Parámetros generales
num_data = 12000
num_intentos = 100
deltaT = 0.01
yaw_rate_max = 270

# Diccionarios para guardar resultados
landing_sinusoidal = {}
landing_noise = {}
landing_turbulence = {}
traj_3d = {}  # Solo para GR = 0.4

# Para guardar las métricas reales
metricas_resultado = []

fig = plt.figure(figsize=(15, 15))  # Más alto y balanceado


# Valores de GR y viento
GR_values = [0.1, 0.4, 0.8]
wind_types = ['sinusoidal', 'noise', 'turbulence']
colors = {'sinusoidal': 'blue', 'noise': 'green', 'turbulence': 'red'}


for i, GRmax in enumerate(GR_values):
    for j, wind in enumerate(wind_types):
        result = dropping(yaw_rate_max, GRmax, num_intentos, num_data, deltaT, wind)
        
        ax = fig.add_subplot(3, 3, 3*i + j + 1, projection='3d')
        all_trajs = result[8]  # all_trajs = {'x': [...], 'y': [...], 'z': [...]}

        
        # Guardar puntos de aterrizaje
        if wind == 'sinusoidal':
            landing_sinusoidal[GRmax] = (result[0], result[1])
        elif wind == 'noise':
            landing_noise[GRmax] = (result[0], result[1])
        elif wind == 'turbulence':
            landing_turbulence[GRmax] = (result[0], result[1])

        # Guardar trayectorias solo para GR = 0.4
        if GRmax == 0.4:
            traj_3d[wind] = result[8]  # all_trajs

        # Extraer listas de métricas reales
        tiempos = result[5]
        velocidades = result[6]
        eficiencias = result[7]

        # Guardar métricas agregadas en el DataFrame
        metricas_resultado.append({
            'GR': GRmax,
            'Viento': wind,
            't_caida_media': np.mean(tiempos),
            't_caida_std': np.std(tiempos),
            'v_term_media': np.mean(velocidades),
            'v_term_std': np.std(velocidades),
            'eff_media': np.mean(eficiencias),
            'eff_std': np.std(eficiencias)
        })
        
        # Graficar las 100 trayectorias
        for x, y, z in zip(all_trajs['x'], all_trajs['y'], all_trajs['z']):
            ax.plot(x, y, z, color=colors[wind], alpha=0.2)

        ax.set_xlim([-100, 100]); ax.set_ylim([-100, 100]); ax.set_zlim([0, 1000])
        ax.set_xlabel("X [m]"); ax.set_ylabel("Y [m]"); ax.set_zlabel("Z [m]")
        ax.set_title(f"GR = {GRmax} ; Wind: {wind}")
        #ax.view_init(elev=15, azim=-61)
        
plt.savefig('sat_fall_3d_9_trials.pdf', bbox_inches='tight')
plt.show()        

# Crear DataFrame final
df_metricas = pd.DataFrame(metricas_resultado)

fig = plt.figure(figsize=(15,30))

# --- Subplot 1: Sinusoidal ---
ax1 = fig.add_subplot(131, projection='3d')
for x, y, z in zip(traj_3d['sinusoidal']['x'], traj_3d['sinusoidal']['y'], traj_3d['sinusoidal']['z']):
    ax1.plot(x, y, z, color='blue', alpha=0.2)
ax1.set_title("Trayectorias - Viento sinusoidal (GR=0.4)")
ax1.set_xlim([-60, 60]); ax1.set_ylim([-60, 60]); ax1.set_zlim([0, 1000])
ax1.set_xlabel("X [m]"); ax1.set_ylabel("Y [m]"); ax1.set_zlabel("Z [m]")
ax1.view_init(elev=15, azim=-61)

# --- Subplot 2: Noise ---
ax2 = fig.add_subplot(132, projection='3d')
for x, y, z in zip(traj_3d['noise']['x'], traj_3d['noise']['y'], traj_3d['noise']['z']):
    ax2.plot(x, y, z, color='green', alpha=0.2)
ax2.set_title("Trayectorias - Viento con ruido (GR=0.4)")
ax2.set_xlim([-60, 60]); ax2.set_ylim([-60, 60]); ax2.set_zlim([0, 1000])
ax2.set_xlabel("X [m]"); ax2.set_ylabel("Y [m]"); ax2.set_zlabel("Z [m]")
ax2.view_init(elev=15, azim=-61)

# --- Subplot 3: Turbulencia ---
ax3 = fig.add_subplot(133, projection='3d')
for x, y, z in zip(traj_3d['turbulence']['x'], traj_3d['turbulence']['y'], traj_3d['turbulence']['z']):
    ax3.plot(x, y, z, color='red', alpha=0.2)
ax3.set_title("Trayectorias - Viento turbulento (GR=0.4)")
ax3.set_xlim([-60, 60]); ax3.set_ylim([-60, 60]); ax3.set_zlim([0, 1000])
ax3.set_xlabel("X [m]"); ax3.set_ylabel("Y [m]"); ax3.set_zlabel("Z [m]")
ax3.view_init(elev=15, azim=-61)

plt.tight_layout()
plt.savefig('sat_fall_3d.pdf', bbox_inches='tight')
plt.show()


plt.figure()
for GR in [0.1, 0.4, 0.8]:
    x, y = landing_sinusoidal[GR]
    plt.scatter(x, y, label=f'GR = {GR}', alpha=0.7)

plt.title("Landing coordinates - sinusoidal Wind")
plt.xlabel("X [m]"); plt.ylabel("Y [m]")
plt.legend(); plt.grid(True)
plt.axis([-60, 60, -60, 60])
plt.gca().set_aspect('equal')
plt.savefig('landing_sin.pdf', bbox_inches='tight')

plt.figure()
for GR in [0.1, 0.4, 0.8]:
    x, y = landing_noise[GR]
    plt.scatter(x, y, label=f'GR = {GR}', alpha=0.7)

plt.title("Landing coordinates - Noisy wind")
plt.xlabel("X [m]"); plt.ylabel("Y [m]")
plt.legend(); plt.grid(True)
plt.axis([-60, 60, -60, 60])
plt.gca().set_aspect('equal')
plt.savefig('landing_noise.pdf', bbox_inches='tight')

plt.figure()
for GR in [0.1, 0.4, 0.8]:
    x, y = landing_turbulence[GR]
    plt.scatter(x, y, label=f'GR = {GR}', alpha=0.7)

plt.title("Landing coordinates - Turbulence Wind")
plt.xlabel("X [m]"); plt.ylabel("Y [m]")
plt.legend(); plt.grid(True)
plt.axis([-60, 60, -60, 60])
plt.gca().set_aspect('equal')
plt.savefig('landing_turb.pdf', bbox_inches='tight')
plt.show()


   