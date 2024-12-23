import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt

# Leer los archivos de texto y extraer los valores de la columna 10 en 10
file_names = ['rules1_linear_velocity'] #, 'rules2_linear_velocity', 'rules3_linear_velocity', 'rules4_linear_velocity', 'rules5_linear_velocity', 'rules6_linear_velocity']
#file_names = ['rules7_angular_velocity', 'rules8_angular_velocity', 'rules9_angular_velocity', 'rules10_angular_velocity', 'rules11_angular_velocity', 'rules12_angular_velocity']
markers = ['o','X','*','s','D','>']
i = 0
for file_name in file_names:
    df = pd.read_csv(file_name + '.txt', sep=' ')
    x = df.iloc[::10, 1]
    y = df.iloc[::10, 0]
    # Dibujar los valores en un diagrama de líneas distintas con diferentes colores y marcadores
    sns.lineplot(x=x, y=y, label=file_name, marker=markers[i])
    i = i+1

plt.xlabel('time (s.)')
plt.ylabel('average minimun distance')
plt.legend()
plt.show()

