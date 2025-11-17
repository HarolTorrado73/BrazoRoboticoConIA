# CONFIGURACIÓN DEL SISTEMA - Brazo Robótico
# Este archivo controla qué componentes están habilitados

# SERVOS (PCA9685) - Todos calibrados y funcionando
SERVOS_HABILITADOS = True

# MOTOR PASO A PASO (TMC2208) - Actualmente deshabilitado
# Razón: Potenciómetro VREF dañado, necesita reemplazo de driver
STEPPER_HABILITADO = False

# CÁMARA
CAMARA_HABILITADA = True

# Nota: El sistema funcionará normalmente con los 4 servos:
# - Hombro (shoulder) - canal 0
# - Codo (elbow) - canal 1  
# - Muñeca (wrist) - canal 2
# - Pinza (gripper) - canal 3
# El movimiento horizontal quedará deshabilitado hasta reemplazar TMC2208
