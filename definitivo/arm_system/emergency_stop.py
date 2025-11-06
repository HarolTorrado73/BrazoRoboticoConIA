#!/usr/bin/env python3
"""
PARADA DE EMERGENCIA - Detiene TODOS los servos inmediatamente
"""
from control.robot_controller import ControladorRobotico
import time

print("="*60)
print("⚠️  PARADA DE EMERGENCIA - DETENIENDO TODOS LOS SERVOS")
print("="*60)

try:
    robot = ControladorRobotico()
    
    # Enviar pulso neutral a TODOS los canales
    print("\n1. Aplicando pulso neutral (1500µs) a todos los canales...")
    for canal in range(16):  # PCA9685 tiene 16 canales
        duty = int(1500 / 20000 * 0xFFFF)
        robot.controlador_servo.pca.channels[canal].duty_cycle = duty
    
    time.sleep(0.5)
    
    # Intentar desactivar completamente los canales
    print("2. Desactivando señal PWM...")
    for canal in range(16):
        robot.controlador_servo.pca.channels[canal].duty_cycle = 0
    
    print("\n✓ Señales enviadas")
    print("\n⚠️  SI LOS SERVOS SIGUEN MOVIÉNDOSE:")
    print("  1. DESCONECTA LA ALIMENTACIÓN del PCA9685 (cable rojo)")
    print("  2. Los servos MG996R pueden tener problema de calibración")
    print("  3. Revisa el trimmer/potenciómetro en la parte trasera del servo")
    
    robot.cerrar()
    
except Exception as e:
    print(f"ERROR: {e}")

print("\n" + "="*60)
