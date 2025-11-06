import time
import board
import busio
from adafruit_pca9685 import PCA9685
from gpiozero import OutputDevice
import logging as log

class ControladorServo:
    """Controlador para servos continuos usando PCA9685 con movimientos temporizados"""

    def __init__(self, direccion_i2c=0x40, frecuencia=50):
        """Inicializar controlador PCA9685"""
        # Para Raspberry Pi 5: usar GPIO 3 (SCL) y GPIO 2 (SDA) - puerto I2C1
        # Estos son los pines físicos 5 y 3 respectivamente
        try:
            self.i2c = busio.I2C(board.D3, board.D2)
            log.info("I2C inicializado en GPIO3/GPIO2 (bus I2C1)")
        except Exception as e:
            log.error(f"Error inicializando I2C en GPIO3/GPIO2: {e}")
            log.error("Verifica que I2C esté habilitado en raspi-config")
            raise
        
        self.pca = PCA9685(self.i2c, address=direccion_i2c)
        self.pca.frequency = frecuencia
        self.servos = {}
        
        # Diagnóstico y seguridad
        # Si es True, se aplicará un pequeño pulso de "hold" en lugar del pulso
        # neutral exacto cuando termine el movimiento.
        # Por defecto: False (mantener comportamiento existente)
        self.hold_after_move = False
        # Cantidad (en microsegundos) para desplazar del neutral cuando está en hold.
        # Usar valores pequeños (ej. 50-200) durante pruebas. Esto no convierte
        # servos continuos en posicionales - es solo un pequeño bias/pulso.
        self.hold_pulse_offset = 100

    def agregar_servo(self, nombre, canal, pulso_min=500, pulso_max=2500, angulo_min=0, angulo_max=180):
        """Agregar servo al controlador"""
        # Usar rango completo de pulso para servos MG996R
        self.servos[nombre] = {
            'canal': canal,
            'pulso_min': pulso_min,
            'pulso_max': pulso_max,
            'angulo_min': angulo_min,
            'angulo_max': angulo_max
        }

    def mover_por_tiempo(self, nombre, direccion, tiempo_segundos, velocidad=0.5):
        """Mover servo continuo por tiempo específico en lugar de ángulos
        
        Args:
            nombre: Nombre del servo
            direccion: -1 = horario, 0 = parar, 1 = antihorario
            tiempo_segundos: Tiempo de movimiento
            velocidad: Factor de velocidad 0.0-1.0 (por defecto 0.5 para movimientos suaves)
        """
        if nombre not in self.servos:
            log.error(f"Servo {nombre} no configurado")
            return

        servo = self.servos[nombre]

        # CONTROL DE SERVOS CONTINUOS - Control de velocidad por tiempo
        # direccion: -1 = giro horario, 0 = parar, 1 = giro antihorario
        if direccion == 0:
            # Detener
            pulso = 1500
        elif direccion == -1:
            # Giro horario (sentido horario)
            pulso = 1500 + (500 * velocidad)  # 1500-2000us
        elif direccion == 1:
            # Giro antihorario (sentido antihorario)
            pulso = 1500 - (500 * velocidad)  # 1500-1000us
        else:
            log.error(f"Dirección inválida: {direccion}")
            return

        ciclo_trabajo = int(pulso / 20000 * 0xFFFF)  # Periodo 50Hz
        log.info(f"[Servo] {nombre}: inicio movimiento dir={direccion} tiempo={tiempo_segundos}s pulso={pulso}us canal={servo['canal']}")
        self.pca.channels[servo['canal']].duty_cycle = ciclo_trabajo

        # Mantener movimiento por el tiempo especificado
        time.sleep(tiempo_segundos)

        # Al finalizar: aplicar stop o (opcionalmente) una pequeña pulse de "hold"
        if self.hold_after_move:
            # Elegir un pulso de hold cercano al neutral pero con pequeño offset
            if pulso > 1500:
                hold_pulso = 1500 + min(500, self.hold_pulse_offset)
            elif pulso < 1500:
                hold_pulso = 1500 - min(500, self.hold_pulse_offset)
            else:
                hold_pulso = 1500

            ciclo_hold = int(hold_pulso / 20000 * 0xFFFF)
            self.pca.channels[servo['canal']].duty_cycle = ciclo_hold
            log.info(f"[Servo] {nombre}: aplicado pulso hold {hold_pulso}us (offset={self.hold_pulse_offset})")
        else:
            # Pulso neutro para detener
            neutro = int(1500 / 20000 * 0xFFFF)
            self.pca.channels[servo['canal']].duty_cycle = neutro
            log.info(f"[Servo] {nombre}: detenido (pulso neutral aplicado)")

    def detener_servo(self, nombre):
        """Detener servo específico"""
        if nombre in self.servos:
            servo = self.servos[nombre]
            # Pulso neutro para detener
            ciclo_trabajo = int(1500 / 20000 * 0xFFFF)
            self.pca.channels[servo['canal']].duty_cycle = ciclo_trabajo
            log.info(f"[Servo] {nombre}: detener_servo -> pulso neutral aplicado")

    def set_hold_after_move(self, enabled: bool, offset_us: int = None):
        """Habilitar/deshabilitar la aplicación de pequeño pulso de hold al terminar un movimiento.

        enabled: True para activar, False para desactivar.
        offset_us: si se pasa, actualiza self.hold_pulse_offset (microsegundos).
        """
        self.hold_after_move = bool(enabled)
        if offset_us is not None:
            try:
                self.hold_pulse_offset = int(offset_us)
            except Exception:
                log.warning("hold_pulse_offset debe ser entero (microsegundos); ignorando valor inválido")
        log.info(f"[Servo] set_hold_after_move={self.hold_after_move} hold_pulse_offset={self.hold_pulse_offset}")

    def detener_todos(self):
        """Detener todos los servos"""
        for nombre in self.servos:
            self.detener_servo(nombre)

class ControladorStepper:
    """Controlador para motores stepper"""

    def __init__(self, pin_paso, pin_direccion, pin_habilitar=None, pasos_por_rev=200, micropasos=16):
        """Inicializar controlador stepper"""
        self.pin_paso = OutputDevice(pin_paso)
        self.pin_direccion = OutputDevice(pin_direccion)
        self.pin_habilitar = OutputDevice(pin_habilitar) if pin_habilitar else None
        self.pasos_por_rev = pasos_por_rev * micropasos
        self.posicion_actual = 0

    def habilitar(self):
        """Habilitar motor stepper"""
        if self.pin_habilitar:
            self.pin_habilitar.off()  # Asumiendo activo bajo

    def deshabilitar(self):
        """Deshabilitar motor stepper"""
        if self.pin_habilitar:
            self.pin_habilitar.on()

    def mover_pasos(self, pasos, direccion=1, velocidad=1000):  # pasos por segundo
        """Mover stepper una cantidad específica de pasos"""
        
        self.pin_direccion.value = 1 if direccion > 0 else 0
        retardo = 1.0 / velocidad
        for _ in range(abs(pasos)):
            self.pin_paso.on()
            time.sleep(retardo / 2)
            self.pin_paso.off()
            time.sleep(retardo / 2)
        self.posicion_actual += pasos * direccion

    def mover_distancia(self, distancia_mm, paso_tuerca=8, direccion=1, velocidad=1000):
        """Mover stepper una distancia específica en mm"""
        pasos = int((distancia_mm / paso_tuerca) * self.pasos_por_rev)
        self.mover_pasos(pasos, direccion, velocidad)

class ControladorRobotico:
    """Controlador principal del brazo robótico con movimientos temporizados y límites físicos"""

    def __init__(self):
        """Inicializar controlador del robot"""
        self.controlador_servo = ControladorServo()
        # Configurar servos: base (canal 0), hombro (1), codo (2), pinza (3)
        # Todos los servos son continuos de 360°
        self.controlador_servo.agregar_servo('base', 0, angulo_min=0, angulo_max=360)
        self.controlador_servo.agregar_servo('shoulder', 1, angulo_min=0, angulo_max=360)
        self.controlador_servo.agregar_servo('elbow', 2, angulo_min=0, angulo_max=360)
        self.controlador_servo.agregar_servo('gripper', 3, angulo_min=0, angulo_max=360)

        # Stepper para elevación del brazo: paso=17, dir=18, hab=19 (pines BCM)
        self.controlador_stepper = ControladorStepper(pin_paso=17, pin_direccion=18, pin_habilitar=19)

        # LÍMITES FÍSICOS DEL BRAZO (en segundos de movimiento)
        # Estos límites previenen que el brazo se salga de su rango físico
        self.limites_fisicos = {
            'base': {'izquierda': 3.0, 'derecha': 3.0},  # Máximo 3 segundos en cada dirección
            'shoulder': {'arriba': 2.5, 'abajo': 2.5},   # Máximo 2.5 segundos en cada dirección
            'elbow': {'extender': 3.5, 'contraer': 3.5}, # Máximo 3.5 segundos en cada dirección
            'gripper': {'abrir': 1.5, 'cerrar': 1.5}     # Máximo 1.5 segundos en cada dirección
        }

        # Estado actual de tiempo acumulado por articulación
        self.tiempo_acumulado = {
            'base': 0.0,
            'shoulder': 0.0,
            'elbow': 0.0,
            'gripper': 0.0
        }

    def mover_base_tiempo(self, direccion, tiempo_segundos, velocidad=0.5):
        """Mover base por tiempo con límites físicos (velocidad reducida por defecto)"""
        tiempo_limitado = min(tiempo_segundos, self.limites_fisicos['base']['derecha' if direccion == 1 else 'izquierda'])
        if tiempo_limitado > 0:
            self.controlador_servo.mover_por_tiempo('base', direccion, tiempo_limitado, velocidad)
            self.tiempo_acumulado['base'] += tiempo_limitado * direccion
        return tiempo_limitado

    def mover_hombro_tiempo(self, direccion, tiempo_segundos, velocidad=0.5):
        """Mover hombro por tiempo con límites físicos (velocidad reducida por defecto)"""
        tiempo_limitado = min(tiempo_segundos, self.limites_fisicos['shoulder']['arriba' if direccion == 1 else 'abajo'])
        if tiempo_limitado > 0:
            self.controlador_servo.mover_por_tiempo('shoulder', direccion, tiempo_limitado, velocidad)
            self.tiempo_acumulado['shoulder'] += tiempo_limitado * direccion
        return tiempo_limitado

    def mover_codo_tiempo(self, direccion, tiempo_segundos, velocidad=0.5):
        """Mover codo por tiempo con límites físicos (velocidad reducida por defecto)"""
        tiempo_limitado = min(tiempo_segundos, self.limites_fisicos['elbow']['extender' if direccion == 1 else 'contraer'])
        if tiempo_limitado > 0:
            self.controlador_servo.mover_por_tiempo('elbow', direccion, tiempo_limitado, velocidad)
            self.tiempo_acumulado['elbow'] += tiempo_limitado * direccion
        return tiempo_limitado

    def mover_pinza_tiempo(self, direccion, tiempo_segundos, velocidad=0.5):
        """Mover pinza por tiempo con límites físicos (velocidad reducida por defecto)"""
        tiempo_limitado = min(tiempo_segundos, self.limites_fisicos['gripper']['abrir' if direccion == 1 else 'cerrar'])
        if tiempo_limitado > 0:
            self.controlador_servo.mover_por_tiempo('gripper', direccion, tiempo_limitado, velocidad)
            self.tiempo_acumulado['gripper'] += tiempo_limitado * direccion
        return tiempo_limitado

    # MÉTODOS LEGACY PARA COMPATIBILIDAD (ya no se usan grados)
    def mover_base(self, angulo, velocidad=5):
        """Mover base del robot (LEGACY - ahora usa tiempo)"""
        log.warning("mover_base con ángulos está obsoleto. Usa mover_base_tiempo")
        # Convertir ángulo aproximado a tiempo (180° ≈ 2 segundos)
        tiempo = abs(angulo - 180) / 90.0  # Aproximación simple
        direccion = 1 if angulo > 180 else -1
        self.mover_base_tiempo(direccion, tiempo, velocidad)

    def mover_hombro(self, angulo, velocidad=5):
        """Mover hombro del robot (LEGACY)"""
        log.warning("mover_hombro con ángulos está obsoleto. Usa mover_hombro_tiempo")
        tiempo = abs(angulo - 180) / 90.0
        direccion = 1 if angulo > 180 else -1
        self.mover_hombro_tiempo(direccion, tiempo, velocidad)

    def mover_codo(self, angulo, velocidad=5):
        """Mover codo del robot (LEGACY)"""
        log.warning("mover_codo con ángulos está obsoleto. Usa mover_codo_tiempo")
        tiempo = abs(angulo - 180) / 90.0
        direccion = 1 if angulo > 180 else -1
        self.mover_codo_tiempo(direccion, tiempo, velocidad)

    def mover_pinza(self, angulo, velocidad=5):
        """Mover pinza del robot (LEGACY)"""
        log.warning("mover_pinza con ángulos está obsoleto. Usa mover_pinza_tiempo")
        tiempo = abs(angulo - 180) / 90.0
        direccion = 1 if angulo > 180 else -1
        self.mover_pinza_tiempo(direccion, tiempo, velocidad)

    def mover_brazo(self, distancia_mm, direccion=1, velocidad=1000):
        """Mover brazo stepper una distancia específica"""
        self.controlador_stepper.mover_distancia(distancia_mm, direccion=direccion, velocidad=velocidad)

    def accion_recoger(self):
        """Abrir pinza para recoger"""
        self.mover_pinza_tiempo(1, 1.0)  # Abrir por 1 segundo

    def accion_soltar(self):
        """Cerrar pinza para soltar"""
        self.mover_pinza_tiempo(-1, 1.0)  # Cerrar por 1 segundo

    def accion_subir(self, distancia=50):
        """Subir brazo stepper"""
        self.mover_brazo(distancia, direccion=1)

    def resetear_tiempos(self):
        """Resetear contadores de tiempo acumulado"""
        self.tiempo_acumulado = {k: 0.0 for k in self.tiempo_acumulado}

    def obtener_estado_tiempos(self):
        """Obtener estado actual de tiempos acumulados"""
        return self.tiempo_acumulado.copy()

    def cerrar(self):
        """Cerrar controladores y liberar recursos"""
        self.controlador_stepper.deshabilitar()
        self.controlador_servo.pca.deinit()
