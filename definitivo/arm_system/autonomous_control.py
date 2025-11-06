#!/usr/bin/env python3
"""
Sistema de Control Autónomo del Brazo Robótico
Integra detección de objetos con movimiento automático del brazo
"""

import time
import logging as log
import numpy as np
from perception.vision.camera.main import CameraManager
from perception.vision.detection.main import DetectionModel
from control.robot_controller import ControladorRobotico

log.basicConfig(level=log.INFO, format="%(asctime)s - %(levelname)s - %(message)s")


class AutonomousRobotController:
    """Controlador autónomo que detecta objetos y mueve el brazo automáticamente"""
    
    def __init__(self):
        """Inicializar componentes del sistema autónomo"""
        log.info("Inicializando sistema autónomo...")
        
        # Inicializar cámara (con flip activado)
        self.camera = CameraManager(flip=True)
        
        # Inicializar modelo de detección
        self.detector = DetectionModel()
        
        # Inicializar controlador del brazo
        self.robot = ControladorRobotico()
        
        # Configuración de detección
        self.confidence_threshold = 0.55
        self.target_classes = ['bottle', 'cup', 'cell phone', 'book']  # Objetos de interés
        
        # Configuración de movimiento
        self.image_width = 1280
        self.image_height = 720
        self.center_x = self.image_width // 2
        self.center_y = self.image_height // 2
        
        # Zona muerta (dead zone) para evitar movimientos pequeños
        self.dead_zone_x = 100  # píxeles
        self.dead_zone_y = 80
        
        log.info("Sistema autónomo inicializado correctamente")
    
    def calculate_object_position(self, bbox):
        """Calcular posición del objeto en la imagen"""
        x1, y1, x2, y2 = bbox
        obj_center_x = (x1 + x2) / 2
        obj_center_y = (y1 + y2) / 2
        obj_width = x2 - x1
        obj_height = y2 - y1
        
        return obj_center_x, obj_center_y, obj_width, obj_height
    
    def calculate_movement(self, obj_x, obj_y):
        """Calcular movimiento necesario para centrar el objeto"""
        # Calcular error de posición
        error_x = obj_x - self.center_x
        error_y = obj_y - self.center_y
        
        # Aplicar zona muerta
        if abs(error_x) < self.dead_zone_x:
            error_x = 0
        if abs(error_y) < self.dead_zone_y:
            error_y = 0
        
        # Si el objeto está centrado, retornar None
        if error_x == 0 and error_y == 0:
            return None
        
        # Calcular tiempo de movimiento proporcional al error
        # Máximo 2 segundos, mínimo 0.2 segundos
        time_base = max(0.2, min(abs(error_x) / self.image_width * 2, 2.0))
        time_shoulder = max(0.2, min(abs(error_y) / self.image_height * 2, 2.0))
        
        # Determinar dirección
        dir_base = 1 if error_x > 0 else -1  # Izq/Der
        dir_shoulder = 1 if error_y > 0 else -1  # Arriba/Abajo
        
        return {
            'base': (dir_base, time_base) if error_x != 0 else None,
            'shoulder': (dir_shoulder, time_shoulder) if error_y != 0 else None
        }
    
    def move_to_object(self, movement):
        """Mover el brazo hacia el objeto detectado"""
        if movement is None:
            log.info("Objeto centrado - no se requiere movimiento")
            return True
        
        # Mover base
        if movement['base']:
            direction, duration = movement['base']
            log.info(f"Moviendo base: dirección={direction}, tiempo={duration:.2f}s")
            self.robot.mover_base_tiempo(direction, duration, velocidad=2.0)
            time.sleep(0.3)
        
        # Mover hombro
        if movement['shoulder']:
            direction, duration = movement['shoulder']
            log.info(f"Moviendo hombro: dirección={direction}, tiempo={duration:.2f}s")
            self.robot.mover_hombro_tiempo(direction, duration, velocidad=2.0)
            time.sleep(0.3)
        
        return False  # Aún no centrado
    
    def detect_and_track(self):
        """Detectar objeto y centrar el brazo en él"""
        log.info("Capturando imagen...")
        
        try:
            image, filename = self.camera.capture_image(save=True)
        except Exception as e:
            log.error(f"Error al capturar imagen: {e}")
            return False, False
        
        if image is None:
            log.error("Error al capturar imagen")
            return False, False
        
        log.info(f"Imagen guardada: {filename}")
        log.info("Ejecutando detección de objetos...")
        
        # Detectar objetos
        results_gen, class_names = self.detector.inference(image)
        
        # Procesar resultados
        best_detection = None
        best_confidence = 0
        
        for results in results_gen:
            boxes = results.boxes
            
            for box in boxes:
                # Obtener información de la detección
                conf = float(box.conf[0])
                cls_id = int(box.cls[0])
                cls_name = class_names[cls_id]
                bbox = box.xyxy[0].cpu().numpy()
                
                # Filtrar por confianza y clase
                if conf >= self.confidence_threshold:
                    log.info(f"Detectado: {cls_name} (confianza: {conf:.2f})")
                    
                    # Si es un objeto de interés y tiene mejor confianza
                    if cls_name in self.target_classes and conf > best_confidence:
                        best_detection = (cls_name, conf, bbox)
                        best_confidence = conf
        
        # Si se detectó un objeto de interés
        if best_detection:
            cls_name, conf, bbox = best_detection
            obj_x, obj_y, obj_w, obj_h = self.calculate_object_position(bbox)
            
            log.info(f"Objetivo seleccionado: {cls_name} en posición ({obj_x:.0f}, {obj_y:.0f})")
            
            # Calcular movimiento
            movement = self.calculate_movement(obj_x, obj_y)
            
            # Mover hacia el objeto
            centered = self.move_to_object(movement)
            
            return True, centered
        else:
            log.info("No se detectaron objetos de interés")
            return False, False
    
    def run_autonomous_mode(self, max_iterations=10):
        """Ejecutar modo autónomo: detectar y centrar objetos automáticamente"""
        log.info("=" * 60)
        log.info("INICIANDO MODO AUTÓNOMO")
        log.info(f"Objetos objetivo: {', '.join(self.target_classes)}")
        log.info("=" * 60)
        
        iteration = 0
        
        while iteration < max_iterations:
            iteration += 1
            log.info(f"\n--- Iteración {iteration}/{max_iterations} ---")
            
            detected, centered = self.detect_and_track()
            
            if not detected:
                log.info("No hay objetos detectados. Esperando 2 segundos...")
                time.sleep(2)
                continue
            
            if centered:
                log.info("¡Objeto centrado! Ejecutando acción de recoger...")
                # Aquí puedes agregar la secuencia completa de recoger
                self.robot.mover_codo_tiempo(1, 1.0)  # Extender codo
                time.sleep(0.5)
                self.robot.accion_recoger()  # Abrir pinza
                time.sleep(0.5)
                self.robot.accion_soltar()  # Cerrar pinza
                log.info("¡Objeto recogido!")
                break
            
            # Esperar antes de la siguiente iteración
            time.sleep(1)
        
        log.info("\n" + "=" * 60)
        log.info("MODO AUTÓNOMO FINALIZADO")
        log.info("=" * 60)
    
    def run_continuous_scan(self):
        """Escaneo continuo: busca objetos constantemente"""
        log.info("Iniciando escaneo continuo...")
        
        try:
            while True:
                log.info("\nEscaneando área...")
                detected, centered = self.detect_and_track()
                
                if detected and centered:
                    log.info("¡Objeto centrado! Listo para acción.")
                
                time.sleep(3)  # Escanear cada 3 segundos
        
        except KeyboardInterrupt:
            log.info("\nEscaneo detenido por el usuario")
    
    def close(self):
        """Cerrar y liberar recursos"""
        log.info("Cerrando sistema autónomo...")
        self.robot.cerrar()


def main():
    """Función principal"""
    import sys
    
    # Crear controlador autónomo
    controller = AutonomousRobotController()
    
    try:
        if len(sys.argv) > 1 and sys.argv[1] == '--scan':
            # Modo escaneo continuo
            controller.run_continuous_scan()
        else:
            # Modo autónomo (detectar, centrar y recoger)
            controller.run_autonomous_mode(max_iterations=10)
    
    except KeyboardInterrupt:
        log.info("\nPrograma interrumpido por el usuario")
    
    finally:
        controller.close()


if __name__ == '__main__':
    main()
