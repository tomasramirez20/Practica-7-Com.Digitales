# main.py - IMPLEMENTACIÓN MEJORADA del sistema Hamming(7,4) para MPU6050
from machine import Pin, I2C, UART
import time
from hamming74 import hamming74_encode, hamming74_decode

# =============================================================================
# CONFIGURACIÓN DE HARDWARE
# =============================================================================

# Comunicación I2C con el sensor MPU6050
i2c_bus = I2C(1, scl=Pin(15), sda=Pin(14), freq=400000)

# Interfaz UART para visualización en osciloscopio
uart_port = UART(0, baudrate=9600, tx=Pin(12), rx=Pin(13))

# Dirección I2C del dispositivo MPU6050
MPU6050_ADDRESS = 0x68

# =============================================================================
# FUNCIONES DE CONFIGURACIÓN Y COMUNICACIÓN
# =============================================================================

def initialize_mpu6050():
    """Inicializa y configura el sensor MPU6050"""
    i2c_bus.writeto_mem(MPU6050_ADDRESS, 0x6B, b'\x00')
    time.sleep(0.2)

def scan_i2c_devices():
    """Escanea y muestra dispositivos conectados al bus I2C"""
    detected_devices = i2c_bus.scan()
    print("Dispositivos I2C detectados:", [hex(addr) for addr in detected_devices])
    return detected_devices

def read_accelerometer_data():
    """Obtiene lecturas del acelerómetro en los tres ejes"""
    try:
        # Lectura de registros de aceleración (6 bytes)
        raw_data = i2c_bus.readfrom_mem(MPU6050_ADDRESS, 0x3B, 6)
        
        # Construcción de valores de 16 bits
        x_axis = (raw_data[0] << 8) | raw_data[1]
        y_axis = (raw_data[2] << 8) | raw_data[3]
        z_axis = (raw_data[4] << 8) | raw_data[5]
        
        # Ajuste para complemento a 2
        if x_axis > 32767: x_axis -= 65536
        if y_axis > 32767: y_axis -= 65536  
        if z_axis > 32767: z_axis -= 65536
        
        return x_axis, y_axis, z_axis
        
    except Exception as error:
        print(f"Error en lectura del acelerómetro: {error}")
        return 0, 0, 0

# =============================================================================
# FUNCIONES DE PROCESAMIENTO DE DATOS
# =============================================================================

def convert_16bit_to_nibbles(data_word):
    """Divide un valor de 16 bits en 4 nibbles de 4 bits cada uno"""
    absolute_value = abs(data_word) & 0xFFFF
    nibble_list = []
    
    for nibble_index in range(4):
        shift_amount = 4 * (3 - nibble_index)
        nibble_value = (absolute_value >> shift_amount) & 0x0F
        
        # Representación en bits [MSB, ..., LSB]
        bit_representation = [
            (nibble_value >> 3) & 1,
            (nibble_value >> 2) & 1,
            (nibble_value >> 1) & 1,
            (nibble_value >> 0) & 1
        ]
        nibble_list.append(bit_representation)
    
    return nibble_list

def hamming_encode_16bit_sample(sample_value):
    """Aplica codificación Hamming(7,4) a una muestra de 16 bits"""
    print(f"Muestra original: 0x{sample_value:04X} = {sample_value:016b}b")
    
    # División en nibbles
    nibble_set = convert_16bit_to_nibbles(sample_value)
    
    print("Nibbles extraídos:")
    for idx, nibble in enumerate(nibble_set):
        decimal_value = nibble[0]*8 + nibble[1]*4 + nibble[2]*2 + nibble[3]
        print(f"  Nibble {idx+1}: {nibble} = 0x{decimal_value:X}")
    
    # Codificación Hamming para cada nibble
    encoded_stream = []
    for idx, nibble in enumerate(nibble_set):
        hamming_encoded = hamming74_encode(nibble)
        encoded_stream.extend(hamming_encoded)
        
        decimal_value = nibble[0]*8 + nibble[1]*4 + nibble[2]*2 + nibble[3]
        print(f"  Nibble 0x{decimal_value:X} → Codificado: {hamming_encoded}")
    
    print(f"Trama codificada ({len(encoded_stream)} bits): {encoded_stream}")
    return encoded_stream

def convert_bitstream_to_bytes(bit_sequence):
    """Convierte una secuencia de bits en bytes para transmisión UART"""
    output_bytes = bytearray()
    
    for byte_start in range(0, len(bit_sequence), 8):
        byte_bits = bit_sequence[byte_start:byte_start+8]
        
        # Completar con ceros si es necesario
        while len(byte_bits) < 8:
            byte_bits.append(0)
        
        # Construcción del byte
        byte_value = 0
        for bit in byte_bits:
            byte_value = (byte_value << 1) | bit
        
        output_bytes.append(byte_value)
    
    return output_bytes

# =============================================================================
# FUNCIONES DE PRUEBA Y VERIFICACIÓN
# =============================================================================

def verify_hamming_operations():
    """Verifica el correcto funcionamiento de las funciones Hamming"""
    print("=== VERIFICACIÓN DE CODIFICACIÓN HAMMING ===")
    
    # Caso de prueba del laboratorio
    test_input = [1, 0, 1, 1]
    print(f"Datos de prueba: {test_input}")
    
    encoded_test = hamming74_encode(test_input)
    print(f"Salida codificada: {encoded_test}")
    
    decoded_result, syndrome_val, was_corrected, corrected_data = hamming74_decode(encoded_test)
    print(f"Decodificación: {decoded_result}, Síndrome: {syndrome_val}, Corrección: {was_corrected}")
    
    # Prueba con error simulado
    erroneous_data = encoded_test.copy()
    erroneous_data[6] = 1 - erroneous_data[6]
    decoded_err, syndrome_err, corrected_err, corrected_err_data = hamming74_decode(erroneous_data)
    print(f"Con error: {decoded_err}, Síndrome: {syndrome_err}, Corregido: {corrected_err}")
    
    print("=== FIN DE VERIFICACIÓN ===\n")

# =============================================================================
# PROGRAMA PRINCIPAL
# =============================================================================

def main_execution():
    """Función principal del sistema"""
    print("Sistema de Codificación Hamming para MPU6050 - Iniciando...")
    print("Configuración: I2C(SDA=GP14, SCL=GP15), UART(TX=GP12)")
    print()
    
    # Verificación inicial de funciones
    verify_hamming_operations()
    
    # Detección de dispositivos I2C
    i2c_devices = scan_i2c_devices()
    if MPU6050_ADDRESS not in i2c_devices:
        print("Error: MPU6050 no detectado. Verificar conexiones:")
        print("   - Alimentación: VCC→3.3V, GND→GND")
        print("   - Datos: SDA→GP14 (Pin 19), SCL→GP15 (Pin 20)")
        return
    
    print("✅ MPU6050 detectado correctamente")
    
    # Configuración del sensor
    initialize_mpu6050()
    
    counter = 0
    
    while True:
        try:
            # Lectura de datos del acelerómetro
            accel_x, accel_y, accel_z = read_accelerometer_data()
            
            # Procesamiento del eje X
            sample_to_encode = accel_x
            
            # Codificación Hamming
            encoded_bits_result = hamming_encode_16bit_sample(abs(sample_to_encode) & 0xFFFF)
            
            # Transmisión UART
            transmission_bytes = convert_bitstream_to_bytes(encoded_bits_result)
            uart_port.write(transmission_bytes)
            
            counter += 1
            print(f"Muestra #{counter} transmitida: {list(transmission_bytes)}")
            print("─" * 50 + "\n")
            
            time.sleep(2)
            
        except Exception as execution_error:
            print(f"Error en ejecución: {execution_error}")
            time.sleep(1)

if __name__ == "__main__":
    main_execution()