# PACECAT LDS-E120-R - LiDAR 2D UART

Proyecto minimo en Python para leer y visualizar en vivo un LiDAR PACECAT
LDS-E120-R desde Windows, sin ROS y sin dependencias pesadas.

El proyecto queda centrado en dos piezas:

- `parsers/parser_cffa.py`: decodifica las tramas binarias del LiDAR.
- `app/live_map.py`: muestra el mapa 2D en vivo y sirve como referencia para usar
  los datos en movimiento autonomo.

## Hardware

- LiDAR: PACECAT LDS-E120-R
- Interfaz: UART TTL
- Rango fisico indicado: 0.05 a 12 m
- Baudrate confirmado: `230400`
- Header confirmado: `CF FA`
- Puerto usado durante pruebas: `COM9`

## Conexion

Usar UART TTL, no RS232 industrial.

| LiDAR | USB-TTL |
|---|---|
| VCC / 5V | 5V del adaptador o fuente externa 5V |
| GND | GND |
| TX LiDAR | RX USB-TTL |
| RX LiDAR | No necesario para lectura inicial |

Reglas importantes:

- Compartir siempre GND entre LiDAR y USB-TTL.
- Usar senales TTL 3.3V si el adaptador tiene selector.
- Si el adaptador no entrega suficiente corriente, alimentar el LiDAR con fuente
  externa de 5V y compartir GND.
- No enviar comandos al LiDAR para esta version; solo se escucha el TX del sensor.

## Instalacion

Desde esta carpeta:

```powershell
py -3 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
pip install -r requirements.txt
```

Si PowerShell bloquea la activacion:

```powershell
Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser
.\.venv\Scripts\Activate.ps1
```

## Ejecutar mapa 2D

Comando recomendado:

```powershell
python .\app\live_map.py --port COM9 --baud 230400 --range-mm 6000
```

Opciones utiles:

```powershell
python .\app\live_map.py --port COM9 --baud 230400 --range-mm 3000 --max-age 1.5
python .\app\live_map.py --port COM9 --baud 230400 --front-angle 0
python .\app\live_map.py --port COM9 --baud 230400 --quality-min 20
```

Parametros:

- `--port`: puerto serial del USB-TTL, por ejemplo `COM9`.
- `--baud`: baudrate. Para este LiDAR usar `230400`.
- `--range-mm`: radio visible del mapa en milimetros.
- `--angle-offset`: rota los datos del mapa. Usar solo si las coordenadas X/Y no coinciden con el entorno fisico.
- `--front-angle`: rota solo la flecha/triangulo del frente fisico. No mueve el mapa.
- `--quality-min`: descarta puntos validos con calidad menor al umbral.
- `--max-age`: tiempo en segundos que un punto queda visible.
- `--invalid-radius-mm`: radio donde se muestran retornos invalidos o sin distancia.

## Calibrar orientacion

El mapa tiene dos orientaciones separadas:

- `angle-offset`: rota los datos. Cambiarlo solo si paredes y objetos aparecen
  girados respecto al entorno fisico.
- `front-angle`: rota solo la flecha/triangulo que marca hacia donde apunta la
  flecha fisica de la carcasa.

Con la carcasa correctamente montada, la flecha fisica del LiDAR corresponde al
frente `0 grados`, por lo que normalmente se usa:

```powershell
python .\app\live_map.py --port COM9 --baud 230400 --front-angle 0
```

Si la carcasa o el LiDAR se montan rotados dentro del robot, ajustar solo
`front-angle` para que la flecha visual coincida con el frente real del robot.

Durante la ejecucion del mapa:

- `A` o flecha izquierda: rota los datos `-1 grado`.
- `D` o flecha derecha: rota los datos `+1 grado`.
- `Q`: rota los datos `-10 grados`.
- `E`: rota los datos `+10 grados`.
- `R`: vuelve `angle-offset` a `0`.
- `J`: rota solo la flecha de frente `-5 grados`.
- `L`: rota solo la flecha de frente `+5 grados`.

Cuando la flecha visual coincida con la flecha fisica del LiDAR, anotar
`Frente` en la barra superior y usar ese valor como `--front-angle`.

Si el LiDAR se monta fijo en el robot, este angulo queda constante. Si se quiere
ubicar el robot dentro de un mapa global mientras el robot gira, se necesita una
fuente externa de orientacion, por ejemplo odometria, IMU o encoder de giro.

En la barra superior del mapa se muestran:

- paquetes recibidos,
- vueltas detectadas,
- puntos validos visibles,
- retornos sin distancia,
- punto mas cercano.

## Protocolo confirmado

Cada trama de datos tiene esta forma:

```text
CF FA              header
1E 00              cantidad de puntos: 30
xx xx              angulo inicial, uint16 little-endian, decimas de grado
B4 00              span angular: 180 decimas = 18.0 grados
30 x 3 bytes       puntos
2 bytes            trailer/checksum pendiente de confirmar
```

Cada punto ocupa 3 bytes:

```text
quality            1 byte
distance_mm        uint16 little-endian
```

Resolucion angular:

```text
18 grados / 30 puntos = 0.6 grados por punto
20 tramas = 360 grados
600 puntos por vuelta completa
```

Transformacion polar a cartesiana:

```python
theta = math.radians(angle_deg)
x_mm = math.sin(theta) * distance_mm
y_mm = math.cos(theta) * distance_mm
```

En `live_map.py` se usa `0 grados` como eje superior del mapa, `x` hacia la
derecha e `y` hacia arriba. La flecha de frente fisico se controla por separado
con `--front-angle`.

## Uso para movimiento autonomo

Para control autonomo no es necesario usar ROS. La salida util es un scan 2D
propio con 600 sectores angulares:

```python
ranges_mm[0..599]
qualities[0..599]
valid[0..599]
angle_step_deg = 0.6
```

Cada indice representa:

```python
angle_deg = index * 0.6
```

Uso recomendado para navegacion:

1. Leer paquetes `CF FA` desde serial.
2. Convertir cada paquete a puntos con `parser_cffa.parse_buffer`.
3. Guardar cada punto en el bin angular correspondiente.
4. Filtrar distancias invalidas: menor a 50 mm o mayor a 12000 mm.
5. Usar sectores frontales para evitar obstaculos.
6. Usar laterales para mantener distancia a paredes u objetos.
7. Usar el punto mas cercano y zonas libres para decidir velocidad y giro.

Ejemplo base para extraer datos:

```python
import serial
from parsers import parser_cffa

buffer = b""
ranges_mm = [None] * 600
qualities = [0] * 600

with serial.Serial("COM9", 230400, timeout=0.05) as ser:
    while True:
        buffer += ser.read(ser.in_waiting or 1)
        packets, buffer = parser_cffa.parse_buffer(buffer)

        for packet in packets:
            for point in packet.all_points:
                index = int(point.angle_deg / 0.6) % 600
                if point.valid:
                    ranges_mm[index] = point.distance_mm
                    qualities[index] = point.quality
                else:
                    ranges_mm[index] = None
                    qualities[index] = point.quality
```

Ejemplo simple de zonas para un robot:

```python
def sector(ranges, center_deg, width_deg=30, step_deg=0.6):
    half_bins = int((width_deg / step_deg) / 2)
    center = int(center_deg / step_deg) % len(ranges)
    return [ranges[(center + offset) % len(ranges)] for offset in range(-half_bins, half_bins + 1)]


front_angle = 0.0  # mismo valor usado en --front-angle
front = sector(ranges_mm, front_angle, width_deg=30)
left = sector(ranges_mm, front_angle - 90.0, width_deg=30)
right = sector(ranges_mm, front_angle + 90.0, width_deg=30)

front_valid = [d for d in front if d is not None]
obstacle_front = bool(front_valid and min(front_valid) < 600)
```

Con esa base se puede implementar una logica propia:

- si hay obstaculo frontal cercano, reducir velocidad o detener,
- si el lado izquierdo esta mas libre, girar a la izquierda,
- si el lado derecho esta mas libre, girar a la derecha,
- si ambos lados estan bloqueados, retroceder o buscar giro seguro.

## Estructura final

```text
pacecat_lidar_windows/
  README.md
  requirements.txt
  app/
    live_map.py
  parsers/
    __init__.py
    parser_cffa.py
  logs/
    .gitkeep
```

## Notas de operacion

- Cerrar cualquier otro programa que tenga abierto `COM9`.
- Si el mapa no cambia, revisar que la barra diga `LIVE COM9 @ 230400`.
- Si aparece `Error serial`, el puerto esta ocupado o mal seleccionado.
- Si al acercar la mano aparecen puntos cerca del centro, la medicion es valida.
- Si el sector se vuelve tenue, el LiDAR esta reportando retorno invalido o sin
  distancia util en esa direccion.
