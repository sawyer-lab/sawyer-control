# Adquisición temporal F/T y estado articular

Esta actualización prepara `contact-implicit-control`. No cambia operations,
no implementa admitancia ni compensación y no mueve el robot desde el demo.

## Instalación

Extraer el ZIP de cambios en la raíz de `contact-implicit-control`, conservando
las carpetas. Incluye los módulos protobuf y gRPC regenerados. Reiniciar el bridge
y actualizar también la instalación de `sawyer-control` utilizada por operations.
Si el bridge se ejecuta desde una imagen Docker, reconstruirla con estos archivos.

```bash
python -m pip install -e '.[dev]'
python -m pytest -q
python scripts/demo/ft_acquisition.py --duration 10 --poll-rate 100
```

El demo requiere un bridge accesible y `FT_SENSOR_IP` habilitado. A diferencia de
`ft_compare.py`, no usa UDP directamente: el bridge es el único propietario del
sensor. No ejecutar los demos UDP antiguos simultáneamente con esta adquisición.

No se fija la frecuencia ni el filtro del dispositivo. Para probar ajustes
explícitos y restaurarlos al terminar:

```bash
python scripts/demo/ft_acquisition.py --sensor-rate 7000 --filter-code 8 --window-ms 10
```

Esos números son un ejemplo, no una selección definitiva. `--zero` solicita un
cero a la carga actual; por defecto nunca se aplica. El demo conserva las muestras,
metadatos y estados articulares en JSONL, además de un resumen de antigüedad,
tiempos de consulta, ventanas repetidas y plazos omitidos. Las modificaciones
solicitadas se restauran en `finally`; un cierre forzado del proceso no puede
garantizar esa restauración.

## Contrato temporal

- `clock_id`: identifica el reloj monotónico del proceso bridge. No comparar sus
  nanosegundos con el reloj de operations u otro equipo. Cambia al reiniciar.
- `received_ns`: recepción en Python después de `recv` para F/T e inicio del
  callback ROS para estado articular. No es tiempo físico de medición.
- `source_stamp_s`: sello ROS original del estado articular, en otro dominio.
  Se conserva para análisis; no se resta directamente de `received_ns`.
- `session_id`: identifica una adquisición continua. Cambia al reconectar,
  modificar ajustes, aplicar cero o reiniciar. Cursor = session_id + sample_id.
- `config_version`: aumenta en cada conexión del lector, incluso reconexiones.
  Evita mezclar configuraciones. La sesión también delimita los promedios.
- Intervalos explícitos: `[start_ns, end_ns)`, con `clock_id` obligatorio.
- `window_s`: última ventana COMPLETA del reloj del bridge, alineada a múltiplos
  de su duración. Dos consultas pueden devolver la misma ventana; se identifica
  con session_id + start_ns + end_ns. Si se pierden ciclos, consultar las ventanas
  faltantes por tiempos explícitos antes de que expire el búfer.
- Consulta por cursor: todas las muestras posteriores, sin consumirlas. El
  intervalo depende del instante de consulta y NO tiene exactamente 10 ms.
- `limit` pagina la respuesta. Si `truncated` es true, no se publica un promedio
  parcial como válido. Usar next_id y mantener el mismo end_ns al paginar.
- Búfer F/T: 35000 registros por defecto, aproximadamente 5 s a 7000 Hz.
  Búfer articular: 10000 actualizaciones reales del callback, no sondeo a 30 Hz.
- Las consultas no esperan un número de paquetes. Incluyen `server_ns` y la
  antigüedad de la última muestra en ese instante, no al recibirla el cliente.
  El tiempo de RPC aporta una cota conservadora adicional, no mide ida unidireccional.

## Validez y unidades

`valid` debe ser true antes de usar `mean`. El promedio contiene seis componentes
Fx,Fy,Fz,Tx,Ty,Tz en las unidades activas informadas. Vacío, antiguo, con pérdidas,
saturación, fallo del lector, sesión distinta, datos sobrescritos, ventana anterior
al inicio de adquisición o respuesta truncada implica `valid=false`, `mean=[]`.
Nunca se representa una lectura ausente como fuerza cero. Una ventana sin pérdidas
observadas NO prueba simultaneidad física ni latencia acotada del sistema operativo.
Los paquetes duplicados se ignoran; regresión de secuencia inicia otra sesión;
el rollover uint32 normal no cuenta como pérdida. El campo dropped_before incluye
el salto anterior a la primera muestra de un intervalo (criterio conservador).

Las escalas se leen de XML sin valores predeterminados. Campos ausentes o inválidos
impiden publicar datos y aparecen en reader_error. La API antigua conserva su
forma, pero sólo entrega N/Nm y rechaza datos antiguos (>100 ms), en lugar de
interpretar otras unidades silenciosamente. La API nueva permite elegir max_age_s.

## Uso desde otro proyecto

```python
from sawyer_control import ForceTorqueClient, SawyerRobotClient

with ForceTorqueClient.connect() as ft:
    # Puede devolver no_samples/reader_error durante inicio y estabilización.
    initial = ft.window(include_samples=True)
    session, cursor = initial.history.session_id, initial.history.next_id
    result = ft.window(session_id=session, after_id=cursor,
                       max_age_s=0.05, include_samples=True, timeout=0.02)
    if result.history.session_changed:
        session, cursor = result.history.session_id, 0
    elif result.valid:
        force_and_torque = list(result.mean)
        cursor = result.history.next_id
    # La aplicación define qué hacer ante resultados inválidos y timeouts.

with SawyerRobotClient.connect() as robot:
    history = robot.state_history(timeout=0.02)
```

Una aplicación continua debe avanzar su cursor incluso al decidir descartar un
intervalo inválido, dejando constancia del motivo. Si la configuración cambia,
reiniciar el cursor, esperar una muestra válida y actualizar los metadatos del
experimento. No reutilizar una corrección de control a partir de datos inválidos.

`command_timed(sample, mode, cycle_id, timeout)` devuelve recepción y retorno de
publicación ROS del comando con el mismo clock_id. Es una operación de movimiento,
no forma parte del demo de diagnóstico. published_ns NO confirma ejecución física.
Un timeout no demuestra que el robot no recibió el comando: no reenviarlo a ciegas.
Las llamadas antiguas `command()` siguen funcionando.

## Configuración, inicio y recuperación

Los setters gRPC pasan por una transacción: detener y unir el hilo, escribir los
ajustes, reiniciar, releer escalas y descartar el periodo de estabilización. No se
mezclan muestras previas y posteriores. Incluso un intento fallido puede reiniciar
la sesión. Las lecturas durante la transición son inválidas.

No se hace bias ni reset del latch al conectar. `zero()` es explícito y sólo informa
éxito al recibir una muestra después del tiempo de estabilización. Esto no certifica
la calidad física del cero: depende de la carga aplicada. El lector reconecta ante
timeout/error sin aplicar otro cero. Sólo se intenta arrancar RDT si ya está habilitado.
El tiempo de estabilización predeterminado de 1 s es conservador y configurable en
el constructor; no representa una medición del retardo del filtro. Se espera antes
de releer escalas y se descartan muestras durante otro periodo tras iniciar UDP.

Cambiar ajustes desde la web del sensor o desde un cliente HTTP externo evita esta
coordinación. Usar el bridge para todos los cambios durante la adquisición. Los
setters devuelven ajustes aceptados, no una garantía de que ya existe una muestra
válida: consultar GetWindow y comprobar valid.

## Verificación

Las pruebas automáticas cubren el contrato del búfer, paginación, sesiones,
ventanas, promedios inválidos, RPC reales en localhost, escalado, cero explícito,
rollover, reconexión y cierre con transporte simulado. No necesitan ROS ni sensor.

En hardware: primero ejecutar el demo con el robot quieto. Después mantenerlo
registrando durante un replay autorizado desde operations. Comparar ciclos,
secuencias, antigüedad y tiempos de recepción. El demo sólo observa y no coordina
el replay. La asociación entre cycle_id, ventana utilizada y comando se añadirá
al ciclo de operations en la siguiente etapa. No se afirma sincronización física
ni ejecución de tiempo real a 100 Hz con esta actualización.

GetStateHistory contiene únicamente los campos articulares y temporales; pose,
enabled y stopped no se reconstruyen históricamente y no deben interpretarse.
El historial devuelve el estado articular real recibido. velocity_valid informa
si ROS suministró todas las velocidades; ceros de relleno no deben usarse como
velocidades medidas para compensación. No se estiman aceleraciones ni se interpolan
estados aquí. Para medir desfase físico hace falta un evento observable por ambos
sistemas o una referencia externa.

## Archivos principales

- ft_buffer.py: historial acotado, consultas no destructivas y sesiones.
- ft_acquisition.py: reducción por intervalo y validez.
- ft_sensor.py: único propietario de UDP, escalado y recuperación.
- sawyer.py / robot.py: historial desde callbacks y snapshot coherente.
- runtime.py / service.py / server.py: coordinación, RPC y cierre.
- control.proto y módulos generados / client.py: contrato y cliente.
- scripts/demo/ft_acquisition.py: diagnóstico sin movimiento.

`ft_config.py` y `ft_status.py` se reutilizan sin cambios y por eso no se incluyen
en el ZIP diferencial. Tampoco se modifican los demos experimentales antiguos.
