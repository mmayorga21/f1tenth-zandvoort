"""Plain terminal presentation; does not control or measure the vehicle."""
import math


STATES = {'waiting': 'Esperando simulador', 'ready': 'Preparado',
          'running': 'Midiendo', 'ended': 'Finalizado', 'invalid': 'INTENTO INVALIDO'}
REASONS = {
    'waiting_for_simulator': 'Esperando datos del simulador',
    'waiting_for_attempt': 'Active el controlador para comenzar',
    'measuring': 'Cruce de meta con recorrido completo requerido',
    'attempt_start_not_observed': 'El contador inicio tarde: reinicie el simulador',
    'session_reset': 'Se reinicio la sesion',
    'collision': 'Se detecto una colision',
    'odometry_gap': 'Faltan muestras de odometria: reinicie',
    'odometry_missing_at_start': 'No se observo la salida: reinicie',
    'position_jump': 'Salto de posicion: reinicie',
    'simulation_time_reversed': 'El reloj retrocedio sin un reinicio de sesion',
    'odometry_time_reversed': 'La odometria retrocedio en el tiempo',
    'source_status_timeout': 'Se perdio el estado del simulador: reinicie',
    'odometry_timeout': 'Se perdio la odometria: reinicie',
    'invalid_odometry': 'Odometria no valida: reinicie',
    'unexpected_odometry_frame': 'La odometria no esta en el marco configurado',
    'start_outside_finish_area': 'La salida no coincide con la meta configurada',
    'drive_timeout': 'El simulador interrumpio el intento por falta de mandos',
    'shutdown': 'El simulador interrumpio el intento',
    'manual_stop': 'Intento cerrado manualmente',
    'target_laps_completed': 'Intento finalizado automaticamente: objetivo de vueltas completado',
}


def format_time(seconds):
    if seconds is None or not math.isfinite(seconds):
        return '--:--.--'
    hundredths = max(0, round(seconds*100))
    minutes, remainder = divmod(hundredths, 6000)
    whole_seconds, fraction = divmod(remainder, 100)
    return f'{minutes:02d}:{whole_seconds:02d}.{fraction:02d}'


def format_panel(status, width=78, height=26):
    title = STATES.get(status['state'], status['state'])
    if status['state'] == 'running' and status['paused']:
        title = 'PAUSA'
    if status['target_reached']:
        title += ' | OBJETIVO ALCANZADO'
    lines = [
        'F1TENTH - ZANDVOORT | CONTADOR Y CRONOMETRO',
        f'Estado: {title}',
        f'Vueltas completadas: {status["lap_count"]} / {status["target_laps"]}',
        f'Vuelta actual: {format_time(status["current_lap_time_s"])}',
        f'Ultima: {format_time(status["last_lap_time_s"])}    Mejor: {format_time(status["best_lap_time_s"])}',
        f'Tiempo del intento: {format_time(status["elapsed_sim_s"])}',
        f'Total vueltas completas: {format_time(status["completed_time_s"])}',
        f'Regiones recorridas: {status["checkpoints_passed"]} / {status["checkpoint_count"]}',
        '', 'VUELTA       TIEMPO',
    ]
    available = max(1, height-len(lines)-3)
    times = status['lap_times_s']
    start = max(0, len(times)-available)
    if not times:
        lines.append('Sin vueltas completadas')
    else:
        for index in range(start, len(times)):
            lines.append(f'{index+1:>6}       {format_time(times[index])}')
    lines += ['', REASONS.get(status['reason'], status['reason'])]
    if status['pause_used']:
        lines.append('Se utilizo pausa durante este intento')
    return '\n'.join(line[:max(20, width)] for line in lines[:max(12, height)])
