#!/usr/bin/env python3
"""Simulador de pista: intento 1 (exploracion) + intento 2 (speed run).

Uso basico:
    python main.py pistas/pista_ejemplo.txt
    python main.py pistas/mi_pista.txt --inicio A1 --meta L8
    python main.py pistas/mi_pista.txt --detalle          # log celda por celda
    python main.py pistas/mi_pista.txt --lidar 3          # LiDAR que ve 3 celdas
    python main.py pistas/mi_pista.txt --celda-cm 25 --exportar mapa.yaml
"""
import argparse
import sys

from sim.maze import cargar, celda, nombre
from sim.explorer import Explorer
from sim.speedrun import (RobotConfig, plan_de_ejecucion, tiempo_estimado,
                          tiempo_exploracion, GLIFO)
from sim.render import dibujar


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("pista", help="archivo .txt con la pista en formato ASCII")
    ap.add_argument("--inicio", help="celda de inicio, ej. A1 (si la pista no tiene 'I')")
    ap.add_argument("--meta", help="celda meta, ej. L8 (si la pista no tiene 'M')")
    ap.add_argument("--detalle", action="store_true",
                    help="log celda por celda: que ve y que decide el robot")
    ap.add_argument("--lidar", type=int, default=1, metavar="N",
                    help="alcance del LiDAR en celdas (1 = solo celda actual)")
    ap.add_argument("--celda-cm", type=float, default=30.0)
    ap.add_argument("--rueda-cm", type=float, default=6.5, help="diametro de rueda")
    ap.add_argument("--track-cm", type=float, default=15.0, help="dist. entre ruedas")
    ap.add_argument("--ticks", type=int, default=1560, help="ticks encoder por vuelta")
    ap.add_argument("--exportar", metavar="ARCHIVO.yaml",
                    help="exporta el mapa descubierto en formato 4-bits (NESO)")
    ap.add_argument("--exportar-plan", metavar="ARCHIVO.json",
                    help="exporta el plan de ejecucion (intento 2) en JSON, referencia/comparacion "
                         "contra robot/granprix_bot (que calcula su propio plan en el robot real)")
    ap.add_argument("--sin-grafico", action="store_true",
                    help="no generar/mostrar la imagen de la pista (matplotlib)")
    args = ap.parse_args()

    mz = cargar(args.pista)
    if args.inicio:
        mz.start = celda(args.inicio)
    if args.meta:
        mz.goal = celda(args.meta)

    errores = mz.validar()
    if errores:
        for e in errores:
            print(f"ERROR: {e}", file=sys.stderr)
        sys.exit(1)

    cfg = RobotConfig(celda_cm=args.celda_cm, diametro_rueda_cm=args.rueda_cm,
                      track_width_cm=args.track_cm, ticks_por_vuelta=args.ticks)

    print(f"Pista {mz.cols}x{mz.rows} | inicio {nombre(mz.start)} | "
          f"meta {nombre(mz.goal)} | muros interiores: {len(mz.walls)}")
    optimo = mz.bfs(mz.start, mz.goal)
    print(f"Optimo teorico (con el mapa completo): {len(optimo)-1} movimientos\n")

    # ---------- INTENTO 1 ----------
    exp = Explorer(mz, lidar_alcance=args.lidar)
    r1 = exp.explorar()
    ruta1 = r1["ruta"]

    print("=" * 62)
    print("INTENTO 1 - EXPLORACION (flood fill)")
    print("=" * 62)
    print(f"Movimientos: {len(ruta1)-1}  "
          f"(fase A hasta meta: {r1['fase_a']}, fase B verificacion: {r1['fase_b']})")
    print(f"Celdas sensadas: {len(r1['sensadas'])}/{mz.cols*mz.rows}  |  "
          f"muros descubiertos: {len(r1['conocidos'])}/{len(mz.walls)}")
    extra = (len(ruta1) - 1) - (len(optimo) - 1)
    print(f"Costo de la ignorancia: +{extra} movimientos sobre el optimo")
    t1 = tiempo_exploracion(len(ruta1) - 1, cfg)
    print(f"Tiempo estimado: {t1:.1f} s "
          f"(parada y sensado en cada celda, {cfg.vel_exp} m/s)\n")
    print(dibujar(mz, ruta1))
    print("\nRuta:", " ".join(nombre(c) for c in ruta1))

    if args.detalle:
        print("\n--- LOG CELDA POR CELDA ---")
        for i, e in enumerate(exp.log):
            linea = f"[{i:>3}] {nombre(e['en'])} (fase {e['fase']})"
            if e["opciones"]:
                ops = ", ".join(f"{d}->{nombre(n)}:{dv if dv < 10**8 else 'inf'}"
                                for d, n, dv in e["opciones"])
                linea += f"\n      opciones [{ops}]"
            if e["va"]:
                d, n = e["va"]
                linea += f"\n      decision: {GLIFO[d]} hacia {nombre(n)}"
                if e["muros_nuevos"]:
                    linea += (f"\n      LiDAR al llegar a {nombre(n)}: "
                              f"MURO NUEVO al {'/'.join(e['muros_nuevos'])}")
            elif e["muros_nuevos"]:
                linea += f"  LiDAR: MURO NUEVO al {'/'.join(e['muros_nuevos'])}"
            print(linea)

    # ---------- INTENTO 2 ----------
    ruta2 = mz.bfs(mz.start, mz.goal, walls=r1["conocidos"])
    valida = all(not mz.hay_muro(ruta2[i], ruta2[i+1]) for i in range(len(ruta2)-1))
    segs, plan = plan_de_ejecucion(ruta2, cfg)

    print("\n" + "=" * 62)
    print("INTENTO 2 - SPEED RUN (BFS sobre el mapa descubierto)")
    print("=" * 62)
    print(f"Movimientos: {len(ruta2)-1}  |  valida en la pista real: {valida}  |  "
          f"optima garantizada: {len(ruta2)-1 == len(optimo)-1}")
    n_giros = sum(1 for p in plan if p["cmd"] == "GIRAR")
    print(f"Segmentos rectos: {len(segs)}  |  giros: {n_giros}")
    t2 = tiempo_estimado(plan, cfg)
    print(f"Tiempo estimado: {t2:.1f} s  ->  {t1/t2:.1f}x mas rapido que explorar\n")
    print(dibujar(mz, ruta2))
    print("\nRuta:", " -> ".join(nombre(c) for c in ruta2))

    print("\n--- PLAN DE EJECUCION (comandos fisicos) ---")
    print(f"(celda {cfg.celda_cm} cm | rueda {cfg.diam} cm | "
          f"track {cfg.track} cm | {cfg.ticks} ticks/vuelta)")
    for p in plan:
        if p["cmd"] == "GIRAR":
            print(f"  GIRAR   {p['lado']:<12} {abs(p['grados']):>3} grados  "
                  f"(arco {p['arco_cm']} cm/rueda = {p['ticks']} ticks)")
        else:
            print(f"  AVANZAR {p['desde']} -> {p['hasta']:<4} "
                  f"{p['celdas']} celdas = {p['cm']:>6.1f} cm = {p['ticks']:>5} ticks  "
                  f"(frenar cuando LiDAR frontal vea pared a {cfg.celda_cm/2:.0f} cm)")

    if args.exportar:
        exportar_yaml(mz, r1["conocidos"], args.exportar)
        print(f"\nMapa descubierto exportado a {args.exportar}")

    if args.exportar_plan:
        exportar_plan_json(mz, cfg, plan, args.pista, args.exportar_plan)
        print(f"Plan de ejecucion exportado a {args.exportar_plan}")

    if not args.sin_grafico:
        try:
            from generar_pista import generar_tres_vistas
        except ImportError as e:
            print(f"\n(No se pudo graficar la pista, falta una dependencia: {e})",
                  file=sys.stderr)
        else:
            generar_tres_vistas(mz, ruta1, ruta2)


def exportar_yaml(mz, conocidos, ruta_archivo):
    """Formato 4 bits por celda: bit0=N bit1=E bit2=S bit3=O (muros conocidos)."""
    from sim.maze import wall
    lineas = ["# mapa descubierto (bit0=N bit1=E bit2=S bit3=O)",
              f"cols: {mz.cols}", f"rows: {mz.rows}",
              f"inicio: {nombre(mz.start)}", f"meta: {nombre(mz.goal)}", "celdas:"]
    for y in range(mz.rows - 1, -1, -1):
        fila = []
        for x in range(mz.cols):
            v = 0
            for bit, (dx, dy) in enumerate([(0, 1), (1, 0), (0, -1), (-1, 0)]):
                n = (x + dx, y + dy)
                if not mz.dentro(n) or wall((x, y), n) in conocidos:
                    v |= 1 << bit
            fila.append(str(v))
        lineas.append(f"  - [{', '.join(fila)}]   # fila {y+1}")
    with open(ruta_archivo, "w", encoding="utf-8") as f:
        f.write("\n".join(lineas) + "\n")


def exportar_plan_json(mz, cfg, plan, ruta_pista, ruta_archivo):
    """Exporta el plan de ejecucion (intento 2) a JSON -- solo como
    referencia/comparacion offline (por ejemplo contra lo que calcula
    robot/granprix_bot/speedrun_node.py en el robot real a partir de su
    propio mapa descubierto). Los "ticks" son informativos, no se usan
    para controlar el robot real (ver robot/README.md)."""
    import json
    import os

    pasos = []
    for p in plan:
        if p["cmd"] == "GIRAR":
            pasos.append({"cmd": "GIRAR", "grados": p["grados"], "lado": p["lado"],
                          "ticks_referencia": p["ticks"]})
        else:
            pasos.append({
                "cmd": "AVANZAR", "celdas": p["celdas"], "cm": p["cm"], "dir": p["dir"],
                "desde": p["desde"], "hasta": p["hasta"],
                "ticks_referencia": p["ticks"],
            })

    data = {
        "pista": os.path.basename(ruta_pista),
        "celda_cm": cfg.celda_cm,
        "celda_inicio": nombre(mz.start),
        "celda_meta": nombre(mz.goal),
        "heading_inicial": "NORTE",
        "pasos": pasos,
    }
    with open(ruta_archivo, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)
        f.write("\n")


if __name__ == "__main__":
    main()
