# -*- coding: utf-8 -*-
"""Leitura e parsing das poses absolutas do mundo no Gazebo Harmonic."""

import subprocess
import time


def parsear_pose_info_gz(texto):
    """Faz o parsing do texto protobuf retornado por `gz topic -e -t .../pose/info -n 1`."""
    poses = {}
    depth = 0
    in_pose = False
    section = None
    nome_atual = None
    pos_atual = None
    ori_atual = None

    for raw_line in texto.splitlines():
        line = raw_line.strip()
        if not line:
            continue

        if line.startswith('pose {'):
            in_pose = True
            depth = 1
            section = None
            nome_atual = None
            pos_atual = {'x': 0.0, 'y': 0.0, 'z': 0.0}
            ori_atual = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
            continue

        if not in_pose:
            continue

        if line == '}':
            depth -= 1
            if depth == 1:
                section = None
            elif depth == 0:
                if nome_atual is not None:
                    poses[nome_atual] = {'position': pos_atual, 'orientation': ori_atual}
                in_pose = False
            continue

        if line.endswith('{'):
            depth += 1
            if depth == 2:
                if line.startswith('position'):
                    section = 'position'
                elif line.startswith('orientation'):
                    section = 'orientation'
            continue

        if ':' in line:
            chave, valor = line.split(':', 1)
            chave = chave.strip()
            valor = valor.strip().strip('"')
            if depth == 1 and chave == 'name':
                nome_atual = valor
            elif depth == 2 and section == 'position' and chave in pos_atual:
                pos_atual[chave] = float(valor)
            elif depth == 2 and section == 'orientation' and chave in ori_atual:
                ori_atual[chave] = float(valor)

    return poses


def ler_poses_absolutas_gazebo(world_name='iris_runway', tentativas=3, timeout_s=5.0):
    """Lê as poses absolutas reais de `drone_coaxial` e `armadilha_alvo` no mundo Gazebo."""
    topico = f'/world/{world_name}/pose/info'
    for _ in range(tentativas):
        try:
            proc = subprocess.run(
                ['gz', 'topic', '-e', '-t', topico, '-n', '1'],
                capture_output=True,
                text=True,
                timeout=timeout_s,
            )
            if proc.returncode == 0 and proc.stdout:
                poses = parsear_pose_info_gz(proc.stdout)
                if 'drone_coaxial' in poses and 'armadilha_alvo' in poses:
                    return poses
        except Exception:
            pass
        time.sleep(0.2)
    return None
