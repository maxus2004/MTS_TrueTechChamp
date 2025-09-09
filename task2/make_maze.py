from dataclasses import dataclass
from pathlib import Path
import random, os
from typing import List, Dict, Set, Tuple
from collections import deque

# --- стороны (битовая маска) ---
L, T, R, B = 1, 2, 4, 8

# ----- соответствие "номер на картинке" → стенки (битмаска) -----
# 1: слева; 2: сверху; 3: справа; 4: снизу
# 5: слева+снизу; 6: справа+снизу; 7: сверху+справа; 8: сверху+слева
# 9: слева+справа; 10: сверху+снизу
# 11: слева+справа+снизу (нет верха)
# 12: сверху+справа+снизу (нет лева)
# 13: сверху+слева+справа (нет низа)
# 14: сверху+слева+снизу (нет права)
# 15: нет стен; 16: все 4 стены
NUM2MASK: Dict[int, int] = {
    0: 0, 1: L, 2: T, 3: R, 4: B,
    5: L | B, 6: R | B, 7: T | R, 8: T | L,
    9: L | R, 10: T | B,
    11: L | R | B, 12: T | R | B, 13: T | L | R, 14: T | L | B,
    15: 0, 16: L | T | R | B,
}
MASK2NUM: Dict[int, int] = {v: k for k, v in NUM2MASK.items()}
MASK2NUM[0] = 0

# ---------------- Геометрия/параметры поля и стен ----------------
ARENA   = float(os.getenv("ARENA", "8.0"))
MARGIN  = float(os.getenv("MARGIN", "0.0"))

THICK   = float(os.getenv("THICK", "0.015"))
HEIGHT  = float(os.getenv("HEIGHT", "0.30"))
Z_CONST = float(os.getenv("Z_CONST", "0.12"))

# Центр сегмента чуть внутрь
EDGE_INSET = THICK / 2.0

# Стартовая клетка (row, col)
START_IJ: Tuple[int, int] = (
    int(os.getenv("START_ROW", "0")),
    int(os.getenv("START_COL", "0")),
)

# «ворота» на внешних границах
ENTRANCE_SPAN = int(os.getenv("ENTRANCE_SPAN", "3"))
EXIT_SPAN     = int(os.getenv("EXIT_SPAN", "3"))

ROTATION = "1 0 0 -1.5707953071795862"
WHITE = "1 1 1"

# ---------------- Цвета ----------------
def wall_color(idx: int) -> str:
    return WHITE

# ---------------- Комната 2×2 по центру с одним входом ----------------
def _apply_center_room(walls: List[List[int]], opening: str = "S") -> None:
    """Центр: комната 2×2 с одним входом (opening in {'N','S','W','E'})."""
    n, m = len(walls), len(walls[0])
    y0, y1 = n // 2 - 1, n // 2
    x0, x1 = m // 2 - 1, m // 2
    a = (y0, x0); b = (y0, x1); c = (y1, x0); d = (y1, x1)

    def set_cell(y: int, x: int, mask: int): walls[y][x] = mask
    def neighbor(y: int, x: int, side: int):
        if side == L and x - 1 >= 0:   return (y, x - 1, R)
        if side == R and x + 1 < m:    return (y, x + 1, L)
        if side == T and y - 1 >= 0:   return (y - 1, x, B)
        if side == B and y + 1 < n:    return (y + 1, x, T)
        return None
    def ensure_wall(y: int, x: int, side: int, on: bool):
        if on: walls[y][x] |= side
        else:  walls[y][x] &= ~side
        nb = neighbor(y, x, side)
        if nb is not None:
            ny, nx, nside = nb
            if on: walls[ny][nx] |= nside
            else:  walls[ny][nx] &= ~nside

    # базовый "замкнутый квадрат" (внутренние рёбра сняты)
    set_cell(*a, L | T); set_cell(*b, T | R)
    set_cell(*c, L | B); set_cell(*d, R | B)
    ensure_wall(*a, R, False); ensure_wall(*b, L, False)
    ensure_wall(*a, B, False); ensure_wall(*c, T, False)
    ensure_wall(*b, B, False); ensure_wall(*d, T, False)
    ensure_wall(*c, R, False); ensure_wall(*d, L, False)

    # закрываем внешний периметр
    for y, x, s in [(y0,x0,T),(y0,x1,T),(y1,x0,B),(y1,x1,B),(y0,x0,L),(y1,x0,L),(y0,x1,R),(y1,x1,R)]:
        ensure_wall(y, x, s, True)

    # один вход
    o = opening.upper()
    if   o == "N": ensure_wall(*a, T, False)
    elif o == "S": ensure_wall(*d, B, False)
    elif o == "W": ensure_wall(*c, L, False)
    elif o == "E": ensure_wall(*b, R, False)
    else: raise ValueError("opening must be one of 'N','S','W','E'")

# ---------------- Проверка достижимости (BFS) ----------------
def _neighbors(y: int, x: int, walls: List[List[int]]):
    h, w = len(walls), len(walls[0])
    mask = walls[y][x]
    if (mask & L) == 0 and x - 1 >= 0: yield (y, x - 1)
    if (mask & R) == 0 and x + 1 < w:  yield (y, x + 1)
    if (mask & T) == 0 and y - 1 >= 0: yield (y - 1, x)
    if (mask & B) == 0 and y + 1 < h:  yield (y + 1, x)

def _bfs_reachable(walls: List[List[int]], start: Tuple[int,int], targets: Set[Tuple[int,int]]) -> bool:
    h, w = len(walls), len(walls[0])
    sy, sx = start
    if not (0 <= sy < h and 0 <= sx < w): return False
    if (sy, sx) in targets: return True
    from collections import deque
    q = deque([(sy, sx)])
    seen = [[False]*w for _ in range(h)]
    seen[sy][sx] = True
    while q:
        y, x = q.popleft()
        for ny, nx in _neighbors(y, x, walls):
            if not seen[ny][nx]:
                if (ny, nx) in targets: return True
                seen[ny][nx] = True
                q.append((ny, nx))
    return False

# ---------------- Генерация ----------------
@dataclass
class Maze:
    n: int = 16
    m: int = 16
    seed: int = 42

    def generate_matrix(self) -> List[List[int]]:
        if self.seed is not None:
            random.seed(self.seed)
        else:
            random.seed()
        n, m = self.n, self.m

        # 1) карвим проходы (backtracker)
        grid = [[0 for _ in range(m)] for _ in range(n)]
        def carve(x: int, y: int):
            dirs = [L, T, R, B]
            random.shuffle(dirs)
            for d in dirs:
                nx = x + (1 if d == R else -1 if d == L else 0)
                ny = y + (1 if d == B else -1 if d == T else 0)
                if 0 <= nx < m and 0 <= ny < n and grid[ny][nx] == 0:
                    grid[y][x] |= d
                    grid[ny][nx] |= {L: R, R: L, T: B, B: T}[d]
                    carve(nx, ny)
        carve(0, 0)

        # 2) "проходы" → "стены"
        walls = [[L | T | R | B for _ in range(m)] for __ in range(n)]
        for y in range(n):
            for x in range(m):
                if grid[y][x] & L:
                    walls[y][x] &= ~L
                    if x > 0: walls[y][x-1] &= ~R
                if grid[y][x] & R:
                    walls[y][x] &= ~R
                    if x+1 < m: walls[y][x+1] &= ~L
                if grid[y][x] & T:
                    walls[y][x] &= ~T
                    if y > 0: walls[y-1][x] &= ~B
                if grid[y][x] & B:
                    walls[y][x] &= ~B
                    if y+1 < n: walls[y+1][x] &= ~T

        # 3) "ворота" на внешних границах (целые ячейки)
        for k in range(ENTRANCE_SPAN):
            if k < n: walls[k][0] &= ~L
        for k in range(EXIT_SPAN):
            if n-1-k >= 0: walls[n-1-k][m-1] &= ~R

        # 4) центральная комната 2×2 — выбираем вход, чтобы была достижимость от старта
        cy0, cy1 = n // 2 - 1, n // 2
        cx0, cx1 = m // 2 - 1, m // 2
        room_cells: Set[Tuple[int,int]] = {(cy0, cx0), (cy0, cx1), (cy1, cx0), (cy1, cx1)}
        sy, sx = START_IJ
        for opening in ("N","E","S","W"):
            walls_try = [row[:] for row in walls]
            _apply_center_room(walls_try, opening)
            if _bfs_reachable(walls_try, (sy, sx), room_cells):
                walls = walls_try
                break
        else:
            raise AssertionError("central room unreachable")

        for y in range(n):
            for x in range(m):
                w = walls[y][x]
                if x + 1 < m:
                    assert bool(w & R) == bool(walls[y][x+1] & L)
                if y + 1 < n:
                    assert bool(w & B) == bool(walls[y+1][x] & T)

        return [[MASK2NUM[walls[y][x]] for x in range(m)] for y in range(n)]

# --------- вывод в Webots ---------
def _wall_x(xc: float, y: float, Llen: float, color: str, name: str) -> str:
    return f"""
  Solid {{
    translation {xc:.5f} {y:.5f} {Z_CONST:.2f}
    rotation {ROTATION}
    children [
      Shape {{
        appearance PBRAppearance {{ baseColor {color} roughness 0 metalness 0 }}
        geometry Box {{ size {Llen:.5f} {HEIGHT:.5f} {THICK:.5f} }}
      }}
    ]
    name "{name}"
    boundingObject Box {{ size {Llen:.5f} {HEIGHT:.5f} {THICK:.5f} }}
  }}""".rstrip()

def _wall_z(xc: float, y: float, Llen: float, color: str, name: str) -> str:
    return f"""
  Solid {{
    translation {xc:.5f} {y:.5f} {Z_CONST:.2f}
    rotation {ROTATION}
    children [
      Shape {{
        appearance PBRAppearance {{ baseColor {color} roughness 0 metalness 0 }}
        geometry Box {{ size {THICK:.5f} {HEIGHT:.5f} {Llen:.5f} }}
      }}
    ]
    name "{name}"
    boundingObject Box {{ size {THICK:.5f} {HEIGHT:.5f} {Llen:.5f} }}
  }}""".rstrip()

def write_proto_from_matrix(matrix: List[List[int]], out_path: Path = Path("protos/GeneratedMaze.proto")) -> None:
    n, m = len(matrix), len(matrix[0])
    avail = ARENA - 2 * MARGIN
    cell_w = avail / m
    cell_h = avail / n

    EPS = 1e-6
    Lx = cell_w - EPS
    Lz = cell_h - EPS

    x_left_line     = -ARENA / 2 + MARGIN
    x_line_center0  = x_left_line + EDGE_INSET
    y_top_line      = +ARENA / 2 - MARGIN
    y_line_center0  = y_top_line - EDGE_INSET
    y_row_center0   = y_top_line - (cell_h / 2.0)

    mask = [[NUM2MASK[v] for v in row] for row in matrix]
    walls_txt: List[str] = []
    wid = 0

    for j in range(n):
        cy = y_row_center0 - j * cell_h
        for i in range(m):
            cx_left_line_center  = x_line_center0 + i * cell_w
            cx_right_line_center = x_line_center0 + (i + 1) * cell_w
            y_top_line_center    = y_line_center0 - j * cell_h
            y_bot_line_center    = y_line_center0 - (j + 1) * cell_h
            w = mask[j][i]

            if w & L:
                color = wall_color(wid); wid += 1
                walls_txt.append(_wall_z(cx_left_line_center, cy, Lz, color, f"cell{j}_{i}_L"))

            if w & T:
                color = wall_color(wid); wid += 1
                cx = x_left_line + cell_w/2 + i*cell_w
                walls_txt.append(_wall_x(cx, y_top_line_center, Lx, color, f"cell{j}_{i}_T"))

            if i == m - 1 and (w & R):
                color = wall_color(wid); wid += 1
                walls_txt.append(_wall_z(cx_right_line_center, cy, Lz, color, f"cell{j}_{i}_R"))

            if j == n - 1 and (w & B):
                color = wall_color(wid); wid += 1
                cx = x_left_line + cell_w/2 + i*cell_w
                walls_txt.append(_wall_x(cx, y_bot_line_center, Lx, color, f"cell{j}_{i}_B"))

    body = "\n".join(walls_txt)
    proto = f"""#VRML_SIM R2025a utf8

PROTO GeneratedMaze [ ] {{
  Group {{
    children [
{body}
    ]
  }}
}}
"""
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(proto, encoding="utf-8")
    print(f"[ok] wrote {out_path}")

def write_cones_proto(
    out_path: Path = Path("protos/GeneratedCones.proto"),
    count: int = 10,
    seed: int = None,
) -> None:
    """Случайные подвижные конусы внутри арены."""
    if seed is not None:
        random.seed(seed)
    else:
        random.seed()  # системное зерно

    cones_txt: List[str] = []
    for i in range(count):
        x = random.uniform(-ARENA/2 + 0.5, ARENA/2 - 0.5)
        y = random.uniform(-ARENA/2 + 0.5, ARENA/2 - 0.5)
        cones_txt.append(f"""
  TrafficCone {{
    translation {x:.2f} {y:.2f} 0.0
    name "cone_{i}"
  }}""".rstrip())

    body = "\n".join(cones_txt)
    proto = f"""#VRML_SIM R2025a utf8

EXTERNPROTO "../protos/TrafficCone.proto"

PROTO GeneratedCones [ ] {{
  Group {{
    children [
{body}
    ]
  }}
}}
"""
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(proto, encoding="utf-8")
    print(f"[ok] wrote {out_path} with {count} cones (seed={seed})")


# ---------------- запуск ----------------
if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=42, help="Seed для генерации лабиринта")
    ap.add_argument("--cones", type=int, default=10, help="Количество конусов")

    args = ap.parse_args()

    maze = Maze(seed=args.seed)
    matrix = maze.generate_matrix()

    write_proto_from_matrix(matrix)
    write_cones_proto(count=args.cones, seed=args.seed)

